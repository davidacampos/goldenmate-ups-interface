#include <HIDPowerDevice.h>    // https://github.com/abratchik/HIDPowerDevice
#include <SPI.h>
#include <usbhub.h>       // https://github.com/felis/USB_Host_Shield_2.0
#include <hiduniversal.h>   // https://github.com/felis/USB_Host_Shield_2.0

#define MINUPDATEINTERVAL   26
#define CHGDCHPIN           4
#define RUNSTATUSPIN        5
#define COMMLOSTPIN         10
#define BATTSOCPIN          A7

// String constants
const char STRING_DEVICECHEMISTRY[] PROGMEM = "Other";
const char STRING_OEMVENDOR[] PROGMEM = "GoldenMate";
const char STRING_SERIAL[] PROGMEM = "UPS10";

const byte bDeviceChemistry = IDEVICECHEMISTRY;
const byte bOEMVendor = IOEMVENDOR;

PresentStatus iPresentStatus = {};

byte bRechargable = 1;
byte bCapacityMode = 2;  // Units are in %%
uint16_t iRunTimeToEmpty = 0;
uint16_t iManufacturerDate = 0; // Initialized in setup()
byte iFullChargeCapacity = 100;
byte iRemaining = 100;
bool externalUpsConnected = false;

enum State {
  WAITING_FOR_EXTERNAL_UPS,
  RUNNING
};

State initialization_state = WAITING_FOR_EXTERNAL_UPS;

USB Usb;
HIDUniversal Hid(&Usb);

// Debounce variables
unsigned long batteryChangeStart = 0;
bool pendingBattery = false;

// UPS state
struct UPSReport {
  bool onBattery;
  uint8_t charge; // %
  uint8_t load; // Watts
};

UPSReport currentUPS = {false, 100, 0};
UPSReport filteredUPS = {false, 100, 0};
UPSReport latestUPS = {false, 100, 255}; // This is updated from parsing USB interface

// --- HID report parser ---
class UPSParser : public HIDReportParser {
    void Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) override {
      if (len < 6) return; // Ensure buffer has enough data

      externalUpsConnected = true;

      latestUPS.onBattery = buf[5] == 0;
      latestUPS.charge = buf[2]; // Charge in %
      latestUPS.load = buf[4]; // Load in Watts, only when in battery mode
      if (latestUPS.charge == 99) latestUPS.charge = 100; // Normalize charge as the UPS commonly stays at 99% instead of hitting 100%
    }
};

UPSParser upsParser;

void sendToPC(UPSReport report);
void externalUpsDisconnected();

void setup() {

  Serial.begin(115200);

  // Start UPS interface with host machine
  PowerDevice.begin();

  // Serial No is set in a special way as it forms Arduino port name
  PowerDevice.setSerial(STRING_SERIAL);

  //  // Used for debugging purposes.
  //  PowerDevice.setOutput(Serial);

  PowerDevice.setFeature(HID_PD_PRESENTSTATUS, &iPresentStatus, sizeof(iPresentStatus));
  PowerDevice.setFeature(HID_PD_RUNTIMETOEMPTY, &iRunTimeToEmpty, sizeof(iRunTimeToEmpty));
  PowerDevice.setFeature(HID_PD_RECHARGEABLE, &bRechargable, sizeof(bRechargable));
  PowerDevice.setFeature(HID_PD_CAPACITYMODE, &bCapacityMode, sizeof(bCapacityMode));
  PowerDevice.setStringFeature(HID_PD_IDEVICECHEMISTRY, &bDeviceChemistry, STRING_DEVICECHEMISTRY);
  PowerDevice.setStringFeature(HID_PD_IOEMINFORMATION, &bOEMVendor, STRING_OEMVENDOR);
  PowerDevice.setFeature(HID_PD_FULLCHRGECAPACITY, &iFullChargeCapacity, sizeof(iFullChargeCapacity));

  // This MUST be called, as it's different from the sendReport for HID_PD_REMAININGCAPACITY. Without this it will always be 0
  PowerDevice.setFeature(HID_PD_REMAININGCAPACITY, &iRemaining, sizeof(iRemaining));

  uint16_t year = 2025, month = 10, day = 1;
  iManufacturerDate = (year - 1980) * 512 + month * 32 + day; // From 4.2.6 Battery Settings in "Universal Serial Bus Usage Tables for HID Power Devices"
  PowerDevice.setFeature(HID_PD_MANUFACTUREDATE, &iManufacturerDate, sizeof(iManufacturerDate));

  // Initialize the UPS as if it was on AC with a full battery
  iPresentStatus.Charging = true;
  iPresentStatus.ACPresent = true;
  iPresentStatus.FullyCharged = true;
  iPresentStatus.Discharging = false;
  iPresentStatus.BatteryPresent = 1;
  PowerDevice.sendReport(HID_PD_PRESENTSTATUS, &iPresentStatus, sizeof(iPresentStatus));

  if (Usb.Init() == -1) {
    Serial.println(F("USB Host Shield init failed"));
    while (1);
  }

  Hid.SetReportParser(0, &upsParser); // attach parser
  Serial.println(F("UPS ready"));
}

void loop() {

  Usb.Task();

  UPSReport newReport = latestUPS;

  switch (initialization_state) {
    case WAITING_FOR_EXTERNAL_UPS: {

        // Give it up to 5 seconds to get data from the external UPS...
        if (millis() > 5000) {
          if (externalUpsConnected == true) {

            Serial.println(F("UPS connected"));

            // Send the latest values coming from the external UPS
            sendToPC(newReport);

            initialization_state = RUNNING;

          } else { // If after waiting, were are still not able to get any data from the external UPS, assume it's not connected

            externalUpsDisconnected();
          }
        }

        break;
      }

    case RUNNING: {

        if (Usb.getUsbTaskState() == USB_STATE_RUNNING) {

          // Check if battery or charge status changed
          if ((newReport.onBattery != currentUPS.onBattery) || (newReport.charge != currentUPS.charge)) {
            currentUPS = newReport;

            if (newReport.onBattery) {
              // In a few seconds, confirm if it's still "on battery", to debounce "false on battery" messages coming from the UPS
              pendingBattery = true;
              batteryChangeStart = millis();
            } else {
              // Back to AC immediately
              filteredUPS = newReport;
              sendToPC(filteredUPS);
              pendingBattery = false;
            }
          }

          // If pending battery, check debounce
          if (pendingBattery && millis() - batteryChangeStart >= 2000) {
            if (latestUPS.onBattery) {
              filteredUPS = latestUPS;
              sendToPC(filteredUPS);
            }
            pendingBattery = false;
          }

          break;
          
        } else {
          Serial.println(F("No external UPS connected"));
          externalUpsDisconnected();
        }
      }
  }
}

// --- Send UPS data to PC ---
void sendToPC(UPSReport report) {

  // Serial.print(F("→ Host update: "));
  // Serial.print(report.onBattery ? "On Battery, " : "On AC, ");
  // Serial.print(report.charge);
  // Serial.println(F("%"));

  //*********** Prepare data from latest values ****************************
  bool bCharging = report.onBattery == false;
  bool bACPresent = bCharging;
  bool bDischarging = !bCharging;

  iRemaining = (byte)(report.charge);
  iRunTimeToEmpty = (uint16_t)round((float)230 / report.load * 60); // The GoldenMate is roughly 230Wh, so calculate in minutes, based on the current load

  iPresentStatus.Charging = bCharging;
  iPresentStatus.ACPresent = bACPresent;
  iPresentStatus.FullyCharged = (iRemaining == 100);
  iPresentStatus.Discharging = (bDischarging ? 1 : 0);
  iPresentStatus.BatteryPresent = 1;

  //************ Send to host (i.e. computer) ***********************

  PowerDevice.sendReport(HID_PD_REMAININGCAPACITY, &iRemaining, sizeof(iRemaining));
  if (bDischarging) PowerDevice.sendReport(HID_PD_RUNTIMETOEMPTY, &iRunTimeToEmpty, sizeof(iRunTimeToEmpty));
  PowerDevice.sendReport(HID_PD_PRESENTSTATUS, &iPresentStatus, sizeof(iPresentStatus));

  //Serial.println(iRemaining);
  //Serial.println(iRunTimeToEmpty);
}

// --- Tells PC the UPS is "disconnected" ---
void externalUpsDisconnected() {
  Serial.println(F("UPS not connected"));

  // Tell the host the UPS is not working by telling there is no charge remaining and no battery present
  iPresentStatus.Charging = false;
  iPresentStatus.ACPresent = false;
  iPresentStatus.FullyCharged = false;
  iPresentStatus.Discharging = true;
  iPresentStatus.BatteryPresent = 0;
  iRemaining = 0;

  PowerDevice.sendReport(HID_PD_REMAININGCAPACITY, &iRemaining, sizeof(iRemaining));
  PowerDevice.sendReport(HID_PD_PRESENTSTATUS, &iPresentStatus, sizeof(iPresentStatus));

  delay(1000); // Wait 1 second before continuing execution...
}
