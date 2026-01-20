#include <HIDPowerDevice.h>    // https://github.com/abratchik/HIDPowerDevice
#include <SPI.h>
#include <usbhub.h>       // https://github.com/felis/USB_Host_Shield_2.0
#include <hiduniversal.h>   // https://github.com/felis/USB_Host_Shield_2.0
#include <avr/wdt.h>

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
bool firstHidReportComplete = false;

unsigned long start;
bool forceReset = false;

enum State {
  WAITING_FOR_EXTERNAL_UPS,
  RUNNING
};

State initialization_state = WAITING_FOR_EXTERNAL_UPS;

USB Usb;
HIDUniversal Hid(&Usb);

// Debounce variables for initial event
unsigned long upsStableSince = 0;
bool isUpsStateStable = false;
const unsigned long UPS_STABLE_DEBOUNCE_TIME_MS = 2000;

// Debounce variables for constant events
unsigned long batteryChangeSince = 0;
bool isBatteryChangePending = false;
const unsigned long UPS_BATTERY_CHANGE_DEBOUNCE_TIME_MS = 2000;

// These flags keep track on when was the last time particular events happened, to decide if the Arduino should be auto-reset
unsigned long upsLastReportSince = 0;

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

      upsLastReportSince = millis();

      if (firstHidReportComplete == true) {

        latestUPS.onBattery = buf[5] == 0;
        latestUPS.charge = buf[2]; // Charge in %
        latestUPS.load = buf[4]; // Load in Watts, only when in battery mode
        if (latestUPS.charge == 99) latestUPS.charge = 100; // Normalize charge as the UPS commonly stays at 99% instead of hitting 100%

        externalUpsConnected = true;

      } else {
        firstHidReportComplete = true; // Ignore the first HID report, as it seems it's reporting "on battery" for an instant even if "on AC"
      }
    }
};

UPSParser upsParser;

void sendToPC(UPSReport report);
void externalUpsDisconnected();

void setup() {

  MCUSR &= ~(1 << WDRF); // Clear any previous WDT reset
  wdt_disable(); // Disable watchdog while booting/uploading

  Serial.begin(115200);

  start = millis();

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
  iPresentStatus.Charging = 1;
  iPresentStatus.Discharging = 0;
  iPresentStatus.ACPresent = 1;
  iPresentStatus.BatteryPresent = 1;
  iPresentStatus.FullyCharged = 1;
  PowerDevice.sendReport(HID_PD_PRESENTSTATUS, &iPresentStatus, sizeof(iPresentStatus));

  if (Usb.Init() == -1) {
    Serial.println(F("USB Host Shield init failed"));
    while (1);
  }

  Hid.SetReportParser(0, &upsParser); // Attach parser
  Serial.println(F("UPS ready"));

  delay(1000);
  wdt_enable(WDTO_8S); // Enable watchdog for every 8 seconds

}

void loop() {

  wdt_reset(); // Call the watchdog to avoid the device's auto reset. This needs to be called at least every 8 seconds to avoid auto resetting the device

  // After ~24 hours of uptime, lock the loop that way wdt_reset() is no longer called, causing the watchdog to reset the Arduino
  if ((millis() - start >= 86400000UL) || forceReset == true) {
    Serial.println(F("Auto-resetting Arduino device..."));
    while (1);
  }

  Usb.Task();

  UPSReport newReport = latestUPS;

  switch (initialization_state) {
    case WAITING_FOR_EXTERNAL_UPS: {

        // Give it up to 30 seconds to get data from the external UPS...
        if (millis() > 30000) {
          if (externalUpsConnected == true) {

            Serial.println(F("External UPS connected"));

            // Wait until UPS state is stable before reporting
            if (!isUpsStateStable) {
              if (newReport.onBattery == filteredUPS.onBattery && newReport.charge == filteredUPS.charge) {

                if (upsStableSince == 0) {
                  upsStableSince = millis();
                }

                if (millis() - upsStableSince >= UPS_STABLE_DEBOUNCE_TIME_MS) {

                  Serial.println(F("External UPS connected and stable. Reporting first real status"));

                  isUpsStateStable = true;

                  // Send the latest values coming from the external UPS
                  sendToPC(newReport);
                  initialization_state = RUNNING;
                }

              } else {

                Serial.println(F("External UPS connected but not stable"));

                // Reset stabilization timer
                filteredUPS = newReport;
                upsStableSince = 0;
              }
            }


          } else { // If after waiting, were are still not able to get any data from the external UPS, assume it's not connected

            externalUpsDisconnected();
          }
        }

        break;
      }

    case RUNNING: {

        uint8_t usbState = Usb.getUsbTaskState();

        if (millis() - upsLastReportSince >= 300000) {
          Serial.println(F("No external UPS report in 5 min. Will trigger an Arduino reset"));
          forceReset = true;
        }

        if (usbState == USB_STATE_DETACHED) {
          Serial.println(F("No external USB device connected"));
          externalUpsDisconnected();
          initialization_state = WAITING_FOR_EXTERNAL_UPS;

        } else {

          // Check if battery or charge status changed
          if ((newReport.onBattery != currentUPS.onBattery) || (newReport.charge != currentUPS.charge)) {
            currentUPS = newReport;

            if (newReport.onBattery) {
              // In a few seconds, confirm if it's still "on battery", to debounce "false on battery" messages coming from the UPS
              isBatteryChangePending = true;
              batteryChangeSince = millis();
            } else {
              // Back to AC immediately
              filteredUPS = newReport;
              sendToPC(filteredUPS);
              isBatteryChangePending = false;
            }
          }

          // If pending battery, check debounce
          if (isBatteryChangePending && millis() - batteryChangeSince >= UPS_BATTERY_CHANGE_DEBOUNCE_TIME_MS) {
            if (latestUPS.onBattery) {
              filteredUPS = latestUPS;
              sendToPC(filteredUPS);
            }
            isBatteryChangePending = false;
          }
        }

        break;
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
  Serial.println(F("External UPS not connected"));

  // Tell the host the UPS is not working by telling there is no charge remaining and no battery present
  iPresentStatus.Charging = 0;
  iPresentStatus.ACPresent = 0;
  iPresentStatus.FullyCharged = 0;
  iPresentStatus.Discharging = 0;
  iPresentStatus.BatteryPresent = 0;
  iRemaining = 0;

  PowerDevice.sendReport(HID_PD_REMAININGCAPACITY, &iRemaining, sizeof(iRemaining));
  PowerDevice.sendReport(HID_PD_PRESENTSTATUS, &iPresentStatus, sizeof(iPresentStatus));

  delay(1000); // Wait 1 second before continuing execution...
}
