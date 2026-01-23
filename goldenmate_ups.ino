#include <HIDPowerDevice.h>   // https://github.com/abratchik/HIDPowerDevice
#include <SPI.h>
#include <usbhub.h>           // https://github.com/felis/USB_Host_Shield_2.0
#include <hiduniversal.h>     // https://github.com/felis/USB_Host_Shield_2.0
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

bool forceReset = false;
bool inititalStatusSentToHost = false;

// These variables keep track on when was the last time particular events happened to decide if the Arduino should be auto-reset
uint16_t externalUpsDisconnectedReportCount = 0;
unsigned long upsLastReportSince = 0; 

USB Usb;
HIDUniversal Hid(&Usb);

// UPS state
struct UPSReport {
  bool onBattery;
  uint8_t charge; // %
  uint8_t load; // Watts
};

void sendToPC(UPSReport report);
void externalUpsDisconnected();

UPSReport previousUpsStatus = {false, 255, 255};

// --- HID report parser ---
class UPSParser : public HIDReportParser {
    void Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) override {
      UPSReport currentUpsStatus;

      if (len < 6) return; // Ensure buffer has enough data

      upsLastReportSince = millis();

      currentUpsStatus.onBattery = buf[5] == 0;
      currentUpsStatus.charge = buf[2]; // Charge in %
      currentUpsStatus.load = buf[4]; // Load in Watts, only when in battery mode
      if (currentUpsStatus.charge == 99) currentUpsStatus.charge = 100; // Normalize charge as the UPS commonly stays at 99% instead of hitting 100%

      if (previousUpsStatus.charge != 255 && previousUpsStatus.load != 255) { // Check if previous status has been initialized

        // The external UPS sends a status report every ~1sec, but it sometimes flaps so we want to submit the report to the host only after there are at least 2 consecutive values that are the same
        if (currentUpsStatus.onBattery == previousUpsStatus.onBattery) {

          // Send the "normalized" UPS status to host PC
          sendToPC(currentUpsStatus);

          if (inititalStatusSentToHost == false) {
            Serial.println(F("External UPS initial status succesfully sent to host"));
            inititalStatusSentToHost = true;
          }

        } else {
          Serial.println(F("External UPS 'OnBattery' status changed but waiting for the next status update to confirm it was not only a flap..."));

        }
      } else {
        Serial.println(F("External UPS status not yet defined"));

      }

      // Keep track of current UPS status for next check
      previousUpsStatus = currentUpsStatus;

    }
};

UPSParser upsParser;

void setup() {

  MCUSR &= ~(1 << WDRF); // Clear any previous WDT reset
  wdt_disable(); // Disable watchdog while booting/uploading

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

  // Lock the loop that way wdt_reset() is no longer called, causing the watchdog to reset the Arduino
  if (forceReset == true) {
    Serial.println(F("Auto-resetting Arduino device in ~8 seconds..."));
    while (1);
  }

  Usb.Task();
  uint8_t usbState = Usb.getUsbTaskState();

  if (millis() - upsLastReportSince >= 300000) {
    Serial.println(F("No external UPS report in 5 min. Will trigger an Arduino reset"));
    forceReset = true;
  }

  if (externalUpsDisconnectedReportCount >= 60) {
    Serial.println(F("External UPS has been disconnected for 1 min. Will trigger an Arduino reset"));
    forceReset = true;
  }

  if (usbState != USB_STATE_RUNNING) {
    externalUpsDisconnected();
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

  externalUpsDisconnectedReportCount = 0;
}

// --- Tells PC the UPS is "disconnected" ---
void externalUpsDisconnected() {
  Serial.print(F("External UPS not connected. USB status:"));
  Serial.println(Usb.getUsbTaskState());

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

  // Count how many consecutive times (i.e. without sending a regular UPS report) the "disconnected" report has been sent to the host... to trigger an Arduino reset if needed
  externalUpsDisconnectedReportCount = externalUpsDisconnectedReportCount + 1;
}
