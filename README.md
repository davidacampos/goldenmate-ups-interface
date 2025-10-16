# goldenmate-ups-interface

## Why?

The `GOLDENMATE 1000VA/800W Lithium UPS Battery Backup and Surge Protector, Sinewave UPS System with LiFePO4 Battery(230.4 Wh), 2025 Upgraded Pro Ver. with Communication Port, 8 Outlets, LCD Display` works well in general from a hardware perspective, but it has a buggy firmware with respect to its USB comm port. This might also be applicable for other models for this same brand...

### Issues

- It keeps flapping between "On Battery" and "On AC" every few minutes, up to 4-5 times an hour
- It just briefly hits 100% charge and then immedtialy drops to 99%, which seems to be why the previous issue happens
   
## What and How?

This is a hardware-based solution to fix those issues in an otherwise good UPS. An Arduino Leonardo is used to interface between both, the physical UPS and the host (i.e. Windows). It fetches the data from the UPS, normalizes it, and then sends that to the host, reducing the noise.

### Needed hardware

- Arduino Leonardo (as it's a model that supports shields + it can be *seen* by a host as an HID device)
- USB Host Shield 2.0

### Needed Software

- At least Arduino IDE 1.x
- https://github.com/abratchik/HIDPowerDevice
- https://github.com/felis/USB_Host_Shield_2.0

### Installation

- Upload the single .ino, through an IDE that has the dependent libraries already configured, into an Arduino that has a USB Host Shield 2.0
- Connect the GOLDENAMTE UPS into USB Host Shield
- Connect the Arduino into the host (e.g Windows computer)
