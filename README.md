# Clickfy – BLE Multimedia Remote

Clickfy is a compact, keychain-sized multimedia remote based on the ESP32-C3. It communicates through the BLE HID profile and provides dedicated controls for play/pause, next track and previous track. A custom PCB integrates the control buttons, Li-Po battery, status LED and battery-voltage monitoring circuit. The firmware uses sleep modes to reduce power consumption, with development focused on minimizing wake-up and BLE reconnection times.

## Technologies

`ESP32-C3` `C/C++` `Bluetooth Low Energy` `BLE HID` `Li-Po battery` `ADC battery monitoring` `Custom PCB` `Low-power operation`

## Usage and development status

After powering on the device, pair Clickfy with a phone, computer or another BLE-compatible media device and use the three buttons to control playback. The current prototype has been built and tested. Planned improvements include further power optimization, shorter reconnection times and support for additional multimedia functions.
