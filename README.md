# mitsubishi2MQTT
Use MQTT and ESP8266/ESP32 module to control Mitsubishi HVAC unit.
It use SwiCago librairies: https://github.com/SwiCago/HeatPump

***
Features:
 - Initial config:  WIFI AP mode and web portal
 - Web interface for configuration, status and control, firmware upgrade
 - Homeassistant autodiscovery and control with MQTT

***
Screenshots:

Main Page
![](https://github.com/gysmo38/mitsubishi2MQTT/blob/master/images/main_page.png)

Control page
![](https://github.com/gysmo38/mitsubishi2MQTT/blob/master/images/control_page.png)

Config page
![](https://github.com/gysmo38/mitsubishi2MQTT/blob/master/images/config_page.png)

***
How to use:
 - Step 1: flash the sketch with flash size include SPIFFS option.
 - Step 2: connect to device AP with name HVAC_XXXX (XXXX last 4 character MAC address)
 - Step 3: You should be automatically redirected to the web portal or go to 192.168.1.1
 - Step 4: set Wifi information, save & reboot. Fall back to AP mode if WiFi connection fails (AP password sets to default SSID name from step 2).
 - Step 5: find the device IP with last 4 character MAC address in your router
 - Step 6: (optional): Set MQTT information for use with Home Assistant
 - Step 7: (optional): Set Login password to prevent unwanted access in SETUP->ADVANCE->Login Password

***
Upgrading from 2025.1.0 (stability and security update):
 - Existing settings are kept. Config files stay on SPIFFS with the same names; saves are now atomic (`.tmp`, previous version kept as `.bak`).
 - Web login uses a random session token (1 hour inactivity timeout). Everyone is logged out after the update or a reboot.
 - Passwords are never shown in the web pages. Leave `********` in a password field to keep the stored value.
 - ArduinoOTA (network upload from PlatformIO/Arduino IDE) only runs when an OTA password is set in SETUP->WIFI. Browser upload via Firmware Upgrade still works.
 - Reboot and Factory Reset only work from the buttons in the web UI (POST), not from a link.
 - `topic/custom/send` is ignored unless debug mode is on (`topic/debug/set on`).
 - Home Assistant: Power and Room temperature sensors report `state_class: measurement`, and every entity becomes unavailable when the device goes offline.
 - Build: `pio run -e esp07 -e esp12e -e wifikit-serial-esp32-s3`. Host tests: `test/run_host_tests.sh`. ESP32-S3 merged image: `"bin release/merge_command.sh"`.

***
For nodered fans MQTT topic use cases
- topic/power/set OFF
- topic/mode/set AUTO HEAT COOL DRY FAN_ONLY OFF ON
- topic/temp/set 16-31
- topic/remote_temp/set also called "room_temp", the implementation defined in "HeatPump" seems not work in some models
- topic/fan/set 1-4 AUTO QUIET
- topic/vane/set 1-5 SWING AUTO
- topic/wideVane/set << < | > >>
- topic/settings
- topic/state
- topic/debug
- topic/debug/set on off
- topic/custom/send (debug mode only) as example "fc 42 01 30 10 02 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 7b " see https://github.com/SwiCago/HeatPump/blob/master/src/HeatPump.h
