#!/bin/sh
command -v arduino-cli >/dev/null 2>&1 || { echo >&2 "arduino-cli not found. Aborting."; exit 1; }
arduino-cli config --additional-urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json init
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli upgrade
arduino-cli lib install ArduinoJson WebSockets uBitcoin base64 Nostr ESP32Servo
arduino-cli compile --build-path build --fqbn esp32:esp32:esp32 --library libraries/ESP32Ping --library libraries/TFT_eSPI nostrasia-iot
# https://github.com/micro-bitcoin/uBitcoin.git#master
# bblanchon/ArduinoJson@^6.21.0
# links2004/WebSockets@^2.3.7
# densaugeo/base64@^1.4.0
# marian-craciunescu/ESP32Ping@^1.7
# lnbits/Nostr@^0.2.0
# madhephaestus/ESP32Servo@^1.1.0
