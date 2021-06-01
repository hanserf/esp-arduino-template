#!/bin/bash
sed -i "s:set(includedirs:set(includedirs\n  tools/sdk/esp32/include/esp_rainmaker/include\n  tools/sdk/esp32/include/qrcode/include:g" components/arduino/CMakeLists.txt
