## AeolusFW:


### Develop enviromnemt
Navigate to where you want the Espressif library installed:
```
git clone https://github.com/espressif/esp-idf.git
git checkout release/v4.0
git submodule init
git submodule update --recursive
./install.sh
. ./export.sh

```
#### Components
https://github.com/espressif/arduino-esp32/blob/master/docs/esp-idf_component.md
#### Expand root CMAKE :
```
# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(AeolusFW)
set(EXTRA_COMPONENT_DIRS "components/arduino")
```

### Build:
``` bash
idf.py build
# Sort Pin0 under reset to activate system bootloader mode.
idf.py flash

```
``` bash
export PYTHONPATH="$IDF_PATH/tools:$IDF_PATH/tools/ci/python_packages:$PYTHONPATH"
```
https://github.com/espressif/esp-idf/blob/master/tools/ci/python_packages/tiny_test_fw/requirements.txt
