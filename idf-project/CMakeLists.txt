# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
set(EXTRA_COMPONENT_DIRS components/extra_components/components)                         
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(aeolusFW)
#extra_component is https://github.com/UncleRus/esp-idf-lib
set(EXTRA_COMPONENT_DIRS components/arduino
                         components/esp_littlefs
			             components/extra_components/components/encoder
                         $ENV{IDF_PATH}/components/nvs_flash                         
                         $ENV{IDF_PATH}/components/console
                         $ENV{IDF_PATH}/components/cmd_nvs
                         $ENV{IDF_PATH}/components/cmd_system)       
                         
# $ENV{IDF_PATH}/examples/system/console/components
                        
