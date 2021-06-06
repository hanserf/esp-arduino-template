set(COMPONENT_SRCS "system/idf_main.c"
                   #"system/tcp_console_driver.c"
                   "system/my_encoder.c"
                   "ota/softap_otaserver.c"
                   "system/button.c" 
                   #"app_console.cpp"
                   "main.cpp")
set(COMPONENT_ADD_INCLUDEDIRS "."
                              system/.
                              ota/.
                              arduino_wrappers/.
                              EMBED_FILES ota/index.html
                              $ENV{IDF_PATH}/examples/common_components/protocol_examples_common/
                              )

register_component()

