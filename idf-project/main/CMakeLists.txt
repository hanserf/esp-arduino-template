set(COMPONENT_SRCS "main.cpp" "system/idf_main.c" "ota/softap_otaserver.c" "system/button.c" )
set(COMPONENT_ADD_INCLUDEDIRS "."
                              $ENV{IDF_PATH}/examples/common_components/protocol_examples_common/include/.
                              system/.
                              ota/.
                              EMBED_FILES ota/index.html
                              )

register_component()

