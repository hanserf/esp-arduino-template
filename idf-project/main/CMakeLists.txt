set(COMPONENT_SRCS "main.cpp" "system/idf_main.c" "ota/softap_otaserver.c" "system/button.c" )
set(COMPONENT_ADD_INCLUDEDIRS "."
                              system/.
                              ota/.
                              EMBED_FILES ota/index.html
                              )

register_component()

