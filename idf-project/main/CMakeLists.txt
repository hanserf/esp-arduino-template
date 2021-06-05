set(COMPONENT_SRCS "system/idf_main.c"
                   "system/tcp_console.c"
                   "ota/softap_otaserver.c"
                   "system/button.c" 
                   "app_console.cpp"
                   "main.cpp")
set(COMPONENT_ADD_INCLUDEDIRS "."
                              system/.
                              ota/.
                              EMBED_FILES ota/index.html
                              )

register_component()

