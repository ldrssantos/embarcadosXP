# mqtt_client included file issue report on this link (esp-tls not included)
# https://stackoverflow.com/questions/70011677/cmake-and-esp-idf-creating-custom-components/70019056

idf_component_register(SRCS "mqtt_app.c"
                    INCLUDE_DIRS "include"
                    REQUIRES mqtt
)
