idf_component_register(
    SRCS "main.c" "../components/wifi_app/wifi_app.c" "../components/mqtt_app/mqtt_app.c" "../components/mpu6050/mpu6050.c"
    INCLUDE_DIRS ""
)

set(COMPONENT_REQUIRES mqtt)
