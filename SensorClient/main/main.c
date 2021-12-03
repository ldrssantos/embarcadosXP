/**
* Company: 
* Engineer:      Leandro Santos
* Create Date:   12/03/2021 
* Design Name:   SensorClient

* Target Devices: ESP32
* Tool versions:  ESP-IDF(v4.3.1) 
* Description: MVP for MQTT Client Publisher device with MPU6050 Sensor
*
* Dependencies: WiFi_app, MQTT_app, MPU6050
*
* Revision: 
* Revision 0.01 - File Created
* Additional Comments: 
 **/

#include <stdio.h>
#include "nvs_flash.h"
#include "esp_spi_flash.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "wifi_app.h"

#include "mqtt_client.h"
#include "mqtt_app.h"

#include "mpu6050.h"
#include "cJSON.h"

static const char *TAG = "MPU6050";

// MQTT client ID and MPU6050_topic must to be unique.
// static const char *clientId = "EmbarcadosXP";
static const char *MPU6050_topic = "EmbarcadosXP/MPU6050";

extern esp_mqtt_client_handle_t client;


/********************************************************************************************
*                           I2C Interface inicialization                                    *
*********************************************************************************************/
#define I2C_MASTER_SCL_IO 22      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

mpu6050_handle_t mpu6050 = NULL; 

/**
 * @brief I2C MASTER interface initialization
 */
static void i2c_bus_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

/**
 * @brief I2C communication with MPU6050 Sensor
 */
static void i2c_sensor_mpu6050_init()
{
    i2c_bus_init();
    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    ESP_ERROR_CHECK(mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS));
    ESP_ERROR_CHECK(mpu6050_wake_up(mpu6050));
}


/********************************************************************************************
*                   FreeRTOS section - Handles and Function prototypes                      *
*********************************************************************************************/
//Semaphore
SemaphoreHandle_t mqttConectionSemaphore;

// Queue handles
QueueHandle_t MPU6050_Queue_data;

//Task prototypes
void vMPU6050_read_dataTask(void *pvParameter);
void vMQTT_PublishTask(void *pvParameter);

//Task handles
TaskHandle_t xMPU6050_read_dataTask;
TaskHandle_t xMQTT_PublishTask;

mpu6050_acce_gyro_value_t MPU6050_MQTT_DATA;


/**
 * @brief MPU6050 Sensor read data 
 */
void vMPU6050_read_dataTask(void *pvParameter)
{
    // Start MQTT Client
    mqtt_app_start();
    
    while (true)
    {    
        if (xSemaphoreTake(mqttConectionSemaphore, portMAX_DELAY))
        {
            mpu6050_acce_value_t acce;
            mpu6050_gyro_value_t gyro;
            
            mpu6050_get_acce(mpu6050, &acce);
            mpu6050_get_gyro(mpu6050, &gyro);

            MPU6050_MQTT_DATA.acce_data = acce;
            MPU6050_MQTT_DATA.gyro_data = gyro;

            xQueueSend(MPU6050_Queue_data, &MPU6050_MQTT_DATA, 10);
            
            vTaskDelay(100 / portTICK_RATE_MS);
            xSemaphoreGive(mqttConectionSemaphore);
        }
    }
}

/**
 * @brief MQTT Publish data 
 */
void vMQTT_PublishTask(void *pvParameter)
{
    while (true)
    {    
        if (xSemaphoreTake(mqttConectionSemaphore, portMAX_DELAY))
        {
            if (xQueueReceive(MPU6050_Queue_data, &MPU6050_MQTT_DATA, 10))
            {
                cJSON *MPU6050_JSON_DATA;
                char *MPU6050_MQTT_TX_DATA = NULL;

                // MPU6050 JSON data structure 
                MPU6050_JSON_DATA = cJSON_CreateObject();
                cJSON_AddNumberToObject(MPU6050_JSON_DATA, "accX", MPU6050_MQTT_DATA.acce_data.acce_x);
                cJSON_AddNumberToObject(MPU6050_JSON_DATA, "accY", MPU6050_MQTT_DATA.acce_data.acce_y);
                cJSON_AddNumberToObject(MPU6050_JSON_DATA, "accZ", MPU6050_MQTT_DATA.acce_data.acce_z);
                cJSON_AddNumberToObject(MPU6050_JSON_DATA, "gyroX", MPU6050_MQTT_DATA.gyro_data.gyro_x);
                cJSON_AddNumberToObject(MPU6050_JSON_DATA, "gyroY", MPU6050_MQTT_DATA.gyro_data.gyro_y);
                cJSON_AddNumberToObject(MPU6050_JSON_DATA, "gyroZ", MPU6050_MQTT_DATA.gyro_data.gyro_z);

                MPU6050_MQTT_TX_DATA = cJSON_Print(MPU6050_JSON_DATA);
                
                ESP_LOGI(TAG, "Sensor data:%s\n", MPU6050_MQTT_TX_DATA);

                esp_mqtt_client_publish(client, MPU6050_topic, MPU6050_MQTT_TX_DATA, 0, 1, 0);

                cJSON_Delete(MPU6050_JSON_DATA);
            }
            vTaskDelay(100 / portTICK_RATE_MS);
            xSemaphoreGive(mqttConectionSemaphore);
        }
    }
}

/********************************************************************************************
*                                  Application section                                      *
*********************************************************************************************/
void app_main(void)
{
    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
        
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    
    //Initialize MPU6050 Sensor
    i2c_sensor_mpu6050_init();

    MPU6050_Queue_data = xQueueCreate(10, sizeof(mpu6050_acce_gyro_value_t));
    if (MPU6050_Queue_data == NULL)
    {
        ESP_LOGE("ERROR", "*** MPU6050 Sensor xQueueCreate error ***\n");
    }
    
    //Initialize FreeRTOS Semaphore
    mqttConectionSemaphore = xSemaphoreCreateBinary();
   
    // Initialize WiFi connection
    wifi_init_app_main();
    
    if (xTaskCreate(&vMPU6050_read_dataTask, "vMPU6050_read_dataTask", configMINIMAL_STACK_SIZE + 8192, NULL, 1, xMPU6050_read_dataTask) != pdTRUE)
    {
        ESP_LOGE("ERROR", "*** vMPU6050_read_dataTask error ***\n");
    }

    if (xTaskCreate(&vMQTT_PublishTask, "vMQTT_PublishTask", configMINIMAL_STACK_SIZE + 8192, NULL, 1, xMQTT_PublishTask) != pdTRUE)
    {
        ESP_LOGE("ERROR", "*** vMQTT_PublishTask error ***\n");
    }
}