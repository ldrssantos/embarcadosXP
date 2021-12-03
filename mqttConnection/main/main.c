/**
* Company: 
* Engineer:      Leandro Santos
* Create Date:   12/03/2021 
* Design Name:   mqttConection

* Target Devices: ESP32
* Tool versions:  ESP-IDF(v4.3.1) 
* Description: MVP for MQTT Client connection
*
* Dependencies: WiFi_app, MQTT
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

//#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "wifi_app.h"
#include "mqtt_app.h"

/********************************************************************************************
*                   FreeRTOS section - Handles and Function prototypes                      *
*********************************************************************************************/
//Semaphore
SemaphoreHandle_t mqttConectionSemaphore;

// Queue handles
// QueueHandle_t xMPU6050_Sensor_Control = 0;

//Task handles
TaskHandle_t xMQTT_PubSubTask;
// TaskHandle_t xMPU6050_read_dataTask;

//Task prototypes
void vMQTT_PubSubTask(void *pvParameter);
//void vMPU6050_read_dataTask(void *pvParameter);


/**
 * @brief MQTT Publish and Subscribe 
 */
void vMQTT_PubSubTask(void *pvParameter)
{
    while (true)
    {    
        if (xSemaphoreTake(mqttConectionSemaphore, portMAX_DELAY))
        {
            // Initialize MQTT Broker connection
            mqtt_app_start();
        }
        
        // if (!xQueueReceive(xMPU6050_Sensor_Control, &MPU6050_MQTT_DATA, 10)){

        //     ESP_LOGI(TAG_MQTT, "*** MPU6050 MQTT publish data error ***\n");
        // } else {
        //     esp_mqtt_client_publish(client, MPU6050_topic, MPU6050_MQTT_DATA, 0, 0, 0);
        // }
        
        // // Free an object. Moves it to the free list. No destructors are run.
        // free(MPU6050_MQTT_DATA);
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


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
    
    //Initialize FreeRTOS Semaphore
    mqttConectionSemaphore = xSemaphoreCreateBinary();
    // xSemaphoreGive(mqttConectionSemaphore);
   
    // Initialize WiFi connection
    wifi_init_app_main();
    
    if (xTaskCreate(&vMQTT_PubSubTask, "vMQTT_PubSubTask", configMINIMAL_STACK_SIZE + 4096, NULL, 1, xMQTT_PubSubTask) != pdTRUE)
    {
        ESP_LOGE("ERROR", "*** vMQTT_PubSubTask taskCreate error process ***\n");
    }   
}
