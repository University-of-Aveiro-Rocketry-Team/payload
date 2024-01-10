#include <string.h>
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_http_client.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "payload.h"
#include "mpu-6500.h"
#include "lora.h"
#include "spiffs.h"
#include "neo-7m.h"


/***** DEFENITIONS *****/
// MPU-6500
#define IMU_ADDRESS 0x68
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_NUM I2C_NUM_0
#define I2C_FREQ_HZ 400000 // 400kHz I2C frequency
// NEO-7M
#define UART_NUM UART_NUM_1
#define BUF_SIZE (2048)
// SPIFFS
#define SPIFFS_MPU_PATH "/spiffs/mpu.txt"
#define SPIFFS_NEO_PATH "/spiffs/neo.txt"
#define LORA_ON 1
#define SPIFFS_ON 0


/***** VARIABLES *****/
static const char *TAG = "PAYLOAD";
bool mpu_mux, neo_mux = false;
data_t *data_to_send = NULL;
// MPU6500
mpu6500 IMU;
calData calib = {0};
AccelData accelData;
GyroData gyroData;
// NEO7M
neo7m gps_data;

/***** FUNCTION PROTOTYPES *****/
// MPU6500
esp_err_t mpu6500_setup();
void mpu6500_loop();
// NEO7M
esp_err_t neo7m_setup();
void neo7m_loop();
// LoRa
esp_err_t lora_setup();
void send_lora_loop();


/***** PROGRAM STARTS HERE *****/
// Initiliazer I2C communication with MPU-6500
esp_err_t mpu6500_setup()
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ};
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0));

    ESP_LOGI("MPU-6500", "Starting MPU-6500 setup");

    int err = mpu6500_init(&IMU, &calib, IMU_ADDRESS);
    if (err != 0)
        return ESP_FAIL;

    ESP_LOGI("MPU-6500", "Keep level");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    mpu6500_calibrateAccelGyro(&IMU, &calib);
    ESP_LOGI("MPU-6500", "Calibration done!");

    mpu6500_init(&IMU, &calib, IMU_ADDRESS);

    return ESP_OK;
}

void mpu6500_loop(void *pvParameters)
{
    float accumulatedGyroX = 0, accumulatedGyroY = 0, accumulatedGyroZ = 0;
    int sampleCount = 0;

    while (1)
    {
        mpu6500_update(&IMU);
        mpu6500_getGyro(&IMU, &gyroData);
        mpu6500_getAccel(&IMU, &accelData);

        // Accumulate gyro data
        accumulatedGyroX += (int)gyroData.gyroX * 0.01; // 0.1 seconds per sample
        accumulatedGyroY += (int)gyroData.gyroY * 0.01;
        accumulatedGyroZ += (int)gyroData.gyroZ * 0.01;

        // Gyro values must be between 0 and 359
        if (accumulatedGyroX >= 360)
            accumulatedGyroX -= 360;
        else if (accumulatedGyroX < 0)
            accumulatedGyroX += 360;

        sampleCount++;

        // Every 100 samples (1 second), log and reset
        if (sampleCount >= 100)
        {
            // Set the data
            data_to_send->acceleration_x = accelData.accelX;
            data_to_send->acceleration_y = accelData.accelX;
            data_to_send->acceleration_z = accelData.accelY;
            data_to_send->gyroscope_x = accumulatedGyroX;
            data_to_send->gyroscope_y = accumulatedGyroY;
            data_to_send->gyroscope_z = accumulatedGyroZ;

            // Log the data
            ESP_LOGI(TAG, "Angular Displacement - x:%.1f y:%.1f z:%.1f", accumulatedGyroX, accumulatedGyroY, accumulatedGyroZ);
            ESP_LOGI(TAG, "Acceleration - x:%.1f y:%.1f z:%.1f", accelData.accelX, accelData.accelY, accelData.accelZ - 1);
            
            // Storing the values in SPIFFS
            char data_string[60];
            sprintf(data_string, "%f,%f,%f,%f,%f,%f", accelData.accelX, accelData.accelY, accelData.accelZ, accumulatedGyroX, accumulatedGyroY, accumulatedGyroZ);
            if (SPIFFS_ON) spiffs_write_file(SPIFFS_MPU_PATH, data_string);

            // Reset for next second
            sampleCount = 0;
            // MPU-6500 data is ready to be sent
            if (LORA_ON) mpu_mux = true;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Initialize UART communication with NEO-7M
esp_err_t neo7m_setup()
{
    neo7m_uart_initialize(9600, 25, 26, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    return ESP_OK;
}

void neo7m_loop(void *pvParameters)
{
    while (1)
    {
        neo7m_uart_read(&gps_data);

        // Set the data
        data_to_send->latitude = gps_data.latitude;
        data_to_send->longitude = gps_data.longitude;
        data_to_send->speed = gps_data.speed;
        data_to_send->day = gps_data.day;
        data_to_send->month = gps_data.month;
        data_to_send->year = gps_data.year;
        data_to_send->hours = gps_data.hours;
        data_to_send->minutes = gps_data.minutes;
        data_to_send->seconds = gps_data.seconds;

        // Log the data
        ESP_LOGI("PAYLOAD", "Latitude: %f, Longitude: %f, Speed: %f, Date: %ud-%ud-%ud, Time: %ud:%ud:%ud",
                 gps_data.latitude, gps_data.longitude, gps_data.speed, gps_data.day, gps_data.month, gps_data.year, gps_data.hours, gps_data.minutes, gps_data.seconds);

        // Storing the values in SPIFFS
        char data_string[60];
        sprintf(data_string, "%f,%f,%f,%u,%u,%u,%u,%u,%u", gps_data.latitude, gps_data.longitude, gps_data.speed, gps_data.day, gps_data.month, gps_data.year, gps_data.hours, gps_data.minutes, gps_data.seconds);
        if (SPIFFS_ON) spiffs_write_file(SPIFFS_NEO_PATH, data_string);

        // NEO-7M data is ready to be sent
        if (LORA_ON) neo_mux = true;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// Initialize UART communication with LoRa
esp_err_t lora_setup()
{
    lora_uart_initialize(9600, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    return ESP_OK;
}

void send_lora_loop(void *pvParameters)
{
    while (1)
    {
        if (mpu_mux && neo_mux)
        {
            // Convert data_to_send to a char * and use it on lora_uart_send
            char data_buffer[256];
            snprintf(data_buffer, sizeof(data_buffer), "%f,%f,%f,%f,%f,%f,%f,%f,%.1f,%u,%u,%u,%u,%u,%u\n",
                     data_to_send->acceleration_x, data_to_send->acceleration_y, data_to_send->acceleration_z,
                     data_to_send->gyroscope_x, data_to_send->gyroscope_y, data_to_send->gyroscope_z,
                     data_to_send->latitude, data_to_send->longitude, data_to_send->speed,
                     data_to_send->day, data_to_send->month, data_to_send->year,
                     data_to_send->hours, data_to_send->minutes, data_to_send->seconds);

            lora_uart_send(data_buffer);

            // Reset the muxs
            mpu_mux = false;
            neo_mux = false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main()
{
    data_to_send = malloc(sizeof(data_t));

    // Initialize SPIFFS
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true};
    esp_err_t ret_spiffs = spiffs_mount(&conf);
    if (ret_spiffs != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize SPIFFS: %d", ret_spiffs);
        return;
    }
    ESP_LOGI(TAG, "SPIFFS initialized successfully");
    spiffs_get_info(&conf);

    // Initialize MPU-6500
    esp_err_t ret_mpu = mpu6500_setup();
    if (ret_mpu != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize MPU-6500: %d", ret_mpu);
        return;
    }
    ESP_LOGI(TAG, "MPU-6500 initialized successfully");

    // Initialize NEO-7M
    esp_err_t ret_gps = neo7m_setup();
    if (ret_gps != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize NEO-7M: %d", ret_gps);
        return;
    }
    ESP_LOGI(TAG, "NEO-7M initialized successfully");

    // Initialize LoRa
    esp_err_t ret_lora = lora_setup();
    if (ret_lora != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize LoRa: %d", ret_lora);
        return;
    }
    ESP_LOGI(TAG, "LoRa initialized successfully");

    // Sensor Tasks
    xTaskCreate(mpu6500_loop, "mpu6500_loop", 4096, NULL, 5, NULL);
    xTaskCreate(neo7m_loop, "neo7m_loop", 4096, NULL, 5, NULL);
    xTaskCreate(send_lora_loop, "send_lora_loop", 4096, NULL, 5, NULL);

    // Infinite loop delay
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Unmount SPIFFS
    ret_spiffs = spiffs_unmount(&conf);
    if (ret_spiffs == ESP_OK)
    {
        ESP_LOGI(TAG, "SPIFFS unmounted successfully");
    }
    else
    {
        ESP_LOGE(TAG, "SPIFFS unmount failed");
        return;
    }
}
