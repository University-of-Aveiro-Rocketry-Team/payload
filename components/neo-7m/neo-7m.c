#include "neo-7m.h"
#include "minmea.h"

#define BUF_SIZE (1024)

void neo7m_uart_initialize(int baud_rate, int tx_pin, int rx_pin, int rts_pin, int cts_pin)
{
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, tx_pin, rx_pin, rts_pin, cts_pin);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
}

void neo7m_uart_read(neo7m *gps_data)
{
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);

    // Buffer to store a complete NMEA sentence
    char *nmea_sentence = (char *)malloc(BUF_SIZE / 2);
    if (!nmea_sentence)
    {
        ESP_LOGE("GPS", "Failed to allocate memory for NMEA sentence");
    }
    int nmea_index = 0;
    int length = 0;

    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_1, (size_t *)&length));
    length = uart_read_bytes(UART_NUM_1, data, length, 100 / portTICK_PERIOD_MS);

    if (length > 0)
    {
        for (int i = 0; i < length; i++)
        {
            // Append character to NMEA sentence buffer
            nmea_sentence[nmea_index++] = data[i];

            // Check if the end of the sentence is reached
            if (data[i] == '\n')
            {
                nmea_sentence[nmea_index] = '\0'; // Null-terminate the sentence

                if (strncmp(nmea_sentence, "$GPRMC", 6) == 0)
                {
                    struct minmea_sentence_rmc rmc;
                    if (minmea_parse_rmc(&rmc, nmea_sentence))
                    {
                        if (rmc.valid)
                        {
                            // Set the GPS data
                            gps_data->latitude = minmea_tocoord(&rmc.latitude);
                            gps_data->longitude = minmea_tocoord(&rmc.longitude);
                            gps_data->speed = minmea_tofloat(&rmc.speed);
                            gps_data->day = rmc.date.day;
                            gps_data->month = rmc.date.month;
                            gps_data->year = rmc.date.year;
                            gps_data->hours = rmc.time.hours;
                            gps_data->minutes = rmc.time.minutes;
                            gps_data->seconds = rmc.time.seconds;

                            // Print the GPS data
                            // ESP_LOGI("GPS", "$GPRMC Latitude: %f \nLongitude: %f \nSpeed: %f \n Date: %d-%d-%d\n Time: %d:%d:%d\n",
                            //         minmea_tocoord(&rmc.latitude), minmea_tocoord(&rmc.longitude), minmea_tofloat(&rmc.speed), rmc.date.day, rmc.date.month, rmc.date.year, rmc.time.hours, rmc.time.minutes, rmc.time.seconds);
                            
                            break;
                        }
                        else
                        {
                            ESP_LOGE("GPS", "Invalid RMC");
                        }
                    }
                }

            }
        }
    }

    free(data);
    free(nmea_sentence);
}
