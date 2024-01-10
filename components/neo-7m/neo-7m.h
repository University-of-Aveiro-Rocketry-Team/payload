#include <string.h>
#include "esp_log.h"
#include <driver/uart.h>
#include <driver/gpio.h>


typedef struct
{
    float latitude;
    float longitude;
    float speed;
    uint8_t day;
    uint8_t month;
    uint8_t year;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} neo7m;

/**
 * @brief Initialize the UART
 *
 * Initialize the UART 1 with the given baud rate and pins to commmunicate with the NEO-7M GPS.
 *
 * @param baud_rate Baud rate
 * @param tx_pin TX pin
 * @param rx_pin RX pin
 * @param rts_pin RTS pin
 * @param cts_pin CTS pin
 */
void neo7m_uart_initialize(int baud_rate, int tx_pin, int rx_pin, int rts_pin, int cts_pin);

/**
 * @brief Read the data from the GPS
 * 
 * Read the data from the GPS and store it in the neo7m struct.
 * 
 * @param neo7m_data Pointer to the neo7m struct
 */
void neo7m_uart_read(neo7m *neo7m_data);
