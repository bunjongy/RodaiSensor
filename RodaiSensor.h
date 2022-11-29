#ifndef RODAI_SENSOR_H_
#define RODAI_SENSOR_H_
#include <Arduino.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#if CONFIG_IDF_TARGET_ESP32
#define UART_PORT UART_NUM_1
#define UART_TX GPIO_NUM_23
#define UART_RX GPIO_NUM_22
#define UART_DIR GPIO_NUM_4
#elif CONFIG_IDF_TARGET_ESP32S2
#define UART_PORT UART_NUM_1
#define UART_TX GPIO_NUM_1
#define UART_RX GPIO_NUM_2
#define UART_DIR GPIO_NUM_3
#elif CONFIG_IDF_TARGET_ESP32C3
#define UART_PORT UART_NUM_1
#define UART_TX GPIO_NUM_4
#define UART_RX GPIO_NUM_5
#define UART_DIR GPIO_NUM_6
#elif CONFIG_IDF_TARGET_ESP32S3
#define UART_PORT UART_NUM_1
#define UART_TX GPIO_NUM_1
#define UART_RX GPIO_NUM_2
#define UART_DIR GPIO_NUM_3
#endif

class RodaiSensor
{
private:
	uart_port_t _uart_num = UART_PORT;
	int _baud_rate = 9600;
	bool crc_tab16_init = false;
	uint16_t crc_tab16[256];
	void init_crc16_tab(void);
	uint16_t crc_modbus(const unsigned char *input_str, size_t num_bytes);
	int readRegisters(uint8_t addr, uint8_t funcCode, uint16_t strAddr, uint16_t count, uint8_t *data);
	int readRegisters(uint8_t addr, uint8_t funcCode, uint16_t strAddr, uint16_t count, uint8_t *data, int tempBaudrate);

public:
	RodaiSensor(uart_port_t uart_num = UART_PORT,
				int tx_io_num = UART_TX,
				int rx_io_num = UART_RX,
				int rts_io_num = UART_DIR,
				int cts_io_num = UART_PIN_NO_CHANGE,
				int baud_rate = 9600);

	int getRodaiSoilWaterContentValue(uint8_t addr);	
};

#endif /* MAIN_DNS_SERVER_H_ */
