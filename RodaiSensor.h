#pragma once
#include <Arduino.h>
#include "driver/uart.h"

class RodaiSensor
{
private:
	uart_port_t _uart_num = UART_NUM_1;
	int _baud_rate = 9600;
	bool crc_tab16_init = false;
	uint16_t crc_tab16[256];
	void init_crc16_tab(void);
	uint16_t crc_modbus(const unsigned char *input_str, size_t num_bytes);
	int readRegisters(uint8_t addr, uint8_t funcCode, uint16_t strAddr, uint16_t count, uint8_t *data);
	int readRegisters(uint8_t addr, uint8_t funcCode, uint16_t strAddr, uint16_t count, uint8_t *data, int tempBaudrate);

public:
	RodaiSensor(uart_port_t uart_num = UART_NUM_1,
				int tx_io_num = GPIO_NUM_23,
				int rx_io_num = GPIO_NUM_22,
				int rts_io_num = GPIO_NUM_4,
				int cts_io_num = UART_PIN_NO_CHANGE,
				int baud_rate = 9600);

	int getRodaiSoilWaterContentValue(uint8_t addr);	
};
