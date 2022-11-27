
#include "RodaiSensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"

#define CRC_START_MODBUS 0xFFFF
#define CRC_POLY_16 0xA001

#define RS485_BUF_SIZE (256)
#define RS485_PACKET_READ_TICS (100 / portTICK_RATE_MS)
#define RS485_READ_TOUT (3)

#define TAG "sensor_rs485"

void RodaiSensor::init_crc16_tab(void)
{
	uint16_t i;
	uint16_t j;
	uint16_t crc;
	uint16_t c;

	for (i = 0; i < 256; i++)
	{

		crc = 0;
		c = i;

		for (j = 0; j < 8; j++)
		{

			if ((crc ^ c) & 0x0001)
				crc = (crc >> 1) ^ CRC_POLY_16;
			else
				crc = crc >> 1;

			c = c >> 1;
		}

		crc_tab16[i] = crc;
	}
	crc_tab16_init = true;
} /* init_crc16_tab */

uint16_t RodaiSensor::crc_modbus(const unsigned char *input_str, size_t num_bytes)
{
	uint16_t crc;
	uint16_t tmp;
	uint16_t short_c;
	const unsigned char *ptr;
	size_t a;

	if (!crc_tab16_init)
		init_crc16_tab();

	crc = CRC_START_MODBUS;
	ptr = input_str;

	if (ptr != NULL)
		for (a = 0; a < num_bytes; a++)
		{

			short_c = 0x00ff & (uint16_t)*ptr;
			tmp = crc ^ short_c;
			crc = (crc >> 8) ^ crc_tab16[tmp & 0xff];

			ptr++;
		}

	return crc;
} /* crc_modbus */

static void sensor_rs485_task(void *arg)
{

	unsigned long time_to_read = 0;
	// Allocate buffers for UART
	uint8_t *data = (uint8_t *)malloc(RS485_BUF_SIZE);

	const char test16[8] = {0x10, 0x03, 0x00, 0x06, 0x00, 0x05, 0x66, 0x89}; // 10 03 00 06 00 05 66 89
	for (;;)
	{
		const unsigned long time1 = (unsigned long)(esp_timer_get_time()) / 1000;

		int len = uart_read_bytes(UART_NUM_1, data, RS485_BUF_SIZE, RS485_PACKET_READ_TICS);

		ESP_LOGI(TAG, "Received len %d :", len);

		if (len >= 4)
		{
			uint16_t crcs = 0; // RodaiSensor::crc_modbus(data, len - 2);
			uint16_t b1 = data[len - 1];
			uint8_t b2 = data[len - 2];
			uint16_t _crcs = b1 << 8 | b2;
			ESP_LOGI(TAG, "Received crc %04X :", crcs);
		}

		if ((time1 - time_to_read) > 250)
		{
			time_to_read = time1;
			uart_write_bytes(UART_NUM_1, test16, 8);
		}
		vTaskDelay(5 / portTICK_PERIOD_MS);
	}
}

int RodaiSensor::ReadHoldingRegisters(uint8_t addr, uint16_t strAddr, uint16_t count, uint8_t *data)
{
	return ReadHoldingRegisters(addr, strAddr, count, data, _baud_rate);
}

int RodaiSensor::ReadHoldingRegisters(uint8_t addr, uint16_t strAddr, uint16_t count, uint8_t *data, int temp_baud_rate)
{
	uint8_t cmd[8] = {};
	cmd[0] = addr;
	cmd[1] = 0x03;
	cmd[2] = (strAddr & 0xff00) >> 8;
	cmd[3] = (uint8_t)strAddr;
	cmd[4] = (count & 0xff00) >> 8;
	cmd[5] = (uint8_t)count;
	uint16_t crcs = crc_modbus(cmd, 8 - 2);
	cmd[6] = (uint8_t)crcs;
	cmd[7] = (crcs & 0xff00) >> 8;

	uart_set_baudrate(_uart_num, temp_baud_rate);
	uart_write_bytes(_uart_num, cmd, 8);

	vTaskDelay(5 / portTICK_PERIOD_MS);

	uint8_t *_data = (uint8_t *)malloc(RS485_BUF_SIZE);
	int len = 0;
	for (int i = 0; i < 10; i++)
	{
		vTaskDelay(75 / portTICK_PERIOD_MS);
		len = uart_read_bytes(_uart_num, _data, RS485_BUF_SIZE, RS485_PACKET_READ_TICS);
		if (len >= 4)
		{
			uint16_t crcs2 = crc_modbus(_data, len - 2);
			uint16_t b1 = _data[len - 1];
			uint8_t b2 = _data[len - 2];
			uint16_t _crcs2 = b1 << 8 | b2;
			// ESP_LOGD(TAG, "Received crc %04X :", crcs2);
			if (crcs2 == _crcs2)
			{
				memset(data, 0, sizeof(data));
				memcpy(data, _data, len);
				// ESP_LOGD(TAG, "Received crc ok");
				break;
			}
		}
	}

	free(_data);
	return len - 2;
}

RodaiSensor::RodaiSensor(uart_port_t uart_num,
						 int tx_io_num,
						 int rx_io_num,
						 int rts_io_num,
						 int cts_io_num,
						 int baud_rate)
{
	_uart_num = uart_num;
	_baud_rate = baud_rate;

	uart_config_t uart_config = {
		.baud_rate = baud_rate,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.rx_flow_ctrl_thresh = 122,
	};

	uart_param_config(uart_num, &uart_config);
	uart_set_pin(uart_num, tx_io_num, rx_io_num, rts_io_num, cts_io_num);
	uart_driver_install(uart_num, RS485_BUF_SIZE * 2, 0, 0, NULL, 0);
	// Set RS485 half duplex mode
	uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX);
	// Set read timeout of UART TOUT feature
	uart_set_rx_timeout(uart_num, RS485_READ_TOUT);

	// xTaskCreate(sensor_rs485_task, "sensor_rs485_task", 1024 * 4, NULL, 5, NULL);
}