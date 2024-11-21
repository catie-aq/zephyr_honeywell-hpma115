#ifndef ZEPHYR_DRIVERS_SENSOR_HPMA115_HPMA115
#define ZEPHYR_DRIVERS_SENSOR_HPMA115_HPMA115

#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/sensor.h>

#define HPMA115_BUF_LEN 16

#define HPMA115_WAIT K_SECONDS(1)

enum hpma115_cmd {
	ReadMeas = 0x04,
	StartMeas = 0x01,
	StopMeas = 0x02,
	SetCoef = 0x08,
	ReadCoef = 0x10,
	EnableAutoSend = 0x40,
	StopAutoSend = 0x20,
};

enum hpma115_header {
	Send = 0x68,
	Resp = 0x40,
	Ack = 0xA5,
	Nack = 0x96,
	AutoData1 = 0x42,
	AutoData2 = 0x4d,
};

/**
 * @brief
 *
 */
struct hpma115_sensor_data {
	/** PM2.5 in µg/m³. */
	uint16_t pm2_5;
	/** PM10 in µg/m³. */
	uint16_t pm10;
	/** PM1.0 in µg/m³. */
	uint16_t pm1_0;
	/** PM4.0 in µg/m³. */
	uint16_t pm4_0;
	/** Validity of PM1.0 and PM4.0 values. */
	bool pm1_pm4_valid;
};

/**
 * @brief Contains runtime mutable data for the UART peripheral.
 */
struct uart_data {
	uint8_t xfer_bytes;
	uint8_t tx_buf[HPMA115_BUF_LEN]; // uart tx buffer
	uint8_t rx_buf[HPMA115_BUF_LEN]; // uart rx buffer
	uint8_t rx_data_len;             // in polling mode
	uint8_t rx_data_len_to_read;     // in interrupt mode
	bool has_rsp;

	struct k_sem tx_sem;
	struct k_sem rx_sem;

	enum hpma115_cmd cmd;
};

/**
 * @brief Build time configurations for the UART peripheral.
 */
struct hpma115_conf {
	struct uart_data *uart_data;
	const struct device *uart_dev; // UART device.
	uart_irq_callback_user_data_t cb;
};

enum sensor_attribute_hpma115 {
	/** Automatic Baseline Correction Self Calibration Function. */
	SENSOR_ATTR_START_MEAS = SENSOR_ATTR_PRIV_START,
};

#endif // ZEPHYR_DRIVERS_SENSOR_HPMA115_HPMA115
