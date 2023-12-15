#define DT_DRV_COMPAT honeywell_hpma115

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <string.h>

#include <errno.h>

#include "hpma115.h"

#include <zephyr/logging/log.h> 
LOG_MODULE_REGISTER(honeywell_hmpa115, CONFIG_SENSOR_LOG_LEVEL);

// define private prototypes
static int hpma115_send_command(const struct device *dev, enum hpma115_cmd cmd);
static bool hpma115_read_data(const struct device *dev, enum hpma115_cmd cmd,  uint8_t *len, uint8_t **data);


/**
 * @brief start measurement.
 * 
 * @param dev hpma115 UART peripheral device.
 */
static int hpma115_attr_start_measurement(const struct device *dev)
{
    return hpma115_send_command(dev, StartMeas);
}

/**
 * @brief stop measurement.
 * 
 * @param dev hpma115 UART peripheral device.
 */
static int hpma115_attr_stop_measurement(const struct device *dev)
{
    return hpma115_send_command(dev, StopMeas);
}

/**
 * @brief read measurement.
 * 
 * @param dev hpma115 UART peripheral device.
 * @param result pointer to structure to store read data
 */
static bool hpma115_read_measurement(const struct device *dev, hpma115_sensor_values *result)
{
    uint8_t len;
    uint8_t *data;

    enum hpma115_cmd command = ReadMeas;
    if (!hpma115_read_data(dev, command, &len, &data)) {
        return false;
    }

    if (data[0] != (uint8_t)(command)) {
        return false;
    } 
    
    if (len == 13) { /* It's a compact sensor. */
        result->pm1_pm4_valid = true;
        result->pm1_0 = ((uint16_t)(data[1]) << 8) + data[2];
        result->pm2_5 = ((uint16_t)(data[3]) << 8) + data[4];
        result->pm4_0 = ((uint16_t)(data[5]) << 8) + data[6];
        result->pm10 = ((uint16_t)(data[7]) << 8) + data[8];
    } else if (len == 5) { /* It's a standard sensor. */
        result->pm1_pm4_valid = false;
        result->pm1_0 = 0;
        result->pm4_0 = 0;
        result->pm2_5 = ((uint16_t)(data[1]) << 8) + data[2];
        result->pm10 = ((uint16_t)(data[3]) << 8) + data[4];
    } else {
        return false;
    }
    
    return true;
}

static int hpma115_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct uart_data *data = dev->data;

	if (chan != SENSOR_CHAN_CO2) {
		return -ENOTSUP;
	}

    hpma115_read_measurement(dev, &data->sensor_values);

    val->val1 = (int32_t)data->sensor_values.pm1_0;
    val->val2 = 0;
    
	return 0;
}

static int hpma115_attr_set(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr, const struct sensor_value *val)
{
	if (chan != SENSOR_CHAN_PM_1_0) {
		return -ENOTSUP;
	}

	switch (attr) {
	case SENSOR_ATTR_START_MEAS:
		return hpma115_attr_start_measurement(dev);

	default:
		return -ENOTSUP;
	}
}

static const struct sensor_driver_api hpma115_api_funcs = {
	.attr_set = hpma115_attr_set,
	.channel_get = hpma115_channel_get,
};

static void hpma115_uart_flush(const struct device *uart_dev)
{
	uint8_t c;

	while (uart_fifo_read(uart_dev, &c, 1) > 0) {
		continue;
	}
}

static uint8_t hpma115_compute_checksum(uint8_t *frame)
{
    uint8_t csum = -frame[0];
    csum -= frame[1];

    for (int i = 0; i <= frame[1]; i++) {
        csum -= frame[2 + i];
    }

    return csum;
}

static int hpma115_read_response(const struct device *dev, int len)
{
    struct uart_data *data = dev->data;
    char c;
    int ret = 0;

    ret = k_sem_take(&data->rx_sem, HPMA115_WAIT);

    if (ret) {
	    return ret;
	}

    while(len) {
        if (!uart_poll_in(dev, &c)) {
            data->rx_buf[data->rx_data_len] = c; 
            data->rx_data_len++;
            len--;
        }
    }

    k_sem_give(&data->rx_sem);

    return ret;
}

static int hpma115_send_command(const struct device *dev, enum hpma115_cmd cmd)
{
    struct uart_conf *conf = dev->config; 
    struct uart_data *data = dev->data;
    int ret = 0;

    ret = k_sem_take(&data->tx_sem, HPMA115_WAIT);

    if (ret) {
		return ret;
	}
    
    data->tx_buf[0] = (char)(Send);
    data->tx_buf[1] = 1;
    data->tx_buf[2] = (char)(cmd);

    /* Invalid checksum and compute it */
    data->tx_buf[3] = 0;
    data->tx_buf[3] = hpma115_compute_checksum(data->tx_buf);

    // clear uart rx data index
    data->rx_data_len = 0;

    for (size_t i = 0;i < 4; i++) {
        uart_poll_out(dev, data->tx_buf[i]);
    }

    k_sem_give(&data->tx_sem);

    ret = hpma115_read_response(dev, 2);
    if (ret) {
		return ret;
	}

    if (data->rx_buf[0] == data->rx_buf[1] && data->rx_buf[0] != (char)(Ack)) {
        return -1;
    }

    return ret;
}

static bool hpma115_read_data(const struct device *dev, enum hpma115_cmd cmd,  uint8_t *len, uint8_t **data_in)
{
    struct uart_conf *conf = dev->config;
    struct uart_data *data = dev->data;

    data->tx_buf[0] = (char)(Send);
    data->tx_buf[1] = 1;
    data->tx_buf[2] = (char)(cmd);

    /* Invalid checksum and compute it */
    data->tx_buf[3] = 0;
    data->tx_buf[3] = hpma115_compute_checksum(data->tx_buf);

    hpma115_send_command(dev, cmd);

    hpma115_read_response(dev, 2);

    if (data->rx_buf[0] == (char)(Resp)) {
        if (data->rx_buf[1] > (HPMA115_BUF_LEN - 3)) {
            return false;
        }

        hpma115_read_response(data->rx_buf + 2, data->rx_buf[1] + 1);

        if (hpma115_compute_checksum(data->rx_buf) != 0) {
            return false;
        }

        *len = data->rx_buf[1];
        *data_in = data->rx_buf + 2;

    }  else {
        *len = 0;
        *data_in = NULL;
        return false;
    }

    return true;
}

// static void hpma115_uart_isr(const struct device *uart_dev, void *user_data)
// {
// 	const struct device *dev = user_data;
// 	struct hpma115_data *data = dev->data;

// 	ARG_UNUSED(user_data);

// 	if (uart_dev == NULL) {
// 		return;
// 	}

// 	if (!uart_irq_update(uart_dev)) {
// 		return;
// 	}

// 	if (uart_irq_rx_ready(uart_dev)) {
// 		data->xfer_bytes += uart_fifo_read(uart_dev, &data->rd_data[data->xfer_bytes],
// 						   HPMA115_BUF_LEN - data->xfer_bytes);

// 		if (data->xfer_bytes == HPMA115_BUF_LEN) {
// 			data->xfer_bytes = 0;
// 			uart_irq_rx_disable(uart_dev);
// 			k_sem_give(&data->rx_sem);
// 			if (data->has_rsp) {
// 				k_sem_give(&data->tx_sem);
// 			}
// 		}
// 	}

// 	if (uart_irq_tx_ready(uart_dev)) {
// 		data->xfer_bytes +=
// 			uart_fifo_fill(uart_dev, &mhz19b_cmds[data->cmd_idx][data->xfer_bytes],
// 				       HPMA115_BUF_LEN - data->xfer_bytes);

// 		if (data->xfer_bytes == HPMA115_BUF_LEN) {
// 			data->xfer_bytes = 0;
// 			uart_irq_tx_disable(uart_dev);
// 			if (!data->has_rsp) {
// 				k_sem_give(&data->tx_sem);
// 			}
// 		}
// 	}
// }

static int hpma115_init(const struct device *dev)
{
	struct uart_data *data = dev->data;
	const struct uart_conf *cfg = dev->config;
	int ret;

	uart_irq_rx_disable(cfg->uart_dev);
	uart_irq_tx_disable(cfg->uart_dev);

	hpma115_uart_flush(cfg->uart_dev);

	//uart_irq_callback_user_data_set(cfg->uart_dev, cfg->cb, (void *)dev);

	k_sem_init(&data->tx_sem, 0, 1);
    k_sem_init(&data->rx_sem, 1, 1);

	/* start measurement */
    ret = hpma115_attr_start_measurement(dev);
	if (ret != 0) {
		LOG_ERR("Error start measurement!");
		return ret;
	}

	return ret;
}
#define HPMA115_INIT(inst)									\
												\
	static struct uart_data uart_data_##inst;						\
												\
	static const struct uart_conf uart_conf_##inst = {					\
		.uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),					\
	};											\
												\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, hpma115_init, NULL,					\
			      &uart_data_##inst, &uart_conf_##inst,				\
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &hpma115_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(HPMA115_INIT)