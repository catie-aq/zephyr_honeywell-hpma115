#define DT_DRV_COMPAT honeywell_hpma115

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <string.h>

#include <errno.h>

#include "hpma115.h"

#include <zephyr/logging/log.h> 
LOG_MODULE_REGISTER(honeywell_hmpa115, CONFIG_SENSOR_LOG_LEVEL);

static uint8_t data_to_read = 0;
// define private prototypes
static int hpma115_send_command(const struct device *dev, enum hpma115_cmd cmd);
static int hpma115_write_command(const struct device *dev, uint8_t cmd_length);
static int hpma115_read_response(const struct device *dev, uint8_t length);
static int hpma155_poll_read_response(const struct device *dev, uint8_t length_to_read);
static int hpma115_read_data(const struct device *dev,  uint8_t *len, uint8_t **data_out);
static uint8_t hpma115_compute_checksum(uint8_t *frame);
// static int hpma115_read_response(const struct device *dev, int len);

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
 */
static bool hpma115_read_measurement(const struct device *dev)
{
    
}

static int hpma115_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct hpma115_conf *conf = dev->config; 
    struct uart_data *data = conf->uart_data;
    struct hpma115_sensor_data *result = dev->data;
    uint8_t len;
    uint8_t *data_out;

    int ret = hpma115_read_data(dev, &len, &data_out);

    if (ret) {
        return ret;
    }
    
    if (len == 13) { /* It's a compact sensor. */
        result->pm1_pm4_valid = true;
        result->pm1_0 = ((uint16_t)(data_out[1]) << 8) + data_out[2];
        result->pm2_5 = ((uint16_t)(data_out[3]) << 8) + data_out[4];
        result->pm4_0 = ((uint16_t)(data_out[5]) << 8) + data_out[6];
        result->pm10 = ((uint16_t)(data_out[7]) << 8) + data_out[8];
    } else if (len == 5) { /* It's a standard sensor. */
        result->pm1_pm4_valid = false;
        result->pm1_0 = 0;
        result->pm4_0 = 0;
        result->pm2_5 = ((uint16_t)(data_out[1]) << 8) + data_out[2];
        result->pm10 = ((uint16_t)(data_out[3]) << 8) + data_out[4];
    } else {
        return false;
    }
    
    return true;

}

static int hpma115_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct hpma115_sensor_data *data = dev->data;

    if (chan == SENSOR_CHAN_PM_1_0) {    
        val->val1 = (int32_t)data->pm1_0;
        val->val2 = 0;
    } else if (chan == SENSOR_CHAN_PM_2_5) {
        val->val1 = (int32_t)data->pm2_5;
        val->val2 = 0;
    } else if (chan == SENSOR_CHAN_PM_10) {
        val->val1 = (int32_t)data->pm10;
        val->val2 = 0;
    } else {        
        // val->val1 = (int32_t)data->sensor_values.pm4_0;
        // val->val2 = 0;
        return -ENOTSUP;
    }
    
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
    .sample_fetch = hpma115_sample_fetch,
};

static void hmpa115_clear_uart_data(struct uart_data *data)
{
    data->rx_data_len = 0;
    data->xfer_bytes = 0;
    memset(data->rx_buf, '\0', sizeof(data->rx_buf));
    memset(data->tx_buf, '\0', sizeof(data->tx_buf));
    
}

static void hpma115_uart_flush(const struct device *dev)
{
    struct hpma115_conf *conf = dev->config;
	uint8_t c;

	while (uart_fifo_read(conf->uart_dev, &c, 1) > 0) {
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

static int hpma115_read_data(const struct device *dev,  uint8_t *len, uint8_t **data_out)
{
    struct hpma115_conf *conf = dev->config;
    struct uart_data *data = conf->uart_data;

    int ret = hpma115_send_command(conf->uart_dev, ReadMeas);
    
    if (!ret) {
        if (data->xfer_bytes > 0) {
            if (data->rx_buf[0] == (uint8_t)Resp) {
                *len = data->rx_buf[1];
                *data_out = data->rx_buf + 2;
            }
        }
    }

    return ret;

    // clear uart rx data index
    // data->rx_data_len = 0;

    // hpma115_read_response(conf->uart_dev, 2);

    // if (data->rx_buf[0] == (char)(Resp)) {
    //     if (data->rx_buf[1] > (HPMA115_BUF_LEN - 3)) {
    //         return false;
    //     }

    //     hpma115_read_response(conf->uart_dev, data->rx_buf[1] + 1);

    //     if (hpma115_compute_checksum(data->rx_buf) != 0) {
    //         return false;
    //     }

    //     *len = data->rx_buf[1];
    //     *data_in = data->rx_buf + 2;

    // }  else {
    //     *len = 0;
    //     *data_in = NULL;
    //     return false;
    // }
}

static int hpma115_send_command(const struct device *dev, enum hpma115_cmd cmd)
{
    const struct hpma115_conf *conf = dev->config; 
    struct uart_data *data = conf->uart_data;
    int ret = 0;
    uint8_t tx_buf[5] = {0};

    hpma115_uart_flush(dev);
    hmpa115_clear_uart_data(data);

    data->tx_buf[0] = (uint8_t)(Send);
    data->tx_buf[1] = 1;
    data->tx_buf[2] = (uint8_t)(cmd);

    /* Invalid checksum and compute it */
    data->tx_buf[3] = 0;
    data->tx_buf[3] = hpma115_compute_checksum(data->tx_buf);
    //LOG_DBG("Tx buf: %X %X %X %X\n",  data->tx_buf[0], data->tx_buf[1], data->tx_buf[2], data->tx_buf[3]);

    // tx_buf[0] = 0x68;
    // tx_buf[1] = 0x01;
    // tx_buf[2] = 0x01;
    // tx_buf[3] = 0x96;
    
    // transmit data
    // for (size_t i = 0; i < 4; i++) {
        // uart_poll_out(conf->uart_dev, data->tx_buf[i]);
        // uart_poll_out(conf->uart_dev, tx_buf[i]);
    // }

    ret = hpma115_write_command(dev, 4);
    // ret = hpma155_poll_read_response(dev, 2);
    ret = hpma115_read_response(dev, 2);
      
    return ret;
}

static int hpma115_write_command(const struct device *dev, uint8_t cmd_length)
{
    const struct hpma115_conf *conf = dev->config;
    struct uart_data *data = conf->uart_data;

    LOG_DBG("Tx buf: %X %X %X %X\n",  data->tx_buf[0], data->tx_buf[1], data->tx_buf[2], data->tx_buf[3]);

    for (uint8_t i = 0; i < cmd_length; i++) {
        uart_poll_out(conf->uart_dev, data->tx_buf[i]);
    }
}

static int hpma155_poll_read_response(const struct device *dev, uint8_t length_to_read)
{
    const struct hpma115_conf *conf = dev->config;
    struct uart_data *data = conf->uart_data;
    uint8_t c;

    // DBG: local variables
    // uint8_t rx_buf[5] = {0};
    // uint8_t rx_data_len = 0;

    data->rx_data_len = 0;
    while(length_to_read) {
        if (!uart_poll_in(&conf->uart_dev, &c)) {
            data->rx_buf[ data->rx_data_len] = c;
            data->rx_data_len++;
            length_to_read--;
        }
    }

    LOG_DBG("Rx buff %X %X\n", data->rx_buf[0], data->rx_buf[1]);
}

static int hpma115_read_response(const struct device *dev, uint8_t length)
{
    const struct hpma115_conf *conf = dev->config;
    struct uart_data *data = conf->uart_data;
    int ret = 0;

    data_to_read = length;
    data->has_rsp = false;
    data->xfer_bytes = 0;

    uart_irq_rx_enable(conf->uart_dev);

    k_msleep(200);
    //int ret = k_sem_take(&data->rx_sem, HPMA115_WAIT);

    // if (ret) {
    //     LOG_DBG("Error occured!\n");
    //     return ret;
    // }

    if (data->has_rsp) {
        // LOG_DBG("Rx buff %X %X\n", data->rx_buf[0], data->rx_buf[1]);
        // LOG_DBG("xfer bytes %d\n", data->xfer_bytes);
        // LOG_DBG("has rsp %d\n", data->has_rsp);
        LOG_DBG("response received!\n");
    }

    //  if (data->has_rsp) {
    //     // data received!
    //     if (data->rx_buf[0] == data->rx_buf[1] && data->rx_buf[0] != (uint8_t)(Ack)) {
    //         LOG_DBG("Ack error\n");
    //         return -1;
    //     }
    //     LOG_DBG("ACK %X %X",  data->rx_buf[0], data->rx_buf[1]);
    // }

    return ret;
}

static void hpma115_uart_isr(const struct device *uart_dev, void *user_data)
{
	const struct device *dev = user_data;
	struct uart_data *data = dev->data;

	ARG_UNUSED(user_data);

	if (uart_dev == NULL) {
		return;
	}

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (uart_irq_rx_ready(uart_dev)) {
		data->xfer_bytes += uart_fifo_read(uart_dev, &data->rx_buf[data->xfer_bytes], 
                        HPMA115_BUF_LEN - data->xfer_bytes);

        LOG_DBG("%X ", data->rx_buf[data->xfer_bytes - 1]);
        
		if ((data->xfer_bytes == data_to_read) || (data->xfer_bytes == HPMA115_BUF_LEN)) {
			data->xfer_bytes = 0;
            data->has_rsp = true;
			uart_irq_rx_disable(uart_dev);
        	//k_sem_give(&data->rx_sem);
		}
	}

	// if (uart_irq_tx_ready(uart_dev)) {
	// 	data->xfer_bytes +=
	// 		uart_fifo_fill(uart_dev, &data->tx_buf[data->xfer_bytes],
	// 			       HPMA115_BUF_LEN - data->xfer_bytes);

	// 	if (data->xfer_bytes == HPMA115_BUF_LEN) {
	// 		data->xfer_bytes = 0;
	// 		uart_irq_tx_disable(uart_dev);
	// 		if (!data->has_rsp) {
	// 			k_sem_give(&data->tx_sem);
	// 		}
	// 	}
	// }
}

static int hpma115_init(const struct device *dev)
{
	struct hpma115_conf *conf = dev->config; 
    struct uart_data *data = conf->uart_data;
	int ret = 0;

	uart_irq_rx_disable(conf->uart_dev);
	// uart_irq_tx_disable(cfg->uart_dev);

	hpma115_uart_flush(conf->uart_dev);

	uart_irq_callback_user_data_set(conf->uart_dev, conf->cb, (void *)dev);

	// k_sem_init(&data->tx_sem, 0, 1);
    k_sem_init(&data->rx_sem, 1, 1);

	/* start measurement */
    // LOG_DBG("Start measurement...");
    // ret = hpma115_attr_start_measurement(dev);
	// if (ret != 0) {
	// 	LOG_ERR("Error start measurement!");
	// 	return ret;
	// }
    // LOG_DBG(" OK\n");

	return ret;
}
#define HPMA115_INIT(inst)									\
												\
	static struct hpma115_sensor_data hpma115_sensor_data_##inst;				\
												\
	static const struct hpma115_conf hpma115_conf_##inst = {					\
		.uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),					\
        .cb = hpma115_uart_isr,                 \
	};											\
												\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, hpma115_init, NULL,					\
			      &hpma115_sensor_data_##inst, &hpma115_conf_##inst,				\
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &hpma115_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(HPMA115_INIT)