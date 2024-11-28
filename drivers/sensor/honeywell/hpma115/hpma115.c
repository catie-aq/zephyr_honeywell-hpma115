#define DT_DRV_COMPAT honeywell_hpma115

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <errno.h>
#include <zephyr/logging/log.h> 
#include "hpma115.h"

LOG_MODULE_REGISTER(honeywell_hmpa115, CONFIG_SENSOR_LOG_LEVEL);

// define private prototypes
static int hpma115_send_command(const struct device *dev, enum hpma115_cmd cmd);
static void hpma115_write_command(const struct device *dev, uint8_t cmd_length);
static int hpma115_read_response(const struct device *dev, uint8_t length);
static int hpma115_read_data(const struct device *dev,  uint8_t *len, uint8_t **data_out);
static uint8_t hpma115_compute_checksum(uint8_t *frame);

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
 * @brief stop autosend.
 * 
 * @param dev hpma115 UART peripheral device.
 */
static int hpma115_attr_stop_autosend(const struct device *dev)
{
    return hpma115_send_command(dev, StopAutoSend);
}

static int hpma115_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct hpma115_sensor_data *result = dev->data;
    uint8_t len = 0;
    uint8_t *data_out = NULL;


    int ret = hpma115_read_data(dev, &len, &data_out);
    
    if (ret) {
        return ret;
    }

    LOG_DBG("data len: %d", len);
    // set data
    if (len == 13) { /* It's a compact sensor. */  
        result->pm1_pm4_valid = true;
        result->pm1_0 = ((uint16_t)(data_out[0]) << 8) + data_out[1];
        LOG_DBG("pm1_0: %d", result->pm1_0);
        result->pm2_5 = ((uint16_t)(data_out[2]) << 8) + data_out[3];
        LOG_DBG("pm2_5: %d", result->pm2_5);
        result->pm4_0 = ((uint16_t)(data_out[4]) << 8) + data_out[5];
        LOG_DBG("pm4_0: %d", result->pm4_0);
        result->pm10 = ((uint16_t)(data_out[6]) << 8) + data_out[7];
        LOG_DBG("pm10: %d", result->pm10);
    } else if (len == 5) { /* It's a standard sensor: not supported */
        result->pm1_pm4_valid = false;
        result->pm1_0 = 0;
        result->pm4_0 = 0;
        result->pm2_5 = ((uint16_t)(data_out[0]) << 8) + data_out[1];
        result->pm10 = ((uint16_t)(data_out[2]) << 8) + data_out[3];
    } else {
        LOG_DBG("Read measures error!");
    }

    return ret;
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

static const struct sensor_driver_api hpma115_api_funcs = {
	.channel_get = hpma115_channel_get,
    .sample_fetch = hpma115_sample_fetch,
};

static void hmpa115_clear_uart_data(struct uart_data *data)
{
    data->rx_data_len = 0;
    data->xfer_bytes = 0;
    data->has_rsp = false;
    memset(data->rx_buf, '\0', sizeof(data->rx_buf));
    memset(data->tx_buf, '\0', sizeof(data->tx_buf));
    
}

static void hpma115_uart_flush(const struct device *dev)
{
    const struct hpma115_conf *conf = dev->config;
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
    const struct hpma115_conf *conf = dev->config;
    struct uart_data *data = conf->uart_data;
    int ret;

    data->tx_buf[0] = (uint8_t)(Send);
    data->tx_buf[1] = 1;
    data->tx_buf[2] = (uint8_t)(ReadMeas);

    /* Invalid checksum and compute it */
    data->tx_buf[3] = 0;
    data->tx_buf[3] = hpma115_compute_checksum(data->tx_buf);

    hpma115_write_command(dev, 4);

    ret = hpma115_read_response(dev, HPMA115_BUF_LEN);

     if (data->has_rsp) {
        if (data->rx_buf[0] == (uint8_t)Resp) {
            *data_out = data->rx_buf + 3;
            *len = data->rx_buf[1];
        }
     }
   
    return ret;
}

static int hpma115_send_command(const struct device *dev, enum hpma115_cmd cmd)
{
    const struct hpma115_conf *conf = dev->config; 
    struct uart_data *data = conf->uart_data;
    int ret = 0;

    hpma115_uart_flush(dev);
    hmpa115_clear_uart_data(data);

    data->tx_buf[0] = (uint8_t)(Send);
    data->tx_buf[1] = 1;
    data->tx_buf[2] = (uint8_t)(cmd);

    /* Invalid checksum and compute it */
    data->tx_buf[3] = 0;
    data->tx_buf[3] = hpma115_compute_checksum(data->tx_buf);

    hpma115_write_command(dev, 4);

    ret = hpma115_read_response(dev, 2);

     if (data->has_rsp) {
        // data received!
        if (data->rx_buf[0] == data->rx_buf[1] && data->rx_buf[0] != (uint8_t)(Ack)) {
            LOG_DBG("Ack error\n");
            return -1;
        }
        LOG_DBG("ACK %X %X",  data->rx_buf[0], data->rx_buf[1]);
    }
      
    return ret;
}

static void hpma115_write_command(const struct device *dev, uint8_t cmd_length)
{
    const struct hpma115_conf *conf = dev->config;
    struct uart_data *data = conf->uart_data;

    LOG_DBG("Tx buf: %X %X %X %X",  data->tx_buf[0], data->tx_buf[1], data->tx_buf[2], data->tx_buf[3]);

    for (uint8_t i = 0; i < cmd_length; i++) {
        uart_poll_out(conf->uart_dev, data->tx_buf[i]);
    }
}

static int hpma115_read_response(const struct device *dev, uint8_t length)
{
    const struct hpma115_conf *conf = dev->config;
    struct uart_data *data = conf->uart_data;
    int ret = 0;

    data->rx_data_len_to_read = length;
    
    uart_irq_rx_enable(conf->uart_dev);

    ret = k_sem_take(&data->rx_sem, HPMA115_WAIT);

    if (ret) {
        LOG_DBG("Error occured!\n");
        return ret;
    }

    return ret;
}

static void hpma115_uart_isr(const struct device *uart_dev, void *user_data)
{
	const struct device *dev = user_data;
    const struct hpma115_conf *conf = dev->config;
	struct uart_data *data = conf->uart_data;

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
        
		if ((data->xfer_bytes == data->rx_data_len_to_read) || (data->xfer_bytes == HPMA115_BUF_LEN)) {
			data->rx_data_len = data->xfer_bytes;
            data->xfer_bytes = 0;
            data->has_rsp = true;
			uart_irq_rx_disable(uart_dev);
        	k_sem_give(&data->rx_sem);
		}
	}
}

static int hpma115_init(const struct device *dev)
{
	const struct hpma115_conf *conf = dev->config; 
    struct uart_data *data = conf->uart_data;
	int ret = 0;

	uart_irq_rx_disable(conf->uart_dev);

	uart_irq_callback_user_data_set(conf->uart_dev, conf->cb, (void *)dev);

	// k_sem_init(&data->tx_sem, 1, 1);
    k_sem_init(&data->rx_sem, 0, 1);

    /* stop autosend */
    ret = hpma115_attr_stop_autosend(dev);
    if (ret) {
        LOG_ERR("Error on stop autosend!");
        return ret;
    }
	/* start measurement */
    ret = hpma115_attr_start_measurement(dev);
	if (ret) {
		LOG_ERR("Error on start measurement!");
		return ret;
	}
    LOG_DBG("sensor init OK\n");

	return ret;
}
#define HPMA115_INIT(inst)									\
												\
	static struct hpma115_sensor_data hpma115_sensor_data_##inst;				\
                                                \
    static struct uart_data hpma115_uart_data; \
												\
	static const struct hpma115_conf hpma115_conf_##inst = {					\
        .uart_data = &hpma115_uart_data, \
		.uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),					\
        .cb = hpma115_uart_isr,                 \
	};											\
												\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, hpma115_init, NULL,					\
			      &hpma115_sensor_data_##inst, &hpma115_conf_##inst,				\
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &hpma115_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(HPMA115_INIT)