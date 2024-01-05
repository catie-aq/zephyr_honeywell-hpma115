#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>

#include "hpma115.h"

#define SLEEP_TIME_MS   1000

void main(void)
{
    int ret;
    const struct device *sensor;
    struct sensor_value val;

    k_msleep(SLEEP_TIME_MS);
    sensor = DEVICE_DT_GET(DT_NODELABEL(hpma115));

    if (!device_is_ready(sensor)) {
        printk("Sensor not ready");
        return 0;
    }

    k_msleep(SLEEP_TIME_MS);

    ret = sensor_sample_fetch(sensor);

    while (1) {
       
        // ret = sensor_channel_get(sensor, SENSOR_CHAN_PM_1_0, &val);
        // printk("PM1_0: %d\n", val.val1);
        // ret = sensor_channel_get(sensor, SENSOR_CHAN_PM_2_5, &val);
        // printk("PM2_5: %d\n", val.val1);
        // ret = sensor_channel_get(sensor, SENSOR_CHAN_PM_10, &val);
        // printk("PM10: %d\n", val.val1);
		// if (ret < 0) {
		// 	printk("Could not get sample (%d)", ret);
		// 	return 0;
		// }        
        k_msleep(SLEEP_TIME_MS);
    }
}