#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>

#define SLEEP_TIME_MS   1000

void main(void)
{
    int ret;
    struct sensor_value val;
    const struct device *sensor;
    sensor = DEVICE_DT_GET(DT_NODELABEL(hpma115));

    if (!device_is_ready(sensor)) {
        printk("Sensor not ready");
        return 0;
    }

    while (1) {
        //printk("Alive!\n");
        ret = sensor_channel_get(sensor, SENSOR_CHAN_PM_1_0, &val);
		if (ret < 0) {
			printk("Could not get sample (%d)", ret);
			return 0;
		}

		printk("Sensor value: %d\n", val.val1);

        k_msleep(SLEEP_TIME_MS);
    }
    

}