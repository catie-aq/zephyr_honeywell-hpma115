# Zephyr Honeywell hpma115 sensor example


## Required hardware
To run this project the following hardware is required:
 - Zest_Sensor_Dustdevicetree overlays.   
 - Zest_Core_stm32l4a6rg

## Setup
1. Install project by running the following commands from your project directory. This will download the project and any dependencies.
```sh
west init -m https://github.com/catie-aq/zephyr_honeywell-hpma115-example
west update
```

4. Install Zephyr Python dependencies as described by the [Zephyr docs](https://docs.zephyrproject.org/2.6.0/getting_started/index.html#install-dependencies).

2. Build the project by entering the directory `./ zephyr_honeywell-hpma115-example` and running
```sh
west build
```
3. Program the target
```sh
west flash
```