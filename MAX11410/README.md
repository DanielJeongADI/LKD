# MAX11410 Linux Driver
The project consists of:
  - MAX11410 driver `max11410.c`
  - An example `Makefile` for building the driver as kernel module
  - An example device tree overlay for Raspberry Pi 4: `iio-adc-max11410.dts`


## Installation on Raspberry Pi
Here are the steps for compiling this driver as a kernel module on Raspberry Pi OS. Please refer to https://www.raspberrypi.com/documentation/computers/linux_kernel.html for more information. Installation steps may differ depending on your system.

Make sure you have the kernel headers installed
```shell
sudo apt install raspberrypi-kernel-headers
```
Compile the driver
```shell
make all
```
Install the module into your system
```shell
sudo make install
```
Compile the device tree overlay and copy it into boot directory. Example overlay assumes that the device is connected to `SPI0` and `GPIO1` of `MAX11410` is connected to `GPIO6` of `RaspberryPi`. You may need to change this configuration according to your system.
```shell
make dts
```
Reboot your system
```shell
sudo reboot
```
# How to use
Once you compile the driver and load it into kernel, a device node will be created under `/sys/bus/iio/devices/iio:deviceX` in which you will find sysfs entries to interact with the device.
## Conversion
You can start a conversion and read raw ADC data by reading from `in_voltage_raw`. Data is displayed in decimal.
```shell
$ cat in_voltage_raw
2526
```
You can also read from indexed channels: `in_voltage0_raw` to `in_voltage9_raw`. When you read from an indexed channel, `ainn` is connected to `agnd` and `ainp` is connected to corresponding channel before starting the conversion.
## Continuous Conversion
This driver supports triggered buffering that can collect continuous samples from the device.

First, enable the scan elements:
```shell
echo 1 > scan_elements/in_voltage_en
echo 1 > scan_elements/in_timestamp_en
```
Then, enable the buffer.
```shell
echo 1 > buffer/enable
```
Once the buffer is enabled you can collect the converted data from character device `/dev/iio:deviceX`. Each sample is 16 bytes. First 8 bytes are adc data and last 8 bytes are unix timestamp of the corresponding sample.
```shell
$ sudo cat /dev/iio\:device0 | hexdump
0000000 00e3 0000 0000 0000 9712 d39f 114a 16bf
0000010 00e4 0000 0000 0000 5815 0cfa 114b 16bf
0000020 0101 0000 0000 0000 f9f4 c174 114f 16bf
0000030 00fc 0000 0000 0000 1b0b fad0 114f 16bf
...
```
## Sysfs Entries for Device Configuration
Here is the list of sysfs entries exposed by this driver for device configuration.

|Configuration|Possible Values|
|---|---|
|`config_input_path`|`buffered`, `bypass`, `pga`
|`config_gain`|`1`, `2`, `4`, `8`, `16`, `32`, `64`, `128`
|`config_ainp`|`ain0`, `ain1`, `ain2`, `ain3`, `ain4`, `ain5`, `ain6`, `ain7`, `ain8`, `ain9`, `avdd` or `unconnected`|
|`config_ainn`|`ain0`, `ain1`, `ain2`, `ain3`, `ain4`, `ain5`, `ain6`, `ain7`, `ain8`, `ain9`, `gnd` or `unconnected`|
|`config_input_range`|`bipolar`, `unipolar`|
|`config_filter`|`fir50/60`, `fir50`, `fir60`, `sinc4`|
|`config_rate`|`0`, `1`, `2`, `3`, `4`, `5`, `6`, `7`, `8`, `9`, `10`, `11`, `12`, `13`, `14`, `15`|
|`config_ref_sel`|`ref0p/ref0n`, `ref1p/ref1n`, `ref2p/ref2n`, `avdd/agnd`, `ref0p/gnd`, `ref1p/gnd`, `ref2p/gnd`, `avdd/agnd`|
|`config_calib_start`|`self`, `pga`|


# References
  - https://www.raspberrypi.com/documentation/computers/linux_kernel.html
  - https://www.raspberrypi.com/documentation/computers/configuration.html#part2
