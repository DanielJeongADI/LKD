# Maxim Max17201/Max17205 Fuel Gauge driver 

* Add following lines to file ./drivers/power/Kconfig
	config BATTERY_MAX1720X
		   tristate "MAX17201/MAX17205 fuel gauge driver"
		   depends on I2C
		   help
			 Say Y to enable support for MAX17201/MAX17205 Fuel Gauge driver


* Add following line to file ./drivers/power/Makefile

	obj-$(CONFIG_BATTERY_MAX1720X)  += max1720x_battery.o
	
* Add config option "CONFIG_BATTERY_MAX1720X"
