TARGET_MODULE:=max31730

PWD=$(shell pwd)
VER=$(shell uname -r)
KERNEL_BUILD=/lib/modules/$(VER)/build
CUSTOM_KERNEL_MOD=/lib/modules/$(VER)/kernel/drivers/iio/temperature

obj-m := $(TARGET_MODULE).o

all:
	$(MAKE) -C $(KERNEL_BUILD) M=$(PWD) modules
	sudo cp $(TARGET_MODULE).ko $(CUSTOM_KERNEL_MOD)
	dtc -I dts -O dtb -o max31730.dtbo max31730.dts
	sudo cp max31730.dtbo /boot/overlays
	
clean:
	$(MAKE) -C $(KERNEL_BUILD) M=$(PWD) clean
install:
	$(MAKE) -C $(KERNEL_BUILD) M=$(PWD) INSTALL_MOD_PATH=$(INSTALL_ROOT) modules_install
	depmod -A
load:
	insmod $(TARGET_MODULE).ko
dtbs:
	dtc -I dts -O dtb -o max31730.dtbo max31730.dts
	sudo cp max31730.dtbo /boot/overlays
