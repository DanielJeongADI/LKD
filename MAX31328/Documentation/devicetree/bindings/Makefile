MODULES := rtc-max31328.o

PWD=$(shell pwd)
VER=$(shell uname -r)
KERNEL_BUILD=/lib/modules/$(VER)/build
#TEST_VER=4.19.59-v7+
CUSTOM_KERNEL_MOD=/lib/modules/$(VER)/kernel/drivers/rtc

obj-m := $(MODULES)

#path of the arm compiled kernel
ROOTDIR := /lib/modules/$(shell uname -r)/build

MAKEARCH := $(MAKE)

all: 
	$(MAKEARCH) -C $(ROOTDIR) M=${shell pwd} modules
	sudo cp rtc-max31328.ko $(CUSTOM_KERNEL_MOD)
clean:
	$(MAKEARCH) -C $(ROOTDIR) M=${shell pwd} clean
install:
	$(MAKE) -C $(KERNEL_BUILD) M=$(PWD) INSTALL_MOD_PATH=$(INSTALL_ROOT) modules_install
	depmod -A
dtbs:
	dtc -I dts -O dtb -o rtc-max31328.dtbo i2c-rtc-overlay-max31328.dts
	sudo cp rtc-max31328.dtbo /boot/overlays
