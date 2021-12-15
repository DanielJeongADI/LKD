obj-m += max11410.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) clean

install: all
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) modules_install
	depmod -A

checkfile:
	/lib/modules/$(shell uname -r)/build/scripts/checkpatch.pl --no-tree -f max11410.c

dtbs:
	cpp -nostdinc -I include -I arch  -undef -x assembler-with-cpp iio-adc-max11410.dts iio-adc-max11410.dts.preprocessed
	dtc -I dts -O dtb -q -o iio-adc-max11410.dtbo iio-adc-max11410.dts.preprocessed
	rm iio-adc-max11410.dts.preprocessed
	sudo cp iio-adc-max11410.dtbo /boot/overlays
