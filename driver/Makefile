CONFIG_MODULE_SIG=n

ifeq ($(KERNELRELEASE), )
KERNELDIR := /lib/modules/$(shell uname -r)/build
PWD :=$(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR)  M=$(PWD)  
clean:
	rm -rf *.mk .tmp_versions Module.symvers *.mod.c *.o *.ko .*.cmd Module.markers modules.order *.a *.mod
load:
	modprobe usbnet
	insmod ch397.ko
unload:
	rmmod ch397
install: default
	rmmod ch397 || true
	modprobe usbnet
	insmod ch397.ko || true
	mkdir -p /lib/modules/$(shell uname -r)/kernel/drivers/net/usb/ || true
	cp -f ./ch397.ko /lib/modules/$(shell uname -r)/kernel/drivers/net/usb/ || true
	depmod -a
uninstall:
	rmmod ch397 || true
	rm -rf /lib/modules/$(shell uname -r)/kernel/drivers/net/usb/ch397.ko || true
	depmod -a
else
	obj-m := ch397.o
endif
