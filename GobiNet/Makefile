obj-m := GobiNet.o
GobiNet-objs := GobiUSBNet.o QMIDevice.o QMI.o usbnet_2_6_32.o usbnet_3_0_6.o \
	            usbnet_2_6_35.o usbnet_3_10_21.o usbnet_3_12_xx.o usbnet_4_4_xx.o
KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
OUTPUTDIR=/lib/modules/`uname -r`/kernel/drivers/net/usb/
#KBUILD_CFLAGS += -DQOS_SIMULATE
#KBUILD_CFLAGS += -DTX_XMIT_SIERRA -DTX_URB_MONITOR
ifdef TX_URB_MONITOR
ifeq ($(TX_URB_MONITOR), 1)
KBUILD_CFLAGS += -DTX_XMIT_SIERRA -DTX_URB_MONITOR
else
TX_URB_MONITOR:=0
endif
else
TX_URB_MONITOR:=0
endif
ifeq ($(TX_URB_MONITOR), 1)
	ccflags-y:=-DTX_URB_MONITOR
endif

RAWIP := 0
ifeq ($(RAWIP), 1)
	ccflags-y:=-DDATA_MODE_RP
endif

PI_KDIR := ~/k/linux-rpi-3.6.y
PI_CCPREFIX=~/toolchain/rpi/tools-master/arm-bcm2708/arm-bcm2708-linux-gnueabi/bin/arm-bcm2708-linux-gnueabi- 

OW_KDIR := ~/openwrt/trunk/build_dir/target-mips_r2_uClibc-0.9.33.2/linux-ar71xx_generic/linux-3.8.11
OW_CCPREFIX=~/openwrt/trunk/staging_dir/toolchain-mips_r2_gcc-4.6-linaro_uClibc-0.9.33.2/bin/mips-openwrt-linux-

MARVELL_KDIR := ~/toolchain/qmi_mxwell/kernel-2.6.31
MARVELL_CCPREFIX := ~/toolchain/qmi_mxwell/toolchain/bin/arm-none-linux-gnueabi-

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

marvell:
	$(MAKE) ARCH=arm CROSS_COMPILE=${MARVELL_CCPREFIX} -C $(MARVELL_KDIR) M=$(PWD) modules

pi: 
	$(MAKE) ARCH=arm CROSS_COMPILE=${PI_CCPREFIX} -C $(PI_KDIR) M=$(PWD) modules

ow: 
	$(MAKE) ARCH=mips CROSS_COMPILE=${OW_CCPREFIX} -C $(OW_KDIR) M=$(PWD) modules

install: all
	mkdir -p $(OUTPUTDIR)
	cp -f GobiNet.ko $(OUTPUTDIR)
	depmod

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions Module.* modules.order


