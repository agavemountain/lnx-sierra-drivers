obj-m := GobiSerial.o
KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
OUTPUTDIR=/lib/modules/`uname -r`/kernel/drivers/usb/serial/

PI_KDIR := ~/k/linux-rpi-3.6.y
PI_CCPREFIX=~/toolchain/rpi/tools-master/arm-bcm2708/arm-bcm2708-linux-gnueabi/bin/arm-bcm2708-linux-gnueabi- 

OW_KDIR := ~/openwrt/trunk/build_dir/target-mips_r2_uClibc-0.9.33.2/linux-ar71xx_generic/linux-3.8.11
OW_CCPREFIX=~/openwrt/trunk/staging_dir/toolchain-mips_r2_gcc-4.6-linaro_uClibc-0.9.33.2/bin/mips-openwrt-linux-

MARVELL_KDIR := ~/toolchain/qmi_mxwell/kernel-2.6.31
MARVELL_CCPREFIX := ~/toolchain/qmi_mxwell/toolchain/bin/arm-none-linux-gnueabi-

M64_KDIR := ~/toolchain/arm64bit/linux/linux-4.4.1-devel-16.04.1
M64_CCPREFIX := ~/toolchain/arm64bit/be/aarch64ebv8-marvell-linux-gnu-5.2.1_x86_64_20151117/bin/aarch64_be-marvell-linux-gnu-

PPC_KDIR := ~/toolchain/ppc/kernel_ppc
PPC_CCPREFIX :=~/toolchain/ppc/fsl/1.1/sysroots/i686-fslsdk-linux/usr/bin/ppc64e5500-fsl-linux/powerpc64-fsl-linux-

HIKEY9_KDIR := /farm/android/Hikey_9.0_Trunk/kernel/hikey-linaro
HIKEY9_CCPREFIX := /farm/android/Hikey_9.0_Trunk/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9/bin/aarch64-linux-android-

all: clean
	$(MAKE) -C $(KDIR) M=$(PWD) modules

marvell: 
	$(MAKE) ARCH=arm CROSS_COMPILE=${MARVELL_CCPREFIX} -C $(MARVELL_KDIR) M=$(PWD) modules

pi: 
	$(MAKE) ARCH=arm CROSS_COMPILE=${PI_CCPREFIX} -C $(PI_KDIR) M=$(PWD) modules

ow: 
	$(MAKE) ARCH=mips CROSS_COMPILE=${OW_CCPREFIX} -C $(OW_KDIR) M=$(PWD) modules

m64:
	$(MAKE) ARCH=arm64 CROSS_COMPILE=${M64_CCPREFIX} -C $(M64_KDIR) M=$(PWD) modules

ppc:
	$(MAKE) ARCH=powerpc CROSS_COMPILE=${PPC_CCPREFIX} -C $(PPC_KDIR) M=$(PWD) modules

hikey9:
	$(MAKE) ARCH=arm64 CROSS_COMPILE=${HIKEY9_CCPREFIX} -C $(HIKEY9_KDIR) M=$(PWD) modules

install: all
	mkdir -p $(OUTPUTDIR)
	cp -f GobiSerial.ko $(OUTPUTDIR)
	depmod

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions Module.* modules.order .cache.mk *.o.ur-safe
