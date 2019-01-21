# lnx-sierra-drivers
This repository contains the GobiNet and GobiSerial device
drivers for Linux.  If you use the Sierra SDK, and not the 
open source main-line drivers, these are the drivers that 
will be used.

Since I do not have access to the official code repository,
this repo was built from source tar-ball releases.

## Original Driver Releases:

The original "official" legacy source releases can be found
from:

+ **[codeaurora.org](https://portland.source.codeaurora.org/patches/quic/gobi/Gobi3000/OldReleases/):** These are the 
original open source drivers, from 2010.

### Releases (contained herein as individual commits)

- Gobi3000Drivers1010_09172010.tar.gz
- Gobi3000Drivers1020_11042010.tar.gz
- Gobi3000Drivers1030_01052011.tar.gz
- Gobi3000Drivers1040_03022011.tar.gz
- Gobi3000Drivers1050_05192011.tar.gz
- Gobi3000Drivers1060_06302011.tar.gz

## Resources

- [GobiNet driver provided by Netgear](http://www.downloads.netgear.com/files/aircard/Linux-Support-S2.13N2.25.zip) to support Kernel versions 3.19 and up
- [Compiling GobiNet on Ubuntu](https://bytefreaks.net/gnulinux/compiling-gobinet-on-ubuntu-16-04-64bit)
- Vedran Krivokuca's [patch](https://github.com/casastorta/gobiserial-patch/blob/master/GobiSerial.patch) to support later kernel versions for GobiSerial


