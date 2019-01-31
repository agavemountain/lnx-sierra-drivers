# Sierra Wireless Drivers for Linux

This repository contains the GPL/BSD GobiNet and GobiSerial device drivers for Linux.  If you use the Sierra SDK, and not the 
open source main-line drivers, these are the drivers that will be used.

This is *NOT* the official repository.  Since I do not have access to the official code repository,
this repo was built from source tar-ball releases.

## Building on Ubuntu 16.04

Add the following entries to the "/etc/modprobe.d/blacklist-modem.conf" file:

```bash
blacklist qcserial
blacklist qmi_wwan
```

Building:

```bash
# install dependancies
sudo apt-get install build-essential make gcc
sudo apt-get install linux-headers=`uname -r`

cd GobiSerial; make; sudo make install
cd GobiNet; make; sudo make install
sudo modprobe GobiSerial
sudo modprobe GobiNet
```

## Sierra Provided Drivers

Each release has been imported as a commit, so you can see what has changed from version to version.  The release
notes are not maintained, so the source becomes valuable in deciding whether or not to take the latest 
version. 

Here are the available USB drivers releases that have been imported in the Linux QMI Software which are in this repository:

- USB drivers Linux QMI Software S2.36N2.55
- USB drivers Linux QMI Software S2.35N2.54
- USB drivers Linux QMI Software S2.34N2.53
- USB drivers Linux QMI Software S2.33N2.52
- USB drivers Linux QMI Software S2.31N2.50
- USB drivers Linux QMI Software S2.30N2.48
- USB drivers Linux QMI Software S2.29N2.47
- USB drivers Linux QMI Software S2.29N2.46
- USB drivers Linux QMI Software S2.29N2.45.1
- USB drivers Linux QMI Software S2.28N2.44
- USB drivers Linux QMI Software S2.28N2.43
- USB drivers Linux QMI Software S2.28N2.42
- USB drivers Linux QMI Software S2.27N2.41
- USB drivers Linux QMI Software S2.27N2.40
- USB drivers Linux QMI Software S2.26N2.39
- USB drivers Linux QMI Software S2.26N2.38
- USB drivers Linux QMI Software S2.25N2.36
- USB drivers Linux QMI Software S2.25N2.35
- USB drivers Linux QMI Software S2.25N2.34
- USB drivers Linux QMI Software S2.24N2.33
- USB drivers Linux QMI Software S2.23N2.31
- USB drivers Linux QMI Software S2.23N2.30
- USB drivers Linux QMI Software S2.22N2.29
- USB drivers Linux QMI Software S2.20N2.27

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


