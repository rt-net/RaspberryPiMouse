This directory stores configuration files and scripts for build test.

To run build test on your local machine, modify config in  `test.sh` and execute `./test.sh`.

# Appendix

## GCC Versions

### https://github.com/raspberrypi/tools

GCC 4.8 or 4.9


```
$ ./tools/arm-bcm2708/arm-rpi-4.9.3-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc --version
arm-linux-gnueabihf-gcc (crosstool-NG crosstool-ng-1.22.0-88-g8460611) 4.9.3
Copyright (C) 2015 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```

```
$ ./tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-gcc --version
arm-linux-gnueabihf-gcc (crosstool-NG linaro-1.13.1+bzr2650 - Linaro GCC 2014.03) 4.8.3 20140303 (prerelease)
Copyright (C) 2013 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```

### Raspbian Stretch

GCC 6.3

### Raspbian Buster

GCC 8.3

```
$ lsb_release -a
No LSB modules are available.
Distributor ID:	Raspbian
Description:	Raspbian GNU/Linux 10 (buster)
Release:	10
Codename:	buster
$ gcc --version
gcc (Raspbian 8.3.0-6+rpi1) 8.3.0
Copyright (C) 2018 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```

## REF

https://gcc.gnu.org/gcc-4.9/changes.html

https://github.com/abhiTronix/raspberry-pi-cross-compilers