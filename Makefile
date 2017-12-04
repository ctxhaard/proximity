ifneq ($(KERNELRELEASE),)
# kbuild part of makefile
include Kbuild
else
#normal makefile
export PATH := /home/ctomasin/development/gcc-linaro-5.1-2015.08-x86_64_arm-linux-gnueabihf/bin/:$(PATH)
KDIR ?= /home/ctomasin/development/raspi/linux
ARCH ?= arm
CROSS_COMPILE ?= arm-linux-gnueabihf-

default:
	$(MAKE) -C $(KDIR) M=$$PWD ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -j8 && scp $$PWD/hcsr04.ko root@uraspi:/root/
# && cp $$PWD/hcsr04.ko /mnt/robottino/home/pi/

# add module specific targets here
endif
