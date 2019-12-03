#
# Peter Yang <turmary@126.com>
# Copyright (c) 2019 Seeed Studio
#
# MIT License
#

# no irq version
_tgt     := adxl34x
_tgt_i2c     := adxl34x-i2c

_uname_r := $(shell uname -r)
_BUILD   := /lib/modules/$(_uname_r)/build
_PWD     := $(shell pwd)
_DEPMOD  ?= depmod -a

# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language
ifneq ($(KERNELRELEASE),)

obj-m += ${_tgt}.o
obj-m += ${_tgt_i2c}.o

else
_class  := drivers/input/misc

DEST  := /lib/modules/$(_uname_r)/kernel

all:
	make -C ${_BUILD} M=$(_PWD) modules

clean:
	make -C ${_BUILD} M=$(_PWD) clean

install:
	cp ${_tgt}.ko ${DEST}/${_class}/
	cp ${_tgt_i2c}.ko ${DEST}/${_class}/
	$(_DEPMOD)

uninstall:
	rm -f ${DEST}/${_class}/${_tgt}.ko
	rm -f ${DEST}/${_class}/${_tgt_i2c}.ko
	$(_DEPMOD)

.PHONY: all clean install uninstall
endif
