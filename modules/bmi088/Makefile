#
# Bbear <ruiqian.tang@seeed.cc>
# Copyright (c) 2025 Seeed Studio
#
# MIT License
#
KVERSION ?= $(shell uname -r)

obj-m := bmi088_accel.o bmi088_gyro.o
bmi088_accel-y := bmi088_accel_core.o bmi088_accel_i2c.o
bmi088_gyro-y  := bmi088_gyro_core.o  bmi088_gyro_i2c.o

all:
	$(MAKE) -C /lib/modules/$(KVERSION)/build M=$(PWD) modules

install:
	$(MAKE) -C /lib/modules/$(KVERSION)/build M=$(shell pwd) modules_install

clean:
	$(MAKE) -C /lib/modules/$(KVERSION)/build M=$(PWD) clean
