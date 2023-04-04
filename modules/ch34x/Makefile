#
# Bbear <953308023@qq.com>
# Copyright (c) 2023 Seeed Studio
#
# MIT License
#
KVERSION ?= $(shell uname -r)

obj-m := ch34x.o
ch34x-objs := ch343.o

all:
	$(MAKE) -C /lib/modules/$(KVERSION)/build M=$(PWD) modules

install:
	$(MAKE) -C /lib/modules/$(KVERSION)/build M=$(shell pwd) modules_install

clean:
	$(MAKE) -C /lib/modules/$(KVERSION)/build M=$(PWD) clean
