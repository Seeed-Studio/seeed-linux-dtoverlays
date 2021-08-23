#
# Baozhu Zuo <zuobaozhu@gmail.com>
# Copyright (c) 2021 Seeed Studio
#
# MIT License
#
KVERSION ?= $(shell uname -r)

obj-m := cc1120.o
net-cc1120-objs := cc1120.o

all:
	$(MAKE) -C /lib/modules/$(KVERSION)/build M=$(PWD) modules

install:
	$(MAKE) -C /lib/modules/$(KVERSION)/build M=$(shell pwd) modules_install

clean:
	$(MAKE) -C /lib/modules/$(KVERSION)/build M=$(PWD) clean
