#
# Hongtai Liu <lht856@foxmail.com>
# Copyright (c) 2023 Seeed Studio
#
# MIT License
#
KVERSION ?= $(shell uname -r)

obj-m				+= rtc-pcf8563w.o

all:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) modules

install:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) modules_install

clean:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) clean
