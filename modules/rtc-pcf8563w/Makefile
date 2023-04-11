#
# Hongtai Liu <lht856@foxmail.com>
# Copyright (c) 2023 Seeed Studio
#
# MIT License
#

obj-m				+= rtc-pcf8563w.o

all:
	make -C $(KDIR) M=$(PWD) modules

install:
	make -C $(KDIR) M=$(PWD) modules_install

clean:
	make -C $(KDIR) M=$(PWD) clean