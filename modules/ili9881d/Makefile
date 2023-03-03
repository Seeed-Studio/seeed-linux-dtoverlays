#
# Zhenhua Liang <zhenhua.liang@seeed.cc>
# Copyright (c) 2023 Seeed Studio
#
# MIT License
#
KDIR ?=/lib/modules/$(shell uname -r)/build
obj-m += ili9881d.o

all:
	make -C $(KDIR) M=$(PWD) modules

install:
	make -C $(KDIR) M=$(PWD) modules_install

clean:
	make -C $(KDIR) M=$(PWD) clean
