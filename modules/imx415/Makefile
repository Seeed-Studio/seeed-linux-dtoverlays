#
# Hongtai.Liu <lht856@foxmail.com>
# Copyright (c) 2021 Seeed Studio
#
# MIT License
#

obj-m += imx415.o

all:
	make -C $(KDIR) M=$(PWD) modules

install:
	make -C $(KDIR) M=$(PWD) modules_install

clean:
	make -C $(KDIR) M=$(PWD) clean