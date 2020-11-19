#
# Peter Yang <turmary@126.com>
# Copyright (c) 2019 Seeed Studio
#
# MIT License
#

obj-m := bma456.o
bma456-objs := bma456_drv.o			\
			   api/bma4.o			\
			   api/bma456.o

all:
	make -C $(KDIR) M=$(PWD) modules

install:
	make -C $(KDIR) M=$(PWD) modules_install

clean:
	make -C $(KDIR) M=$(PWD) clean
