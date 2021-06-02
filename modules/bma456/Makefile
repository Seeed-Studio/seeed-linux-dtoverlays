#
# Peter Yang <turmary@126.com>
# Copyright (c) 2019 Seeed Studio
#
# MIT License
#

obj-m := bma456.o
bma456-objs :=	api/bma4_common.o	\
				api/bma4.o			\
				api/bma456.o		\
				bma456_core.o		\
				bma456_i2c.o		\
				bma456_iio.o		\
				bma456_irq.o		\
				bma456_input.o

all:
	make -C $(KDIR) M=$(PWD) modules

install:
	make -C $(KDIR) M=$(PWD) modules_install

clean:
	make -C $(KDIR) M=$(PWD) clean
