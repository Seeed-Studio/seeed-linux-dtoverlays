#
# Peter Yang <turmary@126.com>
# Copyright (c) 2019 Seeed Studio
#
# MIT License
#

obj-m := mipi_dsi.o
mipi_dsi-objs := 	mipi_dsi_drv.o		\
					touch_panel.o		\
					panel-st7701s.o		\
					panel-ili9881d.o

all:
	make -C $(KDIR) M=$(PWD) modules

install:
	make -C $(KDIR) M=$(PWD) modules_install

clean:
	make -C $(KDIR) M=$(PWD) clean
