#
#Copyright (C) 2019 Seeed Technology Co,Ltd  - https://www.seeed.cc
#Author: Baozhu Zuo <zuobaozhu@gmail.com>#
#This program is free software; you can redistribute it and/or modify
#it under the terms of the GNU General Public License version 2 as
#published by the Free Software Foundation.
#

touch-gt9xx-objs := gt9xx.o gt9xx_update.o goodix_tool.o
obj-m += touch-gt9xx.o

all:
	make -C $(KDIR) M=$(PWD) modules

install:
	make -C $(KDIR) M=$(PWD) modules_install

clean:
	make -C $(KDIR) M=$(PWD) clean