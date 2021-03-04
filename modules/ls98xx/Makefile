#
# Zhangqun Ming <north_sea@qq.com>
# Copyright (c) 2021 Seeed Studio
#
# MIT License
#

obj-m := alps_ls98xx.o
alps_ls98xx-objs := ls98xx.o

all:
	make -C $(KDIR) M=$(PWD) modules

install:
	make -C $(KDIR) M=$(PWD) modules_install

clean:
	make -C $(KDIR) M=$(PWD) clean
