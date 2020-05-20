#
# Peter Yang <turmary@126.com>
# Copyright (c) 2019 Seeed Studio
#
# MIT License
#

subdir-ccflags-y += -I$(srctree)/sound/soc/codecs/
subdir-ccflags-y += -I$(src)/include/

snd-soc-wm8960-objs := wm8960.o
# snd-soc-seeed-tegra-machine-objs := tegra_machine_driver_mobile.o


obj-m += snd-soc-wm8960.o
# obj-m += snd-soc-seeed-tegra-machine.o

all:
	make -C $(KDIR) M=$(PWD) modules

install:
	make -C $(KDIR) M=$(PWD) modules_install

clean:
	make -C $(KDIR) M=$(PWD) clean
