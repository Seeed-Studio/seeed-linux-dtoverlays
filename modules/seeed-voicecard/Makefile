#
# Peter Yang <turmary@126.com>
# Copyright (c) 2019 Seeed Studio
#
# MIT License
#
snd-soc-wm8960-objs := wm8960.o
snd-soc-ac108-objs := ac108.o ac101.o
snd-soc-seeed-voicecard-objs := seeed-voicecard.o


obj-m += snd-soc-wm8960.o
obj-m += snd-soc-ac108.o
obj-m += snd-soc-seeed-voicecard.o

all:
	make -C $(KDIR) M=$(PWD) modules

install:
	make -C $(KDIR) M=$(PWD) modules_install

clean:
	make -C $(KDIR) M=$(PWD) clean