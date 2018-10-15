mcp25xxfd-can-objs := mcp25xxfd.o


obj-m += mcp25xxfd-can.o



all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

install:
	sudo cp mcp25xxfd-can.ko /lib/modules/$(shell uname -r)/kernel/drivers/net/can/spi/
	sudo depmod -a
