obj-m          += mcp25xxfd.o
mcp25xxfd-objs := mcp25xxfd-core.o
mcp25xxfd-objs += mcp25xxfd-crc16.o
mcp25xxfd-objs += mcp25xxfd-dump.o
mcp25xxfd-objs += mcp25xxfd-regmap.o
mcp25xxfd-objs += mcp25xxfd-log.o



all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

install:
	sudo cp mcp25xxfd.ko /lib/modules/$(shell uname -r)/kernel/drivers/net/can/spi/
	sudo depmod -a
