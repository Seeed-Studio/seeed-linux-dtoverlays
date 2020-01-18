mcp25xxfd-can-objs                  := mcp25xxfd_base.o
mcp25xxfd-can-objs                  += mcp25xxfd_can.o
mcp25xxfd-can-objs                  += mcp25xxfd_can_debugfs.o
mcp25xxfd-can-objs                  += mcp25xxfd_can_fifo.o
mcp25xxfd-can-objs                  += mcp25xxfd_can_int.o
mcp25xxfd-can-objs                  += mcp25xxfd_can_rx.o
mcp25xxfd-can-objs                  += mcp25xxfd_can_tx.o
mcp25xxfd-can-objs                  += mcp25xxfd_clock.o
mcp25xxfd-can-objs                  += mcp25xxfd_cmd.o
mcp25xxfd-can-objs                  += mcp25xxfd_crc.o
mcp25xxfd-can-objs                  += mcp25xxfd_debugfs.o
mcp25xxfd-can-objs                  += mcp25xxfd_ecc.o
mcp25xxfd-can-objs                  += mcp25xxfd_gpio.o
mcp25xxfd-can-objs                  += mcp25xxfd_int.o

obj-m				+= mcp25xxfd-can.o



all:
	make -C $(KDIR) M=$(PWD) modules

install:
	make -C $(KDIR) M=$(PWD) modules_install

clean:
	make -C $(KDIR) M=$(PWD) clean