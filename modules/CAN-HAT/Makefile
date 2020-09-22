#
# Copyright (c) 2020 Seeed Studio
#
# MIT License
#

_dts_model := $(shell strings /proc/device-tree/model)
_platform  := $(shell echo $(_dts_model) | sed -nr 's/NVIDIA.*Developer Kit/jtsn/gp')

# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language
ifneq ($(KERNELRELEASE),)
# $(warning KERNELVERSION=$(KERNELVERSION))

obj-m          += mcp25xxfd.o
mcp25xxfd-objs := mcp25xxfd-core.o
mcp25xxfd-objs += mcp25xxfd-crc16.o
mcp25xxfd-objs += mcp25xxfd-dump.o
mcp25xxfd-objs += mcp25xxfd-regmap.o
#mcp25xxfd-objs += mcp25xxfd-log.o


_KERN_VER := $(shell expr $(VERSION) \* 1000 + $(PATCHLEVEL) )
# $(warning _KERN_VER=$(_KERN_VER) )

# rx-offload only exist in kernel version >= 4.11
ifneq (,$(shell if [ $(_KERN_VER) -lt 4011 ]; then echo "NA"; fi ))
mcp25xxfd-objs += rx-offload.o
endif

# will remove when NVIDIA merged the patch
ifeq (jtsn,${_platform})
obj-m              += spi-tegra114p.o
spi-tegra114p-objs += spi-tegra114.o
endif

else # ifneq ($(KERNELRELEASE),)


all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

install:
	cp mcp25xxfd.ko /lib/modules/$(shell uname -r)/kernel/drivers/net/can/spi/
ifeq (jtsn,${_platform})
	cp spi-tegra114p.ko /lib/modules/$(shell uname -r)/drivers/spi/
endif
	depmod -a

.PHONY: all clean install

endif # ifneq ($(KERNELRELEASE),)

