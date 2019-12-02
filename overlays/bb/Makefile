# Peter Yang
# Copyright (c) 2019 Seeed Studio
#
# MIT License
#

DTBOS := $(patsubst %.dts, %.dtbo, $(wildcard *.dts))
# $(warning DTBOS = $(DTBOS))

DEST  := /lib/firmware
SHELL := /bin/bash
CPP   := gcc -E
DTC   := dtc


DTC_FLAGS += -Wno-unit_address_vs_reg
dtc-tmp = $(<).tmp

dtc_cpp_flags  = -nostdinc		\
                 -Iinclude		\
                 -undef -D__DTS__

define cmd_dtc
	$(CPP) $(dtc_cpp_flags) -x assembler-with-cpp -o $(dtc-tmp) $< ;
        $(DTC) -I dts -O dtb -o $@ -b 0 -@	\
                $(DTC_FLAGS) 			\
                $(dtc-tmp) ;
endef

%.dtbo : %.dts
	$(call cmd_dtc)


PHONY += all
all: ${DTBOS}

PHONY += clean
clean:
	rm -f *.dtbo *.tmp

PHONY += install
install:
	@for i in ${DTBOS}; do			\
		cp $$i ${DEST}/;		\
		echo install ${DEST}/$$i;	\
	done
	#
	# What you need to do, for example:
	#   enable LIS331DLH device in system configuration
	# echo 'uboot_overlay_addr7=/lib/firmware/BB-I2C2-LIS331DLH.dtbo' >> /boot/uEnv.txt
	#

PHONY += uninstall
uninstall:
	@for i in ${DTBOS}; do			\
		rm ${DEST}/$$i;			\
		echo uninstall ${DEST}/$$i;	\
	done

PHONY += FORCE
FORCE:

.PHONY: $(PHONY)

