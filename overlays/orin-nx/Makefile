# Peter Yang
# Copyright (c) 2019 Seeed Studio
#
# MIT License
#

#DTBOS := $(patsubst %-overlay.dts, %.dtbo, $(wildcard *.dts))
DTBOS := $(patsubst %.dts, %.dtbo, $(wildcard *.dts))
DT := $(patsubst %-overlay.dtbo, %, ${DTBOS})
# $(warning DTBOS = $(DTBOS))

DEST  := /boot/
SHELL := /bin/bash
CPP   := aarch64-buildroot-linux-gnu-gcc -E
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

#%.dtbo : %-overlay.dts
%.dtbo : %.dts
	$(call cmd_dtc)

PHONY += all
all: ${DTBOS}
	@:

PHONY += clean
clean:
	rm -f *.dtbo *.tmp

PHONY += install
install:
	@for i in ${DT}; do									\
		cp $$i-overlay.dtbo ${DEST}/$$i.dtbo || exit;	\
		echo install $$i-overlay.dtbo ${DEST}/$$i.dtbo;	\
	done

PHONY += uninstall
uninstall:
	@for i in ${DT}; do						\
		rm ${DEST}/$$i.dtbo || exit;		\
		echo uninstall ${DEST}/$$i.dtbo;	\
	done

PHONY += FORCE
FORCE:

.PHONY: $(PHONY)

