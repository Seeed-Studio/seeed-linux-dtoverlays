
DTC ?= dtc
CPP ?= cpp
DESTDIR ?=

DTCVERSION ?= $(shell $(DTC) --version | grep ^Version | sed 's/^.* //g')

MAKEFLAGS += -rR --no-print-directory

ALL_PLATFORMES := $(patsubst overlays/%,%,$(wildcard overlays/*))

PHONY += all
all: help # $(foreach i,$(ALL_PLATFORMES),all_$(i))
	@echo ""
	@echo "nothing to do, please use target all_<platform>"

PHONY += clean
clean: # $(foreach i,$(ALL_PLATFORMES),clean_$(i))
	@echo "nothing to do, please use target clean_<platform>"

PHONY += install
install: # $(foreach i,$(ALL_PLATFORMES),install_$(i))
	@echo "nothing to do, please use target install_<platform>"

# Do not:
# o  use make's built-in rules and variables
#    (this increases performance and avoids hard-to-debug behaviour);
# o  print "Entering directory ...";
MAKEFLAGS += -rR --no-print-directory

# To put more focus on warnings, be less verbose as default
# Use 'make V=1' to see the full commands

ifeq ("$(origin V)", "command line")
  KBUILD_VERBOSE = $(V)
endif
ifndef KBUILD_VERBOSE
  KBUILD_VERBOSE = 0
endif

DTC_FLAGS += -Wno-unit_address_vs_reg
#http://snapshot.debian.org/binary/device-tree-compiler/
#http://snapshot.debian.org/package/device-tree-compiler/1.4.4-1/#device-tree-compiler_1.4.4-1
#http://snapshot.debian.org/archive/debian/20170925T220404Z/pool/main/d/device-tree-compiler/device-tree-compiler_1.4.4-1_amd64.deb

ifeq "$(DTCVERSION)" "1.4.5"
	#GIT BROKEN!!!! Ubuntu Bionic has patches..
	DTC_FLAGS += -Wno-dmas_property
	DTC_FLAGS += -Wno-gpios_property
	DTC_FLAGS += -Wno-pwms_property
	DTC_FLAGS += -Wno-interrupts_property
endif

ifeq "$(DTCVERSION)" "1.4.6"
	#http://snapshot.debian.org/package/device-tree-compiler/1.4.6-1/#device-tree-compiler_1.4.6-1
	#http://snapshot.debian.org/archive/debian/20180426T224735Z/pool/main/d/device-tree-compiler/device-tree-compiler_1.4.6-1_amd64.deb
	#Debian: 1.4.6
	DTC_FLAGS += -Wno-chosen_node_is_root
	DTC_FLAGS += -Wno-alias_paths
	DTC_FLAGS += -Wno-avoid_unnecessary_addr_size
endif

ifeq "$(DTCVERSION)" "1.4.7"
	#http://snapshot.debian.org/package/device-tree-compiler/1.4.7-3/#device-tree-compiler_1.4.7-3
	#http://snapshot.debian.org/archive/debian/20180911T215003Z/pool/main/d/device-tree-compiler/device-tree-compiler_1.4.7-3_amd64.deb
	#Debian: 1.4.6
	DTC_FLAGS += -Wno-chosen_node_is_root
	DTC_FLAGS += -Wno-alias_paths
	DTC_FLAGS += -Wno-avoid_unnecessary_addr_size
endif

ifeq "$(DTCVERSION)" "1.5.0"
	#http://snapshot.debian.org/package/device-tree-compiler/1.5.0-1/#device-tree-compiler_1.5.0-1
	#http://snapshot.debian.org/archive/debian/20190313T032949Z/pool/main/d/device-tree-compiler/device-tree-compiler_1.5.0-1_amd64.deb
	#Debian: 1.4.6
	DTC_FLAGS += -Wno-chosen_node_is_root
	DTC_FLAGS += -Wno-alias_paths
	DTC_FLAGS += -Wno-avoid_unnecessary_addr_size
endif

ifeq "$(DTCVERSION)" "2.0.0"
	#BUILDBOT...http://gfnd.rcn-ee.org:8080/job/beagleboard_overlays/job/master/
	DTC_FLAGS += -Wno-chosen_node_is_root
	DTC_FLAGS += -Wno-alias_paths
endif

# Beautify output
# ---------------------------------------------------------------------------
#
# Normally, we echo the whole command before executing it. By making
# that echo $($(quiet)$(cmd)), we now have the possibility to set
# $(quiet) to choose other forms of output instead, e.g.
#
#         quiet_cmd_cc_o_c = Compiling $(RELDIR)/$@
#         cmd_cc_o_c       = $(CC) $(c_flags) -c -o $@ $<
#
# If $(quiet) is empty, the whole command will be printed.
# If it is set to "quiet_", only the short version will be printed. 
# If it is set to "silent_", nothing will be printed at all, since
# the variable $(silent_cmd_cc_o_c) doesn't exist.
#
# A simple variant is to prefix commands with $(Q) - that's useful
# for commands that shall be hidden in non-verbose mode.
#
#       $(Q)ln $@ :<
#
# If KBUILD_VERBOSE equals 0 then the above command will be hidden.
# If KBUILD_VERBOSE equals 1 then the above command is displayed.

ifeq ($(KBUILD_VERBOSE),1)
  quiet =
  Q =
else
  quiet=quiet_
  Q = @
endif

# If the user is running make -s (silent mode), suppress echoing of
# commands

ifneq ($(filter 4.%,$(MAKE_VERSION)),)	# make-4
ifneq ($(filter %s ,$(firstword x$(MAKEFLAGS))),)
  quiet=silent_
endif
else					# make-3.8x
ifneq ($(filter s% -s%,$(MAKEFLAGS)),)
  quiet=silent_
endif
endif

export quiet Q KBUILD_VERBOSE

MOD_PATH := $(shell pwd)/modules

SRC_FOLDER := $(shell find $(MOD_PATH) -maxdepth 1 -type d)
BASE_SRC_FOLDER := $(basename $(patsubst $(MOD_PATH)/%, %, $(SRC_FOLDER)))
BASE_SRC_FOLDER := $(filter-out $(MOD_PATH), $(BASE_SRC_FOLDER))
BASE_SRC_FOLDER := $(filter-out grove-led, $(BASE_SRC_FOLDER))
BASE_SRC_FOLDER := $(filter-out grove-button, $(BASE_SRC_FOLDER))
ifneq ($(CUSTOM_MOD_FILTER_OUT),)
BASE_SRC_FOLDER := $(filter-out $(CUSTOM_MOD_FILTER_OUT), $(BASE_SRC_FOLDER))
endif
ifneq ($(CUSTOM_MOD_LIST),)
kmods := $(CUSTOM_MOD_LIST)
endif

uname_r = $(shell uname -r)
KBUILD ?= /lib/modules/$(uname_r)/build
KO_DIR ?= /lib/modules/$(uname_r)/extra/seeed

make_options="CROSS_COMPILE=${CC} KDIR=${x86_dir}/KERNEL"

kmods ?= $(shell cat $(shell pwd)/overlays/$*/config.txt)

all_%:
	$(Q)$(MAKE) PLATFORM=$* all_arch
	@for line in $(kmods); do make -C $(KBUILD) M=$(MOD_PATH)/$$line || exit; done

clean_%:
	$(Q)$(MAKE) PLATFORM=$* clean_arch
	@for line in $(kmods); do make -C $(KBUILD) M=$(MOD_PATH)/$$line clean || exit; done

install_%:
	$(Q)$(MAKE) PLATFORM=$* install_arch
	mkdir -p /lib/modules/$(uname_r)/extra/seeed || true
	@for line in $(kmods); do echo $(MOD_PATH)/$$line/*.ko; cp $(MOD_PATH)/$$line/*.ko $(KO_DIR) || exit; done
	@which depmod >/dev/null 2>&1 && depmod -a || true

ifeq ($(PLATFORM),)

ALL_DTS		:= $(shell find overlays/* -name \*.dts)

ALL_DTB		:= $(patsubst %.dts,%.dtbo,$(ALL_DTS))

$(ALL_DTB): PLATFORM=$(word 2,$(subst /, ,$@))
$(ALL_DTB): FORCE
	$(Q)$(MAKE) PLATFORM=$(PLATFORM) $@

else

PLATFORM_DTS	:= $(shell find overlays/$(PLATFORM) -name \*.dts)

PLATFORM_DTB	:= $(patsubst %.dts,%.dtbo,$(PLATFORM_DTS))

src	:= overlays/$(PLATFORM)
obj	:= overlays/$(PLATFORM)

include scripts/Kbuild.include

cmd_files := $(wildcard $(foreach f,$(PLATFORM_DTB),$(dir $(f)).$(notdir $(f)).cmd))

ifneq ($(cmd_files),)
  include $(cmd_files)
endif

quiet_cmd_clean    = CLEAN   $(obj)
      cmd_clean    = rm -f $(__clean-files)

dtc-tmp = $(subst $(comma),_,$(dot-target).dts.tmp)

dtc_cpp_flags  = -Wp,-MD,$(depfile).pre.tmp -nostdinc		\
                 -Iinclude -I$(src) -Ioverlays -Itestcase-data	\
                 -undef -D__DTS__

quiet_cmd_dtc = DTC     $@
cmd_dtc = $(CPP) $(dtc_cpp_flags) -x assembler-with-cpp -o $(dtc-tmp) $< ; \
        $(DTC) -O dtb -o $@ -b 0 -@ \
                -i $(src) -iinclude $(DTC_FLAGS) \
                -d $(depfile).dtc.tmp $(dtc-tmp) ; \
        cat $(depfile).pre.tmp $(depfile).dtc.tmp > $(depfile)

$(obj)/%.dtbo: $(src)/%.dts FORCE
	$(call if_changed_dep,dtc)

PHONY += all_arch
all_arch: $(PLATFORM_DTB)
	@:

PHONY += install_arch
install_arch: $(PLATFORM_DTBO)
	mkdir -p $(DESTDIR)/lib/firmware/
	cp -v $(obj)/*.dtbo $(DESTDIR)/lib/firmware/

RCS_FIND_IGNORE := \( -name SCCS -o -name BitKeeper -o -name .svn -o -name CVS \
                   -o -name .pc -o -name .hg -o -name .git \) -prune -o

PHONY += clean_arch
clean_arch: __clean-files = $(PLATFORM_DTB)
clean_arch: FORCE
	$(call cmd,clean)
	@find . $(RCS_FIND_IGNORE) \
		\( -name '.*.cmd' \
		-o -name '.*.d' \
		-o -name '.*.tmp' \
		\) -type f -print | xargs rm -f

endif

help:
	@echo "Targets:"
	@echo ""
	@echo "  all_<PLATFORM>:            Build all device tree binaries for <PLATFORM>"
	@echo "  clean_<PLATFORM>:          Clean all generated files for <PLATFORM>"
	@echo "  install_<PLATFORM>:        Install all generated files for <PLATFORM> (sudo)"
	@echo ""
	@echo "  overlays/<PLATFORM>/<DTS>.dtbo   Build a single device tree binary"
	@echo ""
	@echo "Architectures: $(ALL_PLATFORMES)"
	@echo ""
	@echo "Obsolete Targets: (no longer supported)"
	@echo "  all:                   Build all device tree binaries for all architectures"
	@echo "  clean:                 Clean all generated files"
	@echo "  install:               Install all generated files (sudo)"

PHONY += FORCE
FORCE:

.PHONY: $(PHONY)

builddeb:
	KO_LIST := $(shell ls $(KO_DIR))
	DTBO_LIST := $(shell ls overlays/bb/*.dtbo)
	cp debian/control control
	cp $(KO_DIR)/*.ko .
	echo "Package: seeed-linux-dtoverlay-bb-${uname_r}" >> control
	echo "Pre-Depends: linux-image-${uname_r}" >> control
	echo "Depends: linux-image-${uname_r}" >> control
	echo "Files: COPYING	/lib/modules/4.19.75-v7l+/extra/seeed/" >> control
	@for dir in ${KO_LIST}; do echo "	$$dir /lib/modules/${uname_r}/extra/seeed" >> control; done
	@for dir in ${DTBO_LIST}; do echo "	$$dir /lib/firmware/" >> control; done
	echo "		extras/tlv320aic3104.state.txt /etc/alsa" >> control
	equivs-build control
	rm -rf control || true
	rm -rf *.ko || true


