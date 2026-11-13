# module/buddha/Makefile -- the buddha root-class loadable module: the
# Buddha / Catweasel Z-II flavour of src/ide.c (src/buddha.c selects it),
# built with amix-dlm's ABI kit like gayle (../Makefile).
#
#   make -C module buddha     buddha on both lines:
#                             build/buddha/<line>/mod.d/buddha
#   make -C module/buddha KIT=<a line's kit>    one line of buddha alone
#
# Its own MODOUT, build/buddha/<line> (build/buddha/<line>-diag with DIAG=1),
# so the kit's staging directory never meets gayle's and the two builds run
# in either order; the last path element stays the line's name.

TOP := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))../..)

ifeq ($(KIT),)
$(error KIT= names one line's kit; ../Makefile passes it)
endif

MODULE = buddha
OBJS   = buddha.o
DIAGFLAGS = $(if $(filter-out 0,$(DIAG)),-DIDE_DIAG=1)
MODOUT = $(TOP)/build/buddha/$(LINE)$(if $(DIAGFLAGS),-diag)
SRC    = $(TOP)/src
ALIEN  = -I$(KIT)/stock/usr/sys/amiga/alien
LINEFLAGS = $(if $(filter k46,$(LINE)),-DIDE_CACHE=1 -DIDE_NBPP=4096)
CFLAGS = -DIDE_MODULE -fno-common $(ALIEN) $(LINEFLAGS) $(DIAGFLAGS)
vpath %.c $(SRC)
include $(KIT)/mk/module.mk

# like gayle: never the Gayle ID register (0xDE1000) in the linked object
nogayleid: $(MODOUT)/mod.d/$(MODULE)
	@python3 -c 'import sys; d = open(sys.argv[1], "rb").read(); \
	  sys.exit("$(LINE): 0xDE1000 found in buddha") if bytes.fromhex("00de1000") in d else print("$(LINE): no 0xDE1000 in buddha")' $<

.PHONY: nogayleid
