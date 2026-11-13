# module/Makefile -- the gayle root-class loadable module, one per kernel line,
# built with amix-dlm's ABI kit (the kit's docs/guide.md; contracts C3-1, C4-1,
# C4-2), and the buddha module (module/buddha, the Buddha / Catweasel
# Z-II flavour of the same source).
#
#   make -C module            both lines: build/k30/mod.d/gayle and
#                             build/k46/mod.d/gayle, each checked by the kit's
#                             modfix and modcheck.py, and scanned for 0xDE1000
#   make -C module buddha     both lines of buddha: build/buddha/k30/mod.d/buddha
#                             and build/buddha/k46/mod.d/buddha, checked the
#                             same way
#   make -C module k30        one line (or k46)
#   make -C module static     the static object of each line compiled against
#                             the kit's headers, and its sha256 (the static
#                             driver must not change: README.md)
#   make -C module clean
#   make -C module DIAG=1     an investigation build (-DIDE_DIAG): adds the
#                             spin high-water and, on k46, the data-cache A/B
#                             switch and its counters; under build/<line>-diag,
#                             never for shipping
#
# DLM is the amix-dlm checkout whose kits are built (`make kit` there gives
# build/kit/k30 and build/kit/k46); KIT_k30 / KIT_k46 name a kit directly.
# The kits must be kabi k30.1.2 / k46.1.2 or later (root_defer on k30).
# The pinned cross tools (m68k-cbm-sysv4-*) and python3 must be on PATH.
# The kit's fragment does the rest: objects and the module under
# build/<line> (MODOUT, here the repo's build/), $cpu from the line.

TOP := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))..)
DLM ?= $(abspath $(TOP)/../amix-dlm)
LINES = k30 k46
kit = $(or $(KIT_$(1)),$(DLM)/build/kit/$(1))

ifeq ($(KIT),)

all: $(LINES)

$(LINES):
	$(MAKE) KIT=$(call kit,$@) all nogayleid

# buddha is not in `all' yet: the module core and the kit's modcheck refuse a
# zorro match key whose product is 00 (amix-dlm C3-1 section 3), and the
# Buddha's product is 0x00.
buddha: $(LINES:%=buddha-%)

$(LINES:%=buddha-%): buddha-%:
	$(MAKE) -C buddha KIT=$(call kit,$*) all nogayleid

static: $(LINES:%=static-%)

$(LINES:%=static-%): static-%:
	$(MAKE) -s KIT=$(call kit,$*) static

clean: $(LINES:%=clean-%)

$(LINES:%=clean-%): clean-%:
	$(MAKE) KIT=$(call kit,$*) clean
	$(MAKE) -C buddha KIT=$(call kit,$*) clean

.PHONY: all static clean buddha $(LINES) $(LINES:%=static-%) $(LINES:%=clean-%) $(LINES:%=buddha-%)

else	# one line, the kit's

MODULE = gayle
OBJS   = ide.o
DIAGFLAGS = $(if $(filter-out 0,$(DIAG)),-DIDE_DIAG=1)
MODOUT = $(TOP)/build/$(LINE)$(if $(DIAGFLAGS),-diag)
SRC    = $(TOP)/src
ALIEN  = -I$(KIT)/stock/usr/sys/amiga/alien
# the 68040/68060 line: the data-cache maintenance, 4 KiB pages, x4 spin budgets
LINEFLAGS = $(if $(filter k46,$(LINE)),-DIDE_CACHE=1 -DIDE_NBPP=4096)
CFLAGS = -DIDE_MODULE -fno-common $(ALIEN) $(LINEFLAGS) $(DIAGFLAGS)
vpath %.c $(SRC)
include $(KIT)/mk/module.mk

# The module must never reach the Gayle ID register (0xDE1000, Gary space on
# the A3000/A4000): no such constant may be in the linked object.
nogayleid: $(MODOUT)/mod.d/$(MODULE)
	@python3 -c 'import sys; d = open(sys.argv[1], "rb").read(); \
	  sys.exit("$(LINE): 0xDE1000 found in the module") if bytes.fromhex("00de1000") in d else print("$(LINE): no 0xDE1000 in the module")' $<

static:
	mkdir -p $(MODOUT)
	$(KIT_CC) $(KIT_CFLAGS) $(ALIEN) $(LINEFLAGS) $(DIAGFLAGS) -c $(SRC)/ide.c -o $(MODOUT)/ide-static.o
	cd $(TOP)/build && sha256sum $(notdir $(MODOUT))/ide-static.o

.PHONY: static nogayleid

endif
