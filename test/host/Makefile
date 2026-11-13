# Host test harness for ../../src/ide.c -- compiles the REAL driver source with
# the host C compiler against a mock Gayle/ATA model and checks its SCSI->ATA
# translation, probe gating, byte order handling and error paths.
#
#   make check     # build + run every variant; exit 0 == all assertions pass
#   make clean
#
# The driver is K&R C (-std=gnu89).  It is compiled in a kernel-only include
# world (-Istubs shadows sys/types.h, sys/errno.h, sys/inline.h, rico.h, sd.h)
# with HOST_TEST defined and mock_regs.h force-included so its register seam
# lands in the mock.  The harness files are ordinary host C.
#
# Two variants of the driver are built and run: the 68030 object as shipped,
# and the 68040/68060 one (-DIDE_CACHE=1 -DIDE_NBPP=4096), whose data-cache
# line operations, ordering barrier and TTR reads land in the mock as well.
#
# The root-class module form (-DIDE_MODULE) is built and run the same two
# ways (ide_modtest, ide_modtest_040): its load, deferred self-test, halt and
# unload entries against the same mock, with the kernel calls it adds mocked
# in ide_modtest.c.
#
# The Buddha / Catweasel Z-II flavour (-DIDE_MODULE -DIDE_BUDDHA, the module
# `buddha') is built and run the same two ways (ide_budtest, ide_budtest_040)
# against the mock's Buddha boards.
#
# The investigation build (-DIDE_DIAG) is built and run the same two ways as
# well (ide_test_diag, ide_test_040_diag): the spin high-water, and on the
# 040/060 line the ide_cache A/B switch and the push counters.  The default
# builds check that those symbols are absent (weak references) and that the
# line operations still run.

CC      ?= cc
CSTD    := -std=gnu89 -fno-builtin
CWARN   := -Wall -Wno-unused-parameter -Wno-unused-variable -Wno-deprecated-non-prototype \
           -Wno-unused-function
CFLAGS  := $(CSTD) $(CWARN) -O0 -g
BUILD   := build
SRC     := ../../src
DRV     := $(SRC)/ide.c

CACHE     := -DIDE_CACHE=1 -DIDE_NBPP=4096
DIAG      := -DIDE_DIAG=1
STUBS     := mock_regs.h stubs/rico.h stubs/sd.h \
             stubs/sys/types.h stubs/sys/errno.h stubs/sys/inline.h stubs/sys/param.h
HOST_OBJS := $(BUILD)/mock_gayle.o $(BUILD)/kstubs.o
BIN       := $(BUILD)/ide_test
BIN040    := $(BUILD)/ide_test_040
MODBIN    := $(BUILD)/ide_modtest
MODBIN040 := $(BUILD)/ide_modtest_040
DIAGBIN   := $(BUILD)/ide_test_diag
DIAGBIN040 := $(BUILD)/ide_test_040_diag
BUDBIN    := $(BUILD)/ide_budtest
BUDBIN040 := $(BUILD)/ide_budtest_040
ALLBIN    := $(BIN) $(BIN040) $(DIAGBIN) $(DIAGBIN040) $(MODBIN) $(MODBIN040) $(BUDBIN) $(BUDBIN040)

.PHONY: all check clean
all: $(ALLBIN)

$(BUILD)/ide.o: $(DRV) $(STUBS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -DHOST_TEST -Istubs -I$(SRC) -include mock_regs.h -c $(DRV) -o $@

$(BUILD)/ide_040.o: $(DRV) $(STUBS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(CACHE) -DHOST_TEST -Istubs -I$(SRC) -include mock_regs.h -c $(DRV) -o $@

$(BUILD)/ide_diag.o: $(DRV) $(STUBS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(DIAG) -DHOST_TEST -Istubs -I$(SRC) -include mock_regs.h -c $(DRV) -o $@

$(BUILD)/ide_040_diag.o: $(DRV) $(STUBS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(CACHE) $(DIAG) -DHOST_TEST -Istubs -I$(SRC) -include mock_regs.h -c $(DRV) -o $@

$(BUILD)/ide_mod.o: $(DRV) $(STUBS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -DHOST_TEST -DIDE_MODULE -Istubs -I$(SRC) -include mock_regs.h -c $(DRV) -o $@

$(BUILD)/ide_mod_040.o: $(DRV) $(STUBS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(CACHE) -DHOST_TEST -DIDE_MODULE -Istubs -I$(SRC) -include mock_regs.h -c $(DRV) -o $@

$(BUILD)/ide_bud.o: $(DRV) $(STUBS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -DHOST_TEST -DIDE_MODULE -DIDE_BUDDHA=1 -Istubs -I$(SRC) -include mock_regs.h -c $(DRV) -o $@

$(BUILD)/ide_bud_040.o: $(DRV) $(STUBS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(CACHE) -DHOST_TEST -DIDE_MODULE -DIDE_BUDDHA=1 -Istubs -I$(SRC) -include mock_regs.h -c $(DRV) -o $@

$(BUILD)/ide_budtest_040.o: ide_budtest.c mock_gayle.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(CACHE) -c $< -o $@

$(BUILD)/ide_modtest_040.o: ide_modtest.c mock_gayle.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(CACHE) -c $< -o $@

$(BUILD)/ide_test_040.o: ide_test.c mock_gayle.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(CACHE) -c $< -o $@

$(BUILD)/ide_test_diag.o: ide_test.c mock_gayle.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(DIAG) -c $< -o $@

$(BUILD)/ide_test_040_diag.o: ide_test.c mock_gayle.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(CACHE) $(DIAG) -c $< -o $@

$(BUILD)/%.o: %.c mock_gayle.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BIN): $(BUILD)/ide.o $(BUILD)/ide_test.o $(HOST_OBJS)
	$(CC) $(CFLAGS) $^ -o $@

$(BIN040): $(BUILD)/ide_040.o $(BUILD)/ide_test_040.o $(HOST_OBJS)
	$(CC) $(CFLAGS) $^ -o $@

$(DIAGBIN): $(BUILD)/ide_diag.o $(BUILD)/ide_test_diag.o $(HOST_OBJS)
	$(CC) $(CFLAGS) $^ -o $@

$(DIAGBIN040): $(BUILD)/ide_040_diag.o $(BUILD)/ide_test_040_diag.o $(HOST_OBJS)
	$(CC) $(CFLAGS) $^ -o $@

$(MODBIN): $(BUILD)/ide_mod.o $(BUILD)/ide_modtest.o $(HOST_OBJS)
	$(CC) $(CFLAGS) $^ -o $@

$(MODBIN040): $(BUILD)/ide_mod_040.o $(BUILD)/ide_modtest_040.o $(HOST_OBJS)
	$(CC) $(CFLAGS) $^ -o $@

$(BUDBIN): $(BUILD)/ide_bud.o $(BUILD)/ide_budtest.o $(HOST_OBJS)
	$(CC) $(CFLAGS) $^ -o $@

$(BUDBIN040): $(BUILD)/ide_bud_040.o $(BUILD)/ide_budtest_040.o $(HOST_OBJS)
	$(CC) $(CFLAGS) $^ -o $@

check: $(ALLBIN)
	./$(BIN)
	./$(BIN040)
	./$(DIAGBIN)
	./$(DIAGBIN040)
	./$(MODBIN)
	./$(MODBIN040)
	./$(BUDBIN)
	./$(BUDBIN040)

clean:
	rm -rf $(BUILD)
