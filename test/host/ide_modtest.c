/*
 * ide_modtest.c -- host test of the root-class module form of ../../src/ide.c
 * (-DIDE_MODULE; docs/MODULE-PORT.md, amix-dlm contracts C4-1, C4-2, C3-6).
 *
 * The same driver source as ide_test.c, built with the module switch and
 * HOST_TEST, against the same mock Gayle/ATA model.  The kernel calls the
 * module form adds are mocked here: bprobe(), sd_register(), root_defer(),
 * intr_attach()/intr_detach(), sleep()/wakeup().  Checks:
 *   - the probe order: one VPOSR read, then bprobe() of ALTSTAT 0xDD303A,
 *     then the task-file readback; no command before the attach's IDENTIFY;
 *   - ENXIO, silent and unregistered, on OCS/ECS (after the single VPOSR
 *     read), on a bprobe() bus error (no task-file access), with no drive,
 *     and on an A1200-shaped machine;
 *   - the registration: sd_register(idequeue, 0, 0xDD2020, "A4000 IDE", 4),
 *     polled requests through it;
 *   - intr: default 1 (root_defer); intr=0 polled, no root_defer; the deferred call
 *     attaches at level 2 and self-tests with one READ VERIFY: pass ->
 *     interrupt I/O, a mute drive or a refused attach -> polled, nIEN set,
 *     the handler detached;
 *   - parameters: defaults, overrides, out-of-range values;
 *   - sd_register and root_defer errors;
 *   - _halt: nIEN set, no command; _unload: nIEN set, the probe forgotten;
 *   - an ATAPI CD-ROM on the slave (slave=2, unit 5), polled and interrupt
 *     driven: every request for unit 5 is a PACKET command with the caller's
 *     LBA, never an ATA read of the master; a dd.c-shaped raw read (512-byte
 *     addressing) of the 2048-byte drive gets exactly the bytes the drive sends;
 *   - a page-crossing request is counted silently (said with debug=1);
 *   - no access to the Gayle ID register 0xDE1000 on any of these paths.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mock_gayle.h"

/* the kernel ABI as the driver sees it (mirrors stubs/sd.h, host types) */
struct sdcom {
	volatile struct sdcom	*next;
	char		reading, okay;
	unsigned char	status, cdb[12];
	char		*addr;
	unsigned	nbyte, card, unit;
	void		(*intr)();
};

extern int	gayle_load(), gayle_unload();
extern void	gayle_halt();
extern char	idequeue();
extern void	idereset();
extern unsigned long	ide_spins, ide_spins_reset, ide_spins_short;
extern unsigned long	ide_intr_count, ide_intr_spurious, ide_timeout_secs;
extern unsigned long	ide_halted, ide_selftest, ide_selftest_ticks;
extern unsigned long	ide_atapi_cmds, ide_atapi_overrun, ide_pagecross_n;
extern int	ide_slave_enable, ide_debug, ide_master_unit, ide_slave_unit, ide_intr_enable;
/* investigation-only symbols, defined only in an IDE_DIAG build (weak: the
 * shipping module must not have them) */
extern unsigned long	ide_spins_hw __attribute__((weak));
#if IDE_CACHE
extern long		ide_cache __attribute__((weak));
extern unsigned long	ide_push_n __attribute__((weak)), ide_push_bytes __attribute__((weak));
#endif

extern int	mock_ipl;
extern long	mock_spl_calls, mock_splx_calls;
extern char	mock_console[];
extern void	mock_spl_reset(), mock_console_reset(), mock_fire_timeouts(), mock_timeouts_reset();
extern int	mock_timeouts_pending();

#define ENXIO_	6
#define EBUSY_	16
#define ENOSPC_	28
#define EEXIST_	17

static int	failures, checks;
static unsigned long	id_total;	/* 0xDE1000 accesses over every case */

#define CHECK( cond, msg) do { checks++; if (!(cond)) { failures++; \
	fprintf( stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, msg); } } while (0)

/* ---- the kernel calls of the module form, mocked ----------------------- */

static int		bp_calls, bp_ret, bp_v;
static unsigned long	bp_addr, bp_accesses_before;

int
bprobe( addr, v)
char	*addr;
int	v;
{
	bp_calls++;
	bp_addr = (unsigned long)addr;
	bp_v = v;
	bp_accesses_before = mock_accesses;	/* register accesses made before the guard */
	return bp_ret;
}

static int		sr_calls, sr_ret;
static char		(*sr_fn)();
static unsigned		sr_c, sr_rank;
static char		*sr_base, *sr_name;

int
sd_register( fn, c, base, name, rank)
char	(*fn)();
unsigned c, rank;
char	*base, *name;
{
	sr_calls++;
	sr_fn = fn; sr_c = c; sr_base = base; sr_name = name; sr_rank = rank;
	return sr_ret;
}

static int		rd_calls, rd_ret;
static int		(*rd_fn)();
static char		*rd_arg;

int
root_defer( fn, arg)
int	(*fn)();
char	*arg;
{
	rd_calls++;
	rd_fn = fn;
	rd_arg = arg;
	return rd_ret;
}

static int		ia_calls, ia_ret, ia_level, id_calls, id_level;
static int		(*ia_fn)(), (*id_fn)();
static char		*ia_arg, *ia_name;
static int		(*handler)();	/* attached at level 2 */
static int		claims, unclaimed;

int
intr_attach( level, fn, arg, name)
int	level;
int	(*fn)();
char	*arg, *name;
{
	ia_calls++;
	ia_level = level; ia_fn = fn; ia_arg = arg; ia_name = name;
	if (ia_ret == 0)
		handler = fn;
	return ia_ret;
}

int
intr_detach( level, fn, arg)
int	level;
int	(*fn)();
char	*arg;
{
	id_calls++;
	id_level = level; id_fn = fn;
	if (fn == handler)
		handler = 0;
	return 0;
}

/* p2int stand-in: run the attached chain while the board asserts INT2 */
static void
deliver()
{
	int	old, guard = 1000;

	while (handler && mock_int2_asserted() && guard-- > 0) {
		old = mock_ipl;
		mock_ipl = 2;
		if ((*handler)( (char *)0))
			claims++;
		else
			unclaimed++;
		mock_ipl = old;
	}
}

static int		sl_calls, sl_badipl, wk_calls;

int
wakeup( chan)
char	*chan;
{
	wk_calls++;
	return 0;
}

/* One clock tick passes while the caller sleeps: the callouts fire, and a
 * pending INT2 is taken (the idle loop runs at level 0). */
int
sleep( chan, pri)
char	*chan;
int	pri;
{
	sl_calls++;
	if (mock_ipl != 2)
		sl_badipl++;
	mock_fire_timeouts();
	deliver();
	return 0;
}

/* ---- helpers ------------------------------------------------------------ */

static int	intr_calls;
static struct sdcom *intr_last;

static void
done( cp)
struct sdcom	*cp;
{
	intr_calls++;
	intr_last = cp;
}

static void
fresh( ndrives)
int	ndrives;
{
	id_total += mock_id_reads + mock_id_writes;
	mock_free();
	mock_reset();
	mock_spl_reset();
	mock_console_reset();
	mock_timeouts_reset();
	idereset();
	mock_board = MOCK_BOARD_A4000;
	ide_spins = 200000;
	ide_spins_reset = 200000;
	ide_spins_short = 20000;
	ide_intr_count = ide_intr_spurious = 0;
	ide_halted = ide_selftest = ide_selftest_ticks = 0;
	bp_calls = bp_ret = 0; bp_addr = 0; bp_v = 0; bp_accesses_before = 0;
	sr_calls = sr_ret = 0; sr_fn = 0; sr_base = sr_name = 0; sr_c = sr_rank = 99;
	rd_calls = rd_ret = 0; rd_fn = 0; rd_arg = (char *)1;
	ia_calls = ia_ret = id_calls = 0; ia_fn = id_fn = 0; handler = 0;
	claims = unclaimed = 0;
	sl_calls = sl_badipl = wk_calls = 0;
	intr_calls = 0; intr_last = 0;
	if (ndrives >= 1)
		mock_add_drive( 0, 4096L, "MOCK CF CARD 2GB", 1);
	if (ndrives >= 2)
		mock_add_drive( 1, 2048L, "MOCK SLAVE", 0);
}

static void
quiet_enxio( msg)
char	*msg;
{
	CHECK( sr_calls == 0, msg);
	CHECK( rd_calls == 0, "ENXIO: no root_defer");
	CHECK( mock_console[0] == '\0', "ENXIO is silent");
	CHECK( mock_cmds() == 0, "ENXIO: no ATA command issued");
	CHECK( mock_id_reads + mock_id_writes == 0, "ENXIO: 0xDE1000 untouched");
}

/* READ(10) of one sector at lba through the registered queue function */
static void
read1( cp, unit, lba, buf)
struct sdcom	*cp;
int		unit;
unsigned long	lba;
char		*buf;
{
	memset( cp, 0, sizeof *cp);
	cp->cdb[0] = 0x28;
	cp->cdb[2] = lba >> 24; cp->cdb[3] = lba >> 16; cp->cdb[4] = lba >> 8; cp->cdb[5] = lba;
	cp->cdb[8] = 1;
	cp->addr  = buf;
	cp->nbyte = 512;
	cp->unit  = unit;
	cp->intr  = done;
	cp->reading = 1;
	intr_calls = 0;
	(*sr_fn)( 0, cp);
	CHECK( mock_spl_calls == mock_splx_calls, "spl bracket balanced after the queue call");
}

/* kernel page size of the line under test (the 040/060 build passes IDE_NBPP) */
#ifdef IDE_NBPP
#define PG	IDE_NBPP
#else
#define PG	2048
#endif

/* The ATA path stores 16-bit words as integers: on a little-endian host the
 * memory image is the disk image with byte pairs swapped (ide_test.c's
 * disk_equals()).  The ATAPI path stores bytes and needs no swap. */
static int
disk_equals( unit, lba, buf, nbyte)
int		unit;
unsigned long	lba;
const char	*buf;
unsigned	nbyte;
{
	const unsigned char	*d = mock_drv[unit].disk + lba * MOCK_SECTOR;
	unsigned short		one = 1;
	unsigned		i;

	if (*(unsigned char *)&one != 1)
		return memcmp( d, buf, nbyte) == 0;
	for (i = 0; i + 1 < nbyte; i += 2)
		if ((unsigned char)buf[i] != d[i + 1] || (unsigned char)buf[i + 1] != d[i])
			return 0;
	return 1;
}

/* READ(10) of `count' blocks at lba into buf (nbyte bytes) for `unit'; the
 * interrupt path is run to the end (deliver() does nothing when polled) */
static void
rd10( cp, unit, lba, count, buf, nbyte)
struct sdcom	*cp;
int		unit;
unsigned long	lba;
unsigned	count, nbyte;
char		*buf;
{
	memset( cp, 0, sizeof *cp);
	cp->cdb[0] = 0x28;
	cp->cdb[2] = lba >> 24; cp->cdb[3] = lba >> 16; cp->cdb[4] = lba >> 8; cp->cdb[5] = lba;
	cp->cdb[7] = count >> 8; cp->cdb[8] = count;
	cp->addr  = buf;
	cp->nbyte = nbyte;
	cp->unit  = unit;
	cp->intr  = done;
	cp->reading = 1;
	intr_calls = 0;
	(*sr_fn)( 0, cp);
	deliver();
	CHECK( mock_spl_calls == mock_splx_calls, "spl bracket balanced after the queue call");
}

/* load with one drive and intr=1, run the deferred call */
static int
load_intr( params)
char	*params;
{
	int	e;

	e = gayle_load( params);
	CHECK( e == 0, "intr=1: loads");
	CHECK( rd_calls == 1 && rd_fn != 0 && rd_arg == 0, "intr=1: root_defer(fn, 0) called once");
	if (rd_fn == 0)
		return -1;
	mock_console_reset();
	mock_spl_reset();
	return (*rd_fn)( rd_arg);
}

/* ---- the cases ---------------------------------------------------------- */

static void
test_chipset_gate()
{
	static unsigned short	ids[] = { 0x0000, 0x1000, 0x2000, 0x3000 };	/* OCS, ECS PAL/NTSC */
	int	i, e;

	for (i = 0; i < 4; ++i) {
		fresh( 1);
		mock_vposr_override = 1;
		mock_vposr_value = ids[i];
		e = gayle_load( (char *)0);
		CHECK( e == ENXIO_, "OCS/ECS: ENXIO");
		CHECK( mock_accesses == 1, "OCS/ECS: the single VPOSR read, nothing else");
		CHECK( bp_calls == 0, "OCS/ECS: no bprobe");
		quiet_enxio( "OCS/ECS: not registered");
	}
	fresh( 1);
	mock_board = MOCK_BOARD_NONE;		/* an A3000: ECS, no Gayle */
	e = gayle_load( (char *)0);
	CHECK( e == ENXIO_ && mock_accesses == 1 && mock_bad_accesses == 0, "A3000: ENXIO after VPOSR only");
	quiet_enxio( "A3000: not registered");
}

static void
test_bprobe_fault()
{
	int	e;

	fresh( 1);
	bp_ret = 1;
	e = gayle_load( (char *)0);
	CHECK( e == ENXIO_, "bprobe bus error: ENXIO");
	CHECK( bp_calls == 1, "bprobe called once");
	CHECK( mock_accesses == 1, "bprobe bus error: no task-file access");
	quiet_enxio( "bprobe bus error: not registered");
}

static void
test_probe_order()
{
	int	e;

	fresh( 1);
	e = gayle_load( (char *)0);
	CHECK( e == 0, "A4000 with a drive: loads");
	CHECK( bp_calls == 1, "bprobe called once");
	CHECK( bp_addr == 0xDD303AL, "bprobe reads ALTSTAT at 0xDD303A");
	CHECK( bp_v < 0, "bprobe is a read");
	CHECK( bp_accesses_before == 1, "VPOSR first, then bprobe, before any task-file access");
	CHECK( mock_cmd_log_n >= 1 && mock_cmd_log[0] == 0xEC, "the first command is the attach's IDENTIFY");
	CHECK( mock_cmd_count[0xEC] == 1, "one IDENTIFY (master only)");
	CHECK( mock_slave_selects == 0, "slave never selected by default");
	CHECK( mock_nien == 1, "polled: nIEN set");
	CHECK( mock_bad_accesses == 0, "no access outside the board");
	CHECK( mock_id_reads + mock_id_writes == 0, "0xDE1000 untouched");
}

static void
test_no_drive()
{
	int	e;

	fresh( 0);				/* AGA, the port empty: an A4000T without a drive */
	e = gayle_load( (char *)0);
	CHECK( e == ENXIO_, "no drive: ENXIO");
	CHECK( bp_calls == 1, "no drive: the guard ran");
	quiet_enxio( "no drive: not registered");
	CHECK( mock_accesses < 40, "no drive: settled in a few accesses, no spin");
}

static void
test_a1200_shape()
{
	int	e;

	fresh( 1);
	mock_board = MOCK_BOARD_A1200;		/* AGA, a Gayle at 0xDA0000 */
	e = gayle_load( (char *)0);
	CHECK( e == ENXIO_, "A1200: ENXIO (the module knows the A4000 row only)");
	quiet_enxio( "A1200: not registered");
}

static void
test_register_polled()
{
	struct sdcom	cp;
	static char	buf[512];
	int		e;

	fresh( 1);
	e = gayle_load( (char *)0);
	CHECK( e == 0, "loads");
	CHECK( sr_calls == 1, "sd_register called once");
	CHECK( sr_fn == idequeue, "registers idequeue");
	CHECK( sr_c == 0, "card argument 0");
	CHECK( (unsigned long)sr_base == 0xDD2020L, "base 0xDD2020");
	CHECK( sr_name && strcmp( sr_name, "A4000 IDE") == 0, "name A4000 IDE");
	CHECK( sr_rank == 4, "rank 4 (ide)");
	CHECK( rd_calls == 1 && rd_fn != 0, "intr default 1: root_defer called");
	CHECK( strstr( mock_console, "ide0: MOCK CF CARD 2GB") != 0, "attach line printed");
	CHECK( strstr( mock_console, "ide: A4000 IDE at 0xdd2020, polled I/O") != 0, "polled banner");
	CHECK( mock_spl_calls == 0, "the load entry takes no spl (the root pass stays at level 7)");
	CHECK( mock_timeouts_pending() == 0, "the load entry sets no timeout");
	CHECK( sl_calls == 0, "the load entry never sleeps");

	read1( &cp, 6, 3L, buf);
	CHECK( intr_calls == 1 && cp.okay && cp.status == 0, "polled READ(10) completes in the call");
	CHECK( mock_cmd_count[0x20] == 1, "one READ SECTOR(S)");
	read1( &cp, 5, 3L, buf);
	CHECK( intr_calls == 1 && !cp.okay, "unit 5 (slave off): no device");
	CHECK( mock_id_reads + mock_id_writes == 0, "0xDE1000 untouched");
}

static void
test_params()
{
	int	e;

	fresh( 2);
	e = gayle_load( "debug=0 slave=1 master=4 slaveunit=3");
	CHECK( e == 0, "params: loads");
	CHECK( ide_slave_enable == 1 && ide_master_unit == 4 && ide_slave_unit == 3 && ide_intr_enable == 1,
	    "params taken, intr by default");
	CHECK( mock_cmd_count[0xEC] == 2 && mock_slave_selects > 0, "slave=1: both drives identified");

	fresh( 1);
	e = gayle_load( "intr=7 slave=x master=9");
	CHECK( e == 0, "bad params: loads with the defaults");
	CHECK( ide_intr_enable == 1 && ide_slave_enable == 0 && ide_master_unit == 6, "bad params: defaults");
	CHECK( strstr( mock_console, "ide: parameter intr is not 0 to 1; 1 taken") != 0, "bad intr said");
	CHECK( strstr( mock_console, "ide: parameter master is not 0 to 7; 6 taken") != 0, "bad master said");
	CHECK( rd_calls == 1, "bad intr: the default, root_defer");

	fresh( 1);
	e = gayle_load( "intr=0");
	CHECK( e == 0 && ide_intr_enable == 0, "intr=0: loads polled");
	CHECK( rd_calls == 0, "intr=0: no root_defer");
	CHECK( mock_nien == 1, "intr=0: nIEN set");
	CHECK( strstr( mock_console, "ide: A4000 IDE at 0xdd2020, polled I/O") != 0, "intr=0: polled banner");

	fresh( 1);
	e = gayle_load( "");
	CHECK( e == 0 && ide_intr_enable == 1 && ide_master_unit == 6 && ide_slave_unit == 5, "empty string: defaults");
	CHECK( rd_calls == 1, "empty string: root_defer (interrupts by default)");

	fresh( 1);
	e = load_intr( (char *)0);
	CHECK( e == 0 && ide_selftest == 1 && mock_nien == 0, "no params: the self-test passes, interrupt I/O");
	CHECK( strstr( mock_console, "ide: A4000 IDE, interrupt I/O") != 0, "no params: interrupt banner");
	CHECK( mock_id_reads + mock_id_writes == 0, "0xDE1000 untouched");
}

static void
test_intr_selftest_pass()
{
	struct sdcom	cp;
	static char	buf[512];
	int		e;

	fresh( 1);
	e = load_intr( "intr=1");
	CHECK( e == 0, "deferred call returns 0");
	CHECK( ia_calls == 1 && ia_level == 2 && ia_arg == 0, "intr_attach at level 2");
	CHECK( ia_name && strcmp( ia_name, "gayle") == 0, "attached as gayle");
	CHECK( mock_cmd_count[0x40] == 1, "self-test: one READ VERIFY");
	CHECK( claims == 1 && unclaimed == 0, "the handler claimed the self-test's interrupt");
	CHECK( sl_calls >= 1 && sl_badipl == 0, "waited in sleep rounds at sdspl");
	CHECK( ide_selftest == 1, "self-test passed");
	CHECK( ide_intr_spurious == 0, "the self-test's interrupt is not spurious");
	CHECK( id_calls == 0, "not detached");
	CHECK( mock_nien == 0, "interrupt I/O: nIEN clear");
	CHECK( strstr( mock_console, "ide: A4000 IDE, interrupt I/O") != 0, "interrupt banner");
	CHECK( mock_spl_calls == mock_splx_calls && mock_ipl == 0, "spl balanced after the deferred call");

	read1( &cp, 6, 7L, buf);
	CHECK( intr_calls == 0, "interrupt I/O: the READ completes later, not in the call");
	deliver();
	CHECK( intr_calls == 1 && cp.okay && cp.status == 0, "the READ completes from the interrupt");

	gayle_halt();
	CHECK( ide_halted == 1, "halt recorded");
	CHECK( mock_nien == 1, "halt: nIEN set");
	CHECK( mock_id_reads + mock_id_writes == 0, "0xDE1000 untouched");
}

static void
test_intr_selftest_fail()
{
	struct sdcom	cp;
	static char	buf[512];
	unsigned long	cmds;
	int		e;

	fresh( 1);
	mock_drv[0].mute_intrq = 1;		/* the interrupt never arrives */
	ide_timeout_secs = 2;
	e = load_intr( "intr=1");
	ide_timeout_secs = 20;
	CHECK( e == 0, "failed self-test: still 0 (the module stays linked)");
	CHECK( ide_selftest == 2, "self-test failed");
	CHECK( ide_selftest_ticks == 2 * 60, "bounded: ide_timeout_secs * HZ ticks");
	CHECK( id_calls == 1 && id_level == 2 && id_fn == ia_fn, "handler detached");
	CHECK( mock_nien == 1, "polled: nIEN set again");
	CHECK( strstr( mock_console, "ide: interrupt self-test failed, polled I/O") != 0, "failure said");
	CHECK( mock_spl_calls == mock_splx_calls && mock_ipl == 0, "spl balanced");
	cmds = mock_cmds();
	read1( &cp, 6, 7L, buf);
	CHECK( intr_calls == 1 && cp.okay && cp.status == 0, "polled READ completes in the call");
	CHECK( mock_cmds() == cmds + 1, "one command for the READ");

	fresh( 1);
	ia_ret = EEXIST_;
	e = load_intr( "intr=1");
	CHECK( e == 0 && ide_selftest == 2, "refused attach: polled");
	CHECK( mock_cmd_count[0x40] == 0, "refused attach: no self-test command");
	CHECK( mock_nien == 1, "refused attach: nIEN set");
	CHECK( strstr( mock_console, "ide: intr_attach failed (errno 17), polled I/O") != 0, "refusal said");

	fresh( 0);
	mock_add_cdrom( 0, 1000L, "MOCK CD-ROM");	/* an ATAPI master: no ATA drive */
	e = load_intr( "intr=1");
	CHECK( e == 0 && ide_selftest == 2, "ATAPI only: no self-test, polled");
	CHECK( mock_cmd_count[0x40] == 0, "ATAPI only: no READ VERIFY");
	CHECK( id_calls == 1, "ATAPI only: handler detached");
	CHECK( mock_id_reads + mock_id_writes == 0, "0xDE1000 untouched");
}

static void
test_errors()
{
	struct sdcom	cp;
	static char	buf[512];
	int		e;

	fresh( 1);
	sr_ret = EBUSY_;
	e = gayle_load( "intr=1");
	CHECK( e == EBUSY_, "sd_register's errno passed back");
	CHECK( strstr( mock_console, "ide: sd_register failed (errno 16)") != 0, "said");
	CHECK( rd_calls == 0, "no root_defer after a failed registration");

	fresh( 1);
	rd_ret = ENOSPC_;
	e = gayle_load( "intr=1");
	CHECK( e == 0, "root_defer refused: still loads, polled");
	CHECK( strstr( mock_console, "ide: root_defer failed (errno 28), polled I/O") != 0, "said");

	fresh( 1);
	e = gayle_load( (char *)0);
	CHECK( e == 0, "loads");
	e = gayle_unload();
	CHECK( e == 0, "_unload returns 0");
	CHECK( mock_nien == 1, "_unload: nIEN set");
	read1( &cp, 6, 1L, buf);
	CHECK( intr_calls == 1 && !cp.okay, "_unload: the probe forgotten, no device");

	fresh( 0);
	gayle_halt();
	CHECK( ide_halted == 1 && mock_accesses == 0, "halt with nothing attached touches no register");
	CHECK( mock_id_reads + mock_id_writes == 0, "0xDE1000 untouched");
}

/*
 * An ATAPI CD-ROM on the slave next to an ATA master, `slave=2' (the bench
 * row: `ide1: UAE-ATAPI, ATAPI CD-ROM', SCSI unit 5).  Sixteen raw reads
 * shaped exactly as dd.c's startio() builds them for `dd bs=2048 count=16'
 * on /dev/rdsk/c5d0s0: READ(10) with the LBA in 512-byte units (offset / 512)
 * and the count bcount / 512, nbyte 2048.  Each must be one PACKET command
 * carrying that CDB unchanged, never an ATA command; the 2048-byte drive
 * answers count * 2048 bytes from its block `lba', the first nbyte are
 * stored and the rest drained.  So the 32 KiB that dd returns are the first
 * 2 KiB of CD blocks 0, 4, ..., 60: the CD's own bytes, addressed in units
 * the drive does not use.  Then a GSIO-shaped read (2048-byte LBA, one
 * block) returns the block itself with nothing drained.
 */
static void
atapi_slave_reads( params, what)
char	*params, *what;
{
	static char	buf[16 * 2048 + 64];
	struct sdcom	cp;
	unsigned char	*cd, *k;
	unsigned long	n, lba, cmds0, ov0, ata0, pkts0;
	int		e, okdone = 1, okpkt = 1, okcnt = 1, okov = 1, okdata = 1, okata = 1;
	char		msg[160];

	fresh( 1);
	mock_add_cdrom( 1, 64L, "UAE-ATAPI");
	cd = mock_drv[1].cd;
	if (strstr( params, "intr=1")) {
		e = load_intr( params);
		CHECK( e == 0 && ide_selftest == 1, "slave=2 intr=1: the self-test passes on the ATA master");
	} else {
		e = gayle_load( params);
		CHECK( e == 0, "slave=2: loads");
	}
	CHECK( sr_calls == 1, "slave=2: registered once");
	memset( buf, 0x55, sizeof buf);
	ata0  = mock_cmd_count[0x20] + mock_cmd_count[0x30] + mock_cmd_count[0x40];
	pkts0 = mock_cmd_count[0xA0];
	for (n = 0; n < 16; ++n) {
		cmds0 = ide_atapi_cmds;
		ov0   = ide_atapi_overrun;
		rd10( &cp, 5, n * 4, 4, buf + n * 2048, 2048);
		k = mock_pkt_last;
		lba = ((unsigned long)k[2] << 24) | ((unsigned long)k[3] << 16) | (k[4] << 8) | k[5];
		if (intr_calls != 1 || !cp.okay || cp.status != 0)
			okdone = 0;
		if (k[0] != 0x28 || lba != n * 4 || ((k[7] << 8) | k[8]) != 4 || mock_pkt_bc != 2048)
			okpkt = 0;
		if (ide_atapi_cmds != cmds0 + 1)
			okcnt = 0;
		if (ide_atapi_overrun - ov0 != 3 * 2048)
			okov = 0;
		if (memcmp( buf + n * 2048, cd + n * 4 * 2048, 2048) != 0)
			okdata = 0;
	}
	if (mock_cmd_count[0x20] + mock_cmd_count[0x30] + mock_cmd_count[0x40] != ata0)
		okata = 0;
	sprintf( msg, "%s: each dd.c-shaped read of unit 5 completes GOOD", what);
	CHECK( okdone, msg);
	sprintf( msg, "%s: READ(10) packets with the caller's LBA 0, 4, ... 60 and count 4", what);
	CHECK( okpkt, msg);
	sprintf( msg, "%s: ide_atapi_cmds +1 per request", what);
	CHECK( okcnt, msg);
	sprintf( msg, "%s: 16 PACKET commands on the cable", what);
	CHECK( mock_cmd_count[0xA0] == pkts0 + 16, msg);
	sprintf( msg, "%s: no ATA read, write or verify for unit 5 (the master untouched)", what);
	CHECK( okata, msg);
	sprintf( msg, "%s: 6144 bytes drained per request (the drive sends 4 x 2048)", what);
	CHECK( okov, msg);
	sprintf( msg, "%s: the 32 KiB are the first 2 KiB of CD blocks 0, 4, ... 60", what);
	CHECK( okdata, msg);
	sprintf( msg, "%s: nothing stored past nbyte", what);
	CHECK( (unsigned char)buf[16 * 2048] == 0x55, msg);

	ov0 = ide_atapi_overrun;
	cmds0 = ide_atapi_cmds;
	rd10( &cp, 5, 16L, 1, buf, 2048);
	sprintf( msg, "%s: a GSIO-shaped READ(10) of block 16 is block 16, nothing drained", what);
	CHECK( intr_calls == 1 && cp.okay && cp.status == 0 && ide_atapi_cmds == cmds0 + 1
	       && ide_atapi_overrun == ov0 && memcmp( buf, cd + 16 * 2048, 2048) == 0, msg);

	read1( &cp, 6, 3L, buf);
	deliver();
	sprintf( msg, "%s: the master still reads as an ATA disk", what);
	CHECK( intr_calls == 1 && cp.okay && cp.status == 0
	       && disk_equals( 0, 3L, buf, 512), msg);
	CHECK( mock_id_reads + mock_id_writes == 0, "0xDE1000 untouched");
}

static void
test_atapi_slave()
{
	atapi_slave_reads( "slave=2", "ATAPI slave, polled");
	atapi_slave_reads( "slave=2 intr=1", "ATAPI slave, interrupt I/O");
}

/*
 * A request whose buffer straddles a page boundary: the boot's sdpart.c RDB
 * read into a kernel static does that on some kernel links.  Counted
 * (ide_pagecross_n), served, and said on the console only with debug.
 */
static void
test_pagecross_quiet()
{
	static char	big[3 * PG];
	struct sdcom	cp;
	char		*p;
	unsigned long	before;

	fresh( 1);
	CHECK( gayle_load( (char *)0) == 0, "loads");
	mock_console_reset();
	p = big + (PG - ((unsigned long)big & (PG - 1))) % PG;	/* first page-aligned byte */
	ide_pagecross_n = before = 0;		/* the first crossing since boot */
	read1( &cp, 6, 12L, p + PG - 256);
	CHECK( intr_calls == 1 && cp.okay && cp.status == 0
	       && disk_equals( 0, 12L, p + PG - 256, 512), "page-crossing read served");
	CHECK( ide_pagecross_n == before + 1, "page-crossing read counted");
	CHECK( strstr( mock_console, "crosses a page boundary") == 0, "and not said on the console");

	fresh( 1);
	CHECK( gayle_load( "debug=1") == 0, "debug=1: loads");
	mock_console_reset();
	read1( &cp, 6, 12L, p + PG - 256);
	CHECK( ide_pagecross_n == before + 2, "debug=1: counted");
	CHECK( strstr( mock_console, "ide: raw request of 512 bytes crosses a page boundary") != 0,
	       "debug=1: said");
}

int
main()
{
#if IDE_CACHE
	const char	*name = "ide_modtest (040/060: IDE_CACHE, IDE_NBPP 4096)";
#else
	const char	*name = "ide_modtest";
#endif
	fprintf( stderr, "== test_chipset_gate\n"); test_chipset_gate();
	fprintf( stderr, "== test_bprobe_fault\n"); test_bprobe_fault();
	fprintf( stderr, "== test_probe_order\n"); test_probe_order();
	fprintf( stderr, "== test_no_drive\n"); test_no_drive();
	fprintf( stderr, "== test_a1200_shape\n"); test_a1200_shape();
	fprintf( stderr, "== test_register_polled\n"); test_register_polled();
	fprintf( stderr, "== test_params\n"); test_params();
	fprintf( stderr, "== test_intr_selftest_pass\n"); test_intr_selftest_pass();
	fprintf( stderr, "== test_intr_selftest_fail\n"); test_intr_selftest_fail();
	fprintf( stderr, "== test_errors\n"); test_errors();
	fprintf( stderr, "== test_atapi_slave\n"); test_atapi_slave();
	fprintf( stderr, "== test_pagecross_quiet\n"); test_pagecross_quiet();
	fresh( 0);
	CHECK( id_total == 0, "no access to 0xDE1000 on any module path");
	CHECK( &ide_spins_hw == 0, "no ide_spins_hw in the module");
#if IDE_CACHE
	CHECK( &ide_cache == 0 && &ide_push_n == 0 && &ide_push_bytes == 0,
	       "no ide_cache switch or push counters in the module");
#endif
	mock_free();
	fprintf( stderr, "%s: %d checks, %d failures\n", name, checks, failures);
	return failures != 0;
}
