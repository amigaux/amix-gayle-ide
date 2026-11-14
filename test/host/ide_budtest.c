/*
 * ide_budtest.c -- host test of the Buddha / Catweasel Z-II flavour of
 * ../../src/ide.c (-DIDE_MODULE -DIDE_BUDDHA, the module `buddha'; amix-dlm
 * contracts C4-2 version 10 section 8.2, C4-1 version 9).
 *
 * The same driver source, built with the Buddha switch and HOST_TEST, against
 * the mock's Buddha boards (mock_gayle.c, MOCK_BOARD_BUDDHA): one cable of
 * two drives per port, ports at board + 0x800 + 0x200 * p, IRQ status at
 * + 0xF00 + 0x40 * p, the write-only enable at + 0xFC0.  Any other access
 * on a board is counted as forbidden.  The kernel calls are mocked here:
 * autocon(), bprobe(), sd_register(), root_defer(), intr_attach() /
 * intr_detach(), sleep() / wakeup().  Checks:
 *   - discovery: autocon() for the Buddha (0x12120000), then the Catweasel
 *     Z-II (0x1212002A); no board, no answering port, or no drive is ENXIO,
 *     silent, unregistered; no chipset gate, no Kickstart ROM read;
 *   - the register map per port: bprobe() of each port's alternate status
 *     (board + 0x91A, 0xB1A, 0xD1A), two ports probed on a Buddha and three
 *     on a Catweasel Z-II, a port that does not answer never touched again;
 *   - the channel table: units 6/5, 3/2, 1/0 reach port 0, 1, 2 master and
 *     slave, reads and writes land on that drive's disk; units 4 and 7 and
 *     the units of a port without a drive are "no such device";
 *   - one registration per board with the board's AutoConfig base and its
 *     index: sd_register(idequeue, b, base, "Buddha IDE" / "Catweasel IDE",
 *     4); with two boards each card reaches its own board, polled and
 *     interrupt driven;
 *   - an ATAPI CD-ROM on a port's slave: PACKET commands, polled and
 *     interrupt driven;
 *   - the interrupt: intr=1 defers; the self-test writes the enable once per
 *     board, tests every port with a drive (READ VERIFY to an ATA drive,
 *     IDENTIFY PACKET DEVICE to a port with only an ATAPI device), clears
 *     nIEN on every port when all pass; a port that does not interrupt
 *     leaves every drive with nIEN set, the handler detached, polled I/O;
 *     an INTRQ on a port with no command is dropped by reading its STATUS;
 *   - interrupts that stop after the self-test passed: the request and the
 *     queued ones still complete (polled), the driver goes polled on every
 *     port with one console line; a hang after an interrupt fails as before;
 *   - parameters (intr, slave, debug), sd_register and root_defer errors;
 *   - _halt and _unload: nIEN on every port, no command;
 *   - the layouts of a port (none, master, slave alone, both) on ports 0 and
 *     1 under three models of an absent device (open bus, Amiberry's, a bus
 *     holding the last byte); a slave alone attached, its master never sent
 *     a command, the self-test run on it (ATA and ATAPI); slave=0 leaving it;
 *     device 0 answering for an absent device 1 giving no ghost slave;
 *   - the forbidden ranges: nothing below 0x800, nothing in the clock port,
 *     the ROM or between the registers, no read of the enable, no access to
 *     a Buddha's (absent) third port -- in every case.
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

extern int	buddha_load(), buddha_unload();
extern void	buddha_halt();
extern char	idequeue();
extern void	idereset();
extern unsigned long	ide_spins, ide_spins_reset, ide_spins_short;
extern unsigned long	ide_intr_count, ide_intr_spurious, ide_timeout_secs;
extern unsigned long	ide_halted, ide_selftest, ide_selftest_ticks, ide_atapi_cmds;
extern int	ide_slave_enable, ide_debug, ide_intr_enable;
extern unsigned long	bud_intr_other, bud_extra_boards, bud_base[];
extern int	bud_nboards, bud_chok[];
extern unsigned long	bud_intr_lost, ide_intr_timeouts;
extern int	mock_timeouts_pending();

extern int	mock_ipl;
extern long	mock_spl_calls, mock_splx_calls;
extern char	mock_console[];
extern void	mock_spl_reset(), mock_console_reset(), mock_fire_timeouts(), mock_timeouts_reset();

#define ENXIO_	6
#define EBUSY_	16
#define ENOSPC_	28

#define BASE0	0xE90000L		/* the first Zorro II I/O slot */
#define BASE1	0xEA0000L

static int	failures, checks;
static unsigned long	forbidden_total;	/* forbidden board accesses over every case */

#define CHECK( cond, msg) do { checks++; if (!(cond)) { failures++; \
	fprintf( stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, msg); } } while (0)

/* ---- the kernel calls of the module form, mocked ----------------------- */

static int		ac_calls;
static long		ac_pc[16];

int
autocon( pc, n, bp, np)
long	pc;
int	n;
long	*bp, *np;
{
	int	b, k;

	if (ac_calls < 16)
		ac_pc[ac_calls] = pc;
	ac_calls++;
	for (b = 0, k = 0; b < MOCK_NBUD; ++b) {
		if (!mock_bud[b].present)
			continue;
		if ((pc == 0x12120000L && !mock_bud[b].cwz2) || (pc == 0x1212002AL && mock_bud[b].cwz2)) {
			if (k++ == n) {
				*bp = (long)mock_bud[b].base;
				*np = 0x10000L;
				return 1;
			}
		}
	}
	return 0;
}

static int		bp_calls, bp_fail_at;	/* bp_fail_at: the n-th call (1..) faults */
static unsigned long	bp_addrs[16];

int
bprobe( addr, v)
char	*addr;
int	v;
{
	if (bp_calls < 16)
		bp_addrs[bp_calls] = (unsigned long)addr;
	bp_calls++;
	return bp_calls == bp_fail_at;
}

static int		sr_calls, sr_ret, sr_fail_at;
static char		(*sr_fn)();
static unsigned		sr_c[4], sr_rank[4];
static char		*sr_base[4], *sr_name[4];

int
sd_register( fn, c, base, name, rank)
char	(*fn)();
unsigned c, rank;
char	*base, *name;
{
	int	i = sr_calls++;

	if (i < 4) {
		sr_c[i] = c; sr_base[i] = base; sr_name[i] = name; sr_rank[i] = rank;
	}
	sr_fn = fn;
	if (sr_fail_at && sr_calls == sr_fail_at)
		return sr_ret;
	return 0;
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

static int		ia_calls, ia_ret, ia_level, id_calls;
static char		*ia_name;
static int		(*handler)();
static int		claims, unclaimed;

int
intr_attach( level, fn, arg, name)
int	level;
int	(*fn)();
char	*arg, *name;
{
	ia_calls++;
	ia_level = level; ia_name = name;
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
	if (fn == handler)
		handler = 0;
	return 0;
}

/* p2int stand-in: run the attached handler while a board asserts INT2 */
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

int
wakeup( chan)
char	*chan;
{
	return 0;
}

/* one clock tick passes while the caller sleeps */
int
sleep( chan, pri)
char	*chan;
int	pri;
{
	mock_fire_timeouts();
	deliver();
	return 0;
}

/* ---- helpers ------------------------------------------------------------ */

static int	intr_calls;

static void
done( cp)
struct sdcom	*cp;
{
	intr_calls++;
}

/* a fresh box: one Buddha at BASE0 (more boards are set up by the case) */
static void
fresh()
{
	forbidden_total += mock_bud_forbidden + mock_bud_intena_reads;
	mock_free();
	mock_reset();
	mock_spl_reset();
	mock_console_reset();
	mock_timeouts_reset();
	idereset();
	mock_board = MOCK_BOARD_BUDDHA;
	mock_bud[0].present = 1;
	mock_bud[0].cwz2 = 0;
	mock_bud[0].base = BASE0;
	ide_spins = 200000;
	ide_spins_reset = 200000;
	ide_spins_short = 20000;
	ide_intr_count = ide_intr_spurious = 0;
	ide_halted = ide_selftest = ide_selftest_ticks = 0;
	bud_intr_other = 0;
	ac_calls = 0;
	bp_calls = bp_fail_at = 0;
	sr_calls = sr_ret = sr_fail_at = 0; sr_fn = 0;
	memset( sr_c, 0xFF, sizeof sr_c); memset( sr_base, 0, sizeof sr_base);
	memset( sr_name, 0, sizeof sr_name); memset( sr_rank, 0, sizeof sr_rank);
	rd_calls = rd_ret = 0; rd_fn = 0; rd_arg = (char *)1;
	ia_calls = ia_ret = id_calls = 0; ia_level = 0; ia_name = 0; handler = 0;
	claims = unclaimed = 0;
	intr_calls = 0;
}

/* the cable of board b's port p */
#define CAB( b, p)	((b) * 3 + (p))

/* a drive on board b, port p, master (0) or slave (1), with a marker byte in
 * every sector so a read shows which drive answered */
static void
disk( b, p, s, nsect, tag)
int		b, p, s, tag;
unsigned long	nsect;
{
	unsigned long	i;
	char		model[41];

	sprintf( model, "MOCK DISK b%d p%d %s", b, p, s ? "slave" : "master");
	mock_add_drive_on( CAB( b, p), s, nsect, model, 1);
	for (i = 0; i < nsect; ++i)
		mock_cdrv[CAB( b, p)][s].disk[i * MOCK_SECTOR + 100] = (unsigned char)tag;
}

static void
quiet_enxio( msg)
char	*msg;
{
	CHECK( sr_calls == 0, msg);
	CHECK( rd_calls == 0, "ENXIO: no root_defer");
	CHECK( mock_console[0] == '\0', "ENXIO is silent");
	CHECK( mock_cmds() == 0, "ENXIO: no ATA command issued");
}

static void
no_forbidden( msg)
char	*msg;
{
	CHECK( mock_bud_forbidden == 0, msg);
	CHECK( mock_bud_intena_reads == 0, "the write-only enable is never read");
}

/* READ(10) of one sector at lba for (card index c, unit) through the queue */
static void
read1( cp, c, card, unit, lba, buf)
struct sdcom	*cp;
int		c, card, unit;
unsigned long	lba;
char		*buf;
{
	memset( cp, 0, sizeof *cp);
	cp->cdb[0] = 0x28;
	cp->cdb[2] = lba >> 24; cp->cdb[3] = lba >> 16; cp->cdb[4] = lba >> 8; cp->cdb[5] = lba;
	cp->cdb[8] = 1;
	cp->addr  = buf;
	cp->nbyte = 512;
	cp->card  = card;
	cp->unit  = unit;
	cp->intr  = done;
	cp->reading = 1;
	intr_calls = 0;
	(*sr_fn)( c, cp);
	deliver();
	CHECK( mock_spl_calls == mock_splx_calls, "spl bracket balanced after the queue call");
}

/* the marker of the sector a read returned (the byte pair is swapped on a
 * little-endian host: ide_test.c's disk_equals()) */
static int
marker( buf)
char	*buf;
{
	unsigned short	one = 1;

	return (unsigned char)buf[*(unsigned char *)&one == 1 ? 101 : 100];
}

/* load with intr=1 and run the deferred call */
static int
load_intr( params)
char	*params;
{
	int	e;

	e = buddha_load( params);
	CHECK( e == 0, "intr=1: loads");
	CHECK( rd_calls == 1 && rd_fn != 0 && rd_arg == 0, "intr=1: root_defer(fn, 0) called once");
	if (rd_fn == 0)
		return -1;
	mock_console_reset();
	mock_spl_reset();
	return (*rd_fn)( rd_arg);
}

/* ---- the cases ---------------------------------------------------------- */

/* the mock itself: an access the board does not allow is counted */
extern unsigned char	mock_rb();
extern void		mock_wb();

static void
test_mock_guards()
{
	fresh();
	mock_wb( BASE0 + 0x7FEL, 0xFF);		/* the speed register */
	mock_wb( BASE0 + 0x0048L, 0x00);	/* AutoConfig space */
	mock_wb( BASE0 + 0xE00L, 0x00);		/* the clock port */
	mock_wb( BASE0 + 0xFC2L, 0x00);		/* past the enable */
	mock_wb( BASE0 + 0x1000L, 0x00);	/* the ROM */
	(void) mock_rb( BASE0 + 0x0D1AL);	/* a Buddha's (absent) third port */
	(void) mock_rb( BASE0 + 0x0806L - 4);	/* between the registers */
	CHECK( mock_bud_forbidden == 7, "the mock counts every forbidden access");
	(void) mock_rb( BASE0 + 0xFC0L);
	CHECK( mock_bud_intena_reads == 1, "the mock counts a read of the enable");
	mock_bud_forbidden = mock_bud_intena_reads = 0;
}

static void
test_no_board()
{
	int	e;

	fresh();
	mock_bud[0].present = 0;
	e = buddha_load( (char *)0);
	CHECK( e == ENXIO_, "no board: ENXIO");
	CHECK( ac_calls == 2 && ac_pc[0] == 0x12120000L && ac_pc[1] == 0x1212002AL,
	       "autocon() asked for the Buddha, then the Catweasel Z-II");
	CHECK( bp_calls == 0 && mock_accesses == 0, "no board: nothing touched");
	CHECK( mock_rom_reads == 0, "no Kickstart ROM read");
	quiet_enxio( "no board: not registered");
}

static void
test_no_drive()
{
	int	e;

	fresh();				/* a Buddha with nothing on its ports */
	e = buddha_load( (char *)0);
	CHECK( e == ENXIO_, "empty ports: ENXIO");
	CHECK( bp_calls == 2, "empty Buddha: both ports probed");
	CHECK( mock_cnien[CAB( 0, 0)] && mock_cnien[CAB( 0, 1)], "the readback left nIEN set");
	no_forbidden( "empty ports: nothing outside the port registers");
	quiet_enxio( "empty ports: not registered");
}

static void
test_discovery()
{
	struct sdcom	cp;
	char		buf[512];
	int		e;

	fresh();
	disk( 0, 0, 0, 4096L, 0x60);
	e = buddha_load( (char *)0);
	CHECK( e == 0, "a Buddha with a drive on port 0: loads");
	CHECK( bp_calls == 2 && bp_addrs[0] == BASE0 + 0x91AL && bp_addrs[1] == BASE0 + 0xB1AL,
	       "bprobe of each port's alternate status, 0x91A and 0xB1A");
	CHECK( bud_nboards == 1 && bud_base[0] == BASE0, "one board, its AutoConfig base");
	CHECK( bud_chok[0] == 1 && bud_chok[1] == 0 && bud_chok[2] == 0,
	       "port 0 answered; port 1 did not; a Buddha has no port 2");
	CHECK( sr_calls == 1 && sr_c[0] == 0 && sr_base[0] == (char *)BASE0 && sr_rank[0] == 4
	       && sr_name[0] && strcmp( sr_name[0], "Buddha IDE") == 0,
	       "sd_register(idequeue, 0, the board base, \"Buddha IDE\", 4)");
	CHECK( strstr( mock_console, "buddha: Buddha IDE at 0xe90000, polled I/O") != 0,
	       "the console names the board");
	CHECK( rd_calls == 1, "intr=1 by default: root_defer");
	CHECK( mock_cmd_count[0xEC] == 2, "IDENTIFY of port 0's master and slave (slave=1), none on port 1");
	CHECK( mock_cnien[CAB( 0, 0)], "the drive is left with nIEN set");
	CHECK( mock_rom_reads == 0, "no Kickstart ROM read");
	read1( &cp, 0, 0, 6, 7L, buf);
	CHECK( intr_calls == 1 && cp.okay && cp.status == 0, "unit 6 reads");
	CHECK( marker( buf) == 0x60, "unit 6 is port 0's master");
	read1( &cp, 0, 0, 5, 0L, buf);
	CHECK( intr_calls == 1 && !cp.okay, "unit 5: no slave, no such device");
	read1( &cp, 0, 0, 3, 0L, buf);
	CHECK( intr_calls == 1 && !cp.okay, "unit 3: no drive on port 1");
	no_forbidden( "discovery: nothing outside the port registers");
}

static void
test_units()
{
	static struct { int p, s, unit, tag; } map[] = {
		{ 0, 0, 6, 0x61 }, { 0, 1, 5, 0x62 }, { 1, 0, 3, 0x63 }, { 1, 1, 2, 0x64 },
		{ 2, 0, 1, 0x65 }, { 2, 1, 0, 0x66 },
	};
	struct sdcom	cp;
	char		buf[512], wbuf[512];
	int		e, i, u;

	fresh();
	mock_bud[0].cwz2 = 1;			/* three ports */
	for (i = 0; i < 6; ++i)
		disk( 0, map[i].p, map[i].s, 2048L + 64 * i, map[i].tag);
	e = buddha_load( "intr=0");
	CHECK( e == 0, "a Catweasel Z-II with six drives: loads");
	CHECK( ac_calls >= 2 && ac_pc[1] == 0x1212002AL, "the Catweasel Z-II is product 0x2A");
	CHECK( bp_calls == 3 && bp_addrs[2] == BASE0 + 0xD1AL, "a Catweasel Z-II: three ports probed");
	CHECK( sr_calls == 1 && sr_name[0] && strcmp( sr_name[0], "Catweasel IDE") == 0,
	       "registered as \"Catweasel IDE\"");
	CHECK( mock_cmd_count[0xEC] == 6, "IDENTIFY of every master and slave (slave=1 by default)");
	for (i = 0; i < 6; ++i) {
		read1( &cp, 0, 0, map[i].unit, 5L, buf);
		CHECK( intr_calls == 1 && cp.okay && cp.status == 0, "each unit reads");
		CHECK( marker( buf) == map[i].tag, "each unit reaches its own port and drive");
	}
	for (u = 0; u < 8; u += 1) {
		if (u != 4 && u != 7)
			continue;
		read1( &cp, 0, 0, u, 0L, buf);
		CHECK( intr_calls == 1 && !cp.okay, "units 4 and 7 are not used");
	}
	memset( wbuf, 0x5A, sizeof wbuf);	/* WRITE(10) to port 1's slave */
	memset( &cp, 0, sizeof cp);
	cp.cdb[0] = 0x2A; cp.cdb[5] = 9; cp.cdb[8] = 1;
	cp.addr = wbuf; cp.nbyte = 512; cp.unit = 2; cp.intr = done;
	intr_calls = 0;
	(*sr_fn)( 0, &cp);
	CHECK( intr_calls == 1 && cp.okay && cp.status == 0, "a write to unit 2");
	CHECK( mock_cdrv[CAB( 0, 1)][1].disk[9 * MOCK_SECTOR] == 0x5A
	       && mock_cdrv[CAB( 0, 1)][0].disk[9 * MOCK_SECTOR] != 0x5A
	       && mock_cdrv[CAB( 0, 0)][1].disk[9 * MOCK_SECTOR] != 0x5A,
	       "the write lands on port 1's slave and nowhere else");
	no_forbidden( "units: nothing outside the port registers");
}

static void
test_slave_param()
{
	struct sdcom	cp;
	char		buf[512];
	int		e;

	fresh();
	disk( 0, 0, 0, 2048L, 0x70);
	disk( 0, 0, 1, 2048L, 0x71);
	e = buddha_load( "intr=0 slave=0");
	CHECK( e == 0, "slave=0: loads");
	CHECK( mock_slave_selects == 0, "slave=0: no slave ever selected");
	read1( &cp, 0, 0, 5, 0L, buf);
	CHECK( intr_calls == 1 && !cp.okay, "slave=0: unit 5 is no device");
	read1( &cp, 0, 0, 6, 0L, buf);
	CHECK( cp.okay && marker( buf) == 0x70, "slave=0: the master reads");
	no_forbidden( "slave=0: nothing outside the port registers");
}

static void
test_bprobe_fault()
{
	int	e;

	fresh();
	disk( 0, 1, 0, 2048L, 0x72);
	bp_fail_at = 1;				/* port 0's alternate status faults */
	e = buddha_load( "intr=0");
	CHECK( e == 0, "port 0 faults, port 1 has a drive: loads");
	CHECK( bud_chok[0] == 0 && bud_chok[1] == 1, "the faulting port is left out");
	CHECK( mock_cnien[CAB( 0, 0)] == 0, "the faulting port is never written");
	no_forbidden( "bprobe fault: nothing outside the port registers");
}

static void
test_two_boards()
{
	struct sdcom	cp;
	char		buf[512];
	int		e;

	fresh();
	mock_bud[1].present = 1;
	mock_bud[1].cwz2 = 1;
	mock_bud[1].base = BASE1;
	disk( 0, 0, 0, 2048L, 0x80);		/* board 0: a Buddha */
	disk( 1, 0, 0, 2048L, 0x81);		/* board 1: a Catweasel Z-II */
	disk( 1, 2, 0, 2048L, 0x82);
	e = load_intr( (char *)0);
	CHECK( e == 0, "two boards: the deferred call returns 0");
	CHECK( sr_calls == 2, "one registration per board");
	CHECK( sr_c[0] == 0 && sr_base[0] == (char *)BASE0 && strcmp( sr_name[0], "Buddha IDE") == 0,
	       "board 0: index 0, its own base");
	CHECK( sr_c[1] == 1 && sr_base[1] == (char *)BASE1 && strcmp( sr_name[1], "Catweasel IDE") == 0,
	       "board 1: index 1, its own base");
	CHECK( mock_bud[0].intena_writes == 1 && mock_bud[1].intena_writes == 1,
	       "the enable written once on each board");
	CHECK( ide_selftest == 1, "both boards' ports interrupted: interrupt I/O");
	/* sd numbers the cards; say the Buddha is card 1 and the Catweasel card 0 */
	read1( &cp, 0, 1, 6, 3L, buf);
	CHECK( intr_calls == 1 && cp.okay && marker( buf) == 0x80, "card 1 (board 0) unit 6: the Buddha's drive");
	read1( &cp, 1, 0, 6, 3L, buf);
	CHECK( intr_calls == 1 && cp.okay && marker( buf) == 0x81, "card 0 (board 1) unit 6: the Catweasel's drive");
	read1( &cp, 1, 0, 1, 3L, buf);
	CHECK( intr_calls == 1 && cp.okay && marker( buf) == 0x82, "board 1 unit 1: its port 2");
	read1( &cp, 0, 1, 1, 3L, buf);
	CHECK( intr_calls == 1 && !cp.okay, "board 0 unit 1: a Buddha has no port 2");
	no_forbidden( "two boards: nothing outside the port registers");
}

static void
test_atapi()
{
	struct sdcom	cp;
	unsigned char	buf[2048];
	int		pass, i, ok, unit, s;

	for (pass = 0; pass < 2; ++pass) {
		fresh();
		disk( 0, 0, 0, 2048L, 0x90);
		if (pass == 0) {		/* a disk and a CD-ROM on port 1 */
			disk( 0, 1, 0, 2048L, 0x91);
			mock_add_cdrom_on( CAB( 0, 1), 1, 64L, "MOCK CD-ROM");
			unit = 2;
			s = 1;
			CHECK( buddha_load( "intr=0") == 0, "ATAPI on port 1's slave, polled: loads");
		} else {			/* a CD-ROM alone on port 1 */
			mock_add_cdrom_on( CAB( 0, 1), 0, 64L, "MOCK CD-ROM");
			unit = 3;
			s = 0;
			CHECK( load_intr( (char *)0) == 0 && ide_selftest == 1,
			       "ATAPI alone on port 1: interrupt I/O");
			CHECK( mock_cmd_count[0xA1] == 2,
			       "IDENTIFY PACKET DEVICE: the attach's and port 1's self-test");
			CHECK( mock_cmd_count[0x40] == 1, "READ VERIFY: port 0's self-test only");
		}
		memset( &cp, 0, sizeof cp);
		cp.cdb[0] = 0x28; cp.cdb[5] = 7; cp.cdb[8] = 1;
		cp.addr = (char *)buf; cp.nbyte = 2048; cp.unit = unit; cp.intr = done; cp.reading = 1;
		intr_calls = 0;
		(*sr_fn)( 0, &cp);
		deliver();
		CHECK( intr_calls == 1 && cp.okay && cp.status == 0, "READ(10) of the CD-ROM's unit");
		CHECK( mock_pkt_log_n >= 1 && mock_pkt_log[mock_pkt_log_n - 1] == 0x28, "sent as a PACKET");
		for (ok = 1, i = 0; i < 2048; ++i)
			if (buf[i] != mock_cdrv[CAB( 0, 1)][s].cd[7 * MOCK_CD_BLOCK + i])
				ok = 0;
		CHECK( ok, "the CD-ROM's block, byte for byte");
		no_forbidden( "ATAPI: nothing outside the port registers");
	}
}

static void
test_intr_selftest_pass()
{
	struct sdcom	cp;
	char		buf[512];
	int		e;

	fresh();
	disk( 0, 0, 0, 2048L, 0xA0);
	disk( 0, 1, 0, 2048L, 0xA1);
	e = load_intr( (char *)0);
	CHECK( e == 0, "the deferred call returns 0");
	CHECK( ia_calls == 1 && ia_level == 2 && ia_name && strcmp( ia_name, "buddha") == 0,
	       "intr_attach at level 2, named buddha");
	CHECK( mock_bud[0].intena_writes == 1, "the enable written exactly once");
	CHECK( mock_cmd_count[0x40] == 2, "one READ VERIFY per port");
	CHECK( ide_selftest == 1 && strstr( mock_console, "buddha: interrupt I/O") != 0,
	       "every port interrupted: interrupt I/O");
	CHECK( mock_cnien[CAB( 0, 0)] == 0 && mock_cnien[CAB( 0, 1)] == 0, "nIEN clear on both ports");
	CHECK( id_calls == 0, "the handler stays attached");
	read1( &cp, 0, 0, 3, 11L, buf);
	CHECK( intr_calls == 1 && cp.okay && marker( buf) == 0xA1, "unit 3 through the interrupt path");
	read1( &cp, 0, 0, 6, 11L, buf);
	CHECK( intr_calls == 1 && cp.okay && marker( buf) == 0xA0, "unit 6 through the interrupt path");
	CHECK( claims > 0 && unclaimed == 0, "every INT2 claimed");
	/* an INTRQ on port 1 while nothing runs there is dropped, not left asserted */
	mock_cintrq[CAB( 0, 1)] = 1;
	read1( &cp, 0, 0, 6, 12L, buf);
	CHECK( cp.okay, "unit 6 reads with port 1's INTRQ up");
	CHECK( mock_cintrq[CAB( 0, 1)] == 0 && bud_intr_other >= 1, "port 1's stray INTRQ dropped");
	CHECK( !mock_int2_asserted(), "INT2 released");
	no_forbidden( "self-test: nothing outside the port registers");
}

static void
test_intr_selftest_fail()
{
	struct sdcom	cp;
	char		buf[512];
	int		e;

	fresh();
	disk( 0, 0, 0, 2048L, 0xB0);
	disk( 0, 1, 0, 2048L, 0xB1);
	mock_cdrv[CAB( 0, 1)][0].mute_intrq = 1;	/* port 1's drive never interrupts */
	ide_timeout_secs = 2;
	e = load_intr( (char *)0);
	ide_timeout_secs = 20;
	CHECK( e == 0, "the deferred call returns 0");
	CHECK( ide_selftest == 2 && strstr( mock_console, "interrupt self-test failed, polled I/O") != 0,
	       "a port that does not interrupt: polled");
	CHECK( id_calls == 1 && handler == 0, "the handler detached");
	CHECK( mock_cnien[CAB( 0, 0)] && mock_cnien[CAB( 0, 1)], "nIEN set on every port");
	read1( &cp, 0, 0, 3, 1L, buf);
	CHECK( intr_calls == 1 && cp.okay && marker( buf) == 0xB1, "polled I/O still reaches port 1");
	fresh();
	disk( 0, 0, 0, 2048L, 0xB2);
	ia_ret = 16;
	e = load_intr( (char *)0);
	CHECK( e == 0 && ide_selftest == 2, "intr_attach refused: polled");
	CHECK( mock_bud[0].intena_writes == 0, "intr_attach refused: the enable never written");
	CHECK( mock_cmds() == 2, "intr_attach refused: no self-test command (the attach's two IDENTIFYs only)");
	no_forbidden( "self-test failure: nothing outside the port registers");
}

static void
test_params_errors()
{
	int	e;

	fresh();
	disk( 0, 0, 0, 2048L, 0xC0);
	e = buddha_load( "intr=0 debug=1");
	CHECK( e == 0 && rd_calls == 0, "intr=0: no root_defer");
	CHECK( ide_debug == 1 && ide_slave_enable == 1, "debug=1 taken, slave=1 the default");
	fresh();
	disk( 0, 0, 0, 2048L, 0xC1);
	e = buddha_load( "slave=7");
	CHECK( e == 0 && ide_slave_enable == 1, "slave=7: out of range, the default kept");
	CHECK( strstr( mock_console, "parameter slave is not 0 to 2") != 0, "and said");
	fresh();
	disk( 0, 0, 0, 2048L, 0xC2);
	sr_fail_at = 1; sr_ret = EBUSY_;
	e = buddha_load( (char *)0);
	CHECK( e == EBUSY_, "sd_register's error is returned");
	CHECK( rd_calls == 0, "no root_defer after a failed registration");
	CHECK( mock_cnien[CAB( 0, 0)], "the drive left with nIEN set");
	fresh();
	disk( 0, 0, 0, 2048L, 0xC3);
	rd_ret = ENOSPC_;
	e = buddha_load( (char *)0);
	CHECK( e == 0 && strstr( mock_console, "root_defer failed (errno 28), polled I/O") != 0,
	       "root_defer refused: still loaded, polled, said");
	no_forbidden( "parameters: nothing outside the port registers");
}

static void
test_halt_unload()
{
	unsigned long	cmds;
	int		e;

	fresh();
	mock_bud[0].cwz2 = 1;
	disk( 0, 0, 0, 2048L, 0xD0);
	disk( 0, 2, 0, 2048L, 0xD1);
	e = load_intr( (char *)0);
	CHECK( e == 0 && ide_selftest == 1, "two ports in interrupt I/O");
	CHECK( mock_cnien[CAB( 0, 0)] == 0 && mock_cnien[CAB( 0, 2)] == 0, "nIEN clear before the halt");
	cmds = mock_cmds();
	buddha_halt();
	CHECK( ide_halted == 1, "the halt pass ran");
	CHECK( mock_cnien[CAB( 0, 0)] && mock_cnien[CAB( 0, 2)], "_halt: nIEN on every port");
	CHECK( mock_cmds() == cmds, "_halt: no command");
	mock_cnien[CAB( 0, 0)] = mock_cnien[CAB( 0, 2)] = 0;
	e = buddha_unload();
	CHECK( e == 0 && mock_cnien[CAB( 0, 0)] && mock_cnien[CAB( 0, 2)], "_unload: nIEN on every port");
	CHECK( bud_nboards == 0 && bud_chok[0] == 0 && bud_chok[2] == 0, "_unload: the boards forgotten");
	no_forbidden( "halt and unload: nothing outside the port registers");
}

/*
 * Which drives a port has: none, the master, the slave alone, or both, on
 * port 0 and on port 1, under each model of what an absent device reads as
 * (the open bus, Amiberry's, a bus that holds the last byte written).  A
 * slave alone is attached as the slave; the master of such a port is never
 * sent a command.
 */
static void
test_layouts()
{
	static int	munit[2] = { 6, 3 }, sunit[2] = { 5, 2 };
	static char	*modes[3] = { "open bus", "Amiberry", "bus hold" };
	struct sdcom	cp;
	char		buf[512], msg[160];
	int		mode, p, lay, e, m, sl;

	for (mode = 0; mode < 3; ++mode)
		for (p = 0; p < 2; ++p)
			for (lay = 0; lay < 4; ++lay) {	/* bit 0 master, bit 1 slave */
				m = lay & 1;
				sl = (lay >> 1) & 1;
				fresh();
				mock_absent_mode = mode;
				if (m)
					disk( 0, p, 0, 2048L, 0x40 + p);
				if (sl)
					disk( 0, p, 1, 2048L, 0x50 + p);
				e = buddha_load( "intr=0");
				sprintf( msg, "%s, port %d, %s", modes[mode], p,
					 lay == 0 ? "no drive" : lay == 1 ? "master only" :
					 lay == 2 ? "slave only" : "master and slave");
				if (lay == 0) {
					CHECK( e == ENXIO_, msg);
					quiet_enxio( msg);
					no_forbidden( msg);
					continue;
				}
				CHECK( e == 0 && sr_calls == 1, msg);
				CHECK( bud_chok[p] == 1 && bud_chok[p ^ 1] == 0, msg);
				/* IDENTIFY: each drive that is there; an absent slave beside a
				 * master may be asked too (unless its status shows BSY, as a
				 * holding bus does); an absent master never */
				CHECK( m ? mock_cmd_count[0xEC] >= (unsigned long)(m + sl)
					   && mock_cmd_count[0xEC] <= 2
					 : mock_cmd_count[0xEC] == 1,
				       "IDENTIFY: every drive there, never an absent master");
				read1( &cp, 0, 0, munit[p], 4L, buf);
				CHECK( intr_calls == 1 && (m ? cp.okay && marker( buf) == 0x40 + p : !cp.okay), msg);
				read1( &cp, 0, 0, sunit[p], 4L, buf);
				CHECK( intr_calls == 1 && (sl ? cp.okay && marker( buf) == 0x50 + p : !cp.okay), msg);
				no_forbidden( msg);
			}
}

/* a slave alone: interrupt I/O through it, ATA and ATAPI; slave=0 leaves it */
static void
test_lone_slave()
{
	struct sdcom	cp;
	unsigned char	cbuf[2048];
	char		buf[512];
	int		e, i, ok;

	fresh();
	disk( 0, 1, 1, 2048L, 0x58);		/* port 1: an ATA slave alone */
	e = load_intr( (char *)0);
	CHECK( e == 0 && ide_selftest == 1, "ATA slave alone: the self-test passes on the slave");
	CHECK( mock_cmd_count[0x40] == 1 && mock_cmd_count[0xEC] == 1,
	       "READ VERIFY to the slave; one IDENTIFY, the slave's");
	CHECK( mock_cnien[CAB( 0, 1)] == 0, "nIEN clear on the port");
	read1( &cp, 0, 0, 2, 6L, (char *)buf);
	CHECK( intr_calls == 1 && cp.okay && marker( buf) == 0x58 && claims > 0,
	       "unit 2 through the interrupt path");
	no_forbidden( "ATA slave alone: nothing outside the port registers");

	fresh();
	mock_absent_mode = MOCK_ABSENT_UAE;
	mock_add_cdrom_on( CAB( 0, 0), 1, 64L, "MOCK CD-ROM");	/* port 0: an ATAPI slave alone */
	e = load_intr( (char *)0);
	CHECK( e == 0 && ide_selftest == 1, "ATAPI slave alone: the self-test passes on it");
	CHECK( mock_cmd_count[0xA1] == 2, "IDENTIFY PACKET DEVICE: the attach's and the self-test's");
	memset( &cp, 0, sizeof cp);
	cp.cdb[0] = 0x28; cp.cdb[5] = 9; cp.cdb[8] = 1;
	cp.addr = (char *)cbuf; cp.nbyte = 2048; cp.unit = 5; cp.intr = done; cp.reading = 1;
	intr_calls = 0;
	(*sr_fn)( 0, &cp);
	deliver();
	CHECK( intr_calls == 1 && cp.okay && cp.status == 0, "unit 5 reads the CD-ROM");
	for (ok = 1, i = 0; i < 2048; ++i)
		if (cbuf[i] != mock_cdrv[CAB( 0, 0)][1].cd[9 * MOCK_CD_BLOCK + i])
			ok = 0;
	CHECK( ok, "the CD-ROM's block, byte for byte");
	read1( &cp, 0, 0, 6, 0L, (char *)buf);
	CHECK( intr_calls == 1 && !cp.okay, "unit 6: no master");
	no_forbidden( "ATAPI slave alone: nothing outside the port registers");

	fresh();
	disk( 0, 0, 1, 2048L, 0x59);
	e = buddha_load( "intr=0 slave=0");
	CHECK( e == ENXIO_ && mock_slave_selects == 0, "slave=0: a slave alone is not probed, ENXIO");
	quiet_enxio( "slave=0, slave alone: not registered");
	no_forbidden( "slave=0: nothing outside the port registers");
}

/* device 0 answering for an absent device 1 does not make a slave */
static void
test_ghost()
{
	struct sdcom	cp;
	char		buf[512];
	int		e;

	fresh();
	mock_ghost = 1;
	disk( 0, 0, 0, 2048L, 0x5A);
	e = buddha_load( "intr=0");
	CHECK( e == 0, "master only, device 0 answering for device 1: loads");
	read1( &cp, 0, 0, 5, 0L, buf);
	CHECK( intr_calls == 1 && !cp.okay, "no ghost slave: unit 5 is no device");
	read1( &cp, 0, 0, 6, 0L, buf);
	CHECK( cp.okay && marker( buf) == 0x5A, "the master reads");
	no_forbidden( "ghost: nothing outside the port registers");
}

/* how often `needle' is in the console */
static int
said( needle)
char	*needle;
{
	char	*p;
	int	n;

	for (n = 0, p = mock_console; (p = strstr( p, needle)) != 0; ++p)
		n++;
	return n;
}

/* a READ(10) of one sector queued (interrupt I/O), not waited for */
static void
queue1( cp, unit, lba, buf)
struct sdcom	*cp;
int		unit;
unsigned long	lba;
char		*buf;
{
	memset( cp, 0, sizeof *cp);
	cp->cdb[0] = 0x28;
	cp->cdb[5] = lba;
	cp->cdb[8] = 1;
	cp->addr  = buf;
	cp->nbyte = 512;
	cp->unit  = unit;
	cp->intr  = done;
	cp->reading = 1;
	(*sr_fn)( 0, cp);
}

/*
 * The self-test passed, then the interrupts stop: the request on the cable
 * and the one queued behind it still complete, with their data, once the
 * watchdog gives up; the driver is polled from then on, nIEN set on every
 * port, one console line; later requests complete in the call, with no
 * watchdog and no second line.
 */
static void
test_intr_stops()
{
	struct sdcom	cp1, cp2, cp3;
	char		b1[512], b2[512], b3[512];
	int		e, i;

	fresh();
	disk( 0, 0, 0, 2048L, 0xE0);
	disk( 0, 1, 0, 2048L, 0xE1);
	e = load_intr( (char *)0);
	CHECK( e == 0 && ide_selftest == 1, "interrupt I/O after the self-test");
	read1( &cp1, 0, 0, 6, 1L, b1);
	CHECK( intr_calls == 1 && cp1.okay && marker( b1) == 0xE0, "a read through the interrupt path");
	mock_console_reset();
	ide_intr_timeouts = 0;
	mock_cdrv[CAB( 0, 0)][0].mute_intrq = 1;	/* the interrupts stop */
	ide_timeout_secs = 3;
	intr_calls = 0;
	queue1( &cp1, 6, 2L, b1);
	queue1( &cp2, 3, 3L, b2);			/* queued behind it, on port 1 */
	deliver();
	CHECK( intr_calls == 0 && mock_timeouts_pending() == 1, "both pending, the watchdog armed");
	for (i = 0; i <= 3 && intr_calls == 0; ++i)
		mock_fire_timeouts();
	CHECK( intr_calls == 2, "the watchdog's turn: both requests complete");
	CHECK( cp1.okay && cp1.status == 0 && marker( b1) == 0xE0, "the timed-out request, run again polled: GOOD, its data");
	CHECK( cp2.okay && cp2.status == 0 && marker( b2) == 0xE1, "the queued request, polled: GOOD, its data");
	CHECK( ide_intr_timeouts == 1 && bud_intr_lost == 1, "one timeout, counted as interrupts stopped");
	CHECK( said( "buddha: interrupts stopped, polled I/O from now on") == 1, "said once");
	CHECK( mock_cnien[CAB( 0, 0)] && mock_cnien[CAB( 0, 1)], "nIEN set on every port");
	CHECK( mock_timeouts_pending() == 0, "the watchdog disarmed");
	CHECK( mock_spl_calls == mock_splx_calls, "spl balanced");
	for (i = 0; i < 3; ++i) {
		read1( &cp3, 0, 0, i == 1 ? 3 : 6, 4L + i, b3);
		CHECK( intr_calls == 1 && cp3.okay && cp3.status == 0, "later requests complete in the call, polled");
	}
	CHECK( marker( b3) == 0xE0, "with their data");
	CHECK( mock_timeouts_pending() == 0, "no watchdog for polled requests");
	CHECK( said( "interrupts stopped") == 1 && said( "timed out") == 1, "no second line");
	ide_timeout_secs = 20;
	no_forbidden( "interrupts stopping: nothing outside the port registers");

	/* a request an interrupt served before it timed out fails as before:
	 * the drive hangs after the first sector of a two-sector read */
	fresh();
	disk( 0, 0, 0, 2048L, 0xE2);
	e = load_intr( (char *)0);
	CHECK( e == 0 && ide_selftest == 1, "interrupt I/O again");
	mock_console_reset();
	ide_timeout_secs = 3;
	memset( &cp1, 0, sizeof cp1);
	cp1.cdb[0] = 0x28; cp1.cdb[5] = 5; cp1.cdb[8] = 2;
	cp1.addr = b1; cp1.nbyte = 512; cp1.unit = 6; cp1.intr = done; cp1.reading = 1;
	intr_calls = 0;
	mock_cdrv[CAB( 0, 0)][0].hang_after = 1;	/* one sector, then nothing */
	(*sr_fn)( 0, &cp1);
	deliver();
	CHECK( intr_calls == 0 && claims > 0, "the first sector's interrupt came");
	for (i = 0; i <= 3 && intr_calls == 0; ++i)
		mock_fire_timeouts();
	CHECK( intr_calls == 1 && cp1.okay && cp1.status == 0x02, "a hang after an interrupt: CHECK CONDITION, as on the Gayle");
	CHECK( bud_intr_lost == 0 && said( "interrupts stopped") == 0, "interrupts kept");
	CHECK( mock_cnien[CAB( 0, 0)] == 0, "nIEN still clear");
	ide_timeout_secs = 20;
	no_forbidden( "a hang after an interrupt: nothing outside the port registers");
}

int
main()
{
#if IDE_CACHE
	const char	*name = "ide_budtest (040/060: IDE_CACHE, IDE_NBPP 4096)";
#else
	const char	*name = "ide_budtest";
#endif
	fprintf( stderr, "== test_mock_guards\n"); test_mock_guards();
	fprintf( stderr, "== test_no_board\n"); test_no_board();
	fprintf( stderr, "== test_no_drive\n"); test_no_drive();
	fprintf( stderr, "== test_discovery\n"); test_discovery();
	fprintf( stderr, "== test_units\n"); test_units();
	fprintf( stderr, "== test_slave_param\n"); test_slave_param();
	fprintf( stderr, "== test_bprobe_fault\n"); test_bprobe_fault();
	fprintf( stderr, "== test_two_boards\n"); test_two_boards();
	fprintf( stderr, "== test_atapi\n"); test_atapi();
	fprintf( stderr, "== test_intr_selftest_pass\n"); test_intr_selftest_pass();
	fprintf( stderr, "== test_intr_selftest_fail\n"); test_intr_selftest_fail();
	fprintf( stderr, "== test_params_errors\n"); test_params_errors();
	fprintf( stderr, "== test_halt_unload\n"); test_halt_unload();
	fprintf( stderr, "== test_layouts\n"); test_layouts();
	fprintf( stderr, "== test_lone_slave\n"); test_lone_slave();
	fprintf( stderr, "== test_ghost\n"); test_ghost();
	fprintf( stderr, "== test_intr_stops\n"); test_intr_stops();
	fresh();
	CHECK( forbidden_total == 0, "no forbidden board access in any case");
	CHECK( mock_id_reads + mock_id_writes == 0, "no access to 0xDE1000");
	mock_free();
	fprintf( stderr, "%s: %d checks, %d failures\n", name, checks, failures);
	return failures != 0;
}
