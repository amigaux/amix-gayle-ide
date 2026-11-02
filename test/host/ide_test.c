/*
 * ide_test.c -- host harness for the Amix Gayle IDE driver (../../src/ide.c).
 * Drives idepresent()/idequeue()/ideintr() with hand-built struct sdcom
 * requests against the mock in mock_gayle.c and checks the SCSI->ATA
 * translation, the board layer and the interrupt path byte for byte.
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

extern int	idepresent();
extern char	idequeue();
extern void	idereset();
extern void	ideintr();
extern unsigned long	ide_spins, ide_spins_reset, ide_spins_short, ide_cq_overflow;
extern unsigned long	ide_intr_count, ide_intr_spurious, ide_intr_timeouts, ide_timeout_secs;
extern unsigned long	ide_dup_submits, ide_resets, ide_pagecross_n, ide_intr_busy;
extern int	ide_slave_enable, ide_debug, ide_master_unit, ide_slave_unit;
extern int	ide_intr_enable, ide_board_force;
extern unsigned long	ide_atapi_cmds, ide_atapi_overrun, ide_spins_atapi, ide_atapi_irlate;
#if IDE_CACHE
extern unsigned long	ide_ci_ok;
#endif

/* the investigation-only symbols: defined only in an IDE_DIAG build; a default
 * build must not have them, which the weak references let the harness check */
#if IDE_DIAG
#define DIAG_SYM
#else
#define DIAG_SYM	__attribute__((weak))
#endif
extern unsigned long	ide_spins_hw DIAG_SYM;
#if IDE_CACHE
extern long		ide_cache DIAG_SYM;
extern unsigned long	ide_push_n DIAG_SYM, ide_push_bytes DIAG_SYM;
#endif

/* page size the driver was built for (the 68040/68060 line uses 4096) */
#ifndef IDE_NBPP
#define IDE_NBPP	2048
#endif
#define PG	IDE_NBPP

/* the kernel's level-2 table (master.d/kernel.c); mutable here so a test can
 * remove the hook and exercise the polled fallback */
void	(*int2_tbl[4])() = { ideintr, 0, 0, 0 };

extern int	mock_ipl;
extern long	mock_spl_calls, mock_splx_calls, mock_timeout_calls;
extern char	mock_console[];
extern void	mock_spl_reset(), mock_console_reset(), mock_fire_timeouts(), mock_timeouts_reset();
extern int	mock_timeouts_pending();

static int	failures, checks;
static int	intr_calls, intr_depth, intr_maxdepth, pumped;
static struct sdcom *intr_last;

#define CHECK( cond, msg) do { checks++; if (!(cond)) { failures++; \
	fprintf( stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, msg); } } while (0)

static void
done( cp)
struct sdcom	*cp;
{
	intr_calls++;
	intr_last = cp;
}

static void
fresh( unit_count)
int	unit_count;
{
	mock_free();
	mock_reset();
	mock_spl_reset();
	mock_console_reset();
	mock_timeouts_reset();
	idereset();
	mock_board = MOCK_BOARD_A4000;
	int2_tbl[0] = ideintr;
	ide_slave_enable = 0;
	ide_intr_enable = 1;
	ide_board_force = 0;
	ide_master_unit = 6; ide_slave_unit = 5;
	ide_spins = 200000;
	ide_spins_reset = 200000;
	ide_spins_short = 20000;
	ide_intr_count = ide_intr_spurious = ide_intr_timeouts = 0;
	ide_dup_submits = ide_resets = ide_pagecross_n = 0;
	if (unit_count >= 1)
		mock_add_drive( 0, 4096L, "MOCK CF CARD 2GB", 1);
	if (unit_count >= 2)
		mock_add_drive( 1, 2048L, "MOCK SLAVE", 0);
	intr_calls = 0;
	intr_last = 0;
}

/* p2int stand-in: call the handler while the board asserts INT2 */
static void
pump()
{
	int	guard = 100000;

	pumped = 0;
	while (mock_int2_asserted() && guard-- > 0) {
		ideintr();
		pumped++;
	}
	CHECK( guard > 0, "interrupt storm: handler never cleared the request");
}

static void
run( cp, unit, cdb, cdblen, buf, nbyte)
struct sdcom	*cp;
int		unit, cdblen;
unsigned char	*cdb;
char		*buf;
unsigned	nbyte;
{
	memset( cp, 0, sizeof *cp);
	memcpy( cp->cdb, cdb, cdblen);
	cp->addr  = buf;
	cp->nbyte = nbyte;
	cp->unit  = unit;
	cp->intr  = done;
	cp->reading = (cdb[0] == 0x28 || cdb[0] == 0x08);
	intr_calls = 0;
	idequeue( 0, cp);
	CHECK( mock_spl_calls == mock_splx_calls, "spl bracket balanced after idequeue");
	CHECK( mock_ipl == 0, "IPL restored after idequeue");
	pump();
	CHECK( intr_calls == 1, "completion delivered exactly once");
	CHECK( intr_last == cp, "completion carries the request");
#if IDE_CACHE
	CHECK( mock_order_violations == 0, "no task-file read ahead of an unsynchronised write");
#endif
}

static void
read10( cp, unit, lba, nblk, buf, nbyte)
struct sdcom	*cp;
int		unit, nblk;
unsigned long	lba;
char		*buf;
unsigned	nbyte;
{
	unsigned char	cdb[10];

	memset( cdb, 0, sizeof cdb);
	cdb[0] = 0x28;
	cdb[2] = lba >> 24; cdb[3] = lba >> 16; cdb[4] = lba >> 8; cdb[5] = lba;
	cdb[7] = nblk >> 8; cdb[8] = nblk;
	run( cp, unit, cdb, 10, buf, nbyte);
}

static void
write10( cp, unit, lba, nblk, buf, nbyte)
struct sdcom	*cp;
int		unit, nblk;
unsigned long	lba;
char		*buf;
unsigned	nbyte;
{
	unsigned char	cdb[10];

	memset( cdb, 0, sizeof cdb);
	cdb[0] = 0x2A;
	cdb[2] = lba >> 24; cdb[3] = lba >> 16; cdb[4] = lba >> 8; cdb[5] = lba;
	cdb[7] = nblk >> 8; cdb[8] = nblk;
	run( cp, unit, cdb, 10, buf, nbyte);
}

static void
reqsense( cp, unit, buf)
struct sdcom	*cp;
int		unit;
unsigned char	*buf;
{
	unsigned char	cdb[6] = { 0x03, 0, 0, 0, 18, 0 };

	memset( buf, 0xEE, 18);
	run( cp, unit, cdb, 6, (char *)buf, 18);
}

static void
tur( cp, unit)
struct sdcom	*cp;
int		unit;
{
	unsigned char	cdb[6] = { 0, 0, 0, 0, 0, 0 };

	run( cp, unit, cdb, 6, 0, 0);
}

/*
 * The driver moves 16-bit register words as integers: on the 68k (big endian)
 * storing the word (stream[0] << 8 | stream[1]) reproduces the drive's byte
 * stream in memory.  On a little-endian host the same store lands the two
 * bytes swapped, so the expected memory image is the disk image with byte
 * pairs swapped.  This keeps the driver source identical and the check exact.
 */
static int
host_little_endian()
{
	unsigned short	one = 1;

	return *(unsigned char *)&one == 1;
}

static int
disk_equals( unit, lba, buf, nbyte)
int		unit;
unsigned long	lba;
const char	*buf;
unsigned	nbyte;
{
	const unsigned char	*d = mock_drv[unit].disk + lba * MOCK_SECTOR;
	unsigned		i;

	if (!host_little_endian())
		return memcmp( d, buf, nbyte) == 0;
	for (i = 0; i + 1 < nbyte; i += 2)
		if ((unsigned char)buf[i] != d[i + 1] || (unsigned char)buf[i + 1] != d[i])
			return 0;
	return 1;
}

/* ------------------------------------------------------- board tests */

static void
test_probe_no_gayle()
{
	char	*pa = 0;

	fresh( 1);
	mock_board = MOCK_BOARD_NONE;			/* A3000 class, ECS */
	CHECK( idepresent( &pa) == 0, "no Gayle: probe reports absent");
	CHECK( mock_taskfile_accesses == 0, "no Gayle: no task-file register touched");
	CHECK( mock_id_reads == 0, "no Gayle: Gayle ID register never read");
	CHECK( mock_accesses == 1 && mock_bad_accesses == 0, "no Gayle: exactly one access, VPOSR");
	CHECK( pa == 0, "no Gayle: base not written");
	CHECK( idepresent( &pa) == 0 && mock_accesses == 1, "repeated probe still silent");
}

static void
test_probe_chipset_ids()
{
	char		*pa;
	static unsigned short	ecs[] = { 0x0000, 0x1000, 0x2000, 0x3000, 0x2100, 0xA000, 0xB000 };
	static unsigned short	aga[] = { 0x2200, 0x2300, 0x3200, 0x3300, 0xA300, 0xB300 };
	unsigned	i;

	for (i = 0; i < sizeof ecs / sizeof ecs[0]; ++i) {
		fresh( 1);				/* an A4000 mock, but the chipset says ECS */
		mock_vposr_value = ecs[i]; mock_vposr_override = 1;
		pa = 0;
		CHECK( idepresent( &pa) == 0 && mock_taskfile_accesses == 0,
		       "OCS/ECS id (incl. NTSC bit): A4000 base never touched");
	}
	for (i = 0; i < sizeof aga / sizeof aga[0]; ++i) {
		fresh( 1);
		mock_vposr_value = aga[i]; mock_vposr_override = 1;
		pa = 0;
		CHECK( idepresent( &pa) == 1 && pa == (char *)0xDD2020L, "Alice id (PAL/NTSC, LOF): A4000 present");
	}
}

static void
test_probe_a4000()
{
	char	*pa = 0;
	unsigned long	n;

	fresh( 0);
	CHECK( idepresent( &pa) == 0, "A4000 without drive: absent");
	CHECK( mock_cmds() == 0, "A4000 without drive: no ATA command issued");
	CHECK( mock_id_reads == 0 && mock_bad_accesses == 0, "A4000 without drive: only VPOSR and the task file");
	CHECK( mock_accesses < 32, "A4000 without drive: open bus settled without a BSY spin");

	fresh( 1);
	mock_drv[0].stuck_bsy = 1;
	CHECK( idepresent( &pa) == 1 && pa == (char *)0xDD2020L, "A4000 + drive holding registers while BSY: present");
	CHECK( mock_accesses < 32, "busy drive: registers held, no BSY spin at probe");

	fresh( 1);
	CHECK( idepresent( &pa) == 1 && pa == (char *)0xDD2020L, "A4000 + drive: present at 0xDD2020");
	CHECK( mock_id_reads == 0, "A4000: the Gayle ID register was not consulted");
	CHECK( mock_bad_accesses == 0, "A4000: no undecoded address touched");
	CHECK( mock_cmds() == 0, "probe issues no ATA command (attach is lazy)");
	CHECK( mock_slave_selects == 0, "probe never selects the slave");
	n = mock_accesses;
	pa = 0;
	CHECK( idepresent( &pa) == 1 && pa == (char *)0xDD2020L, "second probe is cached");
	CHECK( mock_accesses == n, "second probe touches no register");
	CHECK( strstr( mock_console, "A4000 IDE at 0xdd2020 present") != 0, "probe logs the board");
}

static void
test_probe_a1200_a600()
{
	char	*pa = 0;

	fresh( 1);					/* auto probe: the A1200 is never detected */
	mock_board = MOCK_BOARD_A1200;
	CHECK( idepresent( &pa) == 0, "A1200, automatic probe: absent (A4000-only auto-detect)");
	CHECK( mock_id_reads == 0, "A1200, automatic probe: Gayle ID register never read");
	CHECK( mock_bad_accesses == 0, "A1200, automatic probe: no undecoded address touched");

	fresh( 1);
	mock_board = MOCK_BOARD_A1200;
	ide_board_force = 2;
	CHECK( idepresent( &pa) == 1 && pa == (char *)0xDA0000L, "A1200 forced: present at 0xDA0000");
	CHECK( mock_id_reads == 8, "A1200 forced: Gayle ID confirmed once (8 serial bits)");
	CHECK( mock_bad_accesses == 0, "A1200 forced: no undecoded address touched");
	CHECK( strstr( mock_console, "A1200 IDE") != 0, "A1200 named");

	fresh( 1);
	mock_board = MOCK_BOARD_A600;
	ide_board_force = 3;
	CHECK( idepresent( &pa) == 1 && pa == (char *)0xDA0000L, "A600 forced: present at 0xDA0000 (ECS chipset)");
	CHECK( strstr( mock_console, "A600 IDE") != 0, "A600 named");

	fresh( 1);
	mock_board = MOCK_BOARD_A600;
	ide_board_force = 2;
	CHECK( idepresent( &pa) == 0, "A600 forced as A1200: Gayle ID mismatch, absent");
	CHECK( mock_taskfile_accesses == 0, "Gayle ID mismatch: task file never touched");

	fresh( 0);
	mock_board = MOCK_BOARD_A1200;
	ide_board_force = 2;
	CHECK( idepresent( &pa) == 0, "A1200 forced without drive: absent");
	CHECK( mock_bad_accesses == 0, "A1200 forced without drive: no undecoded address touched");
}

static void
test_probe_force()
{
	char	*pa = 0;

	fresh( 1);
	ide_board_force = -1;
	CHECK( idepresent( &pa) == 0 && mock_accesses == 0, "force -1: never probes");

	fresh( 1);
	ide_board_force = 1;
	mock_vposr_value = 0x2000; mock_vposr_override = 1;	/* chipset says ECS */
	CHECK( idepresent( &pa) == 1 && pa == (char *)0xDD2020L, "force 1: A4000 without the chipset gate");
	CHECK( mock_id_reads == 0, "force 1: Gayle ID register not read for the A4000 row");

	fresh( 1);					/* an A4000, operator wrongly forces A1200 */
	ide_board_force = 2;
	CHECK( idepresent( &pa) == 0, "force 2 on an A4000: Gayle ID does not match, absent");
	CHECK( mock_id_reads == 8 && mock_bad_accesses == 0, "force 2 on an A4000: ID read, 0xDA0000 never touched");
}

/* ------------------------------------------------------- attach tests */

static void
test_attach_identify_intr()
{
	struct sdcom	cp;

	fresh( 1);
	tur( &cp, 6);					/* triggers attach */
	CHECK( cp.okay && cp.status == 0, "TUR GOOD");
	CHECK( mock_cmd_count[0xEC] == 1, "exactly one IDENTIFY on attach");
	CHECK( strstr( mock_console, "MOCK CF CARD 2GB") != 0, "model string decoded in order");
	CHECK( strstr( mock_console, "4096 sectors") != 0, "LBA sector count decoded (byte order)");
	CHECK( strstr( mock_console, "interrupt I/O") != 0, "attach chose interrupt mode (hook present)");
	CHECK( mock_nien == 0, "drive interrupts enabled after attach");
	CHECK( mock_slave_selects == 0, "slave never selected while disabled");
	CHECK( ide_intr_spurious <= 1, "at most the IDENTIFY's leftover interrupt");
	tur( &cp, 6);
	CHECK( mock_cmd_count[0xEC] == 1, "IDENTIFY not repeated per request");
}

static void
test_attach_polled_fallback()
{
	struct sdcom	cp;
	static char	buf[1024];

	fresh( 1);
	int2_tbl[0] = 0;				/* kernel not patched */
	tur( &cp, 6);
	CHECK( strstr( mock_console, "polled I/O") != 0, "no hook: attach chose polled mode");
	CHECK( mock_nien == 1, "polled: drive interrupts stay disabled");
	read10( &cp, 6, 3L, 2, buf, 1024);
	CHECK( cp.okay && cp.status == 0 && disk_equals( 0, 3L, buf, 1024), "polled READ(10)");
	CHECK( pumped == 0 && ide_intr_count == 0, "polled: no interrupts involved");
	CHECK( mock_timeout_calls == 0, "polled: no watchdog");

	fresh( 1);
	ide_intr_enable = 0;				/* tunable off */
	tur( &cp, 6);
	CHECK( strstr( mock_console, "polled I/O") != 0, "ide_intr_enable=0 forces polled mode");
}

static void
test_a1200_interrupt_plumbing()
{
	struct sdcom	cp;
	static char	buf[4 * 512];

	fresh( 1);
	mock_board = MOCK_BOARD_A1200;
	ide_board_force = 2;
	tur( &cp, 6);
	CHECK( (mock_intena & 0x80) != 0, "A1200: INTENA bit 7 set at attach");
	read10( &cp, 6, 10L, 4, buf, sizeof buf);
	CHECK( cp.okay && cp.status == 0 && disk_equals( 0, 10L, buf, sizeof buf), "A1200 READ(10) by interrupts");
	CHECK( ide_intr_count == 4, "A1200: one interrupt per sector");
	CHECK( mock_irq_acks == ide_intr_count, "A1200: every interrupt acknowledged at Gayle");
	CHECK( mock_irq_latch == 0, "A1200: latch clear afterwards");
	CHECK( mock_bad_accesses == 0, "A1200: no undecoded address touched");
}

/* ------------------------------------------------------ command tests */

static void
test_inquiry_and_capacity()
{
	struct sdcom	cp;
	unsigned char	inq[6] = { 0x12, 0, 0, 0, 36, 0 };
	unsigned char	cap[10] = { 0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	unsigned char	ms[6]  = { 0x1A, 0, 0x3F, 0, 12, 0 };
	unsigned char	buf[64];

	fresh( 1);
	memset( buf, 0xEE, sizeof buf);
	run( &cp, 6, inq, 6, (char *)buf, 36);
	CHECK( cp.okay && cp.status == 0, "INQUIRY GOOD");
	CHECK( buf[0] == 0 && buf[1] == 0 && buf[2] == 2 && buf[3] == 2 && buf[4] == 0x1F,
	       "INQUIRY header: disk, SCSI-2, format 2, 36 bytes");
	CHECK( memcmp( buf + 8, "AMIGA   ", 8) == 0, "INQUIRY vendor");
	CHECK( memcmp( buf + 16, "MOCK CF CARD 2GB", 16) == 0, "INQUIRY product = model");
	CHECK( buf[36] == 0xEE, "INQUIRY writes only nbyte");

	memset( buf, 0xEE, sizeof buf);
	run( &cp, 6, cap, 10, (char *)buf, 8);
	CHECK( cp.okay && cp.status == 0, "READ CAPACITY GOOD");
	CHECK( buf[0] == 0 && buf[1] == 0 && buf[2] == 0x0F && buf[3] == 0xFF, "READ CAPACITY last LBA 4095");
	CHECK( buf[4] == 0 && buf[5] == 0 && buf[6] == 2 && buf[7] == 0, "READ CAPACITY block size 512");
	CHECK( buf[8] == 0xEE, "READ CAPACITY writes 8 bytes");

	memset( buf, 0xEE, sizeof buf);
	run( &cp, 6, ms, 6, (char *)buf, 12);
	CHECK( cp.okay && cp.status == 0, "MODE SENSE(6) GOOD");
	CHECK( buf[0] == 11 && buf[3] == 8, "MODE SENSE(6) header");
	CHECK( buf[5] == 0 && buf[6] == 0x0F && buf[7] == 0xFF, "MODE SENSE(6) block count");
	CHECK( buf[10] == 2 && buf[11] == 0, "MODE SENSE(6) block size");
}

static void
test_read_single_and_multi()
{
	struct sdcom	cp;
	static char	buf[8192 + 16];
	unsigned long	before;

	fresh( 1);
	memset( buf, 0x55, sizeof buf);
	read10( &cp, 6, 0L, 1, buf, 512);
	CHECK( cp.okay && cp.status == 0, "READ(10) LBA 0 GOOD");
	CHECK( disk_equals( 0, 0L, buf, 512), "READ(10) LBA 0 data in natural byte order");
	CHECK( buf[512] == 0x55, "READ(10) does not overrun nbyte");
	CHECK( mock_cmd_count[0x20] == 1, "one READ SECTORS");
	CHECK( pumped == 1, "one sector = one interrupt");

	memset( buf, 0x55, sizeof buf);
	read10( &cp, 6, 100L, 16, buf, 8192);	/* UFS block */
	CHECK( cp.okay && cp.status == 0, "READ(10) 16 sectors GOOD");
	CHECK( disk_equals( 0, 100L, buf, 8192), "READ(10) 16 sectors data");
	CHECK( buf[8192] == 0x55, "READ(10) 16 sectors no overrun");
	CHECK( mock_cmd_count[0x20] == 2, "16 sectors in one ATA command");
	CHECK( pumped == 16, "16 sectors = 16 interrupts");

	/* nbyte shorter than the block count: the drive still drains */
	memset( buf, 0x55, sizeof buf);
	read10( &cp, 6, 7L, 3, buf, 1024);
	CHECK( cp.okay && cp.status == 0, "READ(10) short nbyte GOOD");
	CHECK( disk_equals( 0, 7L, buf, 1024), "READ(10) short nbyte data");
	CHECK( buf[1024] == 0x55, "READ(10) short nbyte stops at nbyte");
	before = mock_cmd_count[0x20];
	memset( buf, 0x55, 512);
	read10( &cp, 6, 8L, 1, buf, 512);
	CHECK( cp.okay && cp.status == 0 && disk_equals( 0, 8L, buf, 512),
	       "drive is idle again after a short read (phase drained)");
	CHECK( mock_cmd_count[0x20] == before + 1, "follow-up read issued normally");
}

static void
test_read_split_over_256()
{
	struct sdcom	cp;
	char		*buf = malloc( 300 * 512);

	fresh( 1);
	read10( &cp, 6, 500L, 300, buf, 300 * 512);
	CHECK( cp.okay && cp.status == 0, "READ(10) 300 sectors GOOD");
	CHECK( mock_cmd_count[0x20] == 2, "300 sectors split into two ATA commands");
	CHECK( disk_equals( 0, 500L, buf, 300 * 512), "300 sectors data contiguous");
	CHECK( pumped == 300, "300 interrupts for 300 sectors");
	free( buf);
}

static void
test_write()
{
	struct sdcom	cp;
	static char	buf[4 * 512], back[4 * 512];
	int		i;

	fresh( 1);
	for (i = 0; i < (int)sizeof buf; ++i)
		buf[i] = (char)(i * 13 + 1);
	write10( &cp, 6, 1000L, 4, buf, sizeof buf);
	CHECK( cp.okay && cp.status == 0, "WRITE(10) GOOD");
	CHECK( mock_cmd_count[0x30] == 1, "one WRITE SECTORS");
	CHECK( disk_equals( 0, 1000L, buf, sizeof buf), "WRITE(10) data landed in order");
	CHECK( pumped == 4, "4 sectors = 4 interrupts (one per accepted sector)");
	read10( &cp, 6, 1000L, 4, back, sizeof back);
	CHECK( memcmp( buf, back, sizeof buf) == 0, "read back what was written");
	CHECK( !disk_equals( 0, 999L, buf, 512) && !disk_equals( 0, 1004L, buf + 3 * 512, 512),
	       "neighbouring sectors untouched");

	write10( &cp, 6, 2000L, 1, buf, 512);
	CHECK( cp.okay && cp.status == 0 && disk_equals( 0, 2000L, buf, 512), "single-sector WRITE(10)");
	CHECK( pumped == 1, "single-sector write: one (final) interrupt");
}

static void
test_write_split_over_256()
{
	struct sdcom	cp;
	char		*buf = malloc( 300 * 512);
	int		i;

	fresh( 1);
	for (i = 0; i < 300 * 512; ++i)
		buf[i] = (char)(i * 7 + 3);
	write10( &cp, 6, 3000L, 300, buf, 300 * 512);
	CHECK( cp.okay && cp.status == 0, "WRITE(10) 300 sectors GOOD");
	CHECK( mock_cmd_count[0x30] == 2, "300 sectors split into two WRITE commands");
	CHECK( disk_equals( 0, 3000L, buf, 300 * 512), "300 sectors written contiguously");
	free( buf);
}

static void
test_sense_out_of_range()
{
	struct sdcom	cp;
	static char	buf[512];
	unsigned char	sense[18];

	fresh( 1);
	read10( &cp, 6, 4095L, 2, buf, 1024);
	CHECK( cp.okay, "out of range: command executed (okay)");
	CHECK( cp.status == 0x02, "out of range: CHECK CONDITION");
	CHECK( mock_cmd_count[0x20] == 0, "out of range rejected before touching the drive");
	reqsense( &cp, 6, sense);
	CHECK( cp.okay && cp.status == 0, "REQUEST SENSE GOOD");
	CHECK( sense[0] == 0x70 && sense[7] == 0x0A, "sense fixed format");
	CHECK( sense[2] == 0x05 && sense[12] == 0x21, "ILLEGAL REQUEST / LBA OUT OF RANGE");
	reqsense( &cp, 6, sense);
	CHECK( sense[2] == 0 && sense[12] == 0, "sense consumed on read");
}

static void
test_sense_media_error()
{
	struct sdcom	cp;
	static char	buf[4 * 512];
	unsigned char	sense[18];

	fresh( 1);
	mock_drv[0].unc_lba = 202;
	read10( &cp, 6, 200L, 4, buf, sizeof buf);
	CHECK( cp.okay && cp.status == 0x02, "UNC: CHECK CONDITION");
	reqsense( &cp, 6, sense);
	CHECK( sense[2] == 0x03 && sense[12] == 0x11, "MEDIUM ERROR / unrecovered read");
	mock_drv[0].unc_lba = -1;
	read10( &cp, 6, 200L, 4, buf, sizeof buf);
	CHECK( cp.okay && cp.status == 0 && disk_equals( 0, 200L, buf, sizeof buf),
	       "drive usable again after the error");

	mock_drv[0].unc_lba = 301;
	write10( &cp, 6, 300L, 4, buf, sizeof buf);
	CHECK( cp.okay && cp.status == 0x02, "UNC on write: CHECK CONDITION");
	reqsense( &cp, 6, sense);
	CHECK( sense[2] == 0x03 && sense[12] == 0x0C, "MEDIUM ERROR / write error");
	mock_drv[0].unc_lba = -1;
}

static void
test_unknown_opcode()
{
	struct sdcom	cp;
	unsigned char	cdb[6] = { 0xFF, 0, 0, 0, 0, 0 };
	unsigned char	sense[18];

	fresh( 1);
	run( &cp, 6, cdb, 6, 0, 0);
	CHECK( cp.okay && cp.status == 0x02, "unknown opcode: CHECK CONDITION");
	reqsense( &cp, 6, sense);
	CHECK( sense[2] == 0x05 && sense[12] == 0x20, "ILLEGAL REQUEST / invalid opcode");
}

static void
test_slave_disabled_and_enabled()
{
	struct sdcom	cp;
	static char	buf[512];
	unsigned char	cdb[6] = { 0, 0, 0, 0, 0, 0 };

	fresh( 2);					/* slave physically present */
	tur( &cp, 6);
	CHECK( mock_slave_selects == 0, "attach never selects the slave while disabled");
	run( &cp, 5, cdb, 6, 0, 0);
	CHECK( !cp.okay, "unit 5 (slave) disabled: okay == FALSE (no such device)");
	CHECK( mock_slave_selects == 0, "request to unit 5 does not select the slave");
	run( &cp, 7, cdb, 6, 0, 0);
	CHECK( !cp.okay, "unit 7: okay == FALSE");
	run( &cp, 0, cdb, 6, 0, 0);
	CHECK( !cp.okay, "unit 0: nothing mapped by default");
	ide_master_unit = 0;
	run( &cp, 0, cdb, 6, 0, 0);
	CHECK( cp.okay && cp.status == 0, "ide_master_unit = 0 remaps the master");
	ide_master_unit = 6;

	fresh( 2);
	ide_slave_enable = 1;
	run( &cp, 5, cdb, 6, 0, 0);
	CHECK( cp.okay && cp.status == 0, "slave enabled: TUR GOOD");
	CHECK( mock_cmd_count[0xEC] == 2, "both units identified");
	CHECK( strstr( mock_console, "MOCK SLAVE") != 0, "slave model logged");
	read10( &cp, 5, 10L, 1, buf, 512);
	CHECK( cp.okay && cp.status == 0 && disk_equals( 1, 10L, buf, 512), "slave READ(10)");

	fresh( 1);					/* enabled but absent */
	ide_slave_enable = 1;
	run( &cp, 5, cdb, 6, 0, 0);
	CHECK( !cp.okay, "slave enabled but absent: okay == FALSE");
	tur( &cp, 6);
	CHECK( cp.okay && cp.status == 0, "master unaffected by absent slave");
}

static void
test_polled_timeout_does_not_hang()
{
	struct sdcom	cp;
	static char	buf[512];
	unsigned char	sense[18];

	fresh( 1);
	int2_tbl[0] = 0;
	tur( &cp, 6);					/* attach while healthy, polled */
	mock_drv[0].stuck_bsy = 1;
	ide_spins = 1000;
	read10( &cp, 6, 0L, 1, buf, 512);
	CHECK( cp.okay && cp.status == 0x02, "stuck BSY (polled): CHECK CONDITION, returned");
	mock_drv[0].stuck_bsy = 0;
	reqsense( &cp, 6, sense);
	CHECK( sense[2] == 0x04 || sense[2] == 0x02, "stuck BSY: HARDWARE ERROR or NOT READY");
}

static void
test_watchdog()
{
	struct sdcom	cp;
	static char	buf[512];
	unsigned char	sense[18];
	unsigned long	i;

	fresh( 1);
	tur( &cp, 6);
	mock_drv[0].mute_intrq = 1;			/* the interrupt never comes */
	memset( &cp, 0, sizeof cp);
	cp.cdb[0] = 0x28; cp.cdb[5] = 40; cp.cdb[8] = 1;
	cp.addr = buf; cp.nbyte = 512; cp.unit = 6; cp.intr = done;
	intr_calls = 0;
	idequeue( 0, &cp);
	CHECK( intr_calls == 0, "interrupt-driven read is pending");
	CHECK( mock_timeouts_pending() == 1, "watchdog armed with the first request");
	for (i = 0; i < ide_timeout_secs && intr_calls == 0; ++i)
		mock_fire_timeouts();			/* one simulated second each */
	CHECK( intr_calls == 0, "watchdog quiet before the deadline");
	CHECK( mock_timeouts_pending() == 1, "watchdog re-armed while a request is active");
	mock_fire_timeouts();
	CHECK( intr_calls == 1, "watchdog completed the stuck request");
	CHECK( cp.okay && cp.status == 0x02, "timed-out request: CHECK CONDITION");
	CHECK( ide_intr_timeouts == 1, "timeout counted");
	CHECK( strstr( mock_console, "timed out") != 0, "timeout logged");
	CHECK( mock_timeouts_pending() == 0, "watchdog disarmed once idle");
	mock_drv[0].mute_intrq = 0;
	reqsense( &cp, 6, sense);
	CHECK( sense[2] == 0x04 && sense[12] == 0x08, "HARDWARE ERROR / LU communication failure");
	read10( &cp, 6, 41L, 1, buf, 512);
	CHECK( cp.okay && cp.status == 0 && disk_equals( 0, 41L, buf, 512), "drive usable after the timeout");
	CHECK( mock_spl_calls == mock_splx_calls && mock_ipl == 0, "spl balanced after watchdog");
}

/*
 * A drive that hangs mid-command is reset (SRST) when the watchdog gives up,
 * so the abandoned command cannot complete into the next request; a stale
 * interrupt afterwards is discarded; a drive that stays busy even after the
 * reset fails every request without hanging.
 */
static void
test_timeout_recovery()
{
	struct sdcom	cp;
	static char	buf[512];
	unsigned char	sense[18];

	fresh( 1);
	tur( &cp, 6);
	CHECK( mock_nien == 0, "interrupt mode: drive interrupts enabled");
	mock_drv[0].hang_on_cmd = 1;
	ide_timeout_secs = 0;
	memset( &cp, 0, sizeof cp);
	cp.cdb[0] = 0x28; cp.cdb[5] = 100; cp.cdb[8] = 1;
	cp.addr = buf; cp.nbyte = 512; cp.unit = 6; cp.intr = done;
	intr_calls = 0;
	idequeue( 0, &cp);
	pump();
	CHECK( intr_calls == 0 && mock_drv[0].hung == 1, "hung drive: command accepted, then nothing completes");
	mock_fire_timeouts();
	CHECK( intr_calls == 1 && cp.okay && cp.status == 0x02, "hung drive: watchdog failed the request");
	CHECK( ide_intr_timeouts == 1 && ide_resets == 1, "hung drive: timeout and reset counted");
	CHECK( mock_srst_count == 1, "hung drive: SRST reached the drive");
	CHECK( mock_drv[0].hung == 0, "hung drive: the reset un-hung it");
	CHECK( mock_intrq == 0, "after reset: no INTRQ pending");
	CHECK( mock_nien == 0, "after reset: interrupts re-enabled at the drive");
	CHECK( mock_timeouts_pending() == 0, "after reset: watchdog disarmed");
	reqsense( &cp, 6, sense);
	CHECK( sense[2] == 0x04 && sense[12] == 0x08, "after reset: HARDWARE ERROR / LU communication failure");
	read10( &cp, 6, 101L, 1, buf, 512);
	CHECK( cp.okay && cp.status == 0 && disk_equals( 0, 101L, buf, 512), "next request on the reset drive: GOOD, right data");

	/* a stale interrupt with nothing active is discarded */
	intr_calls = 0;
	mock_intrq = 1;
	pump();
	CHECK( intr_calls == 0 && ide_intr_spurious == 1, "stale INTRQ, no request active: counted as spurious, nothing completed");
	read10( &cp, 6, 102L, 1, buf, 512);
	CHECK( cp.okay && cp.status == 0 && disk_equals( 0, 102L, buf, 512), "request after the stale interrupt: GOOD");

	/* a drive that stays busy through the reset: every request fails, nothing hangs */
	mock_drv[0].stuck_bsy = 1;
	ide_spins = 1000;
	memset( &cp, 0, sizeof cp);
	cp.cdb[0] = 0x28; cp.cdb[5] = 103; cp.cdb[8] = 1;
	cp.addr = buf; cp.nbyte = 512; cp.unit = 6; cp.intr = done;
	intr_calls = 0;
	idequeue( 0, &cp);
	CHECK( intr_calls == 1 && cp.okay && cp.status == 0x02, "stuck after reset: request fails at once (NOT READY), no hang");
	CHECK( mock_spl_calls == mock_splx_calls && mock_ipl == 0, "spl balanced after stuck drive");
	mock_drv[0].stuck_bsy = 0;
	ide_spins = 200000;
	read10( &cp, 6, 104L, 1, buf, 512);
	CHECK( cp.okay && cp.status == 0, "drive back: requests succeed again");
	ide_timeout_secs = 20;

	/* polled fallback: a spin timeout resets the drive too */
	fresh( 1);
	int2_tbl[0] = 0;
	tur( &cp, 6);
	mock_drv[0].hang_on_cmd = 1;
	ide_spins = 1000;
	read10( &cp, 6, 105L, 1, buf, 512);
	CHECK( cp.okay && cp.status == 0x02, "polled, hung drive: CHECK CONDITION");
	CHECK( mock_srst_count == 1 && ide_resets == 1 && mock_drv[0].hung == 0, "polled, hung drive: reset issued, drive un-hung");
	CHECK( mock_nien == 1, "polled: interrupts stay disabled after the reset");
	ide_spins = 200000;
	read10( &cp, 6, 106L, 1, buf, 512);
	CHECK( cp.okay && cp.status == 0 && disk_equals( 0, 106L, buf, 512), "polled: next request GOOD");
}

static void
test_nolba_drive_ignored()
{
	struct sdcom	cp;

	fresh( 1);
	mock_drv[0].nolba = 1;
	tur( &cp, 6);
	CHECK( !cp.okay, "CHS-only drive is not attached");
	CHECK( strstr( mock_console, "without LBA") != 0, "CHS-only drive is reported");
}

static void
test_sync_cache()
{
	struct sdcom	cp;
	unsigned char	cdb[10] = { 0x35, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	fresh( 1);
	run( &cp, 6, cdb, 10, 0, 0);
	CHECK( cp.okay && cp.status == 0, "SYNC CACHE GOOD on a flush-capable drive");
	CHECK( mock_cmd_count[0xE7] == 1, "FLUSH CACHE issued");

	fresh( 1);
	mock_drv[0].flush = 0;
	run( &cp, 6, cdb, 10, 0, 0);
	CHECK( cp.okay && cp.status == 0, "SYNC CACHE GOOD on an old drive");
	CHECK( mock_cmd_count[0xE7] == 0, "no FLUSH CACHE on a drive that lacks it");
}

/* the small correctness fixes: DF without ERR, LBA wrap, LUN, FIFO overflow */
static int	flood_calls;
static struct sdcom	flood[40];

static void
flood_done( cp)
struct sdcom	*cp;
{
	int	i;

	flood_calls++;
	if (cp == &flood[0])			/* from inside one completion, submit 39 more */
		for (i = 1; i < 40; ++i) {
			memset( &flood[i], 0, sizeof flood[i]);
			flood[i].unit = 6; flood[i].intr = flood_done;	/* TEST UNIT READY */
			idequeue( 0, &flood[i]);
		}
}

static void
test_small_fixes()
{
	struct sdcom	cp;
	unsigned char	sense[18];
	unsigned char	inq[36];
	unsigned char	cdb[10];
	static char	buf[512];

	/* DF alone: HARDWARE ERROR / internal failure, whatever the stale ERROR register says */
	fresh( 1);
	tur( &cp, 6);
	mock_drv[0].unc_lba = 10;
	read10( &cp, 6, 10L, 1, buf, 512);		/* leaves ERROR = UNC */
	CHECK( cp.okay && cp.status == 0x02, "UNC first: CHECK");
	reqsense( &cp, 6, sense);
	mock_drv[0].unc_lba = -1;
	mock_drv[0].df_lba = 11;
	read10( &cp, 6, 11L, 1, buf, 512);
	CHECK( cp.okay && cp.status == 0x02, "DF without ERR: CHECK");
	reqsense( &cp, 6, sense);
	CHECK( sense[2] == 0x04 && sense[12] == 0x44, "DF without ERR: HARDWARE ERROR / internal failure, not the stale UNC");
	mock_drv[0].df_lba = -1;

	/* LBA + count must not wrap past the end */
	read10( &cp, 6, 0xFFFFFFF0L, 32, buf, 512);
	CHECK( cp.okay && cp.status == 0x02 && mock_cmd_count[0x20] == 2, "LBA near 2^32 + count: rejected before the drive");
	reqsense( &cp, 6, sense);
	CHECK( sense[2] == 0x05 && sense[12] == 0x21, "LBA wrap: ILLEGAL REQUEST / LBA out of range");
	read10( &cp, 6, 4095L, 1, buf, 512);
	CHECK( cp.okay && cp.status == 0 && disk_equals( 0, 4095L, buf, 512), "last sector still readable");
	memset( cdb, 0, sizeof cdb); cdb[0] = 0x2F; cdb[2] = 0xFF; cdb[3] = 0xFF; cdb[4] = 0xFF; cdb[5] = 0xF0; cdb[8] = 32;
	run( &cp, 6, cdb, 10, 0, 0);
	CHECK( cp.okay && cp.status == 0x02 && mock_cmd_count[0x40] == 0, "VERIFY LBA wrap: rejected before the drive");

	/* a LUN other than 0 */
	memset( cdb, 0, sizeof cdb); cdb[0] = 0x28; cdb[1] = 0x20; cdb[5] = 10; cdb[8] = 1;
	run( &cp, 6, cdb, 10, buf, 512);
	CHECK( cp.okay && cp.status == 0x02, "READ to LUN 1: CHECK");
	reqsense( &cp, 6, sense);
	CHECK( sense[2] == 0x05 && sense[12] == 0x25, "READ to LUN 1: ILLEGAL REQUEST / LUN not supported");
	memset( cdb, 0, sizeof cdb); cdb[0] = 0x12; cdb[1] = 0x20; cdb[4] = 36;
	memset( inq, 0xEE, sizeof inq);
	run( &cp, 6, cdb, 6, (char *)inq, 36);
	CHECK( cp.okay && cp.status == 0 && inq[0] == 0x7F && inq[8] == 0, "INQUIRY to LUN 1: GOOD, qualifier 011b / type 1Fh, no vendor text");
	memset( cdb, 0, sizeof cdb); cdb[0] = 0x12; cdb[4] = 36;
	run( &cp, 6, cdb, 6, (char *)inq, 36);
	CHECK( cp.okay && cp.status == 0 && inq[0] == 0x00, "INQUIRY to LUN 0: direct-access device");

	/* completion FIFO full: nothing is lost, delivered nested instead */
	flood_calls = 0;
	memset( &flood[0], 0, sizeof flood[0]);
	flood[0].unit = 6; flood[0].intr = flood_done;
	idequeue( 0, &flood[0]);
	CHECK( flood_calls == 40, "40 synchronous completions from inside one callback: all delivered");
	CHECK( ide_cq_overflow > 0, "FIFO overflow counted");
	CHECK( strstr( mock_console, "completion queue full") != 0, "FIFO overflow logged");
	CHECK( mock_spl_calls == mock_splx_calls && mock_ipl == 0, "spl balanced after the flood");
	tur( &cp, 6);
	CHECK( cp.okay && cp.status == 0, "driver usable after the flood");
	CHECK( ide_intr_busy == 0, "no BSY interrupts seen in these cases");
	ide_cq_overflow = 0;				/* the flood was deliberate; the global check stays strict */
}

/* a raw request (<= one page) that straddles a page boundary is counted */
static void
test_pagecross_counter()
{
	struct sdcom	cp;
	static char	big[6 * PG];
	char		*p;

	fresh( 1);
	tur( &cp, 6);
	p = big + (PG - ((unsigned long)big & (PG - 1))) % PG;	/* first page-aligned byte */
	read10( &cp, 6, 10L, 2, p, 1024);
	CHECK( cp.okay && cp.status == 0 && ide_pagecross_n == 0, "aligned 1 KB request: not counted");
	read10( &cp, 6, 12L, 1, p + PG - 256, 512);
	CHECK( cp.okay && cp.status == 0 && disk_equals( 0, 12L, p + PG - 256, 512), "straddling request still served");
	CHECK( ide_pagecross_n == 1, "512 B request straddling a page boundary: counted");
	CHECK( strstr( mock_console, "crosses a page boundary") != 0, "first crossing logged");
	read10( &cp, 6, 14L, PG / 512, p + PG - 512, PG);
	CHECK( ide_pagecross_n == 2, "one-page request straddling a page boundary: counted");
	read10( &cp, 6, 20L, 4 * PG / 512, p, 4 * PG);
	CHECK( cp.okay && cp.status == 0 && ide_pagecross_n == 2, "multi-page buffer-cache block: spans pages by nature, not counted");
	read10( &cp, 6, 40L, PG / 512, p + PG, PG);
	CHECK( ide_pagecross_n == 2, "page-aligned full page: not counted");
}

static void
test_read6()
{
	struct sdcom	cp;
	static char	buf[2 * 512];
	unsigned char	cdb[6] = { 0x08, 0x00, 0x01, 0x2C, 2, 0 };	/* LBA 300, 2 blocks */

	fresh( 1);
	run( &cp, 6, cdb, 6, buf, sizeof buf);
	CHECK( cp.okay && cp.status == 0 && disk_equals( 0, 300L, buf, sizeof buf), "READ(6)");
}

/* queueing: several requests submitted before any completes */
static struct sdcom	qreq[6];
static char		qbuf[6][1024];
static int		qorder[6], qn;

static void
qdone( cp)
struct sdcom	*cp;
{
	intr_calls++;
	qorder[qn++] = (int)(cp - qreq);
}

static void
test_queue_order()
{
	int	i;

	fresh( 1);
	{ struct sdcom cp; tur( &cp, 6); }
	intr_calls = 0; qn = 0;
	for (i = 0; i < 6; ++i) {
		memset( &qreq[i], 0, sizeof qreq[i]);
		qreq[i].cdb[0] = (i & 1) ? 0x2A : 0x28;
		qreq[i].cdb[5] = (unsigned char)(50 + 2 * i); qreq[i].cdb[8] = 2;
		qreq[i].addr = qbuf[i]; qreq[i].nbyte = 1024; qreq[i].unit = 6; qreq[i].intr = qdone;
		memset( qbuf[i], (i & 1) ? 0xA0 + i : 0, 1024);
		idequeue( 0, &qreq[i]);
	}
	CHECK( intr_calls == 0, "six requests queued, none complete without interrupts");
	pump();
	CHECK( intr_calls == 6, "all six completed by interrupts");
	for (i = 0; i < 6; ++i) {
		CHECK( qorder[i] == i, "requests complete in submission order");
		CHECK( qreq[i].okay && qreq[i].status == 0, "queued request GOOD");
		CHECK( disk_equals( 0, 50L + 2 * i, qbuf[i], 1024), "queued request data");
	}
	CHECK( mock_spl_calls == mock_splx_calls && mock_ipl == 0, "spl balanced after queue drain");
}

static void
verify10( cp, unit, lba, nblk, bytchk)
struct sdcom	*cp;
int		unit, nblk, bytchk;
unsigned long	lba;
{
	unsigned char	cdb[10];

	memset( cdb, 0, sizeof cdb);
	cdb[0] = 0x2F; cdb[1] = bytchk ? 0x02 : 0;
	cdb[2] = lba >> 24; cdb[3] = lba >> 16; cdb[4] = lba >> 8; cdb[5] = lba;
	cdb[7] = nblk >> 8; cdb[8] = nblk;
	run( cp, unit, cdb, 10, 0, 0);
}

/* VERIFY(10) is executed as READ VERIFY SECTOR(S), not answered blindly */
static void
test_verify()
{
	struct sdcom	cp;
	unsigned char	sense[18];
	static char	buf[512];
	int		i;

	fresh( 1);
	tur( &cp, 6);
	verify10( &cp, 6, 100L, 4, 0);
	CHECK( cp.okay && cp.status == 0, "VERIFY 4 sectors: GOOD");
	CHECK( mock_cmd_count[0x40] == 1 && mock_cmd_count[0x20] == 0, "VERIFY: one READ VERIFY, no READ");
	verify10( &cp, 6, 200L, 300, 0);
	CHECK( cp.okay && cp.status == 0 && mock_cmd_count[0x40] == 3, "VERIFY 300 sectors: split into two commands");
	verify10( &cp, 6, 4090L, 6, 0);
	CHECK( cp.okay && cp.status == 0, "VERIFY up to the last sector: GOOD");
	verify10( &cp, 6, 4090L, 7, 0);
	CHECK( cp.okay && cp.status == 0x02 && mock_cmd_count[0x40] == 4, "VERIFY past the end: CHECK, rejected before the drive");
	reqsense( &cp, 6, sense);
	CHECK( sense[2] == 0x05 && sense[12] == 0x21, "VERIFY past the end: ILLEGAL REQUEST / LBA out of range");
	verify10( &cp, 6, 10L, 1, 1);
	CHECK( cp.okay && cp.status == 0x02, "VERIFY with BYTCHK: CHECK");
	reqsense( &cp, 6, sense);
	CHECK( sense[2] == 0x05 && sense[12] == 0x24, "VERIFY with BYTCHK: invalid field in CDB");
	verify10( &cp, 6, 10L, 0, 0);
	CHECK( cp.okay && cp.status == 0 && mock_cmd_count[0x40] == 4, "VERIFY of zero blocks: GOOD, no command");
	mock_drv[0].unc_lba = 302;
	verify10( &cp, 6, 300L, 4, 0);
	CHECK( cp.okay && cp.status == 0x02, "VERIFY over a bad sector: CHECK");
	reqsense( &cp, 6, sense);
	CHECK( sense[2] == 0x03 && sense[12] == 0x11, "VERIFY over a bad sector: MEDIUM ERROR / read error");
	mock_drv[0].unc_lba = -1;

	/* queued behind a READ: the drive sees READ, then READ VERIFY */
	intr_calls = 0; qn = 0;
	memset( qreq, 0, sizeof qreq);
	qreq[0].cdb[0] = 0x28; qreq[0].cdb[5] = 30; qreq[0].cdb[8] = 2;
	qreq[0].addr = qbuf[0]; qreq[0].nbyte = 1024;
	qreq[1].cdb[0] = 0x2F; qreq[1].cdb[5] = 30; qreq[1].cdb[8] = 2;
	for (i = 0; i < 2; ++i) { qreq[i].unit = 6; qreq[i].intr = qdone; }
	mock_cmd_log_n = 0;
	idequeue( 0, &qreq[0]);
	idequeue( 0, &qreq[1]);
	pump();
	CHECK( intr_calls == 2 && qorder[0] == 0 && qorder[1] == 1, "VERIFY queued behind a READ: completes after it");
	CHECK( mock_cmd_log_n == 2 && mock_cmd_log[0] == 0x20 && mock_cmd_log[1] == 0x40, "drive saw READ then READ VERIFY");
	CHECK( qreq[1].okay && qreq[1].status == 0 && disk_equals( 0, 30L, qbuf[0], 1024), "both GOOD, READ data intact");

	/* polled fallback */
	fresh( 1);
	int2_tbl[0] = 0;
	tur( &cp, 6);
	verify10( &cp, 6, 50L, 3, 0);
	CHECK( cp.okay && cp.status == 0 && mock_cmd_count[0x40] == 1, "VERIFY polled: GOOD, one command");
	read10( &cp, 6, 50L, 1, buf, 512);
	CHECK( cp.okay && cp.status == 0 && disk_equals( 0, 50L, buf, 512), "polled READ after VERIFY: GOOD");
}

/*
 * SYNCHRONIZE CACHE in interrupt mode takes the queue: submitted behind a
 * READ in flight and ahead of a WRITE, the FLUSH CACHE reaches the drive
 * after the READ has finished and before the WRITE starts.
 */
static void
test_sync_cache_queued()
{
	unsigned char	sync[10];
	int		i;

	fresh( 1);
	{ struct sdcom cp; tur( &cp, 6); }
	intr_calls = 0; qn = 0;
	memset( sync, 0, sizeof sync); sync[0] = 0x35;
	memset( qreq, 0, sizeof qreq);
	qreq[0].cdb[0] = 0x28; qreq[0].cdb[5] = 90; qreq[0].cdb[8] = 2;
	qreq[0].addr = qbuf[0]; qreq[0].nbyte = 1024;
	memcpy( qreq[1].cdb, sync, 10);
	qreq[2].cdb[0] = 0x2A; qreq[2].cdb[5] = 94; qreq[2].cdb[8] = 2;
	qreq[2].addr = qbuf[2]; qreq[2].nbyte = 1024;
	memset( qbuf[2], 0x3C, 1024);
	for (i = 0; i < 3; ++i) { qreq[i].unit = 6; qreq[i].intr = qdone; }
	mock_cmd_log_n = 0;
	idequeue( 0, &qreq[0]);
	idequeue( 0, &qreq[1]);
	idequeue( 0, &qreq[2]);
	CHECK( intr_calls == 0, "sync queued: nothing completes without interrupts");
	CHECK( mock_cmd_count[0xE7] == 0, "sync queued: FLUSH not issued while the READ is on the cable");
	pump();
	CHECK( intr_calls == 3, "sync queued: READ, SYNC, WRITE all completed");
	CHECK( qorder[0] == 0 && qorder[1] == 1 && qorder[2] == 2, "sync queued: completion order READ, SYNC, WRITE");
	CHECK( mock_cmd_log_n == 3 && mock_cmd_log[0] == 0x20 && mock_cmd_log[1] == 0xE7 && mock_cmd_log[2] == 0x30,
	       "sync queued: drive saw READ, FLUSH CACHE, WRITE in that order");
	CHECK( qreq[1].okay && qreq[1].status == 0, "sync queued: SYNC GOOD");
	CHECK( qreq[0].okay && qreq[0].status == 0 && disk_equals( 0, 90L, qbuf[0], 1024), "sync queued: READ data intact");
	CHECK( qreq[2].okay && qreq[2].status == 0 && disk_equals( 0, 94L, qbuf[2], 1024), "sync queued: WRITE landed");
	mock_fire_timeouts();				/* the armed watchdog tick finds nothing active */
	CHECK( intr_calls == 3 && mock_timeouts_pending() == 0, "sync queued: watchdog tick idle, disarmed");
	CHECK( mock_spl_calls == mock_splx_calls && mock_ipl == 0, "spl balanced after queued sync");

	/* a drive without FLUSH CACHE: GOOD, no command, still through the queue */
	fresh( 0);
	mock_add_drive( 0, 4096L, "OLD DRIVE NO FLUSH", 0);
	{ struct sdcom cp; tur( &cp, 6); run( &cp, 6, sync, 10, 0, 0);
	  CHECK( cp.okay && cp.status == 0, "sync, no FLUSH support (interrupt mode): GOOD");
	  CHECK( mock_cmd_count[0xE7] == 0, "sync, no FLUSH support: no command issued"); }

	/* the FLUSH itself can time out like any queued command */
	fresh( 1);
	{ struct sdcom cp; unsigned char sense[18];
	  tur( &cp, 6);
	  mock_drv[0].mute_intrq = 1;
	  ide_timeout_secs = 0;
	  memset( &cp, 0, sizeof cp); memcpy( cp.cdb, sync, 10); cp.unit = 6; cp.intr = done;
	  intr_calls = 0;
	  idequeue( 0, &cp);
	  pump();
	  CHECK( intr_calls == 0 && mock_cmd_count[0xE7] == 1, "sync timeout: FLUSH issued, no interrupt yet");
	  mock_fire_timeouts();
	  CHECK( intr_calls == 1 && cp.okay && cp.status == 0x02, "sync timeout: watchdog completed it with CHECK CONDITION");
	  mock_drv[0].mute_intrq = 0;
	  ide_timeout_secs = 20;
	  reqsense( &cp, 6, sense);
	  CHECK( sense[2] == 0x04, "sync timeout: HARDWARE ERROR sense"); }
}

/*
 * dd.c's completion order (iodone before the trailing startio) can submit
 * the disk's one sdcom a second time while the first submission is still on
 * the cable or queued.  The duplicate must be ignored, not linked in again.
 */
static struct sdcom	persist;
static char		persistbuf[2][1024];

static void
persist_done( cp)
struct sdcom	*cp;
{
	intr_calls++;
	intr_last = cp;
}

static void
test_persistent_sdcom_resubmit()
{
	struct sdcom	other;
	static char	obuf[512];
	int		i;

	fresh( 1);
	{ struct sdcom cp; tur( &cp, 6); }
	intr_calls = 0; ide_dup_submits = 0;

	/* 1. active: the same sdcom submitted again while its READ is on the cable */
	memset( &persist, 0, sizeof persist);
	persist.cdb[0] = 0x28; persist.cdb[5] = 60; persist.cdb[8] = 2;
	persist.addr = persistbuf[0]; persist.nbyte = 1024; persist.unit = 6; persist.intr = persist_done;
	idequeue( 0, &persist);
	idequeue( 0, &persist);				/* duplicate while active */
	CHECK( ide_dup_submits == 1, "resubmit while active: counted");
	CHECK( intr_calls == 0, "resubmit while active: no completion yet");
	pump();
	CHECK( intr_calls == 1, "resubmit while active: completed exactly once");
	CHECK( persist.okay && persist.status == 0 && disk_equals( 0, 60L, persistbuf[0], 1024),
	       "resubmit while active: data intact");

	/* 2. queued: behind another request, submitted twice before anything runs */
	intr_calls = 0; ide_dup_submits = 0;
	memset( &other, 0, sizeof other);
	other.cdb[0] = 0x28; other.cdb[5] = 70; other.cdb[8] = 1;
	other.addr = obuf; other.nbyte = 512; other.unit = 6; other.intr = persist_done;
	memset( &persist, 0, sizeof persist);
	persist.cdb[0] = 0x2A; persist.cdb[5] = 72; persist.cdb[8] = 2;	/* a WRITE: replaying it would be worst */
	persist.addr = persistbuf[1]; persist.nbyte = 1024; persist.unit = 6; persist.intr = persist_done;
	memset( persistbuf[1], 0x5A, 1024);
	idequeue( 0, &other);
	idequeue( 0, &persist);
	idequeue( 0, &persist);				/* duplicate while queued */
	idequeue( 0, &persist);				/* and again */
	CHECK( ide_dup_submits == 2, "resubmit while queued: counted");
	pump();
	CHECK( intr_calls == 2, "resubmit while queued: both requests completed once each");
	CHECK( persist.okay && persist.status == 0 && disk_equals( 0, 72L, persistbuf[1], 1024),
	       "resubmit while queued: write landed once");
	CHECK( mock_cmd_count[0x30] == 1, "resubmit while queued: WRITE issued to the drive once");
	CHECK( mock_spl_calls == mock_splx_calls && mock_ipl == 0, "spl balanced after resubmits");

	/* 3. after completion the same sdcom is a new request and must be honoured */
	intr_calls = 0; ide_dup_submits = 0;
	for (i = 0; i < 3; ++i) {
		persist.cdb[0] = 0x28; persist.cdb[5] = (unsigned char)(80 + i); persist.cdb[8] = 1;
		persist.addr = persistbuf[0]; persist.nbyte = 512; persist.intr = persist_done;
		idequeue( 0, &persist);
		pump();
	}
	CHECK( intr_calls == 3 && ide_dup_submits == 0, "sequential reuse of one sdcom: three completions, no dup");
	CHECK( disk_equals( 0, 82L, persistbuf[0], 512), "sequential reuse: last read correct");
}

/* re-entrancy: dd.c starts the next request from inside the completion */
static struct sdcom	chain[4];
static char		chainbuf[4][512];
static int		chain_next;

static void
chain_done( cp)
struct sdcom	*cp;
{
	int	i = (int)(cp - chain);

	intr_calls++;
	intr_depth++;
	if (intr_depth > intr_maxdepth)
		intr_maxdepth = intr_depth;
	if (i + 1 < chain_next) {
		struct sdcom	*n = &chain[i + 1];
		memset( n->cdb, 0, 12);
		n->cdb[0] = 0x28; n->cdb[5] = (unsigned char)(20 + i + 1); n->cdb[8] = 1;
		n->addr = chainbuf[i + 1]; n->nbyte = 512; n->unit = 6; n->intr = chain_done;
		idequeue( 0, n);
	}
	intr_depth--;
}

static void
test_reentrant_completion()
{
	int	i;

	fresh( 1);
	{ struct sdcom cp; tur( &cp, 6); }
	chain_next = 4;
	memset( chain, 0, sizeof chain);
	intr_calls = 0; intr_maxdepth = 0;
	chain[0].cdb[0] = 0x28; chain[0].cdb[5] = 20; chain[0].cdb[8] = 1;
	chain[0].addr = chainbuf[0]; chain[0].nbyte = 512; chain[0].unit = 6; chain[0].intr = chain_done;
	idequeue( 0, &chain[0]);
	pump();
	CHECK( intr_calls == 4, "all chained requests completed");
	CHECK( intr_maxdepth == 1, "completions delivered iteratively, not nested");
	for (i = 0; i < 4; ++i)
		CHECK( chain[i].okay && chain[i].status == 0 && disk_equals( 0, 20L + i, chainbuf[i], 512),
		       "chained request data");
	CHECK( ide_cq_overflow == 0, "completion FIFO never overflowed");
	CHECK( mock_spl_calls == mock_splx_calls && mock_ipl == 0, "spl balanced after chain");
}

static void
test_intr_not_mine()
{
	struct sdcom	cp;

	fresh( 1);
	ideintr();
	CHECK( mock_accesses == 0, "ideintr before probe touches nothing");
	tur( &cp, 6);
	mock_accesses = 0;
	ideintr();					/* INT2 from another device: no INTRQ */
	CHECK( mock_accesses == 1, "ideintr with no pending IDE interrupt reads only the Gayle status");
	CHECK( ide_intr_count == 0 && ide_intr_spurious == 0, "not-mine interrupt not counted");
}

#if IDE_DIAG
/* the spin budgets' high-water mark records what a successful wait needed */
static void
test_spins_hw()
{
	struct sdcom	cp;
	static char	buf[512];

	fresh( 1);
	tur( &cp, 6);
	ide_spins_hw = 0;
	mock_bsy_reads = 50;
	read10( &cp, 6, 3L, 1, buf, sizeof buf);
	CHECK( cp.okay && cp.status == 0 && disk_equals( 0, 3L, buf, sizeof buf), "read behind a slow BSY served");
	CHECK( ide_spins_hw == 50, "high-water = the BSY reads the wait sat through");
	mock_bsy_reads = 10;
	read10( &cp, 6, 4L, 1, buf, sizeof buf);
	CHECK( ide_spins_hw == 50, "a shorter wait leaves the high-water alone");
}
#endif	/* IDE_DIAG */

/* the attach banner's MB figure (computed without a shift by 11) */
static void
test_banner_mb()
{
	struct sdcom	cp;

	fresh( 1);
	tur( &cp, 6);
	CHECK( strstr( mock_console, "4096 sectors (2 MB)") != 0, "banner: 4096 sectors = 2 MB");
}

#if IDE_CACHE
/*
 * Every 16-byte line covering [lo, lo + len) saw a CPUSHL and then a CINVL,
 * both with a sequence number in (after, before).
 */
static int
lines_done( lo, len, after, before)
unsigned long	lo, len, after, before;
{
	unsigned long	a;
	int		i, pushed, ok;

	for (a = lo & ~15UL; a < ((lo + len + 15) & ~15UL); a += 16) {
		ok = pushed = 0;
		for (i = 0; i < mock_nlineops && !ok; ++i) {
			if (mock_lineops[i].addr != a || mock_lineops[i].seq <= after
			    || mock_lineops[i].seq >= before)
				continue;
			if (mock_lineops[i].op == MOCK_CPUSHL)
				pushed = 1;
			else if (pushed)
				ok = 1;
		}
		if (!ok)
			return 0;
	}
	return 1;
}

/* No line operation outside the rounded range. */
static int
lines_only( lo, len)
unsigned long	lo, len;
{
	int	i;

	for (i = 0; i < mock_nlineops; ++i)
		if (mock_lineops[i].addr < (lo & ~15UL)
		    || mock_lineops[i].addr >= ((lo + len + 15) & ~15UL)
		    || (mock_lineops[i].addr & 15))
			return 0;
	return mock_lineops_lost == 0;
}

static void
test_cache_spin_scale()
{
	/* before any fresh(): the compiled-in defaults */
	CHECK( ide_spins == 8000000UL && ide_spins_reset == 32000000UL && ide_spins_short == 800000UL,
	       "040/060 build scales the spin budgets by 4");
}

static void
test_cache_ci_check()
{
	struct sdcom	cp;

	fresh( 1);
	tur( &cp, 6);
	CHECK( ide_ci_ok == 1, "shipped DTT0 covers the task file cache-inhibited");
	CHECK( strstr( mock_console, "NOT cache-inhibited") == 0, "no warning on the shipped TTRs");

	fresh( 1);
	mock_dtt0 = 0x003fc020L;			/* same window, copyback */
	tur( &cp, 6);
	CHECK( ide_ci_ok == 2, "a cached DTT0 fails the check");
	CHECK( strstr( mock_console, "NOT cache-inhibited") != 0, "and says so");
	CHECK( cp.okay && cp.status == 0, "the driver carries on");

	fresh( 1);
	mock_dtt0 = 0;					/* disabled; DTT1 covers 0x80.. only */
	tur( &cp, 6);
	CHECK( ide_ci_ok == 2, "an address no TTR covers fails the check");

	fresh( 1);
	mock_dtt0 = 0x003f8060L;			/* user-only TTR */
	tur( &cp, 6);
	CHECK( ide_ci_ok == 2, "a user-only TTR does not cover supervisor data");

	fresh( 1);
	mock_dtt0 = 0x003fc040L;			/* cache-inhibited, serialized */
	tur( &cp, 6);
	CHECK( ide_ci_ok == 1, "CI-serialized passes");
}

/* READ: lines pushed before the first DATA access and again after the last */
static void
cache_read_case( polled)
int	polled;
{
	struct sdcom	cp;
	static char	buf[4 * 512 + 64];
	char		*p = buf + 5;			/* unaligned both ends */
	unsigned long	n0;

	fresh( 1);
	if (polled)
		int2_tbl[0] = 0;
	tur( &cp, 6);
	mock_nlineops = 0;
	mock_data_first = mock_data_last = 0;
#if IDE_DIAG
	n0 = ide_push_n;
#endif
	read10( &cp, 6, 50L, 3, p, 3 * 512);
	CHECK( cp.okay && cp.status == 0 && disk_equals( 0, 50L, p, 3 * 512), "cache build: read data intact");
	CHECK( lines_done( (unsigned long)p, 3 * 512UL, 0UL, mock_data_first), "read: every line pushed+invalidated before the transfer");
	CHECK( lines_done( (unsigned long)p, 3 * 512UL, mock_data_last, ~0UL), "read: and again after it");
	CHECK( lines_only( (unsigned long)p, 3 * 512UL), "read: no line outside the buffer's range");
#if IDE_DIAG
	CHECK( ide_push_n - n0 == 2, "read: two passes");
#endif
}

static void
test_cache_read()
{
	cache_read_case( 0);
	cache_read_case( 1);
}

/* WRITE: lines pushed before the first DATA access; nothing after */
static void
cache_write_case( polled)
int	polled;
{
	struct sdcom	cp;
	static char	buf[2 * 512 + 32];
	char		*p = buf + 7;
	unsigned long	n0;
	int		i;

	fresh( 1);
	if (polled)
		int2_tbl[0] = 0;
	tur( &cp, 6);
	for (i = 0; i < 2 * 512; ++i)
		p[i] = (char)(i * 7 + polled);
	mock_nlineops = 0;
	mock_data_first = mock_data_last = 0;
#if IDE_DIAG
	n0 = ide_push_n;
#endif
	write10( &cp, 6, 60L, 2, p, 2 * 512);
	CHECK( cp.okay && cp.status == 0 && disk_equals( 0, 60L, p, 2 * 512), "cache build: write landed");
	CHECK( lines_done( (unsigned long)p, 2 * 512UL, 0UL, mock_data_first), "write: every line pushed+invalidated before the transfer");
	CHECK( lines_only( (unsigned long)p, 2 * 512UL), "write: no line outside the buffer's range");
#if IDE_DIAG
	CHECK( ide_push_n - n0 == 1, "write: one pass");
#endif
}

static void
test_cache_write()
{
	cache_write_case( 0);
	cache_write_case( 1);
}

/* synthesised data (INQUIRY) is covered too; in an IDE_DIAG build ide_cache = 0
 * turns it all off */
static void
test_cache_synth_and_off()
{
	struct sdcom	cp;
	static char	buf[512];
	unsigned char	inq[6] = { 0x12, 0, 0, 0, 36, 0 };

	fresh( 1);
	tur( &cp, 6);
	mock_nlineops = 0;
	run( &cp, 6, inq, 6, buf + 3, 36);
	CHECK( lines_done( (unsigned long)(buf + 3), 36UL, 0UL, ~0UL), "INQUIRY buffer pushed+invalidated");
	CHECK( lines_only( (unsigned long)(buf + 3), 36UL), "INQUIRY: only its lines");

#if IDE_DIAG
	ide_cache = 0;
	mock_nlineops = 0;
	read10( &cp, 6, 70L, 1, buf, 512);
	CHECK( cp.okay && cp.status == 0 && mock_nlineops == 0, "ide_cache = 0: no line operations");
	tur( &cp, 6);
	ide_cache = 1;
#endif
}
#endif	/* IDE_CACHE */

#if !IDE_DIAG
/*
 * A default build has none of the investigation-only symbols, so nothing can
 * switch the 040/060 line operations off; a READ still pushes and invalidates
 * its buffer before and after the transfer.
 */
static void
test_diag_absent()
{
#if IDE_CACHE
	struct sdcom	cp;
	static char	buf[2 * 512 + 32];
	char		*p = buf + 3;
#endif

	CHECK( &ide_spins_hw == 0, "default build: no ide_spins_hw");
#if IDE_CACHE
	CHECK( &ide_cache == 0, "default build: no ide_cache switch");
	CHECK( &ide_push_n == 0 && &ide_push_bytes == 0, "default build: no push counters");

	fresh( 1);
	tur( &cp, 6);
	mock_nlineops = 0;
	mock_data_first = mock_data_last = 0;
	read10( &cp, 6, 80L, 2, p, 2 * 512);
	CHECK( cp.okay && cp.status == 0 && disk_equals( 0, 80L, p, 2 * 512), "default build: read data intact");
	CHECK( lines_done( (unsigned long)p, 2 * 512UL, 0UL, mock_data_first), "default build: read pushed+invalidated before");
	CHECK( lines_done( (unsigned long)p, 2 * 512UL, mock_data_last, ~0UL), "default build: and after");
#endif
}
#endif	/* !IDE_DIAG */

/* ---------------------------------------------------------------- ATAPI */

#define CD_BLOCKS	64

/* CF master + ATAPI CD-ROM slave, slave mode `mode' (0/1/2), interrupt or polled */
static void
fresh_cd( mode, polled)
int	mode, polled;
{
	fresh( 1);
	mock_add_cdrom( 1, (unsigned long)CD_BLOCKS, "MOCK CD-ROM DRIVE");
	ide_slave_enable = mode;
	if (polled)
		int2_tbl[0] = 0;
}

static int
cd_equals( lba, buf, nbyte)
unsigned long	lba;
const char	*buf;
unsigned	nbyte;
{
	return memcmp( mock_drv[1].cd + lba * MOCK_CD_BLOCK, buf, nbyte) == 0;
}

static void
cdb10( cdb, op, lba, n)
unsigned char	*cdb;
int		op, n;
unsigned long	lba;
{
	memset( cdb, 0, 12);
	cdb[0] = op;
	cdb[2] = lba >> 24; cdb[3] = lba >> 16; cdb[4] = lba >> 8; cdb[5] = lba;
	cdb[7] = n >> 8; cdb[8] = n;
}

static void
test_atapi_attach()
{
	struct sdcom	cp;
	unsigned char	cdb[6] = { 0, 0, 0, 0, 0, 0 };

	fresh_cd( 2, 0);
	tur( &cp, 6);
	CHECK( strstr( mock_console, "ide1: MOCK CD-ROM DRIVE, ATAPI CD-ROM") != 0, "ATAPI slave attached and named");
	CHECK( mock_cmd_count[0xEC] == 2 && mock_cmd_count[0xA1] == 1, "IDENTIFY on both, IDENTIFY PACKET on the CD");
	CHECK( strstr( mock_console, "ide0: MOCK CF CARD 2GB") != 0, "master attached as before");
	run( &cp, 5, cdb, 6, 0, 0);
	CHECK( cp.okay && cp.status == 0, "TUR to the CD: GOOD");
	CHECK( mock_pkt_log_n == 1 && mock_pkt_log[0] == 0x00, "TUR went to the drive as a packet");

	fresh_cd( 0, 0);				/* the CF kit's setting: never touch the slave */
	tur( &cp, 6);
	CHECK( mock_slave_selects == 0 && mock_cmd_count[0xA1] == 0, "slave mode 0: the CD is never touched");
	run( &cp, 5, cdb, 6, 0, 0);
	CHECK( !cp.okay && mock_slave_selects == 0, "slave mode 0: unit 5 is no device");

	fresh_cd( 1, 0);
	run( &cp, 5, cdb, 6, 0, 0);
	CHECK( cp.okay && cp.status == 0, "slave mode 1 attaches the CD too");

	fresh( 2);					/* ATA slave, ATAPI-only mode */
	ide_slave_enable = 2;
	tur( &cp, 6);
	CHECK( strstr( mock_console, "ATA device on the slave ignored") != 0, "slave mode 2 refuses an ATA slave, says so");
	run( &cp, 5, cdb, 6, 0, 0);
	CHECK( !cp.okay, "and unit 5 stays no device");
	tur( &cp, 6);
	CHECK( cp.okay && cp.status == 0, "master unaffected");

	fresh( 0);					/* the CD as master, nothing else */
	mock_add_cdrom( 0, (unsigned long)CD_BLOCKS, "MOCK CD MASTER");
	run( &cp, 6, cdb, 6, 0, 0);
	CHECK( cp.okay && cp.status == 0, "ATAPI master: unit 6 TUR GOOD");
	CHECK( strstr( mock_console, "ide0: MOCK CD MASTER, ATAPI CD-ROM") != 0, "ATAPI master attached");
}

/* the cdfs mount sequence: INQUIRY, READ CAPACITY(10), READ(10) of one 2048-byte block */
static void
atapi_cdfs_case( polled)
int	polled;
{
	struct sdcom	cp;
	static char	buf[3 * 2048];
	char		*b = buf + 2048 - ((unsigned long)buf & 2047);	/* 2 KB aligned */
	unsigned char	cdb[12];
	unsigned long	c0;

	fresh_cd( 2, polled);
	tur( &cp, 6);
	c0 = ide_atapi_cmds;
	memset( cdb, 0, 12); cdb[0] = 0x12; cdb[4] = 36;
	memset( b, 0xEE, 64);
	run( &cp, 5, cdb, 6, b, 36);
	CHECK( cp.okay && cp.status == 0, "INQUIRY: GOOD");
	CHECK( (b[0] & 0x1F) == 5 && memcmp( b + 8, "MOCK    ", 8) == 0, "INQUIRY is the drive's own: type 5");
	CHECK( (unsigned char)b[36] == 0xEE, "INQUIRY: nothing stored past nbyte");
	CHECK( mock_pkt_bc == 36, "byte count limit = nbyte");
	cdb10( cdb, 0x25, 0L, 0);
	run( &cp, 5, cdb, 10, b, 8);
	CHECK( cp.okay && cp.status == 0, "READ CAPACITY: GOOD");
	CHECK( (unsigned char)b[3] == CD_BLOCKS - 1 && (unsigned char)b[6] == 0x08 && b[7] == 0, "READ CAPACITY: last LBA, 2048-byte blocks");
	cdb10( cdb, 0x28, 16L, 1);
	run( &cp, 5, cdb, 10, b, 2048);
	CHECK( cp.okay && cp.status == 0 && cd_equals( 16L, b, 2048), "READ(10) of the PVD block");
	CHECK( memcmp( mock_pkt_last, cdb, 10) == 0 && mock_pkt_last[10] == 0 && mock_pkt_last[11] == 0, "the packet is the CDB");
	CHECK( ide_atapi_cmds - c0 == 3, "three packet commands");
#if IDE_CACHE
	mock_nlineops = 0;
	mock_data_first = mock_data_last = 0;
	cdb10( cdb, 0x28, 17L, 1);
	run( &cp, 5, cdb, 10, b, 2048);
	CHECK( cp.okay && cp.status == 0 && cd_equals( 17L, b, 2048), "cache build: CD block read");
	CHECK( lines_done( (unsigned long)b, 2048UL, 0UL, mock_data_first), "CD read: lines pushed before the transfer");
	CHECK( lines_done( (unsigned long)b, 2048UL, mock_data_last, ~0UL), "CD read: and again after it");
	CHECK( lines_only( (unsigned long)b, 2048UL), "CD read: only the buffer's lines");
#endif
	tur( &cp, 6);
	read10( &cp, 6, 7L, 2, buf, 1024);
	CHECK( cp.okay && cp.status == 0 && disk_equals( 0, 7L, buf, 1024), "the CF master still reads");
}

static void
test_atapi_cdfs_sequence()
{
	atapi_cdfs_case( 0);
	atapi_cdfs_case( 1);
}

/* several DRQ phases (the drive moves 512 bytes per phase), odd lengths, overrun */
static void
atapi_phases_case( polled)
int	polled;
{
	struct sdcom	cp;
	static char	buf[5 * 2048];
	unsigned char	cdb[12];
	unsigned long	ov;

	fresh_cd( 2, polled);
	mock_drv[1].chunk = 512;
	cdb10( cdb, 0x28, 3L, 4);
	run( &cp, 5, cdb, 10, buf, 4 * 2048);
	CHECK( cp.okay && cp.status == 0 && cd_equals( 3L, buf, 4 * 2048), "8 KB read in 16 DRQ phases");
	if (!polled)
		CHECK( ide_intr_count >= 17, "interrupt mode: one interrupt per phase plus the end");

	mock_drv[1].chunk = 0;
	memset( cdb, 0, 12); cdb[0] = 0x12; cdb[4] = 35;	/* odd allocation length */
	memset( buf, 0xEE, 64);
	ov = ide_atapi_overrun;
	run( &cp, 5, cdb, 6, buf, 35);
	CHECK( ide_atapi_overrun == ov, "odd length: no phantom overrun byte");
	CHECK( cp.okay && cp.status == 0 && (buf[0] & 0x1F) == 5, "odd-length INQUIRY");
	CHECK( memcmp( buf + 32, "1.0", 3) == 0 && (unsigned char)buf[35] == 0xEE, "odd length: 35 bytes stored, byte 35 untouched");

	ov = ide_atapi_overrun;
	memset( buf, 0xEE, 2048 + 16);
	cdb10( cdb, 0x28, 9L, 1);
	run( &cp, 5, cdb, 10, buf, 512);			/* a 512-byte caller on a 2048-byte block */
	CHECK( cp.okay && cp.status == 0 && cd_equals( 9L, buf, 512), "short buffer: the first 512 bytes stored");
	CHECK( (unsigned char)buf[512] == 0xEE && (unsigned char)buf[2047] == 0xEE, "short buffer: nothing stored past nbyte");
	CHECK( ide_atapi_overrun - ov == 1536, "short buffer: 1536 bytes drained and counted");
	tur( &cp, 6);
	CHECK( cp.okay && cp.status == 0, "cable sane after the drain");
}

static void
test_atapi_phases()
{
	atapi_phases_case( 0);
	atapi_phases_case( 1);
}

/* CHECK CONDITION: the drive's sense is fetched by a REQUEST SENSE that goes to
 * the drive; a sense the driver synthesised is answered first and locally */
static void
atapi_sense_case( polled)
int	polled;
{
	struct sdcom	cp;
	unsigned char	cdb[12], sense[18];

	fresh_cd( 2, polled);
	mock_drv[1].no_medium = 1;
	memset( cdb, 0, 12);
	run( &cp, 5, cdb, 6, 0, 0);
	CHECK( cp.okay && cp.status == 0x02, "no disc: TUR CHECK CONDITION");
	mock_pkt_log_n = 0;
	reqsense( &cp, 5, sense);
	CHECK( cp.okay && cp.status == 0 && sense[2] == 0x02 && sense[12] == 0x3A, "REQUEST SENSE: the drive's NOT READY / medium not present");
	CHECK( mock_pkt_log_n == 1 && mock_pkt_log[0] == 0x03, "REQUEST SENSE went to the drive");

	cdb10( cdb, 0x28, 0L, 1);
	cdb[1] = 0x20;						/* LUN 1 */
	run( &cp, 5, cdb, 10, 0, 0);
	CHECK( cp.okay && cp.status == 0x02, "LUN 1: CHECK from the driver");
	mock_pkt_log_n = 0;
	reqsense( &cp, 5, sense);
	CHECK( sense[2] == 0x05 && sense[12] == 0x25 && mock_pkt_log_n == 0, "the driver's own sense answered locally first");
	reqsense( &cp, 5, sense);
	CHECK( mock_pkt_log_n == 1, "the next REQUEST SENSE goes to the drive again");

	mock_drv[1].no_medium = 0;
	memset( cdb, 0, 12); cdb[0] = 0xBE;			/* an opcode the mock drive does not know */
	run( &cp, 5, cdb, 12, 0, 0);
	CHECK( cp.okay && cp.status == 0x02, "unknown opcode: the drive's CHECK");
	reqsense( &cp, 5, sense);
	CHECK( sense[2] == 0x05 && sense[12] == 0x20, "and the drive's ILLEGAL REQUEST / invalid opcode");
	mock_drv[1].unc_lba = 20;
	cdb10( cdb, 0x28, 19L, 2);
	run( &cp, 5, cdb, 10, (char *)sense, 18);
	CHECK( cp.okay && cp.status == 0x02, "medium error on the disc: CHECK");
	reqsense( &cp, 5, sense);
	CHECK( sense[2] == 0x03 && sense[12] == 0x11, "MEDIUM ERROR / unrecovered read error");
}

static void
test_atapi_sense()
{
	atapi_sense_case( 0);
	atapi_sense_case( 1);
}

/* READ(6) becomes READ(10); a CDB's tail is not sent; data out; 16-byte packets */
static void
test_atapi_packets()
{
	struct sdcom	cp;
	static char	buf[2 * 2048];
	unsigned char	cdb[12], out[12];
	int		i;

	fresh_cd( 2, 0);
	memset( cdb, 0, 12); cdb[0] = 0x08; cdb[2] = 0; cdb[3] = 5; cdb[4] = 2;	/* READ(6) LBA 5, 2 blocks */
	run( &cp, 5, cdb, 6, buf, 4096);
	CHECK( cp.okay && cp.status == 0 && cd_equals( 5L, buf, 4096), "READ(6) served");
	CHECK( mock_pkt_last[0] == 0x28 && mock_pkt_last[5] == 5 && mock_pkt_last[8] == 2, "READ(6) sent as READ(10)");

	cdb10( cdb, 0x28, 6L, 1);
	cdb[10] = 0x5A; cdb[11] = 0xA5;				/* junk after a 10-byte CDB */
	run( &cp, 5, cdb, 12, buf, 2048);
	CHECK( cp.okay && cp.status == 0 && mock_pkt_last[10] == 0 && mock_pkt_last[11] == 0, "bytes past the 10-byte CDB sent as zero");

	for (i = 0; i < 12; ++i)
		out[i] = (unsigned char)(0x30 + i);
	memset( cdb, 0, 12); cdb[0] = 0x55; cdb[8] = 12;		/* MODE SELECT(10), 12 bytes out */
	run( &cp, 5, cdb, 10, (char *)out, 12);
	CHECK( cp.okay && cp.status == 0 && mock_atapi_out_n == 12 && memcmp( mock_atapi_out, out, 12) == 0, "data-out phase delivered");

	fresh_cd( 2, 1);
	mock_drv[1].pkt16 = 1;
	cdb10( cdb, 0x28, 30L, 1);
	run( &cp, 5, cdb, 10, buf, 2048);
	CHECK( cp.okay && cp.status == 0 && cd_equals( 30L, buf, 2048), "16-byte packet device");
}

/* a drive that never finishes: polled spin budget / interrupt watchdog, SRST */
static void
test_atapi_timeout()
{
	struct sdcom	cp;
	static char	buf[2048];
	unsigned char	cdb[12], sense[18];
	unsigned long	r0, i;

	fresh_cd( 2, 1);
	ide_spins_atapi = 2000;
	tur( &cp, 6);
	r0 = ide_resets;
	mock_drv[1].hang_on_pkt = 1;
	cdb10( cdb, 0x28, 1L, 1);
	run( &cp, 5, cdb, 10, buf, 2048);
	CHECK( cp.okay && cp.status == 0x02 && ide_resets == r0 + 1, "polled: hung CD fails the request and resets the cable");
	reqsense( &cp, 5, sense);
	CHECK( sense[2] == 0x04 && sense[12] == 0x08, "polled: HARDWARE ERROR / LU communication failure");
	run( &cp, 5, cdb, 10, buf, 2048);
	CHECK( cp.okay && cp.status == 0 && cd_equals( 1L, buf, 2048), "polled: CD usable after the reset");
	tur( &cp, 6);
	CHECK( cp.okay && cp.status == 0, "polled: master usable after the reset");

	fresh_cd( 2, 0);
	tur( &cp, 6);
	r0 = ide_resets;
	mock_drv[1].hang_on_pkt = 1;
	memset( &cp, 0, sizeof cp);
	cdb10( cp.cdb, 0x28, 1L, 1);
	cp.addr = buf; cp.nbyte = 2048; cp.unit = 5; cp.intr = done;
	intr_calls = 0;
	idequeue( 0, &cp);
	for (i = 0; i <= ide_timeout_secs && intr_calls == 0; ++i)
		mock_fire_timeouts();
	CHECK( intr_calls == 1 && cp.okay && cp.status == 0x02, "interrupt mode: the watchdog fails the hung CD request");
	CHECK( ide_resets == r0 + 1, "interrupt mode: cable reset");
	cdb10( cdb, 0x28, 2L, 1);
	run( &cp, 5, cdb, 10, buf, 2048);
	CHECK( cp.okay && cp.status == 0 && cd_equals( 2L, buf, 2048), "interrupt mode: CD usable after the reset");
	ide_spins_atapi = 20000000L;
}

/* interrupt mode: CF and CD requests queued together complete in order, each once */
static struct sdcom	aq[4];
static int		aq_order[4], aq_n;

static void
aq_done( cp)
struct sdcom	*cp;
{
	intr_calls++;
	if (aq_n < 4)
		aq_order[aq_n++] = (int)(cp - aq);
}

static void
test_atapi_queue_with_disk()
{
	struct sdcom		*q = aq;
	static char		b[4][2048];
	int			i;

	fresh_cd( 2, 0);
	{ struct sdcom c; tur( &c, 6); }
	aq_n = 0;
	for (i = 0; i < 4; ++i) {
		memset( &q[i], 0, sizeof q[i]);
		if (i & 1) {
			cdb10( q[i].cdb, 0x28, (unsigned long)(10 + i), 1);
			q[i].unit = 5; q[i].nbyte = 2048;
		} else {
			cdb10( q[i].cdb, 0x28, (unsigned long)(100 + i), 4);
			q[i].unit = 6; q[i].nbyte = 2048;
		}
		q[i].addr = b[i];
		q[i].intr = aq_done;
	}
	intr_calls = 0;
	for (i = 0; i < 4; ++i)
		idequeue( 0, &q[i]);
	pump();
	CHECK( intr_calls == 4, "four mixed requests, four completions");
	for (i = 0; i < 4; ++i)
		CHECK( q[i].okay && q[i].status == 0, "mixed request GOOD");
	CHECK( cd_equals( 11L, b[1], 2048) && cd_equals( 13L, b[3], 2048), "CD data intact");
	CHECK( disk_equals( 0, 100L, b[0], 2048) && disk_equals( 0, 102L, b[2], 2048), "CF data intact");
	CHECK( aq_n == 4 && aq_order[0] == 0 && aq_order[1] == 1 && aq_order[2] == 2 && aq_order[3] == 3, "completed in submission order");
}

/*
 * Polled: BSY and DRQ clear is the end only with IREASON CoD|IO.  The mock's
 * `glitch' is Amiberry's model (status 0, IREASON CoD, before the data phase
 * shows); `no_ir3' is a device that never sets CoD|IO at the end.
 */
static void
test_atapi_polled_status_phase()
{
	struct sdcom	cp;
	static char	buf[4 * 2048];
	unsigned char	cdb[12];
	unsigned long	l0;
	int		i;

	fresh_cd( 2, 1);
	mock_drv[1].glitch = 50;
	l0 = ide_atapi_irlate;
	for (i = 0; i < 4; ++i) {
		memset( buf, 0x5A, sizeof buf);
		cdb10( cdb, 0x28, (unsigned long)(20 + i), 2);
		run( &cp, 5, cdb, 10, buf, 2 * 2048);
		CHECK( cp.okay && cp.status == 0 && cd_equals( (unsigned long)(20 + i), buf, 2 * 2048), "glitching drive: polled READ(10) data intact");
	}
	memset( cdb, 0, 12); cdb[0] = 0x12; cdb[4] = 36;
	run( &cp, 5, cdb, 6, buf, 36);
	CHECK( cp.okay && cp.status == 0 && (buf[0] & 0x1F) == 5, "glitching drive: INQUIRY intact");
	memset( cdb, 0, 12);
	run( &cp, 5, cdb, 6, 0, 0);
	CHECK( cp.okay && cp.status == 0, "glitching drive: TUR (no data) completes");
	CHECK( ide_atapi_irlate == l0, "glitching drive: every end seen in the status phase");

	fresh_cd( 2, 0);				/* interrupt mode is not affected */
	mock_drv[1].glitch = 50;
	cdb10( cdb, 0x28, 30L, 1);
	run( &cp, 5, cdb, 10, buf, 2048);
	CHECK( cp.okay && cp.status == 0 && cd_equals( 30L, buf, 2048), "glitch knob: interrupt mode READ intact");

	fresh_cd( 2, 1);
	mock_drv[1].no_ir3 = 1;
	l0 = ide_atapi_irlate;
	cdb10( cdb, 0x28, 40L, 2);
	run( &cp, 5, cdb, 10, buf, 2 * 2048);
	CHECK( cp.okay && cp.status == 0 && cd_equals( 40L, buf, 2 * 2048), "no CoD|IO at the end: polled READ still completes, data intact");
	CHECK( ide_atapi_irlate == l0 + 1, "no CoD|IO at the end: accepted after the quiet reads, counted");
	memset( cdb, 0, 12);
	mock_drv[1].no_medium = 1;
	run( &cp, 5, cdb, 6, 0, 0);
	CHECK( cp.okay && cp.status == 0x02 && ide_atapi_irlate == l0 + 1, "an error end needs no CoD|IO");
}

int
main()
{
#if IDE_CACHE
	fprintf( stderr, "== test_cache_spin_scale\n"); test_cache_spin_scale();
#endif
	fprintf( stderr, "== test_probe_no_gayle\n"); test_probe_no_gayle();
	fprintf( stderr, "== test_probe_chipset_ids\n"); test_probe_chipset_ids();
	fprintf( stderr, "== test_probe_a4000\n"); test_probe_a4000();
	fprintf( stderr, "== test_probe_a1200_a600\n"); test_probe_a1200_a600();
	fprintf( stderr, "== test_probe_force\n"); test_probe_force();
	fprintf( stderr, "== test_attach_identify_intr\n"); test_attach_identify_intr();
	fprintf( stderr, "== test_attach_polled_fallback\n"); test_attach_polled_fallback();
	fprintf( stderr, "== test_a1200_interrupt_plumbing\n"); test_a1200_interrupt_plumbing();
	fprintf( stderr, "== test_inquiry_and_capacity\n"); test_inquiry_and_capacity();
	fprintf( stderr, "== test_read_single_and_multi\n"); test_read_single_and_multi();
	fprintf( stderr, "== test_read_split_over_256\n"); test_read_split_over_256();
	fprintf( stderr, "== test_write\n"); test_write();
	fprintf( stderr, "== test_write_split_over_256\n"); test_write_split_over_256();
	fprintf( stderr, "== test_sense_out_of_range\n"); test_sense_out_of_range();
	fprintf( stderr, "== test_sense_media_error\n"); test_sense_media_error();
	fprintf( stderr, "== test_unknown_opcode\n"); test_unknown_opcode();
	fprintf( stderr, "== test_slave_disabled_and_enabled\n"); test_slave_disabled_and_enabled();
	fprintf( stderr, "== test_polled_timeout_does_not_hang\n"); test_polled_timeout_does_not_hang();
	fprintf( stderr, "== test_watchdog\n"); test_watchdog();
	fprintf( stderr, "== test_timeout_recovery\n"); test_timeout_recovery();
	fprintf( stderr, "== test_nolba_drive_ignored\n"); test_nolba_drive_ignored();
	fprintf( stderr, "== test_sync_cache\n"); test_sync_cache();
	fprintf( stderr, "== test_sync_cache_queued\n"); test_sync_cache_queued();
	fprintf( stderr, "== test_verify\n"); test_verify();
	fprintf( stderr, "== test_small_fixes\n"); test_small_fixes();
	fprintf( stderr, "== test_pagecross_counter\n"); test_pagecross_counter();
	fprintf( stderr, "== test_read6\n"); test_read6();
	fprintf( stderr, "== test_queue_order\n"); test_queue_order();
	fprintf( stderr, "== test_persistent_sdcom_resubmit\n"); test_persistent_sdcom_resubmit();
	fprintf( stderr, "== test_reentrant_completion\n"); test_reentrant_completion();
	fprintf( stderr, "== test_intr_not_mine\n"); test_intr_not_mine();
#if IDE_DIAG
	fprintf( stderr, "== test_spins_hw\n"); test_spins_hw();
#else
	fprintf( stderr, "== test_diag_absent\n"); test_diag_absent();
#endif
	fprintf( stderr, "== test_banner_mb\n"); test_banner_mb();
	fprintf( stderr, "== test_atapi_attach\n"); test_atapi_attach();
	fprintf( stderr, "== test_atapi_cdfs_sequence\n"); test_atapi_cdfs_sequence();
	fprintf( stderr, "== test_atapi_phases\n"); test_atapi_phases();
	fprintf( stderr, "== test_atapi_sense\n"); test_atapi_sense();
	fprintf( stderr, "== test_atapi_packets\n"); test_atapi_packets();
	fprintf( stderr, "== test_atapi_timeout\n"); test_atapi_timeout();
	fprintf( stderr, "== test_atapi_queue_with_disk\n"); test_atapi_queue_with_disk();
	fprintf( stderr, "== test_atapi_polled_status_phase\n"); test_atapi_polled_status_phase();
#if IDE_CACHE
	fprintf( stderr, "== test_cache_ci_check\n"); test_cache_ci_check();
	fprintf( stderr, "== test_cache_read\n"); test_cache_read();
	fprintf( stderr, "== test_cache_write\n"); test_cache_write();
	fprintf( stderr, "== test_cache_synth_and_off\n"); test_cache_synth_and_off();
#endif
	mock_free();
#if IDE_CACHE && IDE_DIAG
	fprintf( stdout, "ide_test (040/060: IDE_CACHE, IDE_NBPP %d, IDE_DIAG): %d checks, %d failures\n", PG, checks, failures);
#elif IDE_CACHE
	fprintf( stdout, "ide_test (040/060: IDE_CACHE, IDE_NBPP %d): %d checks, %d failures\n", PG, checks, failures);
#elif IDE_DIAG
	fprintf( stdout, "ide_test (IDE_DIAG): %d checks, %d failures\n", checks, failures);
#else
	fprintf( stdout, "ide_test: %d checks, %d failures\n", checks, failures);
#endif
	return failures ? 1 : 0;
}
