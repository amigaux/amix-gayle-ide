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
extern int	ide_slave_enable, ide_debug, ide_master_unit, ide_slave_unit;
extern int	ide_intr_enable, ide_board_force;

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

int
main()
{
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
	fprintf( stderr, "== test_nolba_drive_ignored\n"); test_nolba_drive_ignored();
	fprintf( stderr, "== test_sync_cache\n"); test_sync_cache();
	fprintf( stderr, "== test_read6\n"); test_read6();
	fprintf( stderr, "== test_queue_order\n"); test_queue_order();
	fprintf( stderr, "== test_reentrant_completion\n"); test_reentrant_completion();
	fprintf( stderr, "== test_intr_not_mine\n"); test_intr_not_mine();
	mock_free();
	fprintf( stdout, "ide_test: %d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
