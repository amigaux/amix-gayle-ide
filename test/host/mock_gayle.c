/*
 * mock_gayle.c -- behavioural mock of the Gayle IDE port (A4000 / A1200 /
 * A600) and two ATA-2 drives for the host harness, and of the Buddha /
 * Catweasel Z-II (up to two boards, one cable of two drives per port).
 * See mock_gayle.h.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mock_gayle.h"

/* task-file offsets (as in ide.c) */
#define R_DATA		0x0000
#define R_ERROR		0x0006
#define R_NSECT		0x000A
#define R_SECT		0x000E
#define R_LCYL		0x0012
#define R_HCYL		0x0016
#define R_SDH		0x001A
#define R_STATUS	0x001E
#define R_ALTSTAT	0x101A

#define BASE_A4000	0xDD2020L
#define BASE_A1200	0xDA0000L
#define IRQ_A4000	0xDD3020L
#define INTREQ_A1200	0xDA9000L
#define INTENA_A1200	0xDAA000L
#define GAYLE_ID	0xDE1000L
#define VPOSR		0xDFF004L

#define ST_BSY	0x80
#define ST_DRDY	0x40
#define ST_DF	0x20
#define ST_DSC	0x10
#define ST_DRQ	0x08
#define ST_ERR	0x01
#define ER_UNC	0x40
#define ER_IDNF	0x10
#define ER_ABRT	0x04

/* Every cable (a Gayle port, or one port of a Buddha board) has its own
 * task file, drives, data phase and INTRQ.  `ci' is the cable the access in
 * progress goes to; the Gayle is cable 0, Buddha board b's port p is cable
 * b * 3 + p.  mock_gayle.h maps mock_drv, mock_nien and mock_intrq to
 * cable 0 for the harness; inside this file they mean cable `ci'. */
struct mock_drive	mock_cdrv[MOCK_NCABLE][2];
int			mock_cnien[MOCK_NCABLE], mock_cintrq[MOCK_NCABLE];
static int		ci;
#undef mock_drv
#undef mock_nien
#undef mock_intrq
#define mock_drv	(mock_cdrv[ci])
#define mock_nien	(mock_cnien[ci])
#define mock_intrq	(mock_cintrq[ci])

struct mock_bud		mock_bud[MOCK_NBUD];
unsigned long		mock_bud_forbidden;	/* accesses a board does not allow */
unsigned long		mock_bud_intena_reads;	/* reads of the write-only enable */
int			mock_absent_mode;	/* MOCK_ABSENT_*: what an absent selected device reads as */
int			mock_ghost;		/* device 0 answers register reads for an absent device 1 */

int			mock_board = MOCK_BOARD_A4000;
unsigned short		mock_vposr_value;
int			mock_vposr_override;
unsigned long		mock_accesses, mock_bad_accesses, mock_taskfile_accesses, mock_slave_selects;
unsigned long		mock_cmd_count[256], mock_irq_acks, mock_id_reads, mock_id_writes;
unsigned long		mock_srst_count;
int			mock_last_cmd = -1;
int			mock_cmd_log[64];
int			mock_cmd_log_n;
int			mock_irq_latch;
unsigned char		mock_intena;
int			mock_rom;
int			mock_latch_stuck;
unsigned long		mock_rom_reads, mock_dd_accesses, mock_da_accesses;

/* --- the Kickstart ROM window -------------------------------------------- */

#define ROM_BASE	0xF80000L
#define ROM_SIZE	0x80000L
static unsigned char	rom[ROM_SIZE];
static int		rom_built = -1;		/* the kind the image holds */

static void
rom_w( o, w)
unsigned long	o;
unsigned	w;
{
	rom[o] = (w >> 8) & 0xFF;
	rom[o + 1] = w & 0xFF;
}

static void
rom_l( o, l)
unsigned long	o, l;
{
	rom_w( o, (unsigned)(l >> 16));
	rom_w( o + 2, (unsigned)(l & 0xFFFF));
}

/* a resident tag at offset o: rt_MatchWord, rt_MatchTag, rt_EndSkip (o + 0x100,
 * the walk goes on word by word from there), its name at o + 0x40 */
static void
rom_tag( o, name)
unsigned long	o;
const char	*name;
{
	rom_w( o, 0x4AFC);
	rom_l( o + 2, ROM_BASE + o);
	rom_l( o + 6, ROM_BASE + o + 0x100);
	rom_l( o + 14, ROM_BASE + o + 0x40);
	strcpy( (char *)rom + o + 0x40, name);
}

static void
rom_build( kind)
int	kind;
{
	unsigned long	o;

	if (kind == MOCK_ROM_BLANK) {
		memset( rom, 0xFF, sizeof rom);
		rom_built = kind;
		return;
	}
	for (o = 0; o < ROM_SIZE; o += 2)
		rom_w( o, 0x4E71);			/* code-like filler, never a tag */
	rom_tag( 0x00100L, "exec.library");
	rom_tag( 0x08000L, "expansion.library");
	rom_tag( 0x20000L, "scsi.device");
	rom_tag( 0x40000L, kind == MOCK_ROM_CARD ? "card.resource" : "battclock.resource");
	rom_tag( 0x60000L, "intuition.library");
	if (kind == MOCK_ROM_PLAIN) {
		strcpy( (char *)rom + 0x30000L, "card.resource");	/* a string, not a resident */
		rom_w( 0x50000L, 0x4AFC);			/* tag-shaped, wrong rt_MatchTag */
		rom_l( 0x50002L, 0);
		rom_l( 0x50006L, ROM_BASE + 0x50100L);
		rom_l( 0x5000EL, ROM_BASE + 0x30000L);
	}
	rom_built = kind;
}

static int
rom_kind()
{
	if (mock_rom != MOCK_ROM_AUTO)
		return mock_rom;
	return mock_board == MOCK_BOARD_A1200 || mock_board == MOCK_BOARD_A600 ?
	    MOCK_ROM_CARD : MOCK_ROM_PLAIN;
}

static int
in_rom( addr)
unsigned long	addr;
{
	return addr >= ROM_BASE && addr < ROM_BASE + ROM_SIZE;
}

static unsigned char
rom_rb( addr)
unsigned long	addr;
{
	if (rom_built != rom_kind())
		rom_build( rom_kind());
	mock_rom_reads++;
	return rom[addr - ROM_BASE];
}

/* every access: which local-I/O bank it falls in */
static void
bank( addr)
unsigned long	addr;
{
	if ((addr & 0xFF0000L) == 0xDD0000L)
		mock_dd_accesses++;
	if ((addr & 0xFF0000L) == 0xDA0000L)
		mock_da_accesses++;
}

/* CPU-side model for the IDE_CACHE build: an access sequence number, the
 * span of DATA-register accesses, task-file reads that could overtake an
 * unsynchronised write, and a log of data-cache line operations. */
unsigned long		mock_bsy_reads;		/* STATUS reads that still show BSY */
unsigned char		mock_pkt_log[64];
int			mock_pkt_log_n;
unsigned		mock_pkt_bc;
unsigned char		mock_pkt_last[16];
unsigned char		mock_atapi_out[256];
unsigned		mock_atapi_out_n;
unsigned long		mock_seq;
unsigned long		mock_data_first, mock_data_last;	/* seq of DATA accesses, 0 = none */
unsigned long		mock_order_violations;
static int		wr_pending;
unsigned long		mock_dtt0 = 0x003fc060L;	/* pstart040.s */
unsigned long		mock_dtt1 = 0x807fa060L;
unsigned long		mock_cacr = 0x80008000L;
struct mock_lineop	mock_lineops[MOCK_LINEOPS];
int			mock_nlineops;
unsigned long		mock_lineops_lost;

#define ABUF	(64 * 1024)
struct cable {
	/* task file (shared by both drives, as on the real cable) */
	unsigned char	r_nsect, r_sect, r_lcyl, r_hcyl, r_sdh, r_error, r_control;
	unsigned char	r_status[2];
	int		sel;			/* selected drive 0/1 */
	/* data phase */
	unsigned char	xbuf[MOCK_SECTOR];
	int		xidx, xmode;		/* xmode: 0 idle, 1 read, 2 write, 3 ATAPI packet,
						 * 4 ATAPI data in, 5 ATAPI data out */
	unsigned long	xlba, xleft;
	/* ATAPI phases */
	unsigned char	abuf[ABUF];		/* response (in) or received data (out) */
	unsigned	alen, apos, aphase;	/* total, done, left in this DRQ phase */
	unsigned char	pktbuf[16];
	int		pidx;
	unsigned	pkt_bc;
	int		glitch_left;		/* see mock_drive.glitch */
	unsigned char	bus_last;		/* the last byte written (MOCK_ABSENT_HOLD) */
};
static struct cable	cables[MOCK_NCABLE];
#define CC		(cables[ci])
#define r_nsect		CC.r_nsect
#define r_sect		CC.r_sect
#define r_lcyl		CC.r_lcyl
#define r_hcyl		CC.r_hcyl
#define r_sdh		CC.r_sdh
#define r_error		CC.r_error
#define r_control	CC.r_control
#define r_status	CC.r_status
#define sel		CC.sel
#define xbuf		CC.xbuf
#define xidx		CC.xidx
#define xmode		CC.xmode
#define xlba		CC.xlba
#define xleft		CC.xleft
#define abuf		CC.abuf
#define alen		CC.alen
#define apos		CC.apos
#define aphase		CC.aphase
#define pktbuf		CC.pktbuf
#define pidx		CC.pidx
#define pkt_bc		CC.pkt_bc
#define glitch_left	CC.glitch_left

static int		id_cnt;			/* Gayle ID serial read position */

static void		add_drive(), add_cdrom();
static int		bud_nports();

static struct mock_drive *
cur()
{
	return &mock_drv[sel];
}

static unsigned long
base()
{
	return mock_board == MOCK_BOARD_A4000 ? BASE_A4000 : BASE_A1200;
}

static int
has_gayle()
{
	return mock_board != MOCK_BOARD_NONE && mock_board != MOCK_BOARD_BUDDHA;
}

/* the Buddha board an address falls on, or -1 */
static int
bud_board_of( addr)
unsigned long	addr;
{
	int	b;

	if (mock_board != MOCK_BOARD_BUDDHA)
		return -1;
	for (b = 0; b < MOCK_NBUD; ++b)
		if (mock_bud[b].present && addr >= mock_bud[b].base && addr < mock_bud[b].base + 0x10000L)
			return b;
	return -1;
}

/*
 * A port register of board b at board offset `off': selects the port's cable
 * (ci) and returns the offset the Gayle model uses (R_DATA, R_ERROR ...
 * R_STATUS, R_ALTSTAT), or -1 for anything else.  Only the registers the
 * specification names are decoded: DATA at the port base, register n at
 * + 2 + 4 * n, alternate status / device control at + 0x11A.
 */
static long
bud_reg( b, off)
int		b;
unsigned long	off;
{
	unsigned long	rel;
	int		p;

	if (off < 0x800 || off >= 0x800 + 0x200L * bud_nports( b))
		return -1;
	p   = (int)((off - 0x800) / 0x200);
	rel = (off - 0x800) % 0x200;
	ci  = b * 3 + p;
	if (rel == 0x11A)
		return R_ALTSTAT;
	if (rel <= 0x1E && (rel == 0 || (rel & 3) == 2))
		return (long)rel;
	return -1;
}

/* an access the board does not allow (outside its ports' registers, the
 * IRQ status registers and the enable) */
static void
bud_forbidden()
{
	mock_bud_forbidden++;
	mock_bad_accesses++;
}

void
mock_reset()
{
	mock_bsy_reads = 0;
	mock_data_first = mock_data_last = 0;
	mock_order_violations = 0;
	wr_pending = 0;
	mock_nlineops = 0;
	mock_lineops_lost = 0;
	mock_dtt0 = 0x003fc060L;
	mock_dtt1 = 0x807fa060L;
	mock_cacr = 0x80008000L;
	memset( mock_cmd_count, 0, sizeof mock_cmd_count);
	mock_accesses = mock_bad_accesses = mock_taskfile_accesses = mock_slave_selects = 0;
	mock_irq_acks = mock_id_reads = mock_id_writes = 0;
	mock_rom = MOCK_ROM_AUTO;
	mock_latch_stuck = 0;
	mock_rom_reads = mock_dd_accesses = mock_da_accesses = 0;
	mock_last_cmd = -1;
	mock_srst_count = 0;
	mock_cmd_log_n = 0;
	mock_irq_latch = 0;
	mock_intena = 0;
	mock_vposr_value = 0; mock_vposr_override = 0;
	for (ci = 0; ci < MOCK_NCABLE; ++ci) {
		mock_nien = 0;			/* ATA default after reset; the driver sets it */
		mock_intrq = 0;
		r_nsect = r_sect = r_lcyl = r_hcyl = r_error = r_control = 0;
		r_sdh = 0xA0;
		r_status[0] = r_status[1] = ST_DRDY | ST_DSC;
		sel = 0;
		xmode = 0; xidx = 0; xlba = xleft = 0;
		alen = apos = aphase = 0; pidx = 0; glitch_left = 0;
	}
	ci = 0;
	id_cnt = 0;
	memset( mock_bud, 0, sizeof mock_bud);
	mock_bud_forbidden = mock_bud_intena_reads = 0;
	mock_absent_mode = MOCK_ABSENT_OPEN;
	mock_ghost = 0;
	mock_pkt_log_n = 0; mock_pkt_bc = 0; mock_atapi_out_n = 0;
	memset( mock_pkt_last, 0, sizeof mock_pkt_last);
}

void
mock_free()
{
	int	i, c;

	for (c = 0; c < MOCK_NCABLE; ++c)
		for (i = 0; i < 2; ++i) {
			free( mock_cdrv[c][i].disk);
			free( mock_cdrv[c][i].cd);
			memset( &mock_cdrv[c][i], 0, sizeof mock_cdrv[c][i]);
		}
}

/* mock_add_drive() / mock_add_cdrom() on cable c (a Buddha port) */
void
mock_add_drive_on( c, unit, nsectors, model, flush)
int		c, unit, flush;
unsigned long	nsectors;
const char	*model;
{
	ci = c;
	add_drive( unit, nsectors, model, flush);
	ci = 0;
}

void
mock_add_cdrom_on( c, unit, blocks, model)
int		c, unit;
unsigned long	blocks;
const char	*model;
{
	ci = c;
	add_cdrom( unit, blocks, model);
	ci = 0;
}

void
mock_add_drive( unit, nsectors, model, flush)
int		unit, flush;
unsigned long	nsectors;
const char	*model;
{
	ci = 0;
	add_drive( unit, nsectors, model, flush);
}

void
mock_add_cdrom( unit, blocks, model)
int		unit;
unsigned long	blocks;
const char	*model;
{
	ci = 0;
	add_cdrom( unit, blocks, model);
}

/* on cable `ci' */
static void
add_drive( unit, nsectors, model, flush)
int		unit, flush;
unsigned long	nsectors;
const char	*model;
{
	struct mock_drive	*d = &mock_drv[unit];
	unsigned long		i;

	free( d->disk);
	free( d->cd);
	memset( d, 0, sizeof *d);
	d->present  = 1;
	d->nsectors = nsectors;
	d->flush    = flush;
	d->unc_lba  = -1; d->df_lba  = -1;
	strncpy( d->model, model, 40);
	d->disk = malloc( nsectors * MOCK_SECTOR);
	for (i = 0; i < nsectors * MOCK_SECTOR; ++i)	/* recognisable pattern */
		d->disk[i] = (unsigned char)((i / MOCK_SECTOR) * 7 + (i % MOCK_SECTOR));
}

/* An ATAPI CD-ROM with `blocks' 2048-byte blocks of a recognisable pattern. */
static void
add_cdrom( unit, blocks, model)
int		unit;
unsigned long	blocks;
const char	*model;
{
	struct mock_drive	*d = &mock_drv[unit];
	unsigned long		i;

	free( d->disk);
	free( d->cd);
	memset( d, 0, sizeof *d);
	d->present   = 1;
	d->atapi     = 1;
	d->cd_blocks = blocks;
	d->unc_lba   = -1; d->df_lba = -1;
	strncpy( d->model, model, 40);
	d->cd = malloc( blocks * MOCK_CD_BLOCK);
	for (i = 0; i < blocks * MOCK_CD_BLOCK; ++i)
		d->cd[i] = (unsigned char)((i / MOCK_CD_BLOCK) * 13 + (i % MOCK_CD_BLOCK) * 3 + 1);
	r_status[unit] = 0;			/* no DRDY: the driver must not need it */
}

unsigned long
mock_cmds()
{
	unsigned long	n = 0;
	int		i;

	for (i = 0; i < 256; ++i)
		n += mock_cmd_count[i];
	return n;
}

static int
bud_nports( b)
int	b;
{
	return mock_bud[b].cwz2 ? 3 : 2;
}

int
mock_int2_asserted()
{
	int	b, p;

	if (mock_board == MOCK_BOARD_BUDDHA) {	/* the enabled boards' ports, ORed */
		for (b = 0; b < MOCK_NBUD; ++b)
			if (mock_bud[b].present && mock_bud[b].intena)
				for (p = 0; p < bud_nports( b); ++p)
					if (mock_cintrq[b * 3 + p])
						return 1;
		return 0;
	}
	if (mock_board == MOCK_BOARD_A4000)
		return mock_cintrq[0];
	if (mock_board == MOCK_BOARD_NONE)
		return 0;
	return mock_irq_latch && (mock_intena & 0x80);
}

/* the drive raises INTRQ (if allowed); Gayle latches it on A1200/A600 */
static void
raise_intrq()
{
	struct mock_drive	*d = cur();

	if (mock_nien || !d->present || d->mute_intrq)
		return;
	mock_intrq = 1;
	if (mock_board == MOCK_BOARD_A1200 || mock_board == MOCK_BOARD_A600)
		mock_irq_latch = 1;
}

/* --- IDENTIFY image: words little-endian in the byte stream ------------- */

static void
idw( o, w)
int		o;
unsigned	w;
{
	xbuf[2 * o]     = w & 0xFF;
	xbuf[2 * o + 1] = (w >> 8) & 0xFF;
}

static void
idstr( o, n, s)
int		o, n;
const char	*s;
{
	int	i, len = (int)strlen( s);

	for (i = 0; i < n; ++i) {		/* first char in bits 15:8 */
		int	c0 = 2 * i < len ? s[2 * i] : ' ';
		int	c1 = 2 * i + 1 < len ? s[2 * i + 1] : ' ';
		idw( o + i, (c0 << 8) | c1);
	}
}

static void
identify( d)
struct mock_drive	*d;
{
	memset( xbuf, 0, sizeof xbuf);
	idw( 0, 0x0040);
	idw( 1, 1024); idw( 3, 16); idw( 6, 63);
	idstr( 10, 10, "SN0000000001");
	idstr( 23, 4, "1.00");
	idstr( 27, 20, d->model);
	idw( 47, 0x8010);
	idw( 49, d->nolba ? 0x0000 : 0x0200);
	idw( 60, d->nsectors & 0xFFFF);
	idw( 61, (d->nsectors >> 16) & 0xFFFF);
	idw( 83, 0x4000 | (d->flush ? 0x1000 : 0));
}

/* --- command execution --------------------------------------------------- */

static unsigned long
lba()
{
	return ((unsigned long)(r_sdh & 0x0F) << 24) | ((unsigned long)r_hcyl << 16)
	     | ((unsigned long)r_lcyl << 8) | r_sect;
}

static void
fail( er)
int	er;
{
	r_error = er;
	r_status[sel] = ST_DRDY | ST_DSC | ST_ERR;
	xmode = 0;
	raise_intrq();
}

static void
load_sector()
{
	struct mock_drive	*d = cur();

	if (d->unc_lba >= 0 && (unsigned long)d->unc_lba == xlba) {
		fail( ER_UNC);
		return;
	}
	if (d->df_lba >= 0 && (unsigned long)d->df_lba == xlba) {	/* device fault: ERROR register stale */
		r_status[sel] = ST_DRDY | ST_DSC | ST_DF;
		xmode = 0;
		raise_intrq();
		return;
	}
	memcpy( xbuf, d->disk + xlba * MOCK_SECTOR, MOCK_SECTOR);
	xidx = 0;
	r_status[sel] = ST_DRDY | ST_DSC | ST_DRQ;
	raise_intrq();
}

/* --- ATAPI CD-ROM ---------------------------------------------------------
 * PACKET: DRQ with IREASON = CoD for the packet; then data phases of at most
 * the host's byte count limit (or d->chunk), IREASON = IO for data in, 0 for
 * data out, each announced by INTRQ; completion IREASON = CoD|IO, DRQ clear,
 * INTRQ.  Failure: ERR with the sense key in ERROR 7:4, sense held for
 * REQUEST SENSE.  Status never shows DRDY.
 */
#define IR_COD	0x01
#define IR_IO	0x02

static void
atapi_done( d)
struct mock_drive	*d;
{
	xmode = 0;
	r_nsect = d->no_ir3 ? IR_IO : IR_COD | IR_IO;
	r_status[sel] = ST_DSC;
	raise_intrq();
}

static void
atapi_check( d, skey, asc)
struct mock_drive	*d;
int			skey, asc;
{
	d->skey = skey;
	d->asc  = asc;
	r_error = (skey << 4) | ER_ABRT;
	xmode = 0;
	r_nsect = IR_COD | IR_IO;
	r_status[sel] = ST_DSC | ST_ERR;
	raise_intrq();
}

/* start the next DRQ phase, or complete */
static void
atapi_phase( d)
struct mock_drive	*d;
{
	unsigned	n;

	if (apos >= alen) {
		atapi_done( d);
		return;
	}
	n = alen - apos;
	if (n > pkt_bc)
		n = pkt_bc;
	if (d->chunk && n > d->chunk)
		n = d->chunk;
	aphase = n;
	r_lcyl = n & 0xFF;
	r_hcyl = (n >> 8) & 0xFF;
	r_nsect = xmode == 4 ? IR_IO : 0;
	r_status[sel] = ST_DSC | ST_DRQ;
	raise_intrq();
}

static void
atapi_data_in( d, src, n, alloc)
struct mock_drive	*d;
const unsigned char	*src;
unsigned		n, alloc;
{
	if (n > alloc)
		n = alloc;
	if (n > ABUF)
		n = ABUF;
	memcpy( abuf, src, n);
	alen = n; apos = 0;
	xmode = 4;
	atapi_phase( d);
}

static void
atapi_packet( d)
struct mock_drive	*d;
{
	unsigned char	*p = pktbuf, r[36];
	unsigned long	lba, cnt;
	unsigned	alloc;

	memcpy( mock_pkt_last, pktbuf, 16);
	glitch_left = d->glitch;
	if (mock_pkt_log_n < 64)
		mock_pkt_log[mock_pkt_log_n++] = p[0];
	if (d->hang_on_pkt) {
		d->hang_on_pkt = 0;
		d->hung = 1;
		mock_intrq = 0;
		xmode = 0;
		return;
	}
	switch (p[0]) {
	case 0x00:				/* TEST UNIT READY */
		if (d->no_medium) { atapi_check( d, 2, 0x3A); return; }
		atapi_done( d);
		return;
	case 0x03:				/* REQUEST SENSE */
		memset( r, 0, sizeof r);
		r[0] = 0x70; r[2] = d->skey; r[7] = 10; r[12] = d->asc;
		d->skey = d->asc = 0;
		atapi_data_in( d, r, 18, p[4]);
		return;
	case 0x12:				/* INQUIRY */
		memset( r, ' ', sizeof r);
		r[0] = 0x05; r[1] = 0x80; r[2] = 0x00; r[3] = 0x21; r[4] = 31;
		r[5] = r[6] = r[7] = 0;
		memcpy( r + 8, "MOCK    ", 8);
		memcpy( r + 16, "ATAPI CD-ROM    ", 16);
		memcpy( r + 32, "1.00", 4);
		atapi_data_in( d, r, 36, p[4]);
		return;
	case 0x25:				/* READ CAPACITY(10) */
		if (d->no_medium) { atapi_check( d, 2, 0x3A); return; }
		lba = d->cd_blocks - 1;
		r[0] = lba >> 24; r[1] = lba >> 16; r[2] = lba >> 8; r[3] = lba;
		r[4] = 0; r[5] = 0; r[6] = MOCK_CD_BLOCK >> 8; r[7] = 0;
		atapi_data_in( d, r, 8, 8);
		return;
	case 0x28:				/* READ(10) */
		if (d->no_medium) { atapi_check( d, 2, 0x3A); return; }
		lba = ((unsigned long)p[2] << 24) | ((unsigned long)p[3] << 16) | (p[4] << 8) | p[5];
		cnt = (p[7] << 8) | p[8];
		if (lba + cnt > d->cd_blocks || cnt * MOCK_CD_BLOCK > ABUF) {
			atapi_check( d, 5, 0x21);
			return;
		}
		if (d->unc_lba >= 0 && (unsigned long)d->unc_lba >= lba && (unsigned long)d->unc_lba < lba + cnt) {
			atapi_check( d, 3, 0x11);
			return;
		}
		atapi_data_in( d, d->cd + lba * MOCK_CD_BLOCK, (unsigned)(cnt * MOCK_CD_BLOCK), (unsigned)(cnt * MOCK_CD_BLOCK));
		return;
	case 0x5A:				/* MODE SENSE(10): an empty header */
		memset( r, 0, 8);
		r[1] = 6;
		alloc = (p[7] << 8) | p[8];
		atapi_data_in( d, r, 8, alloc);
		return;
	case 0x55:				/* MODE SELECT(10): data out */
		alloc = (p[7] << 8) | p[8];
		if (alloc > sizeof mock_atapi_out)
			alloc = sizeof mock_atapi_out;
		alen = alloc; apos = 0;
		mock_atapi_out_n = 0;
		xmode = 5;
		atapi_phase( d);
		return;
	default:
		atapi_check( d, 5, 0x20);
		return;
	}
}

/* one 16-bit DATA read in an ATAPI data-in phase */
static unsigned short
atapi_rw( d)
struct mock_drive	*d;
{
	unsigned short	w;

	w = (unsigned short)(abuf[apos] << 8);
	if (aphase >= 2)
		w |= abuf[apos + 1];
	if (aphase >= 2) { apos += 2; aphase -= 2; }
	else             { apos += 1; aphase = 0; }
	if (aphase == 0)
		atapi_phase( d);
	return w;
}

/* one 16-bit DATA write: the packet, or a data-out phase */
static void
atapi_ww( d, v)
struct mock_drive	*d;
unsigned short		v;
{
	if (xmode == 3) {
		pktbuf[pidx++] = v >> 8;
		pktbuf[pidx++] = v & 0xFF;
		if (pidx >= (d->pkt16 ? 16 : 12)) {
			r_status[sel] = ST_BSY;
			atapi_packet( d);
		}
		return;
	}
	abuf[apos] = v >> 8;
	if (aphase >= 2)
		abuf[apos + 1] = v & 0xFF;
	if (aphase >= 2) { apos += 2; aphase -= 2; }
	else             { apos += 1; aphase = 0; }
	if (aphase == 0) {
		if (apos >= alen) {
			memcpy( mock_atapi_out, abuf, alen);
			mock_atapi_out_n = alen;
		}
		atapi_phase( d);
	}
}

static void
identify_packet( d)
struct mock_drive	*d;
{
	memset( xbuf, 0, sizeof xbuf);
	idw( 0, 0x85C0 | (d->pkt16 ? 1 : 0));	/* ATAPI, CD-ROM, removable, 50 us DRQ */
	idstr( 10, 10, "CD0000000001");
	idstr( 23, 4, "1.00");
	idstr( 27, 20, d->model);
	idw( 49, 0x0200);
}

static void
atapi_command( d, cmd)
struct mock_drive	*d;
int			cmd;
{
	r_error = 0;
	switch (cmd) {
	case 0xA1:			/* IDENTIFY PACKET DEVICE */
		identify_packet( d);
		xidx = 0; xmode = 1; xleft = 1;
		xlba = (unsigned long)-1;
		r_status[sel] = ST_DSC | ST_DRQ;
		raise_intrq();
		return;
	case 0xA0:			/* PACKET */
		pkt_bc = r_lcyl | (r_hcyl << 8);
		mock_pkt_bc = pkt_bc;
		pidx = 0;
		memset( pktbuf, 0, sizeof pktbuf);
		xmode = 3;
		r_nsect = IR_COD;
		r_status[sel] = ST_DSC | ST_DRQ;	/* no INTRQ: accelerated DRQ */
		return;
	default:			/* IDENTIFY DEVICE and every ATA command: abort + signature */
		r_lcyl = 0x14; r_hcyl = 0xEB;
		r_error = ER_ABRT;
		xmode = 0;
		r_status[sel] = ST_DSC | ST_ERR;
		raise_intrq();
		return;
	}
}

static void
command( cmd)
int	cmd;
{
	struct mock_drive	*d = cur();
	unsigned long		n;

	mock_cmd_count[cmd]++;
	mock_last_cmd = cmd;
	if (mock_cmd_log_n < 64)
		mock_cmd_log[mock_cmd_log_n++] = cmd;
	if (!d->present)
		return;
	if (d->atapi) {
		atapi_command( d, cmd);
		return;
	}
	r_error = 0;
	switch (cmd) {
	case 0xEC:			/* IDENTIFY DEVICE */
		identify( d);
		xidx = 0; xmode = 1; xleft = 1;
		xlba = (unsigned long)-1;
		r_status[sel] = ST_DRDY | ST_DSC | ST_DRQ;
		raise_intrq();
		break;
	case 0x20:			/* READ SECTOR(S) */
	case 0x30:			/* WRITE SECTOR(S) */
		if (!(r_sdh & 0x40)) { fail( ER_ABRT); break; }
		n = r_nsect ? r_nsect : 256;
		xlba = lba();
		if (xlba + n > d->nsectors) { fail( ER_IDNF); break; }
		xleft = n;
		if (d->hang_on_cmd) {		/* command accepted, then the drive dies */
			d->hang_on_cmd = 0;
			d->hung = 1;
			mock_intrq = 0;
			break;
		}
		if (cmd == 0x20) {
			xmode = 1;
			load_sector();
		} else {
			xmode = 2; xidx = 0;
			r_status[sel] = ST_DRDY | ST_DSC | ST_DRQ;	/* no INTRQ for the first sector */
		}
		break;
	case 0x40:			/* READ VERIFY SECTOR(S): no data phase */
		if (!(r_sdh & 0x40)) { fail( ER_ABRT); break; }
		n = r_nsect ? r_nsect : 256;
		xlba = lba();
		if (xlba + n > d->nsectors) { fail( ER_IDNF); break; }
		if (d->unc_lba >= 0 && (unsigned long)d->unc_lba >= xlba && (unsigned long)d->unc_lba < xlba + n) {
			fail( ER_UNC);
			break;
		}
		r_status[sel] = ST_DRDY | ST_DSC;
		raise_intrq();
		break;
	case 0xE7:			/* FLUSH CACHE */
		if (d->flush) {
			r_status[sel] = ST_DRDY | ST_DSC;
			raise_intrq();
		} else
			fail( ER_ABRT);
		break;
	default:
		fail( ER_ABRT);
		break;
	}
}

/* --- register seam ------------------------------------------------------- */

static unsigned char
status()
{
	struct mock_drive	*d = cur();

	if (!d->present) {
		if (mock_absent_mode == MOCK_ABSENT_HOLD)
			return CC.bus_last;
		if (mock_absent_mode == MOCK_ABSENT_UAE)	/* Amiberry: 01h when the other is there */
			return mock_drv[sel ^ 1].present ? 0x01 : 0xFF;
		return sel == 0 ? 0xFF : 0x00;	/* open bus / device 1 absent */
	}
	if (d->atapi && glitch_left > 0 && mock_nien) {
		glitch_left--;
		return 0x00;
	}
	if (d->stuck_bsy || d->hung)
		return ST_BSY;
	return r_status[sel];
}

/* a register read with the selected device absent: device 0 answering for
 * device 1 (mock_ghost), or what the bus gives */
static unsigned char
absent_reg( v)
unsigned char	v;
{
	if (mock_ghost && sel == 1 && mock_drv[0].present)
		return v;
	if (mock_absent_mode == MOCK_ABSENT_HOLD)
		return CC.bus_last;
	if (mock_absent_mode == MOCK_ABSENT_UAE)
		return 0x00;
	return 0xFF;
}

/* register writes land in a device's copy only if it is there; with
 * mock_ghost, device 0 takes them for an absent device 1 as well */
static int
latches()
{
	return cur()->present || (mock_ghost && sel == 1 && mock_drv[0].present);
}

/* task-file access; `off' relative to the board base.  Returns 1 if handled. */
static int
tf_read( off, vp)
unsigned long	off;
unsigned char	*vp;
{
	struct mock_drive	*d = cur();

	mock_taskfile_accesses++;
	switch (off) {
	case R_STATUS:
		if (mock_bsy_reads > 0 && d->present) {	/* a drive slow to finish */
			mock_bsy_reads--;
			*vp = ST_BSY;
			return 1;
		}
		*vp = status(); mock_intrq = 0; return 1;
	case R_ALTSTAT:	*vp = status(); return 1;
	case R_ERROR:	*vp = d->present ? r_error : absent_reg( r_error); return 1;
	case R_NSECT:	*vp = d->present ? (d->atapi && glitch_left > 0 && mock_nien ? IR_COD : r_nsect)
				     : absent_reg( r_nsect); return 1;
	case R_SECT:	*vp = d->present ? r_sect  : absent_reg( r_sect); return 1;
	case R_LCYL:	*vp = d->present ? r_lcyl  : absent_reg( r_lcyl); return 1;
	case R_HCYL:	*vp = d->present ? r_hcyl  : absent_reg( r_hcyl); return 1;
	case R_SDH:	*vp = r_sdh; return 1;
	}
	return 0;
}

static int
tf_write( off, v)
unsigned long	off;
unsigned char	v;
{
	mock_taskfile_accesses++;
	CC.bus_last = v;
	switch (off) {
	case R_SDH:
		r_sdh = v;
		sel = (v & 0x10) ? 1 : 0;
		if (sel)
			mock_slave_selects++;
		return 1;
	case R_NSECT: if (latches()) r_nsect = v; return 1;
	case R_SECT:  if (latches()) r_sect  = v; return 1;
	case R_LCYL:  if (latches()) r_lcyl  = v; return 1;
	case R_HCYL:  if (latches()) r_hcyl  = v; return 1;
	case R_ERROR: return 1;				/* FEATURES: ignored */
	case R_STATUS: command( v); return 1;		/* COMMAND */
	case R_ALTSTAT:					/* CONTROL */
		r_control = v;
		mock_nien = (v & 0x02) != 0;
		if (v & 0x04) {				/* SRST: both drives */
			mock_srst_count++;
			r_status[0] = r_status[1] = ST_DRDY | ST_DSC;
			mock_drv[0].hung = mock_drv[1].hung = 0;
			if (mock_drv[0].atapi) r_status[0] = 0;
			if (mock_drv[1].atapi) r_status[1] = 0;
			if (mock_drv[0].atapi || mock_drv[1].atapi) {
				r_lcyl = 0x14; r_hcyl = 0xEB;
			}
			xmode = 0; mock_intrq = 0;
		}
		return 1;
	}
	return 0;
}

static void
data_access()
{
	mock_seq++;
	if (mock_data_first == 0)
		mock_data_first = mock_seq;
	mock_data_last = mock_seq;
}

static void
lineop( addr, op)
unsigned long	addr;
int		op;
{
	mock_seq++;
	if (mock_nlineops >= MOCK_LINEOPS) {
		mock_lineops_lost++;
		return;
	}
	mock_lineops[mock_nlineops].addr = addr;
	mock_lineops[mock_nlineops].op   = op;
	mock_lineops[mock_nlineops].seq  = mock_seq;
	mock_nlineops++;
}

void mock_cpushl( addr) unsigned long addr; { lineop( addr, MOCK_CPUSHL); }
void mock_cinvl( addr) unsigned long addr; { lineop( addr, MOCK_CINVL); }
void mock_sync() { mock_seq++; wr_pending = 0; }

unsigned char
mock_rb( addr)
unsigned long	addr;
{
	unsigned char	v = 0xFF;

	int		b, p;
	long		r;
	unsigned long	off;

	if (in_rom( addr))
		return rom_rb( addr);
	mock_accesses++;
	bank( addr);
	ci = 0;
	if ((b = bud_board_of( addr)) >= 0) {
		off = addr - mock_bud[b].base;
		r = bud_reg( b, off);
		if (r > 0 && tf_read( (unsigned long)r, &v)) {
			mock_seq++;
			if (wr_pending)
				mock_order_violations++;
			return v;
		}
		for (p = 0; p < bud_nports( b); ++p)
			if (off == 0xF00 + 0x40L * p)
				return mock_cintrq[b * 3 + p] ? 0x80 : 0x00;
		if (off >= 0xFC0 && off < 0x1000)
			mock_bud_intena_reads++;
		bud_forbidden();
		return 0xFF;
	}
	if (addr == GAYLE_ID) {
		mock_id_reads++;
		if (mock_board == MOCK_BOARD_A1200 || mock_board == MOCK_BOARD_A600) {
			unsigned id = mock_board == MOCK_BOARD_A1200 ? 0xD1 : 0xD0;
			v = (id_cnt < 8 && ((id >> (7 - id_cnt)) & 1)) ? 0x80 : 0x00;
			id_cnt++;
			return v;
		}
		return 0xFF;			/* [verify] real A4000/A3000 behaviour */
	}
	if (mock_board == MOCK_BOARD_A4000 && addr == IRQ_A4000)
		return mock_intrq ? 0x80 : 0x00;
	if (has_gayle() && addr >= base() && addr < base() + 0x1100 && tf_read( addr - base(), &v)) {
		mock_seq++;
		if (wr_pending)
			mock_order_violations++;
		return v;
	}
	if ((mock_board == MOCK_BOARD_A1200 || mock_board == MOCK_BOARD_A600)) {
		if (addr == INTREQ_A1200)
			return (mock_irq_latch ? 0x80 : 0x00) | 0x00;
		if (addr == INTENA_A1200)
			return mock_intena;
		if (addr >= 0xD80000L && addr < 0xE00000L)
			return 0x00;			/* Gayle decodes the range: harmless */
	}
	if (mock_board == MOCK_BOARD_A4000 && addr >= 0xDD0000L && addr < 0xDE0000L)
		return 0xFF;				/* Gayle bank, unused register */
	mock_bad_accesses++;
	return 0xFF;
}

void
mock_wb( addr, v)
unsigned long	addr;
unsigned char	v;
{
	int		b;
	long		r;
	unsigned long	off;

	mock_accesses++;
	bank( addr);
	ci = 0;
	if ((b = bud_board_of( addr)) >= 0) {
		off = addr - mock_bud[b].base;
		r = bud_reg( b, off);
		if (r > 0 && tf_write( (unsigned long)r, v)) {
			mock_seq++;
			wr_pending = 1;
			return;
		}
		if (off == 0xFC0) {		/* the enable: any value, no way back */
			mock_bud[b].intena = 1;
			mock_bud[b].intena_writes++;
			return;
		}
		bud_forbidden();
		return;
	}
	if (addr == GAYLE_ID) {
		mock_id_writes++;
		id_cnt = 0;
		return;
	}
	if (has_gayle() && addr >= base() && addr < base() + 0x1100 && tf_write( addr - base(), v)) {
		mock_seq++;
		wr_pending = 1;
		return;
	}
	if ((mock_board == MOCK_BOARD_A1200 || mock_board == MOCK_BOARD_A600)) {
		if (addr == INTREQ_A1200) {
			mock_irq_acks++;
			if (!(v & 0x80) && !mock_latch_stuck)
				mock_irq_latch = 0;
			return;
		}
		if (addr == INTENA_A1200) {
			mock_intena = v;
			return;
		}
		if (addr >= 0xD80000L && addr < 0xE00000L)
			return;
	}
	if (mock_board == MOCK_BOARD_A4000 && addr >= 0xDD0000L && addr < 0xDE0000L)
		return;
	mock_bad_accesses++;
}

unsigned short
mock_rw( addr)
unsigned long	addr;
{
	unsigned short	w;
	int		b;

	if (in_rom( addr) && !(addr & 1))
		return ((unsigned short)rom_rb( addr) << 8) | rom_rb( addr + 1);
	mock_accesses++;
	bank( addr);
	ci = 0;
	if ((b = bud_board_of( addr)) >= 0) {
		if (bud_reg( b, addr - mock_bud[b].base) != R_DATA) {
			bud_forbidden();
			return 0xFFFF;
		}
	} else if (addr == VPOSR) {
		if (mock_vposr_override)
			return mock_vposr_value;
		return mock_board == MOCK_BOARD_A600 || mock_board == MOCK_BOARD_NONE ? 0x2000 : 0x2300;
	} else if (!(has_gayle() && addr == base() + R_DATA)) {
		mock_bad_accesses++;
		return 0xFFFF;
	}
	mock_taskfile_accesses++;
	data_access();
	if (wr_pending)
		mock_order_violations++;
	if (xmode == 4 && cur()->present && cur()->atapi)
		return atapi_rw( cur());
	if (xmode != 1 || !cur()->present)
		return xmode == 0 ? 0xFFFF : 0;
	w = ((unsigned short)xbuf[xidx] << 8) | xbuf[xidx + 1];
	xidx += 2;
	if (xidx >= MOCK_SECTOR) {
		if (xlba == (unsigned long)-1 || --xleft == 0) {	/* IDENTIFY or last sector */
			xmode = 0;
			r_status[sel] = ST_DRDY | ST_DSC;
		} else if (cur()->hang_after && --cur()->hang_after == 0) {
			cur()->hung = 1;			/* sectors moved, then the drive dies */
			xmode = 0;
		} else {
			xlba++;
			load_sector();
		}
	}
	return w;
}

void
mock_ww( addr, v)
unsigned long	addr;
unsigned short	v;
{
	struct mock_drive	*d;
	int			b;

	mock_accesses++;
	bank( addr);
	ci = 0;
	if ((b = bud_board_of( addr)) >= 0) {
		if (bud_reg( b, addr - mock_bud[b].base) != R_DATA) {
			bud_forbidden();
			return;
		}
	} else if (!(has_gayle() && addr == base() + R_DATA)) {
		mock_bad_accesses++;
		return;
	}
	d = cur();
	mock_taskfile_accesses++;
	data_access();
	wr_pending = 1;
	if ((xmode == 3 || xmode == 5) && d->present && d->atapi) {
		atapi_ww( d, v);
		return;
	}
	if (xmode != 2 || !d->present)
		return;
	xbuf[xidx]     = v >> 8;
	xbuf[xidx + 1] = v & 0xFF;
	xidx += 2;
	if (xidx >= MOCK_SECTOR) {
		if (d->unc_lba >= 0 && (unsigned long)d->unc_lba == xlba) {
			fail( ER_UNC);
			return;
		}
		memcpy( d->disk + xlba * MOCK_SECTOR, xbuf, MOCK_SECTOR);
		xidx = 0;
		if (--xleft == 0) {
			xmode = 0;
			r_status[sel] = ST_DRDY | ST_DSC;
		} else {
			xlba++;
			r_status[sel] = ST_DRDY | ST_DSC | ST_DRQ;
		}
		raise_intrq();				/* sector accepted / command done */
	}
}
