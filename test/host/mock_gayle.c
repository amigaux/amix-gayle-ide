/*
 * mock_gayle.c -- behavioural mock of the Gayle IDE port (A4000 / A1200 /
 * A600) and two ATA-2 drives for the host harness.  See mock_gayle.h.
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

struct mock_drive	mock_drv[2];
int			mock_board = MOCK_BOARD_A4000;
unsigned short		mock_vposr_value;
int			mock_vposr_override;
unsigned long		mock_accesses, mock_bad_accesses, mock_taskfile_accesses, mock_slave_selects;
unsigned long		mock_cmd_count[256], mock_irq_acks, mock_id_reads;
unsigned long		mock_srst_count;
int			mock_last_cmd = -1;
int			mock_cmd_log[64];
int			mock_cmd_log_n;
int			mock_nien, mock_intrq, mock_irq_latch;
unsigned char		mock_intena;

/* task file (shared by both drives, as on the real cable) */
static unsigned char	r_nsect, r_sect, r_lcyl, r_hcyl, r_sdh, r_error, r_control;
static unsigned char	r_status[2];
static int		sel;			/* selected drive 0/1 */
static int		id_cnt;			/* Gayle ID serial read position */

/* data phase */
static unsigned char	xbuf[MOCK_SECTOR];
static int		xidx, xmode;		/* xmode: 0 idle, 1 read, 2 write */
static unsigned long	xlba, xleft;

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
	return mock_board != MOCK_BOARD_NONE;
}

void
mock_reset()
{
	memset( mock_cmd_count, 0, sizeof mock_cmd_count);
	mock_accesses = mock_bad_accesses = mock_taskfile_accesses = mock_slave_selects = 0;
	mock_irq_acks = mock_id_reads = 0;
	mock_last_cmd = -1;
	mock_srst_count = 0;
	mock_cmd_log_n = 0;
	mock_nien = 1;				/* ATA default after reset: interrupts off? no -- 0.  Driver sets it. */
	mock_nien = 0;
	mock_intrq = mock_irq_latch = 0;
	mock_intena = 0;
	mock_vposr_value = 0; mock_vposr_override = 0;
	r_nsect = r_sect = r_lcyl = r_hcyl = r_error = r_control = 0;
	r_sdh = 0xA0;
	r_status[0] = r_status[1] = ST_DRDY | ST_DSC;
	sel = 0; id_cnt = 0;
	xmode = 0; xidx = 0; xlba = xleft = 0;
}

void
mock_free()
{
	int	i;

	for (i = 0; i < 2; ++i) {
		free( mock_drv[i].disk);
		memset( &mock_drv[i], 0, sizeof mock_drv[i]);
	}
}

void
mock_add_drive( unit, nsectors, model, flush)
int		unit, flush;
unsigned long	nsectors;
const char	*model;
{
	struct mock_drive	*d = &mock_drv[unit];
	unsigned long		i;

	free( d->disk);
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

unsigned long
mock_cmds()
{
	unsigned long	n = 0;
	int		i;

	for (i = 0; i < 256; ++i)
		n += mock_cmd_count[i];
	return n;
}

int
mock_int2_asserted()
{
	if (mock_board == MOCK_BOARD_A4000)
		return mock_intrq;
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

	if (!d->present)
		return sel == 0 ? 0xFF : 0x00;	/* open bus / device 1 absent */
	if (d->stuck_bsy || d->hung)
		return ST_BSY;
	return r_status[sel];
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
	case R_STATUS:	*vp = status(); mock_intrq = 0; return 1;
	case R_ALTSTAT:	*vp = status(); return 1;
	case R_ERROR:	*vp = d->present ? r_error : 0xFF; return 1;
	case R_NSECT:	*vp = d->present ? r_nsect : 0xFF; return 1;
	case R_SECT:	*vp = d->present ? r_sect  : 0xFF; return 1;
	case R_LCYL:	*vp = d->present ? r_lcyl  : 0xFF; return 1;
	case R_HCYL:	*vp = d->present ? r_hcyl  : 0xFF; return 1;
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
	switch (off) {
	case R_SDH:
		r_sdh = v;
		sel = (v & 0x10) ? 1 : 0;
		if (sel)
			mock_slave_selects++;
		return 1;
	case R_NSECT: if (cur()->present) r_nsect = v; return 1;
	case R_SECT:  if (cur()->present) r_sect  = v; return 1;
	case R_LCYL:  if (cur()->present) r_lcyl  = v; return 1;
	case R_HCYL:  if (cur()->present) r_hcyl  = v; return 1;
	case R_ERROR: return 1;				/* FEATURES: ignored */
	case R_STATUS: command( v); return 1;		/* COMMAND */
	case R_ALTSTAT:					/* CONTROL */
		r_control = v;
		mock_nien = (v & 0x02) != 0;
		if (v & 0x04) {				/* SRST: both drives */
			mock_srst_count++;
			r_status[0] = r_status[1] = ST_DRDY | ST_DSC;
			mock_drv[0].hung = mock_drv[1].hung = 0;
			xmode = 0; mock_intrq = 0;
		}
		return 1;
	}
	return 0;
}

unsigned char
mock_rb( addr)
unsigned long	addr;
{
	unsigned char	v = 0xFF;

	mock_accesses++;
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
	if (has_gayle() && addr >= base() && addr < base() + 0x1100 && tf_read( addr - base(), &v))
		return v;
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
	mock_accesses++;
	if (addr == GAYLE_ID) {
		id_cnt = 0;
		return;
	}
	if (has_gayle() && addr >= base() && addr < base() + 0x1100 && tf_write( addr - base(), v))
		return;
	if ((mock_board == MOCK_BOARD_A1200 || mock_board == MOCK_BOARD_A600)) {
		if (addr == INTREQ_A1200) {
			mock_irq_acks++;
			if (!(v & 0x80))
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

	mock_accesses++;
	if (addr == VPOSR) {
		if (mock_vposr_override)
			return mock_vposr_value;
		return mock_board == MOCK_BOARD_A600 || mock_board == MOCK_BOARD_NONE ? 0x2000 : 0x2300;
	}
	if (!(has_gayle() && addr == base() + R_DATA)) {
		mock_bad_accesses++;
		return 0xFFFF;
	}
	mock_taskfile_accesses++;
	if (xmode != 1 || !cur()->present)
		return xmode == 0 ? 0xFFFF : 0;
	w = ((unsigned short)xbuf[xidx] << 8) | xbuf[xidx + 1];
	xidx += 2;
	if (xidx >= MOCK_SECTOR) {
		if (xlba == (unsigned long)-1 || --xleft == 0) {	/* IDENTIFY or last sector */
			xmode = 0;
			r_status[sel] = ST_DRDY | ST_DSC;
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
	struct mock_drive	*d = cur();

	mock_accesses++;
	if (!(has_gayle() && addr == base() + R_DATA)) {
		mock_bad_accesses++;
		return;
	}
	mock_taskfile_accesses++;
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
