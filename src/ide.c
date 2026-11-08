/* SPDX-License-Identifier: MIT */
/*
 * ide.c -- Amix (Commodore Amiga UNIX 2.1, SVR4 m68k) pseudo SCSI host adapter
 *          for the Gayle IDE port of the Amiga 4000 (and, through the board
 *          table, the A1200 and A600).
 *
 * Phase 2: interrupt-driven READ/WRITE (level 2, via int2_tbl[]) with a polled
 * fallback, PIO only, LBA28, master drive by default.
 *
 * Copyright (c) 2026 Emre Bastuz.  See ../LICENSE.
 *
 * Own work: no Commodore/AT&T or GPL code.  ATA behaviour follows the ATA-2
 * standard; NetBSD's wdc.c and wdc_amiga.c (BSD) were read as documentation.
 *
 * How it plugs in (facts verified against the Amix 2.1 /usr/sys tree; see
 * docs/decision-hba-vs-block.md):
 *
 *  - The SCSI disk driver (block major 18) calls sdqueue(cp) with a struct
 *    sdcom (sd.h): a 12-byte CDB, the PHYSICAL buffer address in cp->addr,
 *    the byte count, the unit, and a completion callback cp->intr.  sd.c routes
 *    it to idequeue(c, cp).  We execute the CDB, set cp->okay (command executed
 *    at all) and cp->status (SCSI status byte), then call (*cp->intr)(cp).
 *    okay == FALSE is a hard failure (no device) -> B_ERROR at once; okay ==
 *    TRUE with status CHECK CONDITION makes dd.c issue REQUEST SENSE, log
 *    sense[2]/sense[12] and retry once.
 *  - Only READ(10), WRITE(10) and REQUEST SENSE come from dd.c.  INQUIRY, TEST
 *    UNIT READY, READ CAPACITY(10), MODE SENSE(6), START/STOP UNIT and
 *    SYNCHRONIZE CACHE are answered for the GSIO user passthrough (scsi.c).
 *  - Kernel virtual == physical (sys/immu.h: phystokv(p) == p), so PIO copies
 *    go straight to cp->addr.  Requests are <= NBPP (2048) bytes for raw I/O
 *    and 8 KB (UFS block) from the buffer cache, physically contiguous, exactly
 *    as the stock DMA host adapters receive them.  A raw request that crosses
 *    a page boundary is the one shape whose second page is not guaranteed to
 *    follow the first; ide_pagecross_n counts those (IDE_NBPP).
 *  - sdspl is spl2 (sd.h).  Level-2 handlers (int2_tbl[] in master.d/kernel.c,
 *    walked by amiga/ml/ttrap.s p2int for every INT2 after it clears the
 *    INTREQ PORTS bit) run at IPL 2, so idequeue()'s spl2 bracket and ideintr()
 *    exclude each other.  The clock tick is a level-2 interrupt too, so lbolt
 *    does not advance inside the bracket: polled waits are bounded spin
 *    counts, the interrupt path uses a timeout() watchdog instead.
 *  - Registration: driver.conf's probe=idepresent.  sd.c's init() calls it once
 *    when the autocon table has no entry; it must be safe on an A3000 (no
 *    Gayle): the chipset/ID gates run before any task-file address is touched.
 *  - Interrupt hook: kernel-patches/kernel.c.patch adds ideintr to int2_tbl[].
 *    attach() looks for itself in that table and falls back to polled I/O when
 *    the hook is missing, so an unpatched kernel still works.
 *  - Completion is delivered from ideintr() (interrupt path) or synchronously
 *    (synthesised commands, polled path).  A small FIFO flattens the dd.c
 *    completion -> startio -> sdqueue re-entry into iteration.
 *
 * BOARD ABSTRACTION: every board-specific address and interrupt convention is
 * confined to `struct ideboard' and the ide_irq_*() / ide_detect() helpers
 * below.  The rest of the driver sees a task-file base and three interrupt
 * operations, nothing else.
 *
 * Register access goes through the RB/WB/RW/WW seam so the host test harness
 * (test/host) can compile this file unmodified against a mock (-DHOST_TEST
 * -include mock_regs.h).  Without HOST_TEST the macros are plain volatile
 * dereferences.
 */
#include	"sys/param.h"		/* HZ */
#include	"sys/types.h"
#include	"sys/errno.h"
#include	"sys/inline.h"		/* spl2()/splx() */
#include	"rico.h"
#include	"sd.h"

extern int	printf();
extern int	timeout();
extern void	(*int2_tbl[])();	/* master.d/kernel.c, NULL terminated */

/*
 * ---- Register map and constants (kept in this file: amix-kerntools ships one
 * source file per driver; see also docs/) ----
 * Sources: NetBSD gayle.h / wdc_amiga.c (BSD, read as documentation), ATA-2
 * (X3.221-1996), SCSI-2 (X3.131-1994).  Nothing from Commodore/AT&T or GPL code.
 */
/*
 * Gayle task file (A4000/A4000T at 0xDD2020, A1200/A600 at 0xDA0000 -- the
 * layout is the same): byte-wide ATA registers spaced 4 bytes apart, the
 * 16-bit DATA register at the base itself, the control block (alternate
 * status / device control) 0x101A above the base (NetBSD: ctl subregion 0x406
 * in a stride-4 space, +2).  Interrupt registers differ per board and are
 * described in the board table.
 *
 * Kernel virtual == physical for these ranges (sys/immu.h: phystokv(p) == p;
 * the low 1 GB is identity mapped through the 030 transparent translation
 * registers), so the driver dereferences plain addresses.  Cache inhibition
 * of the ranges is assumed from the TT registers (works on the real
 * A4000; the TT0/TT1 CI bits themselves are not yet read back).
 */
/* Base addresses live in the board table (struct ideboard) -- never here. */
#define IDE_SPAN	0x101BL		/* bytes from a task-file base to the last register + 1 */

#define IDE_REG_DATA	0x0000		/* R/W 16 bit */
#define IDE_REG_ERROR	0x0006		/* R          */
#define IDE_REG_FEATURE	0x0006		/*   W        */
#define IDE_REG_NSECT	0x000A		/* R/W sector count */
#define IDE_REG_SECT	0x000E		/* R/W sector number  / LBA  7:0  */
#define IDE_REG_LCYL	0x0012		/* R/W cylinder low   / LBA 15:8  */
#define IDE_REG_HCYL	0x0016		/* R/W cylinder high  / LBA 23:16 */
#define IDE_REG_SDH	0x001A		/* R/W drive/head     / LBA 27:24 */
#define IDE_REG_STATUS	0x001E		/* R   (clears INTRQ) */
#define IDE_REG_COMMAND	0x001E		/*   W                */
#define IDE_REG_ALTSTAT	0x101A		/* R   (does not clear INTRQ) */
#define IDE_REG_CONTROL	0x101A		/*   W device control */

#define IDE_IRQ_IDE	0x80		/* Gayle IRQ status: IDE interrupt pending */

/*
 * Chipset gate: VPOSR bits 8..14 hold the Agnus/Alice id.  OCS 0x00/0x10, ECS
 * 0x20/0x30, AGA Alice 0x22/0x23/0x32/0x33 -- bit 4 (0x10) is the video
 * standard, the low nibble is >= 2 only for Alice.  (Testing "id >= 0x22", as
 * some sources do, misclassifies an NTSC ECS machine, id 0x30, as AGA.)
 * Verified against the Amiberry/WinUAE VPOSR model; [verify] on real hardware.
 */
#define IDE_VPOSR_ADDR	0xDFF004L
#define IDE_VPOSR_ID( v)	(((v) >> 8) & 0x7F)
#define IDE_VPOSR_IS_AGA( v)	((IDE_VPOSR_ID( v) & 0x0F) >= 2)

/* Status register bits (ATA-2 7.15) */
#define IDE_ST_BSY	0x80
#define IDE_ST_DRDY	0x40
#define IDE_ST_DF	0x20
#define IDE_ST_DSC	0x10
#define IDE_ST_DRQ	0x08
#define IDE_ST_CORR	0x04
#define IDE_ST_IDX	0x02
#define IDE_ST_ERR	0x01

/* Error register bits (ATA-2 7.11) */
#define IDE_ER_BBK	0x80
#define IDE_ER_UNC	0x40
#define IDE_ER_MC	0x20
#define IDE_ER_IDNF	0x10
#define IDE_ER_MCR	0x08
#define IDE_ER_ABRT	0x04
#define IDE_ER_TK0NF	0x02
#define IDE_ER_AMNF	0x01

/* Device control register bits (ATA-2 7.9) */
#define IDE_CTL_SRST	0x04
#define IDE_CTL_NIEN	0x02		/* 1 = drive does not assert INTRQ */

/* Drive/head register (ATA-2 7.10) */
#define IDE_SDH_FIXED	0xA0		/* bits 7 and 5 always set */
#define IDE_SDH_LBA	0x40
#define IDE_SDH_DRV1	0x10		/* slave */
#define IDE_SDH_HEAD	0x0F		/* CHS head or LBA 27:24 */

/* Commands (ATA-2 8) */
#define IDE_CMD_RECAL		0x10
#define IDE_CMD_READ		0x20	/* READ SECTOR(S) with retry */
#define IDE_CMD_WRITE		0x30	/* WRITE SECTOR(S) with retry */
#define IDE_CMD_VERIFY		0x40	/* READ VERIFY SECTOR(S) with retry: no data */
#define IDE_CMD_READMULT	0xC4
#define IDE_CMD_WRITEMULT	0xC5
#define IDE_CMD_SETMULT		0xC6
#define IDE_CMD_FLUSH		0xE7	/* FLUSH CACHE (ATA-4+, optional) */
#define IDE_CMD_IDENTIFY	0xEC
#define IDE_CMD_SETFEATURES	0xEF

/* IDENTIFY DEVICE words (ATA-2 8.12) */
#define IDE_ID_CONFIG		0
#define IDE_ID_CYLS		1
#define IDE_ID_HEADS		3
#define IDE_ID_SPT		6
#define IDE_ID_SERIAL		10	/* 10..19 */
#define IDE_ID_FIRMWARE		23	/* 23..26 */
#define IDE_ID_MODEL		27	/* 27..46 */
#define IDE_ID_MULTI		47	/* low byte: max sectors per READ/WRITE MULTIPLE */
#define IDE_ID_CAPS		49	/* bit 9: LBA supported */
#define IDE_ID_LBASECTORS	60	/* 60..61 total LBA28 sectors */
#define IDE_ID_CMDSET2		83	/* bit 12: FLUSH CACHE supported */
#define IDE_ID_CAPS_LBA		0x0200
#define IDE_ID_CMDSET2_FLUSH	0x1000
#define IDE_ID_CMDSET2_VALID	0xC000	/* bit 15 clear, bit 14 set => word valid */

#define IDE_SECTOR	512
#define IDE_MAXSECT	256		/* per READ/WRITE SECTOR(S) command (NSECT 0 == 256) */
#define IDE_LBA28_MAX	0x0FFFFFFFL

/* SCSI-2 opcodes the queue function understands */
#define SC_TUR		0x00
#define SC_REZERO	0x01
#define SC_REQ_SENSE	0x03
#define SC_READ6	0x08
#define SC_WRITE6	0x0A
#define SC_INQUIRY	0x12
#define SC_MODE_SELECT6	0x15
#define SC_MODE_SENSE6	0x1A
#define SC_START_STOP	0x1B
#define SC_READ_CAP10	0x25
#define SC_READ10	0x28
#define SC_WRITE10	0x2A
#define SC_VERIFY10	0x2F
#define SC_SYNC_CACHE	0x35

/* SCSI status bytes and sense keys / ASC values we synthesise */
#define SC_GOOD		0x00
#define SC_CHECK	0x02
#define SK_NONE		0x00
#define SK_NOT_READY	0x02
#define SK_MEDIUM	0x03
#define SK_HARDWARE	0x04
#define SK_ILLEGAL	0x05
#define SK_DATA_PROT	0x07
#define ASC_LUN_NOT_READY	0x04
#define ASC_LU_COMM_FAILURE	0x08
#define ASC_WRITE_ERROR		0x0C
#define ASC_READ_ERROR		0x11
#define ASC_INVALID_OPCODE	0x20
#define ASC_LBA_OUT_OF_RANGE	0x21
#define ASC_INVALID_CDB_FIELD	0x24
#define ASC_LUN_NOT_SUPPORTED	0x25
#define ASC_INTERNAL_FAILURE	0x44

/* ---- end of register map ---- */

/* ---------------------------------------------------------------- tunables */

/*
 * Spin budgets for the polled waits.  One iteration is one (slow, chip-bus)
 * status read plus loop overhead; on a 25 MHz 68030 that is on the order of a
 * microsecond, so the defaults allow several seconds.  [verify] calibrate on
 * the real machine; plain globals so the kernel debugger can change them.
 */
ulong	ide_spins       = 2000000L;	/* BSY/DRQ wait inside a polled command */
ulong	ide_spins_reset = 8000000L;	/* BSY wait after reset / for FLUSH CACHE */
ulong	ide_spins_short = 200000L;	/* DRQ wait inside the interrupt path */

int	ide_slave_enable = 0;		/* 0: never touch the slave (FlashAir rule) */
int	ide_intr_enable  = 1;		/* 0: force polled I/O even if hooked */
int	ide_board_force  = 0;		/* 0 auto-detect (A4000 only), 1.. = ideboards[] index+1, -1 = never probe */
ulong	ide_timeout_secs = 20;		/* watchdog for an interrupt-driven command */
int	ide_debug        = 0;		/* 0 quiet, 1 attach/errors, 2 every command */

/*
 * SCSI unit numbers under which the drives appear.  The Amix disk convention
 * puts the system disk at target 6 (root c6d0s1, the installer's default, the
 * stock CONFIG and vfstab); presenting the IDE master as unit 6 lets an image
 * installed on an A3000 boot unchanged from IDE on an A4000, and lets the
 * installer be answered with "6" for an IDE disk.
 */
int	ide_master_unit = 6;
int	ide_slave_unit  = 5;

/*
 * Kernel page size, for the page-crossing check on raw requests: 2048 on the
 * 68030 kernel, 4096 on the 68040/68060 line (build with -DIDE_NBPP=4096).
 */
#ifndef IDE_NBPP
#define IDE_NBPP	2048L
#endif
ulong	ide_pagecross_n;		/* raw (<= IDE_NBPP) requests spanning two pages */

#define IDE_DATA_SWAP	0		/* 1 if sector data must be byte swapped [verify] */

/* ------------------------------------------------------ register seam */

#ifndef HOST_TEST
#define RB( a)		(*(volatile uchar *)(a))
#define WB( a, v)	(*(volatile uchar *)(a) = (uchar)(v))
#define RW( a)		(*(volatile ushort *)(a))
#define WW( a, v)	(*(volatile ushort *)(a) = (ushort)(v))
#endif

/* ------------------------------------------------------------ boards */

/*
 * One entry per Gayle flavour.  Facts from NetBSD gayle.h / wdc_amiga.c and
 * the WinUAE/Amiberry Gayle model:
 *   A4000  task file 0xDD2020; INTRQ mirrored in bit 7 of 0xDD3020; nothing to
 *          acknowledge or enable (reading the drive's STATUS drops INTRQ).
 *   A1200  task file 0xDA0000; Gayle INTREQ 0xDA9000 (bit 7 IDE, latched; ack
 *          by writing the register with bit 7 clear, bits 2..6 set so the
 *          PCMCIA sources are left alone, bits 0..1 preserved), INTENA 0xDAA000
 *          (bit 7 must be set to route IDE to INT2).  Gayle ID 0xD1.
 *   A600   as the A1200 but ECS chipset and Gayle ID 0xD0.
 * The Gayle ID is read bit-serially from 0xDE1000 (write resets, then eight
 * reads deliver the bits, MSB first, in bit 7).  It is consulted ONLY when
 * the operator names an A1200/A600 row through ide_board_force: 0xDExxxx is
 * Gary/Fat Gary space on an A3000/A4000, and the automatic probe must not
 * write there.  Automatic detection is A4000-only (chipset gate + task-file
 * readback); the A1200/A600 rows are untested and never auto-selected.
 */
struct ideboard {
	char	*name;
	ulong	base;		/* task file: DATA at base, others at base + IDE_REG_* */
	ulong	irqstat;	/* register whose bit 7 mirrors / latches INTRQ */
	ulong	irqack;		/* 0: none; else the INTREQ register to acknowledge */
	ulong	irqena;		/* 0: none; else the INTENA register (bit 7 = IDE) */
	uint	gayleid;	/* expected Gayle ID, 0: identified by chipset + readback */
	int	aga;		/* 1: AGA chipset expected, 0: OCS/ECS */
};

static struct ideboard	ideboards[] = {
	{ "A4000 IDE", 0xDD2020L, 0xDD3020L, 0L,        0L,        0x00, 1 },
	{ "A1200 IDE", 0xDA0000L, 0xDA9000L, 0xDA9000L, 0xDAA000L, 0xD1, 1 },
	{ "A600 IDE",  0xDA0000L, 0xDA9000L, 0xDA9000L, 0xDAA000L, 0xD0, 0 },
};
#define IDE_NBOARDS	((int)(sizeof ideboards / sizeof ideboards[0]))
#define IDE_GAYLE_ID	0xDE1000L
#define IDE_IRQ_BIT	0x80

static struct ideboard	*ide_bd;	/* the detected board, NULL until then */

/* task-file access relative to the board base */
#define TR( o)		RB( ide_bd->base + (o))
#define TW( o, v)	WB( ide_bd->base + (o), (v))
#define TRW()		RW( ide_bd->base + IDE_REG_DATA)
#define TWW( v)		WW( ide_bd->base + IDE_REG_DATA, (v))

static int
ide_irq_pending( bd)
struct ideboard	*bd;
{
	return (RB( bd->irqstat) & IDE_IRQ_BIT) != 0;
}

static void
ide_irq_ack( bd)
struct ideboard	*bd;
{
	uchar	v;

	if (bd->irqack) {
		v = RB( bd->irqack);
		WB( bd->irqack, 0x7C | (v & 0x03));	/* clear bit 7 only */
	}
}

static void
ide_irq_enable( bd, on)
struct ideboard	*bd;
int		on;
{
	uchar	v;

	if (bd->irqena) {
		v = RB( bd->irqena);
		WB( bd->irqena, on ? (v | IDE_IRQ_BIT) : (v & ~IDE_IRQ_BIT));
	}
}

static uint
ide_gayle_id()
{
	uint	v;
	int	i;

	WB( IDE_GAYLE_ID, 0);
	v = 0;
	for (i = 0; i < 8; ++i)
		v = (v << 1) | ((RB( IDE_GAYLE_ID) >> 7) & 1);
	return v;
}

/* ------------------------------------------------------------ unit state */

#define IDE_NDRIVES	2
#define IDE_MODEL_LEN	40

struct ideunit {
	bool	present;		/* IDENTIFY succeeded, LBA usable */
	bool	flush;			/* FLUSH CACHE supported (IDENTIFY word 83 bit 12) */
	ulong	nsectors;		/* LBA28 sector count */
	ushort	multi;			/* IDENTIFY word 47 low byte (informational) */
	char	model[IDE_MODEL_LEN + 1];
	uchar	skey, asc, ascq;	/* pending sense for REQUEST SENSE */
	ulong	errors;			/* kmem-readable error counter */
	uchar	lasterr;		/* last ATA error register value */
	uchar	laststat;		/* last ATA status register value */
};

static struct ideunit	ideunits[IDE_NDRIVES];
static int		ide_state;	/* 0 unprobed, 1 no controller, 2 present */
static int		ide_attached;	/* attach() done */
static int		ide_use_intr;	/* attach(): ideintr is in int2_tbl[] and enabled */

/* interrupt-path bookkeeping (kmem-readable) */
ulong	ide_intr_count, ide_intr_spurious, ide_intr_timeouts;
ulong	ide_intr_busy;		/* interrupts taken with the drive still BSY */
ulong	ide_dup_submits;	/* requests re-submitted while active or queued (ignored) */
ulong	ide_resets;		/* software resets issued after a timeout */

/* --------------------------------------------------- low-level helpers */

/*
 * Wait until the bits in `mask' equal `want' in the status register, or the
 * spin budget is exhausted.  Returns the final status, or -1 on timeout.
 * Reading STATUS (not ALTSTATUS) also drops a pending INTRQ, which the polled
 * paths want.
 */
static int
ide_wait( mask, want, spins)
uint	mask, want;
ulong	spins;
{
	int	st;

	do {
		st = TR( IDE_REG_STATUS);
		if ((st & mask) == want)
			return st;
	} while (spins-- != 0);
	return -1;
}

/* BSY clear and DRQ set, or BSY clear with ERR/DF set (caller inspects). */
static int
ide_wait_drq( spins)
ulong	spins;
{
	int	st;

	do {
		st = TR( IDE_REG_STATUS);
		if (!(st & IDE_ST_BSY) && (st & (IDE_ST_DRQ | IDE_ST_ERR | IDE_ST_DF)))
			return st;
	} while (spins-- != 0);
	return -1;
}

/* The ATA-2 400 ns settle after selecting a drive or issuing a command. */
static void
ide_settle()
{
	int	i;

	for (i = 0; i < 4; ++i)
		(void) TR( IDE_REG_ALTSTAT);
}

/* Select a drive and wait for BSY to drop.  Returns status or -1. */
static int
ide_select( drive)
int	drive;
{
	TW( IDE_REG_SDH, IDE_SDH_FIXED | IDE_SDH_LBA | (drive ? IDE_SDH_DRV1 : 0));
	ide_settle();
	return ide_wait( IDE_ST_BSY, 0, ide_spins);
}

static void
ide_setsense( u, skey, asc, ascq)
struct ideunit	*u;
int		skey, asc, ascq;
{
	u->skey = skey;
	u->asc  = asc;
	u->ascq = ascq;
	u->errors++;
}

/* Translate the ATA status/error registers after a failed command into sense. */
static void
ide_ataerror( u, st, write)
struct ideunit	*u;
int		st, write;
{
	int	er;

	er = TR( IDE_REG_ERROR);
	u->laststat = st;
	u->lasterr  = er;
	if (!(st & IDE_ST_ERR))			/* DF alone: the ERROR register is stale */
		ide_setsense( u, SK_HARDWARE, ASC_INTERNAL_FAILURE, 0);
	else if (er & IDE_ER_IDNF)
		ide_setsense( u, SK_ILLEGAL, ASC_LBA_OUT_OF_RANGE, 0);
	else if (er & (IDE_ER_UNC | IDE_ER_BBK | IDE_ER_AMNF))
		ide_setsense( u, SK_MEDIUM, write ? ASC_WRITE_ERROR : ASC_READ_ERROR, 0);
	else if (er & IDE_ER_ABRT)
		ide_setsense( u, SK_ILLEGAL, ASC_INVALID_CDB_FIELD, 0);
	else
		ide_setsense( u, SK_HARDWARE, ASC_INTERNAL_FAILURE, 0);
	if (ide_debug)
		printf( "ide%d: ATA error status 0x%x error 0x%x\n", (int)(u - ideunits), st, er);
}

/*
 * Move one 512-byte sector through the DATA register.  `nwords' < 256 keeps
 * the transfer with the drive complete while storing only what fits nbyte.
 */
static void
ide_pio_in( buf, nwords)
ushort	*buf;
int	nwords;
{
	int	i;
	ushort	w;

	for (i = 0; i < IDE_SECTOR / 2; ++i) {
		w = TRW();
		if (i < nwords) {
#if IDE_DATA_SWAP
			w = (w << 8) | (w >> 8);
#endif
			buf[i] = w;
		}
	}
}

static void
ide_pio_out( buf, nwords)
ushort	*buf;
int	nwords;
{
	int	i;
	ushort	w;

	for (i = 0; i < IDE_SECTOR / 2; ++i) {
		w = i < nwords ? buf[i] : 0;
#if IDE_DATA_SWAP
		w = (w << 8) | (w >> 8);
#endif
		TWW( w);
	}
}

/* -------------------------------------------------------- IDENTIFY */

/*
 * IDENTIFY DEVICE (polled).  On the Amiga the 16-bit DATA register delivers
 * the little-endian ATA words byte swapped relative to the 68k's view, so
 * every IDENTIFY word is swapped once: numeric fields become correct and the
 * ASCII strings (first character in bits 15:8 per ATA-2) come out in order.
 * Verified in the emulator (model string and sector count decode correctly);
 * [verify] on real hardware.  Sector data needs no swap (IDE_DATA_SWAP 0).
 */
static int
ide_identify( drive, id)
int	drive;
ushort	*id;
{
	int	st, i;
	ushort	w;

	if (ide_select( drive) < 0)
		return -1;
	TW( IDE_REG_COMMAND, IDE_CMD_IDENTIFY);
	ide_settle();
	st = ide_wait( IDE_ST_BSY, 0, ide_spins);
	if (st < 0 || (st & IDE_ST_ERR) || !(st & IDE_ST_DRQ))
		return -1;
	for (i = 0; i < IDE_SECTOR / 2; ++i) {
		w = TRW();
		id[i] = (w << 8) | (w >> 8);
	}
	return 0;
}

static void
ide_attach_unit( drive)
int	drive;
{
	static ushort	id[IDE_SECTOR / 2];	/* static: kernel stacks are small */
	struct ideunit	*u;
	int		i;
	char		*m;

	u = &ideunits[drive];
	u->present = FALSE;
	if (ide_identify( drive, id) < 0) {
		if (ide_debug)
			printf( "ide%d: no drive\n", drive);
		return;
	}
	if (!(id[IDE_ID_CAPS] & IDE_ID_CAPS_LBA)) {
		printf( "ide%d: drive without LBA support ignored\n", drive);
		return;
	}
	u->nsectors = ((ulong)id[IDE_ID_LBASECTORS + 1] << 16) | id[IDE_ID_LBASECTORS];
	if (u->nsectors == 0 || u->nsectors > IDE_LBA28_MAX + 1)
		u->nsectors = IDE_LBA28_MAX + 1;
	u->multi = id[IDE_ID_MULTI] & 0xFF;
	u->flush = ((id[IDE_ID_CMDSET2] & IDE_ID_CMDSET2_VALID) == 0x4000
		    && (id[IDE_ID_CMDSET2] & IDE_ID_CMDSET2_FLUSH)) ? TRUE : FALSE;
	m = u->model;
	for (i = 0; i < IDE_MODEL_LEN / 2; ++i) {
		*m++ = id[IDE_ID_MODEL + i] >> 8;
		*m++ = id[IDE_ID_MODEL + i] & 0xFF;
	}
	*m = '\0';
	while (m > u->model && m[-1] == ' ')	/* trim trailing blanks */
		*--m = '\0';
	u->present = TRUE;
	printf( "ide%d: %s, %d sectors (%d MB)\n", drive, u->model,
		(int)u->nsectors, (int)(u->nsectors / 2048));
}

/* Is ideintr in the kernel's level-2 table?  (NULL-terminated, kernel.c.) */
static int
ide_hooked()
{
	extern void	ideintr();
	int		i;

	for (i = 0; int2_tbl[i] != 0; ++i)
		if (int2_tbl[i] == ideintr)
			return 1;
	return 0;
}

static void
ide_attach()
{
	int	drive;

	ide_attached = 1;
	ide_use_intr = 0;
	TW( IDE_REG_CONTROL, IDE_CTL_NIEN);		/* quiet during the polled attach */
	for (drive = 0; drive < IDE_NDRIVES; ++drive)
		if (drive == 0 || ide_slave_enable)
			ide_attach_unit( drive);
		else
			ideunits[drive].present = FALSE;
	if (ide_intr_enable && ide_hooked()) {
		ide_use_intr = 1;
		ide_irq_enable( ide_bd, 1);
		TW( IDE_REG_CONTROL, 0);		/* drive may assert INTRQ */
	} else
		ide_irq_enable( ide_bd, 0);
	printf( "ide: %s, %s I/O\n", ide_bd->name, ide_use_intr ? "interrupt" : "polled");
}

/*
 * Software reset of the cable after a command timed out.  The drive may
 * still be executing the abandoned command and would otherwise complete it
 * -- interrupt, data and all -- into whatever request comes next.  SRST
 * (held for well over the 5 us ATA-2 asks) aborts it on both drives; then
 * BSY is waited out (bounded), STATUS is read to drop a pending INTRQ, the
 * Gayle latch is acknowledged where there is one, and interrupt enable is
 * restored to the attach-time setting.  No SET FEATURES state is lost: the
 * driver sets none.
 */
static void
ide_reset_drive()
{
	int	i, st;

	ide_resets++;
	TW( IDE_REG_CONTROL, IDE_CTL_SRST | IDE_CTL_NIEN);
	for (i = 0; i < 16; ++i)
		(void) TR( IDE_REG_ALTSTAT);
	TW( IDE_REG_CONTROL, IDE_CTL_NIEN);
	ide_settle();
	st = ide_wait( IDE_ST_BSY, 0, ide_spins_reset);
	(void) TR( IDE_REG_STATUS);
	ide_irq_ack( ide_bd);
	TW( IDE_REG_CONTROL, ide_use_intr ? 0 : IDE_CTL_NIEN);
	if (ide_debug)
		printf( "ide: drive reset after timeout, status 0x%x\n", st);
}

/* --------------------------------------------------------- polled I/O */

/* Program the task file for `n' sectors at `lba' and issue ATA command `cmd'. */
static void
ide_program( drive, cmd, lba, n)
int	drive, cmd;
ulong	lba, n;
{
	TW( IDE_REG_NSECT, n & 0xFF);			/* 256 -> 0 */
	TW( IDE_REG_SECT,  lba & 0xFF);
	TW( IDE_REG_LCYL, (lba >> 8) & 0xFF);
	TW( IDE_REG_HCYL, (lba >> 16) & 0xFF);
	TW( IDE_REG_SDH,   IDE_SDH_FIXED | IDE_SDH_LBA
			 | (drive ? IDE_SDH_DRV1 : 0) | ((lba >> 24) & IDE_SDH_HEAD));
	TW( IDE_REG_COMMAND, cmd);
	ide_settle();
}

/*
 * Polled READ VERIFY SECTOR(S): the drive reads and checks `nsect' sectors
 * from `lba' without transferring them.  Returns 0 or -1 (sense set).
 */
static int
ide_verify_polled( u, drive, lba, nsect)
struct ideunit	*u;
int		drive;
ulong		lba, nsect;
{
	ulong	chunk;
	int	st;

	while (nsect > 0) {
		chunk = nsect > IDE_MAXSECT ? IDE_MAXSECT : nsect;
		st = ide_select( drive);
		if (st < 0 || !(st & IDE_ST_DRDY)) {
			ide_setsense( u, SK_NOT_READY, ASC_LUN_NOT_READY, 0);
			return -1;
		}
		ide_program( drive, IDE_CMD_VERIFY, lba, chunk);
		st = ide_wait( IDE_ST_BSY, 0, ide_spins);
		if (st < 0 || (st & (IDE_ST_ERR | IDE_ST_DF))) {
			if (st < 0) {
				ide_setsense( u, SK_HARDWARE, ASC_LU_COMM_FAILURE, 0);
				ide_reset_drive();
			} else
				ide_ataerror( u, st, 0);
			return -1;
		}
		lba   += chunk;
		nsect -= chunk;
	}
	return 0;
}

/*
 * Polled READ/WRITE SECTOR(S): up to IDE_MAXSECT sectors per command, split
 * as needed.  Returns 0 or -1 (sense set).
 */
static int
ide_rw_polled( u, drive, write, lba, nsect, data, nbyte)
struct ideunit	*u;
int		drive, write;
ulong		lba, nsect;
uchar		*data;
ulong		nbyte;
{
	ulong	chunk, done, left;
	int	st, n;

	done = 0;
	while (nsect > 0) {
		chunk = nsect > IDE_MAXSECT ? IDE_MAXSECT : nsect;
		st = ide_select( drive);
		if (st < 0 || !(st & IDE_ST_DRDY)) {
			ide_setsense( u, SK_NOT_READY, ASC_LUN_NOT_READY, 0);
			return -1;
		}
		ide_program( drive, write ? IDE_CMD_WRITE : IDE_CMD_READ, lba, chunk);
		for (n = 0; n < (int)chunk; ++n) {
			st = ide_wait_drq( ide_spins);
			if (st < 0 || (st & (IDE_ST_ERR | IDE_ST_DF))) {
				if (st < 0) {
					u->laststat = TR( IDE_REG_STATUS);
					ide_setsense( u, SK_HARDWARE, ASC_LU_COMM_FAILURE, 0);
					if (ide_debug)
						printf( "ide%d: timeout, status 0x%x\n", drive, u->laststat);
					ide_reset_drive();
				} else
					ide_ataerror( u, st, write);
				return -1;
			}
			left = nbyte > done ? nbyte - done : 0;
			if (left > IDE_SECTOR)
				left = IDE_SECTOR;
			if (write)
				ide_pio_out( (ushort *)(data + done), (int)(left / 2));
			else
				ide_pio_in( (ushort *)(data + done), (int)(left / 2));
			done += IDE_SECTOR;
		}
		st = ide_wait( IDE_ST_BSY, 0, ide_spins);
		if (st < 0 || (st & (IDE_ST_ERR | IDE_ST_DF))) {
			if (st < 0) {
				ide_setsense( u, SK_HARDWARE, ASC_LU_COMM_FAILURE, 0);
				ide_reset_drive();
			} else
				ide_ataerror( u, st, write);
			return -1;
		}
		lba   += chunk;
		nsect -= chunk;
	}
	return 0;
}

static int
ide_flush( u, drive)
struct ideunit	*u;
int		drive;
{
	int	st;

	if (!u->flush)
		return 0;			/* nothing to do on an old drive */
	if (ide_select( drive) < 0)
		return -1;
	TW( IDE_REG_COMMAND, IDE_CMD_FLUSH);
	ide_settle();
	st = ide_wait( IDE_ST_BSY, 0, ide_spins_reset);	/* flush can take a while */
	if (st < 0 || (st & IDE_ST_ERR)) {
		if (st >= 0)
			ide_ataerror( u, st, 1);
		else {
			ide_reset_drive();
			ide_setsense( u, SK_HARDWARE, ASC_LU_COMM_FAILURE, 0);
		}
		return -1;
	}
	return 0;
}

/* --------------------------------------------- completion delivery */

/*
 * Completion FIFO: dd.c's completion routine starts the next request from
 * inside (*cp->intr)(cp), which re-enters idequeue().  The first completion
 * drains the FIFO; completions issued from inside an intr() merely append and
 * return.  Protected by sdspl (spl2) like everything else on this path; from
 * ideintr() (already at IPL 2) the bracket is a no-op.
 */
#define IDE_CQ	32			/* power of two */
static struct sdcom	*ide_cq[IDE_CQ];
static uint		ide_cq_head, ide_cq_tail;
static int		ide_completing;
ulong			ide_cq_overflow;	/* completions delivered nested because the FIFO was full */

static void
ide_complete( cp)
struct sdcom	*cp;
{
	struct sdcom	*p;
	uint		nt;
	int		s;

	s = sdspl();
	nt = (ide_cq_tail + 1) & (IDE_CQ - 1);
	if (nt == ide_cq_head) {		/* full: deliver now rather than lose it */
		if (ide_cq_overflow++ == 0)
			printf( "ide: completion queue full, delivering directly\n");
		splx( s);
		(*cp->intr)( cp);
		return;
	}
	ide_cq[ide_cq_tail] = cp;
	ide_cq_tail = nt;
	if (ide_completing) {
		splx( s);
		return;
	}
	ide_completing = 1;
	while (ide_cq_head != ide_cq_tail) {
		p = ide_cq[ide_cq_head];
		ide_cq_head = (ide_cq_head + 1) & (IDE_CQ - 1);
		splx( s);
		(*p->intr)( p);
		s = sdspl();
	}
	ide_completing = 0;
	splx( s);
}

/* ------------------------------------------- interrupt-driven I/O */

/*
 * One request is active on the cable at a time; the others wait in a FIFO
 * built on cp->next (ours while we own the request).  ide_x describes the
 * active transfer.  All of this is touched only at IPL 2 (sdspl bracket or
 * the level-2 handler).
 */
static struct sdcom	*ide_qhead, *ide_qtail, *ide_active;
static struct {
	struct ideunit	*u;
	int		drive, write;
	int		nodata;		/* 0: READ/WRITE; else the ATA command that moves no data
				 * (FLUSH CACHE, READ VERIFY) -- one interrupt per command */
	ulong		cmdn;		/* sectors covered by the current no-data command */
	ulong		lba;		/* next LBA to issue / transfer */
	ulong		left;		/* sectors of the request not yet transferred */
	ulong		cmdleft;	/* sectors still to move in the current ATA command */
	uchar		*data;
	ulong		nbyte, done;
	ulong		gen;		/* bumped per started request: the watchdog counts ticks per gen */
} ide_x;
static int	ide_watch_armed;
static ulong	ide_watch_ticks, ide_watch_gen;

static void	ide_start();
static void	ide_watchdog();
static void	ide_reset_drive();
int		idepresent();

/* Is cp on the cable or waiting for it?  (dd.c can hand us the same sdcom
 * twice: its completion routine calls iodone() before it starts the next
 * buffer, and iodone's callback may resubmit the disk's single sdcom while
 * the first submission is still in flight.  Linking it again would make the
 * queue eat its own tail.) */
static int
ide_pending( cp)
struct sdcom	*cp;
{
	struct sdcom	*p;

	if (cp == ide_active)
		return 1;
	for (p = ide_qhead; p; p = (struct sdcom *)p->next)
		if (p == cp)
			return 1;
	return 0;
}

static void
ide_enqueue( cp)
struct sdcom	*cp;
{
	cp->next = 0;
	if (ide_qtail)
		ide_qtail->next = cp;
	else
		ide_qhead = cp;
	ide_qtail = cp;
}

static struct sdcom *
ide_dequeue()
{
	struct sdcom	*cp;

	cp = ide_qhead;
	if (cp) {
		ide_qhead = (struct sdcom *)cp->next;
		if (ide_qhead == 0)
			ide_qtail = 0;
		cp->next = 0;
	}
	return cp;
}

/* Copy one sector of the active transfer and advance the counters. */
static void
ide_xfer_sector()
{
	ulong	left;

	left = ide_x.nbyte > ide_x.done ? ide_x.nbyte - ide_x.done : 0;
	if (left > IDE_SECTOR)
		left = IDE_SECTOR;
	if (ide_x.write)
		ide_pio_out( (ushort *)(ide_x.data + ide_x.done), (int)(left / 2));
	else
		ide_pio_in( (ushort *)(ide_x.data + ide_x.done), (int)(left / 2));
	ide_x.done += IDE_SECTOR;
	ide_x.lba++;
	ide_x.left--;
	ide_x.cmdleft--;
}

/* Finish the active request with `err' (0 ok, -1 sense set) and start the next. */
static void
ide_finish( err)
int	err;
{
	struct sdcom	*cp;

	cp = ide_active;
	ide_active = 0;
	ide_watch_ticks = 0;
	if (cp) {
		cp->status = err ? SC_CHECK : SC_GOOD;
		cp->okay   = TRUE;
		ide_complete( cp);		/* may re-enter idequeue(): it just enqueues */
	}
	if (ide_active == 0 && ide_qhead)
		ide_start();
}

/*
 * Issue the next ATA command of the active request.  A write must be primed
 * with its first sector before the drive interrupts; a read interrupts as soon
 * as the first sector is ready; a no-data command (FLUSH CACHE, READ VERIFY)
 * interrupts once, when done (ideintr() sees cmdleft == 0, accounts for the
 * sectors it covered and finishes or issues the next chunk).
 */
static void
ide_issue()
{
	ulong	n;
	int	st;

	if (ide_x.nodata == IDE_CMD_FLUSH && !ide_x.u->flush) {	/* old drive: nothing to flush */
		ide_finish( 0);
		return;
	}
	n = ide_x.left > IDE_MAXSECT ? IDE_MAXSECT : ide_x.left;
	st = ide_select( ide_x.drive);
	if (st < 0 || !(st & IDE_ST_DRDY)) {
		ide_setsense( ide_x.u, SK_NOT_READY, ASC_LUN_NOT_READY, 0);
		ide_finish( -1);
		return;
	}
	if (ide_x.nodata) {
		ide_x.cmdleft = 0;
		ide_x.cmdn = n;
		if (ide_x.nodata == IDE_CMD_FLUSH) {
			TW( IDE_REG_COMMAND, IDE_CMD_FLUSH);
			ide_settle();
		} else
			ide_program( ide_x.drive, ide_x.nodata, ide_x.lba, n);
	} else {
		ide_x.cmdleft = n;
		ide_program( ide_x.drive, ide_x.write ? IDE_CMD_WRITE : IDE_CMD_READ, ide_x.lba, n);
	}
	if (ide_x.write) {
		st = ide_wait_drq( ide_spins_short);
		if (st < 0 || (st & (IDE_ST_ERR | IDE_ST_DF))) {
			if (st < 0)
				ide_setsense( ide_x.u, SK_HARDWARE, ASC_LU_COMM_FAILURE, 0);
			else
				ide_ataerror( ide_x.u, st, 1);
			ide_finish( -1);
			return;
		}
		ide_xfer_sector();
	}
	if (!ide_watch_armed) {
		ide_watch_armed = 1;
		timeout( ide_watchdog, (char *)0, HZ);
	}
}

/* Take the next request off the FIFO and put it on the cable. */
static void
ide_start()
{
	struct sdcom	*cp;

	cp = ide_dequeue();
	if (cp == 0)
		return;
	ide_active = cp;
	ide_x.gen++;
	/* the transfer description is rebuilt from the CDB, so queued requests
	 * need no side storage beyond cp->next */
	ide_x.write = (cp->cdb[0] == SC_WRITE10 || cp->cdb[0] == SC_WRITE6);
	ide_x.nodata = cp->cdb[0] == SC_SYNC_CACHE ? IDE_CMD_FLUSH
		     : cp->cdb[0] == SC_VERIFY10   ? IDE_CMD_VERIFY : 0;
	ide_x.cmdn = 0;
	if (ide_x.nodata == IDE_CMD_FLUSH) {
		ide_x.lba  = 0;
		ide_x.left = 0;
	} else if (cp->cdb[0] == SC_READ6 || cp->cdb[0] == SC_WRITE6) {
		ide_x.lba  = ((ulong)(cp->cdb[1] & 0x1F) << 16) | ((ulong)cp->cdb[2] << 8) | cp->cdb[3];
		ide_x.left = cp->cdb[4] ? cp->cdb[4] : 256;
	} else {
		ide_x.lba  = ((ulong)cp->cdb[2] << 24) | ((ulong)cp->cdb[3] << 16)
			   | ((ulong)cp->cdb[4] << 8)  | cp->cdb[5];
		ide_x.left = ((ulong)cp->cdb[7] << 8) | cp->cdb[8];
	}
	ide_x.drive = (int)cp->unit == ide_master_unit ? 0 : 1;
	ide_x.u     = &ideunits[ide_x.drive];
	ide_x.data  = (uchar *)cp->addr;
	ide_x.nbyte = cp->nbyte;
	ide_x.done  = 0;
	ide_issue();
}

/*
 * Level-2 handler (int2_tbl[]).  The dispatcher has already cleared INTREQ
 * PORTS; we test the board's own status bit and return quietly when the
 * interrupt is not ours (INT2 is shared with CIA-A and Zorro II).
 */
void
ideintr()
{
	int	st;

	if (ide_bd == 0 || ide_state != 2 || !ide_irq_pending( ide_bd))
		return;
	st = TR( IDE_REG_STATUS);		/* drops INTRQ at the drive */
	ide_irq_ack( ide_bd);			/* and at the Gayle latch, where there is one */
	ide_intr_count++;
	if (ide_active == 0) {			/* polled command's leftover, or noise */
		ide_intr_spurious++;
		return;
	}
	if (st & IDE_ST_BSY) {			/* not for us yet; the drive will interrupt again */
		ide_intr_busy++;
		return;
	}
	if (st & (IDE_ST_ERR | IDE_ST_DF)) {
		ide_ataerror( ide_x.u, st, ide_x.write);
		ide_finish( -1);
		return;
	}
	if (ide_x.cmdleft > 0) {
		if (!(st & IDE_ST_DRQ)) {
			st = ide_wait_drq( ide_spins_short);
			if (st < 0 || (st & (IDE_ST_ERR | IDE_ST_DF))) {
				if (st < 0)
					ide_setsense( ide_x.u, SK_HARDWARE, ASC_LU_COMM_FAILURE, 0);
				else
					ide_ataerror( ide_x.u, st, ide_x.write);
				ide_finish( -1);
				return;
			}
		}
		ide_xfer_sector();
		if (ide_x.cmdleft > 0 || ide_x.write)
			return;			/* more sectors, or the write's final interrupt is due */
	}
	/* command complete: settle, check the final status */
	st = ide_wait( IDE_ST_BSY, 0, ide_spins_short);
	if (st < 0 || (st & (IDE_ST_ERR | IDE_ST_DF))) {
		if (st < 0)
			ide_setsense( ide_x.u, SK_HARDWARE, ASC_LU_COMM_FAILURE, 0);
		else
			ide_ataerror( ide_x.u, st, ide_x.write);
		ide_finish( -1);
		return;
	}
	if (ide_x.nodata) {			/* the command covered cmdn sectors */
		ide_x.lba  += ide_x.cmdn;
		ide_x.left  = ide_x.left > ide_x.cmdn ? ide_x.left - ide_x.cmdn : 0;
	}
	if (ide_x.left > 0)
		ide_issue();
	else
		ide_finish( 0);
}

/*
 * Clock callout, once per second while interrupt-driven requests exist.  A
 * command that has not completed within ide_timeout_secs is failed with a
 * hardware-error sense; dd.c will retry it once.
 */
static void
ide_watchdog()
{
	int	s;

	s = sdspl();
	if (ide_active) {
		if (ide_watch_gen != ide_x.gen) {	/* a new request since the last tick */
			ide_watch_gen = ide_x.gen;
			ide_watch_ticks = 0;
		}
		if (++ide_watch_ticks > ide_timeout_secs) {
			ide_intr_timeouts++;
			ide_x.u->laststat = TR( IDE_REG_STATUS);
			printf( "ide%d: command timed out (status 0x%x)\n", ide_x.drive, ide_x.u->laststat);
			ide_setsense( ide_x.u, SK_HARDWARE, ASC_LU_COMM_FAILURE, 0);
			ide_reset_drive();		/* quiet cable before the next request starts */
			ide_finish( -1);
		}
	}
	if (ide_active || ide_qhead)
		timeout( ide_watchdog, (char *)0, HZ);
	else
		ide_watch_armed = 0;
	splx( s);
}

/* ------------------------------------------------ SCSI CDB emulation */

static void
ide_zero( d, n)
uchar	*d;
uint	n;
{
	while (n-- > 0)
		*d++ = 0;
}

/* SCSI unit -> drive index (0 master, 1 slave), or -1 when nothing is mapped. */
static int
ide_drive_of( unit)
uint	unit;
{
	if ((int)unit == ide_master_unit)
		return 0;
	if ((int)unit == ide_slave_unit && ide_slave_enable)
		return 1;
	return -1;
}

/*
 * The host-adapter queue entry: execute cp->cdb for cp->unit and complete.
 * Second argument `c' is the card index sd.c registered us with (always 0 from
 * the probe path); unused.
 */
bool
idequeue( c, cp)
int		c;
struct sdcom	*cp;
{
	struct ideunit	*u;
	uchar		*data;
	uchar		op;
	ulong		lba, nblk, n;
	int		drive, write, s, i;

	s = sdspl();
	if (ide_pending( cp)) {			/* already in flight: it completes once, later */
		ide_dup_submits++;
		if (ide_debug)
			printf( "ide: request re-submitted while in flight, ignored\n");
		splx( s);
		return TRUE;
	}
	if (ide_state == 0) {			/* sd.c probes first; be robust anyway */
		char	*pa;
		(void) idepresent( &pa);
	}
	if (ide_state == 2 && !ide_attached)
		ide_attach();

	drive = ide_drive_of( cp->unit);
	data  = (uchar *)cp->addr;
	op    = cp->cdb[0];
	cp->status = SC_GOOD;
	cp->okay   = TRUE;
	write      = 0;

	if (ide_state != 2 || drive < 0 || !ideunits[drive].present) {
		cp->okay = FALSE;		/* no such device */
		splx( s);
		ide_complete( cp);
		return TRUE;
	}
	u = &ideunits[drive];
	if (ide_debug > 1)
		printf( "ide%d: unit %d cdb 0x%x nbyte %d\n", drive, (int)cp->unit, op, cp->nbyte);

	if (cp->cdb[1] & 0xE0) {		/* a LUN other than 0: there is none */
		if (op == SC_INQUIRY) {		/* SCSI-2 8.2.5.1: qualifier 011b, type 1Fh */
			if (data && cp->nbyte) {
				ide_zero( data, cp->nbyte);
				data[0] = 0x7F;
			}
		} else {
			ide_setsense( u, SK_ILLEGAL, ASC_LUN_NOT_SUPPORTED, 0);
			cp->status = SC_CHECK;
		}
		splx( s);
		ide_complete( cp);
		return TRUE;
	}

	switch (op) {
	case SC_TUR:
	case SC_REZERO:
	case SC_START_STOP:
	case SC_MODE_SELECT6:
		break;

	case SC_VERIFY10:
		/* READ VERIFY SECTOR(S): the drive checks the sectors, no data
		 * moves.  BYTCHK (compare against supplied data) is not offered. */
		if (cp->cdb[1] & 0x02) {
			ide_setsense( u, SK_ILLEGAL, ASC_INVALID_CDB_FIELD, 0);
			cp->status = SC_CHECK;
			break;
		}
		lba  = ((ulong)cp->cdb[2] << 24) | ((ulong)cp->cdb[3] << 16)
		     | ((ulong)cp->cdb[4] << 8)  | cp->cdb[5];
		nblk = ((ulong)cp->cdb[7] << 8) | cp->cdb[8];
		if (nblk == 0)
			break;				/* nothing to verify: GOOD */
		if (lba >= u->nsectors || nblk > u->nsectors - lba) {
			ide_setsense( u, SK_ILLEGAL, ASC_LBA_OUT_OF_RANGE, 0);
			cp->status = SC_CHECK;
			break;
		}
		if (ide_use_intr) {
			ide_enqueue( cp);
			if (ide_active == 0)
				ide_start();
			splx( s);
			return TRUE;		/* completes from ideintr() */
		}
		if (ide_verify_polled( u, drive, lba, nblk) < 0)
			cp->status = SC_CHECK;
		break;

	case SC_REQ_SENSE:
		if (data && cp->nbyte) {
			n = cp->nbyte < 18 ? cp->nbyte : 18;
			ide_zero( data, cp->nbyte);
			if (n > 0)  data[0]  = 0x70;	/* current error, fixed format */
			if (n > 2)  data[2]  = u->skey;
			if (n > 7)  data[7]  = 0x0A;	/* additional sense length */
			if (n > 12) data[12] = u->asc;
			if (n > 13) data[13] = u->ascq;
			u->skey = u->asc = u->ascq = 0;	/* consumed on read */
		}
		break;

	case SC_INQUIRY:
		if (data && cp->nbyte) {
			static char	vendor[] = "AMIGA   ";
			static char	rev[]    = "0.2 ";
			ide_zero( data, cp->nbyte);
			if (cp->nbyte > 4) {
				data[0] = 0x00;		/* direct access */
				data[1] = 0x00;		/* fixed medium */
				data[2] = 0x02;		/* SCSI-2 */
				data[3] = 0x02;		/* response data format 2 */
				data[4] = 0x1F;		/* additional length (36 bytes) */
			}
			for (i = 0; i < 8 && 8 + i < (int)cp->nbyte; ++i)
				data[8 + i] = vendor[i];
			for (i = 0; i < 16 && 16 + i < (int)cp->nbyte; ++i)
				data[16 + i] = u->model[i] ? u->model[i] : ' ';
			for (i = 0; i < 4 && 32 + i < (int)cp->nbyte; ++i)
				data[32 + i] = rev[i];
		}
		break;

	case SC_READ_CAP10:
		if (data && cp->nbyte >= 8) {
			lba = u->nsectors - 1;
			data[0] = lba >> 24; data[1] = lba >> 16;
			data[2] = lba >> 8;  data[3] = lba;
			data[4] = 0; data[5] = 0;
			data[6] = IDE_SECTOR >> 8; data[7] = IDE_SECTOR & 0xFF;
		}
		break;

	case SC_MODE_SENSE6:
		if (data && cp->nbyte >= 12) {
			ide_zero( data, cp->nbyte);
			n = (u->nsectors - 1) & 0xFFFFFF;
			data[0] = 3 + 8;		/* mode data length */
			data[3] = 8;			/* block descriptor length */
			data[5] = n >> 16; data[6] = n >> 8; data[7] = n;
			data[10] = IDE_SECTOR >> 8; data[11] = IDE_SECTOR & 0xFF;
		} else {
			ide_setsense( u, SK_ILLEGAL, ASC_INVALID_CDB_FIELD, 0);
			cp->status = SC_CHECK;
		}
		break;

	case SC_SYNC_CACHE:
		/* Interrupt mode: take the queue like a transfer, so the flush
		 * runs after the writes queued before it and never overwrites
		 * the task file of a command in flight.  Polled mode has
		 * nothing in flight and flushes inline. */
		if (ide_use_intr) {
			ide_enqueue( cp);
			if (ide_active == 0)
				ide_start();
			splx( s);
			return TRUE;		/* completes from ideintr() */
		}
		if (ide_flush( u, drive) < 0)
			cp->status = SC_CHECK;
		break;

	case SC_WRITE6:
		write = 1;
		/* fall through */
	case SC_READ6:
		lba  = ((ulong)(cp->cdb[1] & 0x1F) << 16) | ((ulong)cp->cdb[2] << 8) | cp->cdb[3];
		nblk = cp->cdb[4];
		if (nblk == 0)
			nblk = 256;
		goto rw;

	case SC_WRITE10:
		write = 1;
		/* fall through */
	case SC_READ10:
		lba  = ((ulong)cp->cdb[2] << 24) | ((ulong)cp->cdb[3] << 16)
		     | ((ulong)cp->cdb[4] << 8)  | cp->cdb[5];
		nblk = ((ulong)cp->cdb[7] << 8) | cp->cdb[8];
	rw:
		if (data == 0 || cp->nbyte == 0) {
			ide_setsense( u, SK_ILLEGAL, ASC_INVALID_CDB_FIELD, 0);
			cp->status = SC_CHECK;
			break;
		}
		if ((ulong)cp->nbyte <= IDE_NBPP
		    && (((ulong)data ^ ((ulong)data + cp->nbyte - 1)) & ~(IDE_NBPP - 1))) {
			if (ide_pagecross_n++ == 0 || ide_debug)
				printf( "ide: raw request of %d bytes crosses a page boundary\n", cp->nbyte);
		}
		if (nblk == 0 || lba >= u->nsectors || nblk > u->nsectors - lba) {
			ide_setsense( u, SK_ILLEGAL, ASC_LBA_OUT_OF_RANGE, 0);
			cp->status = SC_CHECK;
			break;
		}
		if (ide_use_intr) {
			ide_enqueue( cp);
			if (ide_active == 0)
				ide_start();
			splx( s);
			return TRUE;		/* completes from ideintr() */
		}
		if (ide_rw_polled( u, drive, write, lba, nblk, data, (ulong)cp->nbyte) < 0)
			cp->status = SC_CHECK;
		break;

	default:
		ide_setsense( u, SK_ILLEGAL, ASC_INVALID_OPCODE, 0);
		cp->status = SC_CHECK;
		break;
	}

	splx( s);
	ide_complete( cp);
	return TRUE;
}

/* ------------------------------------------------------------- probe */

/*
 * Write two distinct values to the master's sector count / sector number
 * registers and read them back.  Open bus or a missing drive does not hold
 * both.  Caller has set ide_bd.
 */
static int
ide_readback_regs()
{
	TW( IDE_REG_NSECT, 0x55);
	TW( IDE_REG_SECT,  0xAA);
	if (TR( IDE_REG_NSECT) != 0x55 || TR( IDE_REG_SECT) != 0xAA)
		return 0;
	TW( IDE_REG_NSECT, 0xAA);
	TW( IDE_REG_SECT,  0x55);
	return TR( IDE_REG_NSECT) == 0xAA && TR( IDE_REG_SECT) == 0x55;
}

/*
 * Is a drive answering at the board's task file?  The readback runs first,
 * so a port with nothing attached (open bus, status reads 0xFF) is settled in
 * a dozen accesses.  Only when the registers do not hold AND the status shows
 * a drive still busy (power-on reset, not 0xFF) is BSY waited out, bounded,
 * and the readback retried.  Leaves the drive with interrupts disabled.
 */
static int
ide_readback( bd)
struct ideboard	*bd;
{
	int	st, ok;

	ide_bd = bd;
	TW( IDE_REG_CONTROL, IDE_CTL_NIEN);
	TW( IDE_REG_SDH, IDE_SDH_FIXED | IDE_SDH_LBA);
	ide_settle();
	ok = ide_readback_regs();
	if (!ok) {
		st = TR( IDE_REG_ALTSTAT);
		if (st != 0xFF && (st & IDE_ST_BSY)) {
			st = ide_wait( IDE_ST_BSY, 0, ide_spins_reset);
			if (st >= 0)
				ok = ide_readback_regs();
		}
	}
	ide_bd = 0;
	return ok;
}

/*
 * Board detection.  The automatic probe knows the A4000 only, and no address
 * outside the custom chips is touched before a gate says a controller can be
 * there: read VPOSR; on an AGA chipset look for a drive answering at the
 * A4000 task file; anything else (A3000, A2000 ...) is "no controller" after
 * that single VPOSR read.  The A1200/A600 rows are reached only through
 * ide_board_force (1.. = row): the row's Gayle ID, where it has one, is then
 * confirmed at 0xDE1000 -- the one place that register is ever read -- and
 * the readback still has to succeed.  -1 keeps the driver from probing at all.
 */
static struct ideboard *
ide_detect()
{
	struct ideboard	*bd;
	ushort		vposr;
	uint		id;

	if (ide_board_force < 0)
		return 0;
	if (ide_board_force > 0 && ide_board_force <= IDE_NBOARDS) {
		bd = &ideboards[ide_board_force - 1];
		if (bd->gayleid) {
			id = ide_gayle_id();
			if (id != bd->gayleid) {
				if (ide_debug)
					printf( "ide: forced %s but Gayle id is 0x%x, not 0x%x\n",
						bd->name, id, bd->gayleid);
				return 0;
			}
		}
		return ide_readback( bd) ? bd : 0;
	}
	vposr = RW( IDE_VPOSR_ADDR);
	if (IDE_VPOSR_IS_AGA( vposr) && ide_readback( &ideboards[0]))
		return &ideboards[0];
	if (ide_debug)
		printf( "ide: no Gayle IDE (VPOSR id 0x%x)\n", IDE_VPOSR_ID( vposr));
	return 0;
}

/*
 * Probe fallback named in driver.conf.  Called once by sd.c's init() with no
 * autocon entry to go on.  Registration also succeeds with a drive that later
 * fails IDENTIFY -- then every request completes with okay == FALSE.
 */
int
idepresent( ap)
char	**ap;
{
	if (ide_state == 0) {
		ide_state = 1;
		ide_bd = ide_detect();
		if (ide_bd) {
			ide_state = 2;
			printf( "ide: %s at 0x%x present\n", ide_bd->name, (int)ide_bd->base);
		}
	}
	if (ide_state != 2)
		return 0;
	*ap = (char *)ide_bd->base;
	return 1;
}

/*
 * Forget the probe and attach results so the next idepresent()/idequeue()
 * redo them (kernel debugger aid; the host harness uses it between cases).
 */
void
idereset()
{
	int	i;

	ide_state = 0;
	ide_attached = 0;
	ide_use_intr = 0;
	ide_bd = 0;
	ide_qhead = ide_qtail = ide_active = 0;
	ide_watch_armed = 0;
	ide_watch_ticks = 0;
	for (i = 0; i < IDE_NDRIVES; ++i)
		ideunits[i].present = FALSE;
}
