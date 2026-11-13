/* SPDX-License-Identifier: MIT */
/*
 * ide.c -- Amix (Commodore Amiga UNIX 2.1, SVR4 m68k) pseudo SCSI host adapter
 *          for the Gayle IDE port of the Amiga 4000.  (The board table also
 *          carries A1200/A600 rows: in the static driver untested and reachable
 *          only by ide_board_force; the module drives the A1200 row, see below.)
 *
 * Phase 2: interrupt-driven READ/WRITE (level 2, via int2_tbl[]) with a polled
 * fallback, PIO only, LBA28, master drive by default.  On the 68040/68060
 * line (-DIDE_CACHE=1) the fallback is the only mode: int2_tbl[] there is a
 * fixed table without ideintr (README, "The 68040/68060 kernel line").
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
 * Built with -DIDE_MODULE the same source is the root-class loadable kernel
 * module `gayle' (the section at the end of this file; module/ and
 * docs/MODULE-PORT.md): the module probes and attaches in its load entry,
 * registers with sd_register(), and attaches its interrupt later through
 * intr_attach() after a self-test.  It drives the A4000 and the A1200 rows,
 * told apart by the Kickstart ROM, never by the Gayle ID register.  The
 * int2_tbl[] search, the sd.c probe hook idepresent() and the A600 row are
 * not in the module.  Without the switch this file is the static driver,
 * byte for byte.
 *
 * Built with -DIDE_DIAG (module: make -C module DIAG=1) the object also
 * carries the investigation-only symbols: the spin high-water ide_spins_hw,
 * and on the 68040/68060 line the data-cache A/B switch ide_cache and the
 * ide_push_n / ide_push_bytes counters.  A default build has none of them,
 * so the line operations cannot be switched off from kmem.
 *
 * Built with -DIDE_BUDDHA (implies IDE_MODULE; make -C module, the second
 * module) the same source is the root-class module `buddha': the Individual
 * Computers Buddha and the Catweasel Z-II, Zorro II IDE boards found through
 * the AutoConfig list.  Everything specific to them is in the sections marked
 * BUDDHA; the shared code reaches the board through the macros those sections
 * define (IDE_CHAN, IDE_DRV1, ...), which a Gayle build defines to exactly
 * what it compiled before, so the static driver and `gayle' keep their bytes.
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
#if IDE_BUDDHA && !defined(IDE_MODULE)
#define IDE_MODULE	1		/* the Buddha back end exists only as a module */
#endif
#include	"sys/param.h"		/* HZ */
#include	"sys/types.h"
#include	"sys/errno.h"
#include	"sys/inline.h"		/* spl2()/splx() */
#include	"rico.h"
#include	"sd.h"

extern int	printf();
extern int	timeout();
#ifndef IDE_MODULE
extern void	(*int2_tbl[])();	/* master.d/kernel.c, NULL terminated */
#endif

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
 * the low 1 GB is identity mapped through the transparent translation
 * registers), so the driver dereferences plain addresses.  Cache inhibition
 * of these ranges on the 68030 rests on the stock kernel's transparent-
 * translation setup (works on the real A4000; the TT0/TT1 CI bits are not
 * read back there); on the 68040/68060 line DTT0 (0x003fc060, pstart040.s)
 * covers them cache-inhibited for supervisor data, which an IDE_CACHE build
 * checks at attach (see "68040/68060 data cache" below).
 */
/* Base addresses live in the board table (struct ideboard) -- never here. */
#if IDE_BUDDHA	/* BUDDHA: the control block lies 0x11A above a port's base */
#define IDE_SPAN	0x011BL
#else
#define IDE_SPAN	0x101BL		/* bytes from a task-file base to the last register + 1 */
#endif

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
#if IDE_BUDDHA	/* BUDDHA: CS1 register 6, 0x100 + 2 + 6 * 4 above the port */
#define IDE_REG_ALTSTAT	0x011A
#define IDE_REG_CONTROL	0x011A
#else
#define IDE_REG_ALTSTAT	0x101A		/* R   (does not clear INTRQ) */
#define IDE_REG_CONTROL	0x101A		/*   W device control */
#endif

/* the same registers under their ATAPI names (ATA/ATAPI-4 9.10) */
#define IDE_REG_IREASON	IDE_REG_NSECT	/* R   interrupt reason */
#define IDE_REG_BCLO	IDE_REG_LCYL	/* R/W byte count 7:0  */
#define IDE_REG_BCHI	IDE_REG_HCYL	/* R/W byte count 15:8 */
#define IDE_IR_COD	0x01		/* 1: the DRQ phase moves the command packet */
#define IDE_IR_IO	0x02		/* 1: device to host */
#define IDE_ATAPI_SIGLO	0x14		/* LCYL/HCYL after an ATAPI device aborts IDENTIFY */
#define IDE_ATAPI_SIGHI	0xEB

#define IDE_IRQ_IDE	0x80		/* Gayle IRQ status: IDE interrupt pending */

/*
 * Chipset gate: VPOSR bits 8..14 hold the Agnus/Alice id.  OCS 0x00/0x10, ECS
 * 0x20/0x30, AGA Alice 0x22/0x23/0x32/0x33 -- bit 4 (0x10) is the video
 * standard, the low nibble is >= 2 only for Alice.  (Testing "id >= 0x22", as
 * some sources do, misclassifies an NTSC ECS machine, id 0x30, as AGA.)
 * Verified against the Amiberry/WinUAE VPOSR model and on a real PAL A4000
 * (2026-10-10: VPOSR 0xA300, id 0x23).
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
#define IDE_CMD_PACKET		0xA0	/* ATAPI: a SCSI-style command packet follows */
#define IDE_CMD_PIDENTIFY	0xA1	/* ATAPI: IDENTIFY PACKET DEVICE */
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
/* IDENTIFY PACKET DEVICE word 0 (ATA/ATAPI-4 8.13) */
#define IDE_PID_ATAPI_MASK	0xC000	/* bits 15:14 == 10b: an ATAPI device */
#define IDE_PID_ATAPI		0x8000
#define IDE_PID_TYPE( w)	(((w) >> 8) & 0x1F)	/* SCSI peripheral type, 5 = CD-ROM */
#define IDE_PID_PKT16( w)	(((w) & 0x03) == 0x01)	/* 16-byte packets, else 12 */

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
 *
 * The loop is bus bound: the status read is a motherboard cycle of fixed
 * length whatever the CPU, and only the loop overhead shrinks on a faster
 * one.  An IDE_CACHE (68040/68060) build therefore scales the budgets by
 * IDE_SPIN_SCALE (4), which keeps the wall-clock floor at or above the 030's
 * for any per-iteration speed-up up to 4x.  In an IDE_DIAG build ide_spins_hw
 * records the most iterations a successful wait has needed (a real 68060
 * A4000: 2765).
 */
#ifndef IDE_SPIN_SCALE
#if IDE_CACHE
#define IDE_SPIN_SCALE	4L
#else
#define IDE_SPIN_SCALE	1L
#endif
#endif
ulong	ide_spins       = 2000000L * IDE_SPIN_SCALE;	/* BSY/DRQ wait inside a polled command */
ulong	ide_spins_reset = 8000000L * IDE_SPIN_SCALE;	/* BSY wait after reset / for FLUSH CACHE */
ulong	ide_spins_short = 200000L * IDE_SPIN_SCALE;	/* DRQ wait inside the interrupt path */
#if IDE_DIAG
ulong	ide_spins_hw;			/* high-water: iterations a successful wait needed */
#endif

int	ide_slave_enable = 0;		/* 0: never touch the slave (FlashAir rule),
					 * 1: attach whatever answers there,
					 * 2: attach it only if it is an ATAPI device (a CD-ROM) */
int	ide_intr_enable  = 1;		/* 0: force polled I/O even if hooked */
#ifndef IDE_MODULE
int	ide_board_force  = 0;		/* 0 auto-detect (A4000 only), 1.. = ideboards[] index+1, -1 = never probe */
#endif
ulong	ide_timeout_secs = 20;		/* watchdog for an interrupt-driven command */
ulong	ide_spins_atapi = 20000000L * IDE_SPIN_SCALE;	/* polled ATAPI phase wait (spin-up, seek) */
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

#define IDE_DATA_SWAP	0		/* 1 if sector data must be byte swapped; 0 confirmed by the
					 * 2026-09-27 A4000 boot (root mounted, files intact) */

/* ------------------------------------------------------ register seam */

#ifndef HOST_TEST
#define RB( a)		(*(volatile uchar *)(a))
#define WB( a, v)	(*(volatile uchar *)(a) = (uchar)(v))
#define RW( a)		(*(volatile ushort *)(a))
#define WW( a, v)	(*(volatile ushort *)(a) = (ushort)(v))
#endif

/* -------------------------------------------- 68040/68060 data cache */

/*
 * Built only with -DIDE_CACHE=1, for the 68040/68060 kernel line; the 68030
 * object contains none of it.
 *
 * PIO is done by the CPU, but not through a cached alias: the driver stores
 * and loads the buffer at cp->addr, a physical address dereferenced through
 * the low identity range, which DTT0 makes cache-inhibited for supervisor
 * data.  The same physical page can sit in the copyback data cache under
 * another alias -- a raw-I/O buffer is a user page, and user pages are
 * copyback on this line.  A WRITE would then send RAM that is older than the
 * dirty line, and a READ would leave stale lines that are later read, or
 * evicted over the fresh sectors.  So every buffer is pushed and invalidated
 * (CPUSHL + CINVL, by physical line: one op covers every alias) before the
 * transfer, and a READ's buffer again after it.  CINVL follows each CPUSHL
 * because the 68060 leaves a pushed line valid when CACR.DPI is set.  The
 * range rounds outward to 16-byte lines; push-before-invalidate never
 * discards a neighbour's dirty bytes in the edge lines.  The same sequence as
 * amix-z3660scsi (BLIZZARD F3), with the raw words verified there:
 *   f468 = cpushl dc,(a0)   f448 = cinvl dc,(a0)   4e71 = nop
 *   4e7a 0006 / 0007 / 0002 = movec dtt0 / dtt1 / cacr,d0
 *
 * Ordering: DTT0 is cache-inhibited NONSERIALIZED, so with the 68060 store
 * buffer enabled a task-file read could overtake a buffered write (a status
 * poll ahead of the COMMAND write that it is meant to follow).  TR() issues a
 * NOP first, which waits for pending writes.  [verify] against the MC68060UM
 * before the store buffer is turned on; with it off the NOP costs a few
 * cycles next to a motherboard bus cycle.
 *
 * Only in an IDE_DIAG build, ide_cache (kmem) turns the line operations off
 * for an A/B run (the NOP and the attach-time check stay), and ide_push_n /
 * ide_push_bytes count them.  A default build always does the line
 * operations: without them data can be silently corrupted on the 68060.
 */
#if IDE_CACHE
#if IDE_DIAG
long	ide_cache = 1;			/* 0: no line operations (A/B only) */
ulong	ide_push_n, ide_push_bytes;	/* push+invalidate passes, bytes covered */
#endif
ulong	ide_dtt0, ide_dtt1, ide_cacr;	/* captured at attach */
ulong	ide_ci_ok;			/* 0 not checked, 1 task file cache-inhibited, 2 NOT */

#ifndef HOST_TEST
#define IDE_CPUSHL( p)	__asm__ __volatile__( ".word 0xf468" : : "a" (p) : "memory")
#define IDE_CINVL( p)	__asm__ __volatile__( ".word 0xf448" : : "a" (p) : "memory")
#define IDE_SYNC()	__asm__ __volatile__( ".word 0x4e71" : : : "memory")
#define IDE_LINEPTR	register uchar *p __asm__("a0")

static ulong
ide_rd_dtt0()
{
	register ulong	v __asm__("d0");

	__asm__ __volatile__( ".word 0x4e7a,0x0006" : "=d" (v));
	return v;
}

static ulong
ide_rd_dtt1()
{
	register ulong	v __asm__("d0");

	__asm__ __volatile__( ".word 0x4e7a,0x0007" : "=d" (v));
	return v;
}

static ulong
ide_rd_cacr()
{
	register ulong	v __asm__("d0");

	__asm__ __volatile__( ".word 0x4e7a,0x0002" : "=d" (v));
	return v;
}
#else	/* HOST_TEST: mock_regs.h routes IDE_CPUSHL/IDE_CINVL/IDE_SYNC and the
	 * control-register reads into the harness */
#define IDE_LINEPTR	uchar *p
#define ide_rd_dtt0()	mock_dtt0
#define ide_rd_dtt1()	mock_dtt1
#define ide_rd_cacr()	mock_cacr
#endif	/* HOST_TEST */

/* Push and invalidate every data-cache line covering [pa, pa + len). */
static void
ide_cache_push( pa, len)
ulong	pa, len;
{
	IDE_LINEPTR;
	ulong	end;

#if IDE_DIAG
	if (ide_cache == 0)
		return;
#endif
	if (pa == 0 || len == 0 || pa + len < pa)
		return;
	p   = (uchar *)(pa & ~15L);
	end = (pa + len + 15) & ~15L;
	while ((ulong)p < end) {
		IDE_CPUSHL( p);
		IDE_CINVL( p);
		p += 16;
	}
#if IDE_DIAG
	ide_push_n++;
	ide_push_bytes += len;
#endif
}

/*
 * Classify a transparent-translation register against one address for
 * supervisor data: 0 does not cover it, 1 covers it cache-inhibited, 2 covers
 * it cached.  Layout (040 and 060): 31-24 base, 23-16 mask, 15 E, 14-13 S,
 * 6-5 CM (0 writethrough, 1 copyback, 2/3 cache-inhibited).
 */
static int
ide_ttr_class( t, a)
ulong	t, a;
{
	ulong	base, mask;

	if ((t & 0x8000) == 0 || ((t >> 13) & 3) == 0)	/* disabled, or user only */
		return 0;
	base = (t >> 24) & 0xFF;
	mask = (t >> 16) & 0xFF;
	if ((((a >> 24) & 0xFF) & ~mask) != (base & ~mask))
		return 0;
	return ((t >> 5) & 3) >= 2 ? 1 : 2;
}

/*
 * Attach-time check that the board's registers are reached cache-inhibited:
 * a cached status register would never change.  Both ends of the span are
 * tested (TTR granularity is 16 MB).  An address no TTR covers fails too: it
 * would go through page tables whose cache mode the driver cannot see.  The
 * driver carries on either way -- refusing the root disk would only hide the
 * message -- but says so on the console.
 */
static void
ide_ci_check( lo, hi)
ulong	lo, hi;				/* lo inclusive, hi exclusive */
{
	ulong	a;
	int	i, c0, c1;

	ide_dtt0 = ide_rd_dtt0();
	ide_dtt1 = ide_rd_dtt1();
	ide_cacr = ide_rd_cacr();
	ide_ci_ok = 1;
	for (i = 0; i < 2; ++i) {
		a  = i ? hi - 1 : lo;
		c0 = ide_ttr_class( ide_dtt0, a);
		c1 = ide_ttr_class( ide_dtt1, a);
		if (c0 == 2 || c1 == 2 || (c0 == 0 && c1 == 0)) {
			ide_ci_ok = 2;
			printf( "ide: 0x%x is NOT cache-inhibited (dtt0 0x%x dtt1 0x%x cacr 0x%x)\n",
				(int)a, (int)ide_dtt0, (int)ide_dtt1, (int)ide_cacr);
		}
	}
}
#else	/* !IDE_CACHE: the 68030 line has no copyback data cache to maintain */
#define IDE_SYNC()
#define ide_cache_push( pa, len)
#endif	/* IDE_CACHE */

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
 * The module form never reads the Gayle ID either: it takes the A1200 row
 * when the Kickstart ROM carries card.resource (gayle_load, below).
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

#if !IDE_BUDDHA	/* BUDDHA: no fixed rows; the ports come from the AutoConfig list */
static struct ideboard	ideboards[] = {
	{ "A4000 IDE", 0xDD2020L, 0xDD3020L, 0L,        0L,        0x00, 1 },
	{ "A1200 IDE", 0xDA0000L, 0xDA9000L, 0xDA9000L, 0xDAA000L, 0xD1, 1 },
#ifndef IDE_MODULE	/* the module has no A600 row: an ECS machine is not told apart safely */
	{ "A600 IDE",  0xDA0000L, 0xDA9000L, 0xDA9000L, 0xDAA000L, 0xD0, 0 },
#endif
};
#define IDE_NBOARDS	((int)(sizeof ideboards / sizeof ideboards[0]))
#endif
#define IDE_GAYLE_ID	0xDE1000L
#define IDE_IRQ_BIT	0x80

static struct ideboard	*ide_bd;	/* the detected board, NULL until then */

/* task-file access relative to the board base */
#if IDE_CACHE
static __inline__ int
ide_tr( o)
ulong	o;
{
	IDE_SYNC();			/* earlier task-file writes land first */
	return RB( ide_bd->base + o);
}
#define TR( o)		ide_tr( (ulong)(o))
#else
#define TR( o)		RB( ide_bd->base + (o))
#endif
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

#if IDE_BUDDHA
static void	bud_irq_enable();	/* BUDDHA: the board's write-once enable */
#define ide_irq_enable( bd, on)	bud_irq_enable( (bd), (on))
#else
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
#endif

#ifndef IDE_MODULE
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
#endif	/* !IDE_MODULE */

/* ------------------------------------------------------------ unit state */

#if IDE_BUDDHA
/* ============================================================================
 * BUDDHA: the boards (Individual Computers' register specification, as the
 * Linux kernel carries it in Documentation/arch/m68k/buddha-driver.rst, and
 * Linux pata_buddha.c; the WinUAE/Amiberry Buddha model agrees).
 *
 *   AutoConfig  Zorro II, 64 KB, manufacturer 0x1212; product 0x00 Buddha
 *               (two ports), 0x2A Catweasel Z-II (three ports).
 *   port p      task file at board + 0x800 + 0x200 * p: DATA at the base,
 *               register n at + 2 + 4 * n (as the Gayle's); alternate
 *               status / device control at + 0x11A.
 *   IRQ status  board + 0xF00 + 0x40 * p, read only, bit 7 = the port's
 *               INTRQ line (a level, not a latch: reading the drive's STATUS
 *               drops it, nothing to acknowledge).
 *   IRQ enable  board + 0xFC0, write only: any write lets the ports reach
 *               INT2, and nothing but a reset takes it back.  So a drive is
 *               kept quiet with nIEN, never at the board.
 *
 * The driver writes nothing else on the board.  Below 0x800 lie the
 * AutoConfig space and the speed register (the AmigaOS driver's setting is
 * kept), 0xE00-0xEFF is the clock port, and from 0x1000 up the ROM (a flash
 * chip on the Buddha Flash): none of it is read or written.
 * ============================================================================ */
#define BUD_MANUF	0x1212L		/* Individual Computers */
#define BUD_PC_BUDDHA	0x12120000L	/* autocon(): manufacturer << 16 | product */
#define BUD_PC_CWZ2	0x1212002AL
#define BUD_NBOARDS	4		/* sd's four cards */
#define BUD_NPORTS	3		/* per board, the Catweasel Z-II's */
#define BUD_NCHAN	(BUD_NBOARDS * BUD_NPORTS)
#define BUD_PORT0	0x800L
#define BUD_PORTSTEP	0x200L
#define BUD_IRQSTAT0	0xF00L
#define BUD_IRQSTEP	0x40L
#define BUD_INTENA	0xFC0L

/* drive index d: board d / 6, port (d / 2) % 3, slave d & 1; channel d / 2 */
#define IDE_NDRIVES	(BUD_NCHAN * 2)
#else
#define IDE_NDRIVES	2
#endif
#define IDE_MODEL_LEN	40

struct ideunit {
	bool	present;		/* IDENTIFY succeeded, LBA usable (or an ATAPI device) */
	bool	atapi;			/* packet device: CDBs go to the drive as they are */
	uchar	devtype;		/* ATAPI: SCSI peripheral type (5 = CD-ROM) */
	uchar	pktlen;			/* ATAPI: command packet length, 12 or 16 */
	bool	synth;			/* the pending sense is ours, not the drive's */
	bool	flush;			/* FLUSH CACHE supported (IDENTIFY word 83 bit 12) */
	ulong	nsectors;		/* LBA28 sector count */
	ushort	multi;			/* IDENTIFY word 47 low byte (informational) */
	char	model[IDE_MODEL_LEN + 1];
	uchar	skey, asc, ascq;	/* pending sense for REQUEST SENSE */
	ulong	errors;			/* kmem-readable error counter */
	uchar	lasterr;		/* last ATA error register value */
	uchar	laststat;		/* last ATA status register value */
};

#ifdef IDE_MODULE
struct ideunit		ideunits[IDE_NDRIVES];	/* global: getksym finds the per-unit counters */
#else
static struct ideunit	ideunits[IDE_NDRIVES];
#endif
static int		ide_state;	/* 0 unprobed, 1 no controller, 2 present */
static int		ide_attached;	/* attach() done */
static int		ide_use_intr;	/* attach(): ideintr is in int2_tbl[] and enabled */
#ifdef IDE_MODULE
static int		ide_st_armed;	/* gayle_defer(): the self-test's command is out */
#endif

/* interrupt-path bookkeeping (kmem-readable) */
ulong	ide_intr_count, ide_intr_spurious, ide_intr_timeouts;
ulong	ide_intr_busy;		/* interrupts taken with the drive still BSY */
ulong	ide_dup_submits;	/* requests re-submitted while active or queued (ignored) */
ulong	ide_resets;		/* software resets issued after a timeout */
ulong	ide_atapi_cmds;		/* ATAPI packet commands issued */
ulong	ide_atapi_overrun;	/* ATAPI bytes the drive sent beyond nbyte (dropped) */
ulong	ide_atapi_irlate;	/* polled ATAPI ends accepted without IREASON CoD|IO */

#if IDE_BUDDHA
/* ============================================================================
 * BUDDHA: the channel table.  One `struct ideboard' per port, whose base is
 * the port's task file; ide_bd points at the port of the command on the
 * cable.  One command is in flight across all boards and ports, so the
 * queue, ide_x and the watchdog stay as they are and only the port changes
 * per command: ide_select() switches ide_bd to the drive's port (IDE_CHAN).
 * ============================================================================ */
struct ideboard	bud_ch[BUD_NCHAN];	/* base 0: no such port */
int		bud_chok[BUD_NCHAN];	/* 1: a drive answered the port's readback */
ulong		bud_base[BUD_NBOARDS];	/* the boards' AutoConfig bases, 0: none */
int		bud_nports[BUD_NBOARDS];
int		bud_nboards;
ulong		bud_intr_other;		/* INTRQs dropped on a port with no command */
static int	bud_intron[BUD_NBOARDS];	/* the enable at 0xFC0 written */
static int	bud_cardbd[4];		/* sd card -> board index, learnt in idequeue() */

/* SCSI units per port, master and slave; 4 and 7 are not used */
static int	bud_unit[BUD_NPORTS][2] = { { 6, 5 }, { 3, 2 }, { 1, 0 } };

/* SCSI unit of board b -> drive index, or -1 when no port answers to it */
static int
bud_drive( b, unit)
int	b;
uint	unit;
{
	int	p, s;

	if (b < 0 || b >= BUD_NBOARDS)
		return -1;
	for (p = 0; p < BUD_NPORTS; ++p)
		for (s = 0; s < 2; ++s)
			if ((int)unit == bud_unit[p][s])
				return (b * BUD_NPORTS + p) * 2 + s;
	return -1;
}

/* idequeue(c, cp): `c' is the board's index (sd_register's); keep it per card */
static int
bud_drive_of( c, cp)
int		c;
struct sdcom	*cp;
{
	if (c >= 0 && c < BUD_NBOARDS)
		bud_cardbd[cp->card & 3] = c;
	return bud_drive( c, cp->unit);
}

/* a queued request's drive: its card's board, as idequeue() saw it */
static int
bud_cp_drive( cp)
struct sdcom	*cp;
{
	return bud_drive( bud_cardbd[cp->card & 3], cp->unit);
}

/* The board's enable at 0xFC0: written once, and only to turn it on. */
static void
bud_irq_enable( bd, on)
struct ideboard	*bd;
int		on;
{
	int	b;

	b = (bd - bud_ch) / BUD_NPORTS;
	if (on && bd->irqena && !bud_intron[b]) {
		bud_intron[b] = 1;
		WB( bd->irqena, 0);
	}
}

/* Device control (nIEN) on every port that answered. */
static void
bud_ctl_all( v)
int	v;
{
	int	i;

	for (i = 0; i < BUD_NCHAN; ++i)
		if (bud_chok[i])
			WB( bud_ch[i].base + IDE_REG_CONTROL, v);
}

/*
 * ideintr() serves the port of the command on the cable.  A drive on another
 * port that holds INTRQ (nothing should, but the line is a level on a shared
 * INT2) would keep the interrupt asserted for ever: read its STATUS to drop it.
 */
static void
bud_intr_others()
{
	int	i;

	if (ide_state != 2)
		return;
	for (i = 0; i < BUD_NCHAN; ++i) {
		if (!bud_chok[i] || &bud_ch[i] == ide_bd)
			continue;
		if (RB( bud_ch[i].irqstat) & IDE_IRQ_BIT) {
			IDE_SYNC();
			(void) RB( bud_ch[i].base + IDE_REG_STATUS);
			bud_intr_other++;
		}
	}
}

/* Forget the boards (idereset). */
static void
bud_forget()
{
	int	i;

	for (i = 0; i < BUD_NCHAN; ++i) {
		bud_ch[i].name = 0;
		bud_ch[i].base = bud_ch[i].irqstat = bud_ch[i].irqack = bud_ch[i].irqena = 0;
		bud_chok[i] = 0;
	}
	for (i = 0; i < BUD_NBOARDS; ++i) {
		bud_base[i] = 0;
		bud_nports[i] = 0;
		bud_intron[i] = 0;
	}
	for (i = 0; i < 4; ++i)
		bud_cardbd[i] = 0;
	bud_nboards = 0;
}

#define IDE_CHAN( drive)		ide_bd = &bud_ch[(drive) >> 1]
#define IDE_DRV1( drive)		((drive) & 1 ? IDE_SDH_DRV1 : 0)
#define IDE_IS_SLAVE( drive)		((drive) & 1)
#define IDE_ATTACH_WANTED( drive)	(bud_chok[(drive) >> 1] && (!((drive) & 1) || ide_slave_enable))
#define IDE_CP_DRIVE( cp)		bud_cp_drive( cp)
#define IDE_DRIVE_OF( c, cp)		bud_drive_of( c, cp)
#define IDE_INTR_OTHERS()		bud_intr_others()
#define IDE_RESET_EXTRA()		bud_forget()
#else	/* the Gayle: one port, drive 0 the master, 1 the slave */
#define IDE_CHAN( drive)
#define IDE_DRV1( drive)		(drive ? IDE_SDH_DRV1 : 0)
#define IDE_IS_SLAVE( drive)		(drive == 1)
#define IDE_ATTACH_WANTED( drive)	(drive == 0 || ide_slave_enable)
#define IDE_CP_DRIVE( cp)		((int)cp->unit == ide_master_unit ? 0 : 1)
#define IDE_DRIVE_OF( c, cp)		ide_drive_of( cp->unit)
#define IDE_INTR_OTHERS()
#define IDE_RESET_EXTRA()
#endif

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
#if IDE_DIAG
	ulong	n;

	n = spins;
#endif
	do {
		st = TR( IDE_REG_STATUS);
		if ((st & mask) == want) {
#if IDE_DIAG
			if (n - spins > ide_spins_hw)
				ide_spins_hw = n - spins;
#endif
			return st;
		}
	} while (spins-- != 0);
	return -1;
}

/* BSY clear and DRQ set, or BSY clear with ERR/DF set (caller inspects). */
static int
ide_wait_drq( spins)
ulong	spins;
{
	int	st;
#if IDE_DIAG
	ulong	n;

	n = spins;
#endif
	do {
		st = TR( IDE_REG_STATUS);
		if (!(st & IDE_ST_BSY) && (st & (IDE_ST_DRQ | IDE_ST_ERR | IDE_ST_DF))) {
#if IDE_DIAG
			if (n - spins > ide_spins_hw)
				ide_spins_hw = n - spins;
#endif
			return st;
		}
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
	IDE_CHAN( drive);
	TW( IDE_REG_SDH, IDE_SDH_FIXED | IDE_SDH_LBA | IDE_DRV1( drive));
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
	u->synth = TRUE;
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
 * Verified in the emulator and on the real A4000 (2026-09-27: model string
 * and sector count decode correctly).  Sector data needs no swap
 * (IDE_DATA_SWAP 0).
 */
static int
ide_identify( drive, id, cmd)
int	drive, cmd;		/* IDE_CMD_IDENTIFY, or IDE_CMD_PIDENTIFY for a packet device */
ushort	*id;
{
	int	st, i;
	ushort	w;

	if (ide_select( drive) < 0)
		return -1;
	TW( IDE_REG_COMMAND, cmd);
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

/*
 * Sectors per MB for the attach banner.  A variable, not the constant 2048:
 * the 68040/68060 relink pass rejects any shift by 11 (the 030 page shift)
 * in a linked object, and /2048 compiles to exactly that.
 */
static ulong	ide_secs_per_mb = 1048576L / IDE_SECTOR;

/* IDENTIFY words 27..46: the model string, two characters per word */
static void
ide_model( u, id)
struct ideunit	*u;
ushort		*id;
{
	char	*m;
	int	i;

	m = u->model;
	for (i = 0; i < IDE_MODEL_LEN / 2; ++i) {
		*m++ = id[IDE_ID_MODEL + i] >> 8;
		*m++ = id[IDE_ID_MODEL + i] & 0xFF;
	}
	*m = '\0';
	while (m > u->model && m[-1] == ' ')	/* trim trailing blanks */
		*--m = '\0';
}

/*
 * An ATAPI device (a CD-ROM): IDENTIFY PACKET DEVICE gives its type and
 * packet length.  It has no capacity of its own here -- the caller's READ
 * CAPACITY goes to the drive like every other CDB.
 */
static void
ide_attach_atapi( drive, u, id)
int		drive;
struct ideunit	*u;
ushort		*id;
{
	if (ide_identify( drive, id, IDE_CMD_PIDENTIFY) < 0
	    || (id[0] & IDE_PID_ATAPI_MASK) != IDE_PID_ATAPI) {
		printf( "ide%d: ATAPI signature but no IDENTIFY PACKET DEVICE, ignored\n", drive);
		return;
	}
	u->atapi   = TRUE;
	u->devtype = IDE_PID_TYPE( id[0]);
	u->pktlen  = IDE_PID_PKT16( id[0]) ? 16 : 12;
	u->nsectors = 0;
	u->flush   = FALSE;
	ide_model( u, id);
	u->present = TRUE;
	printf( "ide%d: %s, ATAPI %s\n", drive, u->model,
		u->devtype == 5 ? "CD-ROM" : "device");
}

static void
ide_attach_unit( drive)
int	drive;
{
	static ushort	id[IDE_SECTOR / 2];	/* static: kernel stacks are small */
	struct ideunit	*u;

	u = &ideunits[drive];
	u->present = FALSE;
	u->atapi = FALSE;
	if (ide_identify( drive, id, IDE_CMD_IDENTIFY) < 0) {
		/* a packet device aborts IDENTIFY DEVICE and leaves its signature */
		if (TR( IDE_REG_LCYL) == IDE_ATAPI_SIGLO && TR( IDE_REG_HCYL) == IDE_ATAPI_SIGHI) {
			ide_attach_atapi( drive, u, id);
			return;
		}
		if (ide_debug)
			printf( "ide%d: no drive\n", drive);
		return;
	}
	if (IDE_IS_SLAVE( drive) && ide_slave_enable == 2) {
#if IDE_BUDDHA
		printf( "ide%d: ATA device on a slave ignored (slave=2: ATAPI only)\n", drive);
#else
		printf( "ide1: ATA device on the slave ignored (ide_slave_enable 2: ATAPI only)\n");
#endif
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
	ide_model( u, id);
	u->present = TRUE;
	printf( "ide%d: %s, %d sectors (%d MB)\n", drive, u->model,
		(int)u->nsectors, (int)(u->nsectors / ide_secs_per_mb));
}

#ifndef IDE_MODULE
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
#endif	/* !IDE_MODULE */

static void
ide_attach()
{
	int	drive;

	ide_attached = 1;
	ide_use_intr = 0;
#if IDE_CACHE
	ide_ci_check( ide_bd->base, ide_bd->base + IDE_SPAN);
#endif
	TW( IDE_REG_CONTROL, IDE_CTL_NIEN);		/* quiet during the polled attach */
	for (drive = 0; drive < IDE_NDRIVES; ++drive)
		if (IDE_ATTACH_WANTED( drive))
			ide_attach_unit( drive);
		else
			ideunits[drive].present = FALSE;
#ifndef IDE_MODULE
	if (ide_intr_enable && ide_hooked()) {
		ide_use_intr = 1;
		ide_irq_enable( ide_bd, 1);
		TW( IDE_REG_CONTROL, 0);		/* drive may assert INTRQ */
	} else
		ide_irq_enable( ide_bd, 0);
	printf( "ide: %s, %s I/O\n", ide_bd->name, ide_use_intr ? "interrupt" : "polled");
#else	/* the module: polled until its deferred self-test passes (gayle_defer) */
	ide_irq_enable( ide_bd, 0);
#endif
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
			 | IDE_DRV1( drive) | ((lba >> 24) & IDE_SDH_HEAD));
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
	int		atapi;		/* the request is an ATAPI packet command */
	uchar		pkt[16];	/* ATAPI: the command packet */
} ide_x;
static int	ide_watch_armed;
static ulong	ide_watch_ticks, ide_watch_gen;

static void	ide_start();
static void	ide_watchdog();
static void	ide_watch_arm();
static void	ide_reset_drive();
#ifndef IDE_MODULE
int		idepresent();
#endif

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

/* ------------------------------------------------ ATAPI packet commands */

/*
 * An ATAPI device (a CD-ROM) takes the caller's SCSI CDB itself, as a 12-
 * or 16-byte packet after the PACKET command; the data then moves in DRQ
 * phases whose direction (IREASON) and length (byte count) the device
 * chooses.  The same step function runs polled (ide_atapi_polled loops to
 * the end) and from ideintr() (one phase per interrupt).  READ(6)/WRITE(6),
 * which MMC devices do not implement, become READ(10)/WRITE(10); everything
 * else goes as it is, so INQUIRY, READ CAPACITY and REQUEST SENSE are the
 * drive's own answers and a CHECK CONDITION leaves the real sense in the
 * drive for the caller's REQUEST SENSE.  Bytes the drive sends beyond nbyte
 * are drained and counted (ide_atapi_overrun), never stored.
 */

/* Build ide_x.pkt from the CDB; bytes past the CDB's group length are zero. */
static void
ide_atapi_packet( c)
uchar	*c;
{
	uchar	*p;
	ulong	lba, cnt;
	int	i, n;

	p = ide_x.pkt;
	for (i = 0; i < 16; ++i)
		p[i] = 0;
	if (c[0] == SC_READ6 || c[0] == SC_WRITE6) {
		lba = ((ulong)(c[1] & 0x1F) << 16) | ((ulong)c[2] << 8) | c[3];
		cnt = c[4] ? c[4] : 256;
		p[0] = c[0] == SC_READ6 ? SC_READ10 : SC_WRITE10;
		p[3] = lba >> 16; p[4] = lba >> 8; p[5] = lba;
		p[7] = cnt >> 8;  p[8] = cnt;
		p[9] = c[5];
		return;
	}
	switch (c[0] >> 5) {			/* SCSI-2 group code */
	case 0:  n = 6;  break;
	case 1:
	case 2:  n = 10; break;
	default: n = 12; break;
	}
	for (i = 0; i < n; ++i)
		p[i] = c[i];
}

/* Describe cp as the ATAPI transfer in ide_x. */
static void
ide_atapi_setup( cp, drive)
struct sdcom	*cp;
int		drive;
{
	ide_x.atapi  = 1;
	ide_x.drive  = drive;
	ide_x.u      = &ideunits[drive];
	ide_x.write  = 0;			/* the device picks the direction per phase */
	ide_x.nodata = 0;
	ide_x.cmdleft = 0;
	ide_x.left   = 0;
	ide_x.data   = (uchar *)cp->addr;
	ide_x.nbyte  = cp->nbyte;
	ide_x.done   = 0;
	ide_atapi_packet( cp->cdb);
}

/*
 * One step with the device not busy: move a DRQ phase, or finish.  Returns
 * 1 (more phases follow), 0 (done, GOOD) or -1 (CHECK CONDITION).
 */
static int
ide_atapi_step( st)
int	st;
{
	struct ideunit	*u;
	ulong		n, i, end, from;
	ushort		w;
	uchar		*d;
	int		ir;

	u = ide_x.u;
	if (!(st & IDE_ST_DRQ)) {
		if (st & IDE_ST_ERR) {		/* the drive holds the sense; key in ERROR 7:4 */
			u->laststat = st;
			u->lasterr  = TR( IDE_REG_ERROR);
			u->skey     = u->lasterr >> 4;
			u->errors++;
			return -1;
		}
		if (st & IDE_ST_DF) {
			ide_setsense( u, SK_HARDWARE, ASC_INTERNAL_FAILURE, 0);
			return -1;
		}
		return 0;
	}
	ir = TR( IDE_REG_IREASON);
	n  = TR( IDE_REG_BCLO) | ((ulong)TR( IDE_REG_BCHI) << 8);
	if ((ir & IDE_IR_COD) || n == 0) {	/* a packet phase again, or nothing to move */
		u->laststat = st;
		ide_setsense( u, SK_HARDWARE, ASC_LU_COMM_FAILURE, 0);
		ide_reset_drive();
		return -1;
	}
	d = ide_x.data;
	from = ide_x.done;
	for (i = 0; i < n; i += 2) {
		end = from + i;
		if (ir & IDE_IR_IO) {
			w = TRW();
			if (end < ide_x.nbyte)
				d[end] = w >> 8;
			if (i + 1 < n && end + 1 < ide_x.nbyte)
				d[end + 1] = w;
		} else {
			w = 0;
			if (end < ide_x.nbyte)
				w = (ushort)d[end] << 8;
			if (i + 1 < n && end + 1 < ide_x.nbyte)
				w |= d[end + 1];
			TWW( w);
		}
	}
	end = from + n;
	if (end > ide_x.nbyte)
		ide_atapi_overrun += end - (from > ide_x.nbyte ? from : ide_x.nbyte);
	ide_x.done = end;
	ide_settle();
	return 1;
}

/*
 * PACKET, then the command packet once the device asks for it.  Returns 0
 * with the command running, or -1 (sense set or held by the drive).
 */
static int
ide_atapi_issue()
{
	struct ideunit	*u;
	ulong		bc;
	int		st, i;

	u = ide_x.u;
	ide_atapi_cmds++;
	if (ide_select( ide_x.drive) < 0) {
		ide_setsense( u, SK_NOT_READY, ASC_LUN_NOT_READY, 0);
		return -1;
	}
	bc = (ide_x.nbyte + 1) & ~1L;		/* byte count limit per DRQ phase */
	if (bc == 0 || bc > 0xFFFE)
		bc = 0xFFFE;
	TW( IDE_REG_FEATURE, 0);		/* PIO, no overlap */
	TW( IDE_REG_IREASON, 0);
	TW( IDE_REG_BCLO, bc & 0xFF);
	TW( IDE_REG_BCHI, (bc >> 8) & 0xFF);
	TW( IDE_REG_COMMAND, IDE_CMD_PACKET);
	ide_settle();
	st = ide_wait_drq( ide_spins);
	if (st < 0) {
		ide_setsense( u, SK_HARDWARE, ASC_LU_COMM_FAILURE, 0);
		ide_reset_drive();
		return -1;
	}
	if (!(st & IDE_ST_DRQ)) {		/* PACKET itself refused (ERR or DF) */
		(void) ide_atapi_step( st);
		return -1;
	}
	if ((TR( IDE_REG_IREASON) & (IDE_IR_COD | IDE_IR_IO)) != IDE_IR_COD) {
		ide_setsense( u, SK_HARDWARE, ASC_LU_COMM_FAILURE, 0);
		ide_reset_drive();
		return -1;
	}
	for (i = 0; i < u->pktlen; i += 2)
		TWW( ((ushort)ide_x.pkt[i] << 8) | ide_x.pkt[i + 1]);
	ide_settle();
	return 0;
}

/*
 * The whole command, polled.  0 GOOD, -1 CHECK CONDITION.
 *
 * Polling sees states an interrupt never shows.  BSY and DRQ clear with no
 * error is the end of the command only in the status phase, where IREASON
 * reads CoD|IO (ATA/ATAPI-4 9.10); Amiberry's ATAPI model passes through
 * BSY=0/DRQ=0 with IREASON still CoD while it prepares the data, and taking
 * that as the end completed reads with nothing transferred.  A device that
 * never sets CoD|IO at the end is accepted after ide_spins_short quiet reads
 * (ide_atapi_irlate counts those).
 */
static int
ide_atapi_polled( cp, drive)
struct sdcom	*cp;
int		drive;
{
	ulong	quiet;
	int	st, r;

	ide_atapi_setup( cp, drive);
	if (ide_atapi_issue() < 0)
		return -1;
	quiet = 0;
	for (;;) {
		st = ide_wait( IDE_ST_BSY, 0, ide_spins_atapi);
		if (st < 0) {
			ide_x.u->laststat = TR( IDE_REG_ALTSTAT);
			ide_setsense( ide_x.u, SK_HARDWARE, ASC_LU_COMM_FAILURE, 0);
			if (ide_debug)
				printf( "ide%d: ATAPI timeout, status 0x%x\n", drive, ide_x.u->laststat);
			ide_reset_drive();
			return -1;
		}
		if (!(st & (IDE_ST_DRQ | IDE_ST_ERR | IDE_ST_DF))
		    && (TR( IDE_REG_IREASON) & (IDE_IR_COD | IDE_IR_IO)) != (IDE_IR_COD | IDE_IR_IO)) {
			if (quiet++ < ide_spins_short)
				continue;		/* not the status phase yet */
			ide_atapi_irlate++;
		}
		quiet = 0;
		r = ide_atapi_step( st);
		if (r <= 0)
			return r;
	}
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
		if (!ide_x.write && !ide_x.nodata)
			ide_cache_push( (ulong)cp->addr, (ulong)cp->nbyte);	/* lines refetched meanwhile */
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
	ide_watch_arm();
}

static void
ide_watch_arm()
{
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
	if (ideunits[IDE_CP_DRIVE( cp)].atapi) {
		ide_atapi_setup( cp, IDE_CP_DRIVE( cp));
		if (ide_atapi_issue() < 0)
			ide_finish( -1);
		else
			ide_watch_arm();
		return;
	}
	ide_x.atapi = 0;
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
	ide_x.drive = IDE_CP_DRIVE( cp);
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
	int	st, i;

	IDE_INTR_OTHERS();
	if (ide_bd == 0 || ide_state != 2 || !ide_irq_pending( ide_bd))
		return;
	st = TR( IDE_REG_STATUS);		/* drops INTRQ at the drive */
	ide_irq_ack( ide_bd);			/* and at the Gayle latch, where there is one */
	ide_intr_count++;
	if (ide_active == 0) {			/* polled command's leftover, or noise */
#ifdef IDE_MODULE
		if (!ide_st_armed)		/* not the self-test's READ VERIFY */
#endif
		ide_intr_spurious++;
		return;
	}
	if (st & IDE_ST_BSY) {			/* not for us yet; the drive will interrupt again */
		ide_intr_busy++;
		return;
	}
	if (ide_x.atapi) {			/* one DRQ phase, or the end of the command */
		i = ide_atapi_step( st);
		if (i <= 0)
			ide_finish( i);
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

#if !IDE_BUDDHA	/* BUDDHA: bud_drive_of() */
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
#endif

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
#ifndef IDE_MODULE
	if (ide_state == 0) {			/* sd.c probes first; be robust anyway */
		char	*pa;
		(void) idepresent( &pa);
	}
#endif
	if (ide_state == 2 && !ide_attached)
		ide_attach();
	ide_cache_push( (ulong)cp->addr, (ulong)cp->nbyte);	/* before the CPU reads or fills it */

	drive = IDE_DRIVE_OF( c, cp);
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

	/* ATAPI: the drive executes the CDB (our own sense, if pending, first) */
	if (u->atapi && !(op == SC_REQ_SENSE && u->synth)) {
		u->synth = FALSE;
		if (ide_use_intr) {
			ide_enqueue( cp);
			if (ide_active == 0)
				ide_start();
			splx( s);
			return TRUE;		/* completes from ideintr() */
		}
		if (ide_atapi_polled( cp, drive) < 0)
			cp->status = SC_CHECK;
		ide_cache_push( (ulong)data, (ulong)cp->nbyte);
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
			u->synth = FALSE;
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
#ifdef IDE_MODULE
			/* counted, said only with debug: the one crossing seen at
			 * boot is sdpart.c's RDB read into a kernel static, which
			 * is physically contiguous (README, Hardware notes) */
			ide_pagecross_n++;
			if (ide_debug)
#else
			if (ide_pagecross_n++ == 0 || ide_debug)
#endif
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
		if (!write)
			ide_cache_push( (ulong)data, (ulong)cp->nbyte);
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
#ifndef IDE_MODULE
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
#endif	/* !IDE_MODULE */

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
	IDE_RESET_EXTRA();
}

#ifdef IDE_MODULE
/* ----------------------------------------------------------------------------
 * The root-class module form: the loadable kernel module `gayle' (amix-dlm
 * contracts C4-2 version 3, C4-1 version 2, C3-6 version 5; docs/MODULE-PORT.md).
 * Built with -DIDE_MODULE once per kernel line (k30; k46 with -DIDE_CACHE=1
 * -DIDE_NBPP=4096); module/Master gives the .moddata keys: class=root,
 * match=mobo:gayle, rank=ide, unload=never, no majors (sd owns the disk
 * majors).
 *
 * gayle_load() runs in the module core's root pass: process 0, before the root
 * is mounted, at LEVEL 7 WITH THE CLOCK STOPPED.  So it sleeps on nothing,
 * sets no timeout, waits for no interrupt, allocates nothing (the driver's
 * state is its own .data/.bss) and every wait is a bounded spin.  In order:
 *   1. the parameters (defaults: interrupts after the self-test, no slave,
 *      units 6 and 5, the board found automatically; intr=0 is polled I/O,
 *      board=-1 touches nothing at all);
 *   2. the chipset gate: one VPOSR read; anything but AGA Alice is ENXIO
 *      after that single read (an A3000 is never touched further);
 *   3. the board: with board=0 (the default) the Kickstart ROM decides.  A
 *      ROM whose resident modules include card.resource is an A1200's (every
 *      A600/A1200 ROM carries one, no A3000/A4000/A4000T ROM does); then the
 *      A1200 row (0xDA0000), else the A4000 row (0xDD2020).  Only ROM space
 *      is read for this: it is decoded on every Amiga.  A wrong guess would
 *      read local I/O nothing decodes, which on some CPU cards never ends,
 *      so neither row's registers are touched before this step.  board=1 or
 *      2 names the row and skips the ROM;
 *   4. the guard: bprobe() reads the row's alternate status register
 *      (0xDD303A, 0xDA101A), which has no side effect; a bus error is ENXIO;
 *   5. the identifying probe registered as mobo:gayle: the task-file
 *      write/readback of the parameter registers (nIEN, SDH, NSECT/SECT), no
 *      command; open bus or no drive is ENXIO;
 *   6. the attach: polled IDENTIFY of the master (and the slave if `slave'
 *      asks for it); no drive that answers it is ENXIO as well; on the A1200
 *      the Gayle's IDE interrupt enable (0xDAA000 bit 7) is cleared;
 *   7. sd_register(idequeue, 0, 0xDD2020, "A4000 IDE", 4), or
 *      sd_register(idequeue, 0, 0xDA0000, "A1200 IDE", 4);
 *   8. with intr=1 (the default), root_defer(gayle_defer, 0).
 * ENXIO is silent and leaves nothing behind: an empty A4000T port takes no
 * card.  The Gayle ID register at 0xDE1000 lies in Gary space on the A3000
 * and A4000 and is never touched, on either row.  The A600 is not driven:
 * its ECS chipset and its ROM (also the A500's and A2000's) say nothing
 * certain about the machine.  The PCMCIA side of the A1200 Gayle is left
 * alone: its interrupt bits are preserved in every INTENA and INTREQ write.
 *
 * gayle_defer() runs at the first sysinit, in process context at level 0.
 * It attaches ide_intr2() at level 2 and self-tests with a real command: the
 * A4000's INTRQ mirror (0xDD3020 bit 7) cannot be set by an INTREQ raise, so
 * the driver clears nIEN, issues READ VERIFY SECTOR(S) for one sector at LBA
 * 0 on the first ATA drive, and waits in one-tick timeout/sleep rounds for
 * ideintr() to count it.  On the A1200 a stale Gayle latch is acknowledged
 * and INTENA bit 7 set before the command, and the test passes only when the
 * latch reads clear after ideintr()'s acknowledge.  Pass: interrupt I/O from
 * then on.  Fail: INTENA bit 7 cleared, nIEN set again, the handler
 * detached, polled I/O, and a console line.
 *
 * gayle_halt() masks the drive's interrupt (nIEN) and, on the A1200, the
 * Gayle's IDE interrupt enable; it does not flush and does not wait for a
 * command in flight.
 * ------------------------------------------------------------------------- */

#ifndef HOST_TEST
#include	"sys/moddefs.h"		/* the ABI kit's wrapper */
#include	"sys/amixddi.h"		/* sd_register, root_defer, intr_attach */
#else
extern int	sd_register(), root_defer(), intr_attach(), intr_detach();
#endif
extern int	bprobe();		/* (caddr_t addr, int v): 0 ok, 1 bus error */
extern int	sleep();
extern int	wakeup();

#define IDE_SD_RANK	4		/* C4-1 section 4: class 4 ide = .moddata rank=ide */
#define IDE_LEVEL	2		/* INT2: PORTS, shared with CIA-A and Zorro II */

/* kmem/getksym-readable state of the module form */
ulong	ide_halted;			/* the halt pass ran (count) */
ulong	ide_selftest;			/* 0 not run, 1 passed (interrupt I/O), 2 failed (polled) */
ulong	ide_selftest_ticks;		/* clock ticks the self-test waited */
#if !IDE_BUDDHA
int	ide_board;			/* param board: 0 auto, 1 A4000, 2 A1200, -1 never probe */
int	ide_cardres = -1;		/* the ROM walk: -1 not run, 0 no card.resource, 1 found */
ulong	ide_romtags;			/* resident tags the ROM walk passed */
#endif
static char	ide_st_chan;		/* the self-test's sleep channel */

#if !IDE_BUDDHA
#define IDE_BD_A4000	0		/* ideboards[] rows; board=N takes row N-1 */
#define IDE_BD_A1200	1
#define IDE_NBD_MODULE	2
#endif

/*
 * One integer parameter from the string the core passes to the load entry:
 * "name=value ..." (decimal; a leading minus only where lo is negative), or
 * 0.  Absent: the default.  A value that is not a number in [lo, hi] is said
 * and gives the default.
 */
static int
gayle_iparam( params, name, dflt, lo, hi)
char	*params, *name;
int	dflt, lo, hi;
{
	char	*p, *n;
	long	v, lim;
	int	nd, neg;

	if ((p = params) == 0)
		return dflt;
	for (;;) {
		while (*p == ' ')
			p++;
		if (*p == 0)
			return dflt;
		for (n = name; *n && *p == *n; n++, p++)
			;
		if (*n == 0 && *p == '=')
			break;
		while (*p && *p != ' ')
			p++;
	}
	p++;
	neg = *p == '-' && lo < 0;	/* a sign only where a negative value is allowed */
	if (neg)
		p++;
	lim = neg ? -(long)lo : (long)hi;
	for (v = 0, nd = 0; *p >= '0' && *p <= '9'; p++, nd++)
		v = v <= lim ? v * 10 + (*p - '0') : lim + 1;
	if (neg)
		v = -v;
	if (nd == 0 || (*p != 0 && *p != ' ') || v < lo || v > hi) {
		printf( "ide: parameter %s is not %d to %d; %d taken\n", name, lo, hi, dflt);
		return dflt;
	}
	return (int)v;
}

#if !IDE_BUDDHA	/* the Gayle's module entries; BUDDHA's are below */
/*
 * Does the Kickstart ROM carry card.resource among its resident modules?
 * That is how the module tells an A1200 from an A4000, both AGA, without a
 * single access to local I/O: every A600/A1200 ROM (2.05 to 3.2.3) has the
 * resident, no A3000/A4000/A4000T ROM has.  The walk is the one exec makes
 * at reset: a resident tag is the word 0x4AFC whose next long points at the
 * word itself (rt_MatchTag); its name pointer is at +14 and rt_EndSkip at +6
 * says where the next one may start.  Only 0xF80000-0xFFFFFF is read, the
 * ROM itself (or the copy a CPU card maps there), which every Amiga decodes;
 * the name is compared as a resident tag's name, so the plain string that
 * A3000/A4000 ROMs also contain is never mistaken for the module.  Bounded:
 * at most one pass over the 512 KB, word by word.
 */
#define IDE_ROM_BASE	0xF80000L
#define IDE_ROM_END	0x1000000L
#define IDE_RTC_MATCHWORD 0x4AFC
#define IDE_RT_ENDSKIP	6
#define IDE_RT_NAME	14
#define IDE_RT_SIZE	26

static ulong
ide_rl( a)
ulong	a;
{
	return ((ulong)RW( a) << 16) | (ulong)RW( a + 2);
}

/* the NUL-terminated string at ROM address `a' is `s' */
static int
ide_rom_streq( a, s)
ulong	a;
char	*s;
{
	if (a < IDE_ROM_BASE || a >= IDE_ROM_END - 32)
		return 0;
	for (; *s; ++a, ++s)
		if (RB( a) != (uchar)*s)
			return 0;
	return RB( a) == 0;
}

static int
ide_rom_cardres()
{
	ulong	a, skip;

	ide_romtags = 0;
	for (a = IDE_ROM_BASE; a <= IDE_ROM_END - IDE_RT_SIZE; ) {
		if (RW( a) != IDE_RTC_MATCHWORD || ide_rl( a + 2) != a) {
			a += 2;
			continue;
		}
		ide_romtags++;
		if (ide_rom_streq( ide_rl( a + IDE_RT_NAME), "card.resource"))
			return 1;
		skip = ide_rl( a + IDE_RT_ENDSKIP);
		a = skip > a + 2 && skip <= IDE_ROM_END ? (skip + 1) & ~1L : a + 2;
	}
	return 0;
}

/*
 * The level-2 entry.  ideintr() counts ide_intr_count exactly when the
 * board's INTRQ mirror showed the interrupt and it took it, so nonzero here
 * means "ours", as intr_attach() asks.
 */
static int
ide_intr2( arg)
char	*arg;
{
	ulong	before;

	before = ide_intr_count;
	ideintr();
	return ide_intr_count != before;
}

/*
 * The self-test (process context, level 0, the queue idle: every request
 * before it was polled and has completed).  Returns 0 when the drive's
 * interrupt was taken, -1 otherwise; on failure nIEN is set again and the
 * command's end is waited out polled.
 */
static int
ide_selftest_run()
{
	ulong	before, t;
	int	drive, s, st;

	for (drive = 0; drive < IDE_NDRIVES; ++drive)
		if (ideunits[drive].present && !ideunits[drive].atapi)
			break;
	if (drive == IDE_NDRIVES)
		return -1;			/* no ATA drive to give READ VERIFY to */
	s = sdspl();
	st = ide_select( drive);
	if (st < 0 || !(st & IDE_ST_DRDY)) {
		splx( s);
		return -1;
	}
	before = ide_intr_count;
	ide_st_armed = 1;
	ide_irq_ack( ide_bd);			/* A1200: a latch from before is not the test's */
	ide_irq_enable( ide_bd, 1);		/* A1200: route the IDE latch to INT2 */
	TW( IDE_REG_CONTROL, 0);		/* nIEN clear: the drive may assert INTRQ */
	ide_program( drive, IDE_CMD_VERIFY, 0L, 1L);
	for (t = 0; ide_intr_count == before && t < ide_timeout_secs * HZ; ++t) {
		/* one tick; at sdspl, so the wakeup cannot come before the sleep */
		timeout( wakeup, (caddr_t)&ide_st_chan, 1);
		(void) sleep( (caddr_t)&ide_st_chan, PRIBIO);
	}
	ide_st_armed = 0;
	ide_selftest_ticks = t;
	if (ide_intr_count != before) {
		/* a Gayle latch that stays set after the acknowledge would hold
		 * INT2 asserted for good: polled I/O instead */
		if (ide_bd->irqack == 0 || !ide_irq_pending( ide_bd)) {
			ide_use_intr = 1;
			splx( s);
			return 0;
		}
		printf( "ide: the Gayle interrupt latch did not clear\n");
	}
	ide_irq_enable( ide_bd, 0);
	TW( IDE_REG_CONTROL, IDE_CTL_NIEN);
	st = ide_wait( IDE_ST_BSY, 0, ide_spins);	/* the command's end, polled */
	if (st < 0)
		ide_reset_drive();
	ide_irq_ack( ide_bd);
	splx( s);
	return -1;
}

/* The deferred call (root_defer): attach the interrupt, self-test, decide. */
static int
gayle_defer( arg)
char	*arg;
{
	int	e;

	e = intr_attach( IDE_LEVEL, ide_intr2, (char *)0, "gayle");
	if (e) {
		ide_selftest = 2;
		printf( "ide: intr_attach failed (errno %d), polled I/O\n", e);
		return 0;
	}
	if (ide_selftest_run() == 0) {
		ide_selftest = 1;
		printf( "ide: %s, interrupt I/O\n", ide_bd->name);
		return 0;
	}
	ide_selftest = 2;
	(void) intr_detach( IDE_LEVEL, ide_intr2, (char *)0);
	printf( "ide: interrupt self-test failed, polled I/O\n");
	return 0;
}

/* The load entry, in the root pass (see the block comment above). */
int
gayle_load( params)
char	*params;
{
	struct ideboard	*bd;
	ushort	vposr;
	int	e, i;

	ide_intr_enable  = gayle_iparam( params, "intr", 1, 0, 1);
	ide_slave_enable = gayle_iparam( params, "slave", 0, 0, 2);
	ide_debug        = gayle_iparam( params, "debug", 0, 0, 2);
	ide_master_unit  = gayle_iparam( params, "master", 6, 0, 7);
	ide_slave_unit   = gayle_iparam( params, "slaveunit", 5, 0, 7);
	ide_board        = gayle_iparam( params, "board", 0, -1, IDE_NBD_MODULE);

	if (ide_board < 0)
		return ENXIO;
	vposr = RW( IDE_VPOSR_ADDR);
	if (!IDE_VPOSR_IS_AGA( vposr))
		return ENXIO;
	if (ide_board > 0)
		bd = &ideboards[ide_board - 1];
	else {
		ide_cardres = ide_rom_cardres();
		bd = &ideboards[ide_cardres ? IDE_BD_A1200 : IDE_BD_A4000];
	}
	if (bprobe( (caddr_t)(bd->base + IDE_REG_ALTSTAT), -1) != 0)
		return ENXIO;
	if (!ide_readback( bd))
		return ENXIO;
	ide_bd = bd;
	ide_state = 2;
	ide_attach();
	for (i = 0; i < IDE_NDRIVES; ++i)
		if (ideunits[i].present)
			break;
	if (i == IDE_NDRIVES) {
		idereset();
		return ENXIO;
	}
	e = sd_register( idequeue, 0, (char *)ide_bd->base, ide_bd->name, IDE_SD_RANK);
	if (e) {
		printf( "ide: sd_register failed (errno %d)\n", e);
		idereset();
		return e;
	}
	printf( "ide: %s at 0x%x, polled I/O\n", ide_bd->name, (int)ide_bd->base);
	if (ide_intr_enable) {
		e = root_defer( gayle_defer, (char *)0);
		if (e)
			printf( "ide: root_defer failed (errno %d), polled I/O\n", e);
	}
	return 0;
}

/*
 * The core calls this only when a load fails after gayle_load() returned 0:
 * mask the drive and forget the probe.
 */
int
gayle_unload()
{
	if (ide_bd) {
		TW( IDE_REG_CONTROL, IDE_CTL_NIEN);
		ide_irq_enable( ide_bd, 0);
	}
	idereset();
	return 0;
}

/* The halt pass: mask the drive's interrupt (nIEN) and the Gayle's.  No flush, no wait. */
void
gayle_halt()
{
	ide_halted++;
	if (ide_bd == 0)
		return;
	TW( IDE_REG_CONTROL, IDE_CTL_NIEN);
	ide_irq_enable( ide_bd, 0);
}

#ifndef HOST_TEST
/* unload=never: a root-class module is pinned */
MOD_HDRV_WRAPPER(gayle, gayle_load, gayle_unload, gayle_halt, "Gayle IDE (A4000, A1200)");
#endif
#endif	/* !IDE_BUDDHA */

#if IDE_BUDDHA
/* ============================================================================
 * BUDDHA: the module form, `buddha' (amix-dlm contracts C4-2 version 11
 * section 8.2, C4-1 version 9).  module/buddha/Master gives the .moddata:
 * class=root, rank=ide, unload=never, no majors, and the match keys (the
 * Catweasel Z-II's, and a stand-in for the Buddha's on the 1.1 kernels: see
 * there).  Nothing here depends on the key: the boards are found with
 * autocon() by manufacturer and product, on any Amiga, with no chipset or
 * Kickstart gate.
 *
 * buddha_load() runs in the root pass, at level 7 with the clock stopped
 * (see the Gayle's block above for what that rules out).  In order:
 *   1. the parameters: intr (1: interrupts after the self-test), slave
 *      (1: master and slave on every port; 0 masters only; 2 slaves only if
 *      ATAPI), debug (0);
 *   2. the boards: autocon() for every Buddha (0x1212/0x00), then every
 *      Catweasel Z-II (0x1212/0x2A), at most four in all;
 *   3. each port (two on a Buddha, three on a Catweasel Z-II): bprobe() of
 *      its alternate status, then the task-file readback (on the 68030 line
 *      bprobe() catches no empty window; the readback is what the open bus
 *      cannot fake).  A port that does not answer is not touched again;
 *   4. the attach: polled IDENTIFY of the master of every answering port,
 *      and of its slave unless slave=0; every drive is left with nIEN set;
 *   5. sd_register(idequeue, b, board base, "Buddha IDE" or "Catweasel IDE",
 *      4), once per board with a drive, b the board's index here and the
 *      base its AutoConfig address, never a port's;
 *   6. with intr=1, root_defer(buddha_defer, 0).
 * No board, no answering port, or no drive: ENXIO, silent.
 *
 * The units are fixed per port and drive (bud_unit[]): port 0 master 6,
 * slave 5; port 1 3 and 2; port 2 (Catweasel Z-II) 1 and 0.
 *
 * buddha_defer() runs at the first sysinit, in process context at level 0:
 * intr_attach() at level 2, then a self-test on every port with a drive.
 * For each, the board's enable at 0xFC0 is written (once per board), nIEN
 * is cleared on that port alone and a command that interrupts is issued:
 * READ VERIFY SECTOR(S) of LBA 0 to an ATA drive, IDENTIFY PACKET DEVICE to
 * a port with only an ATAPI device (its data drained afterwards).  When
 * every port interrupted, nIEN is cleared on all of them: interrupt I/O.
 * Otherwise every drive keeps nIEN, the handler is detached and the driver
 * stays polled.  The enable cannot be taken back and need not be: with
 * nIEN set no drive drives INTRQ.
 *
 * buddha_halt() sets nIEN on every drive; buddha_unload() does too and
 * forgets the boards.
 * ========================================================================= */

extern int	autocon();		/* (long pc, int n, long *base, long *size): 1 found */

ulong	bud_extra_boards;		/* boards found beyond BUD_NBOARDS, not driven */
static char	bud_name_buddha[] = "Buddha IDE";
static char	bud_name_cwz2[]   = "Catweasel IDE";

/* The boards of one product: their ports set up and probed. */
static void
bud_find( pc, nports, name)
long	pc;
int	nports;
char	*name;
{
	struct ideboard	*ch;
	long		base, size;
	int		n, b, p;

	for (n = 0; autocon( pc, n, &base, &size); ++n) {
		if (bud_nboards >= BUD_NBOARDS) {
			bud_extra_boards++;
			continue;
		}
		b = bud_nboards++;
		bud_base[b]   = (ulong)base;
		bud_nports[b] = nports;
		for (p = 0; p < nports; ++p) {
			ch = &bud_ch[b * BUD_NPORTS + p];
			ch->name    = name;
			ch->base    = (ulong)base + BUD_PORT0 + BUD_PORTSTEP * p;
			ch->irqstat = (ulong)base + BUD_IRQSTAT0 + BUD_IRQSTEP * p;
			ch->irqack  = 0;
			ch->irqena  = (ulong)base + BUD_INTENA;
			if (bprobe( (caddr_t)(ch->base + IDE_REG_ALTSTAT), -1) == 0 && ide_readback( ch))
				bud_chok[b * BUD_NPORTS + p] = 1;
		}
	}
}

/* Has board b a drive the attach found? */
static int
bud_has_drive( b)
int	b;
{
	int	d;

	for (d = b * BUD_NPORTS * 2; d < (b + 1) * BUD_NPORTS * 2; ++d)
		if (ideunits[d].present)
			return 1;
	return 0;
}

/* The level-2 entry: "ours" when ideintr() took an interrupt on any port. */
static int
bud_intr2( arg)
char	*arg;
{
	ulong	before, other;

	before = ide_intr_count;
	other  = bud_intr_other;
	ideintr();
	return ide_intr_count != before || bud_intr_other != other;
}

/*
 * The self-test of port i (process context, level 0, the queue idle).
 * Returns 0 when the port's drive interrupted, -1 when it did not, 1 when
 * the port has no drive to ask.  nIEN is set again before it returns.
 */
static int
bud_selftest_port( i)
int	i;
{
	struct ideunit	*u;
	ulong		before, t;
	int		d, s, st, n, ok;

	d = i * 2;
	if (!ideunits[d].present)
		d++;
	if (!ideunits[d].present)
		return 1;
	u = &ideunits[d];
	s = sdspl();
	st = ide_select( d);			/* ide_bd is now port i */
	if (st < 0 || (!u->atapi && !(st & IDE_ST_DRDY))) {
		splx( s);
		return -1;
	}
	before = ide_intr_count;
	ide_st_armed = 1;
	ide_irq_enable( ide_bd, 1);		/* the board's enable, once */
	TW( IDE_REG_CONTROL, 0);		/* nIEN clear on this port only */
	if (u->atapi) {
		TW( IDE_REG_COMMAND, IDE_CMD_PIDENTIFY);
		ide_settle();
	} else
		ide_program( d, IDE_CMD_VERIFY, 0L, 1L);
	for (t = 0; ide_intr_count == before && t < ide_timeout_secs * HZ; ++t) {
		/* one tick; at sdspl, so the wakeup cannot come before the sleep */
		timeout( wakeup, (caddr_t)&ide_st_chan, 1);
		(void) sleep( (caddr_t)&ide_st_chan, PRIBIO);
	}
	ide_st_armed = 0;
	ide_selftest_ticks += t;
	ok = ide_intr_count != before;
	TW( IDE_REG_CONTROL, IDE_CTL_NIEN);
	if (u->atapi) {				/* IDENTIFY PACKET DEVICE's data */
		st = ide_wait_drq( ide_spins);
		if (st >= 0 && (st & IDE_ST_DRQ))
			for (n = 0; n < IDE_SECTOR / 2; ++n)
				(void) TRW();
	}
	st = ide_wait( IDE_ST_BSY, 0, ide_spins);	/* the command's end, polled */
	if (st < 0)
		ide_reset_drive();
	splx( s);
	return ok ? 0 : -1;
}

/* Every port with a drive must interrupt; then interrupt I/O on all. */
static int
bud_selftest()
{
	int	i, r, n, s;

	n = 0;
	for (i = 0; i < BUD_NCHAN; ++i) {
		if (!bud_chok[i])
			continue;
		r = bud_selftest_port( i);
		if (r < 0) {
			if (ide_debug)
				printf( "buddha: port %d of board %d did not interrupt\n",
					i % BUD_NPORTS, i / BUD_NPORTS);
			return -1;
		}
		if (r == 0)
			n++;
	}
	if (n == 0)
		return -1;
	s = sdspl();
	ide_use_intr = 1;
	bud_ctl_all( 0);			/* nIEN clear: the drives may assert INTRQ */
	splx( s);
	return 0;
}

/* The deferred call (root_defer): attach the interrupt, self-test, decide. */
static int
buddha_defer( arg)
char	*arg;
{
	int	e;

	e = intr_attach( IDE_LEVEL, bud_intr2, (char *)0, "buddha");
	if (e) {
		ide_selftest = 2;
		printf( "buddha: intr_attach failed (errno %d), polled I/O\n", e);
		return 0;
	}
	if (bud_selftest() == 0) {
		ide_selftest = 1;
		printf( "buddha: interrupt I/O\n");
		return 0;
	}
	ide_selftest = 2;
	bud_ctl_all( IDE_CTL_NIEN);
	(void) intr_detach( IDE_LEVEL, bud_intr2, (char *)0);
	printf( "buddha: interrupt self-test failed, polled I/O\n");
	return 0;
}

/* The load entry, in the root pass (see the block comment above). */
int
buddha_load( params)
char	*params;
{
	int	b, i, e, nreg;

	ide_intr_enable  = gayle_iparam( params, "intr", 1, 0, 1);
	ide_slave_enable = gayle_iparam( params, "slave", 1, 0, 2);
	ide_debug        = gayle_iparam( params, "debug", 0, 0, 2);

	idereset();
	bud_extra_boards = 0;
	bud_find( BUD_PC_BUDDHA, 2, bud_name_buddha);
	bud_find( BUD_PC_CWZ2, 3, bud_name_cwz2);
	for (i = 0; i < BUD_NCHAN; ++i)
		if (bud_chok[i])
			break;
	if (i == BUD_NCHAN) {
		idereset();
		return ENXIO;
	}
	ide_bd = &bud_ch[i];
	ide_state = 2;
	ide_attach();
	nreg = 0;
	for (b = 0; b < bud_nboards; ++b) {
		if (!bud_has_drive( b))
			continue;
		e = sd_register( idequeue, (uint)b, (char *)bud_base[b],
				 bud_ch[b * BUD_NPORTS].name, IDE_SD_RANK);
		if (e) {		/* the core undoes the earlier registrations */
			printf( "buddha: sd_register failed (errno %d)\n", e);
			bud_ctl_all( IDE_CTL_NIEN);
			idereset();
			return e;
		}
		nreg++;
		printf( "buddha: %s at 0x%x, polled I/O\n", bud_ch[b * BUD_NPORTS].name,
			(int)bud_base[b]);
	}
	if (nreg == 0) {
		idereset();
		return ENXIO;
	}
	if (bud_extra_boards)
		printf( "buddha: %d more board(s) found, not driven\n", (int)bud_extra_boards);
	if (ide_intr_enable) {
		e = root_defer( buddha_defer, (char *)0);
		if (e)
			printf( "buddha: root_defer failed (errno %d), polled I/O\n", e);
	}
	return 0;
}

/*
 * The core calls this only when a load fails after buddha_load() returned 0:
 * mask the drives and forget the boards.
 */
int
buddha_unload()
{
	bud_ctl_all( IDE_CTL_NIEN);
	idereset();
	return 0;
}

/* The halt pass: nIEN on every drive.  No flush, no wait. */
void
buddha_halt()
{
	ide_halted++;
	bud_ctl_all( IDE_CTL_NIEN);
}

#ifndef HOST_TEST
/* unload=never: a root-class module is pinned */
MOD_HDRV_WRAPPER(buddha, buddha_load, buddha_unload, buddha_halt, "Buddha and Catweasel Z-II IDE");
#endif
#endif	/* IDE_BUDDHA */
#endif	/* IDE_MODULE */
