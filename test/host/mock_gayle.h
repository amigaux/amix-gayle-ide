/*
 * Mock of the Amiga Gayle IDE port (A4000, A1200 and A600 flavours) with up
 * to two ATA-2 drives, for the host harness.  Addresses are absolute, exactly
 * as the driver issues them; the selected board decides which respond.
 *
 * Byte order model: a 16-bit DATA read returns (stream[0] << 8) | stream[1]
 * where `stream' is the byte sequence the drive puts on the 16-bit bus in
 * little-endian word order -- the WinUAE/Amiberry Gayle model (ide.cpp
 * ide_get_data_2), which is how sector bytes arrive in natural order on the
 * 68k and IDENTIFY words arrive swapped.
 *
 * Interrupts: the drive asserts INTRQ (mock_intrq) when nIEN is clear and a
 * sector is ready (READ), a sector was accepted (WRITE), or a command ends.
 * Reading STATUS drops it.  On the A4000 the Gayle status register mirrors it
 * live; on the A1200/A600 it is latched into INTREQ until acknowledged and
 * only reaches INT2 when INTENA bit 7 is set.  mock_int2_asserted() tells the
 * harness when the kernel's p2int would call the handlers.
 */
#ifndef MOCK_GAYLE_H
#define MOCK_GAYLE_H

#define MOCK_SECTOR	512

#define MOCK_BOARD_NONE		0	/* no Gayle at all (A3000/A2000 class) */
#define MOCK_BOARD_A4000	1
#define MOCK_BOARD_A1200	2
#define MOCK_BOARD_A600		3

struct mock_drive {
	int		present;
	unsigned long	nsectors;
	int		flush;		/* advertise FLUSH CACHE */
	int		nolba;		/* advertise no LBA support */
	char		model[41];
	unsigned char	*disk;		/* nsectors * 512 */
	long		unc_lba;	/* -1, or LBA that fails with UNC */
	long		df_lba;		/* -1, or LBA that fails with DF set and ERR clear */
	int		stuck_bsy;	/* status always BSY, even after SRST */
	int		hung;		/* status BSY, no INTRQ, until a software reset */
	int		hang_on_cmd;	/* accept the next READ/WRITE, then go `hung' */
	int		mute_intrq;	/* never assert INTRQ (interrupt lost) */
	/* ATAPI CD-ROM (mock_add_cdrom) */
	int		atapi;
	int		pkt16;		/* 16-byte command packets */
	unsigned long	cd_blocks;	/* 2048-byte blocks on the disc */
	unsigned char	*cd;
	int		no_medium;	/* TUR/READ/READ CAPACITY: NOT READY, 3A */
	unsigned	chunk;		/* bytes per DRQ phase (0: the host's byte count limit) */
	unsigned char	skey, asc;	/* sense held for REQUEST SENSE */
	int		hang_on_pkt;	/* accept the next packet, then stay BSY until SRST */
	int		glitch;		/* after each packet, polled status reads 0 (BSY, DRQ clear, IREASON
					 * CoD) this many times before the real phase shows (Amiberry) */
	int		no_ir3;		/* completion leaves IREASON at IO instead of CoD|IO */
};

#define MOCK_CD_BLOCK	2048

extern struct mock_drive	mock_drv[2];
extern int			mock_board;
extern unsigned short		mock_vposr_value;	/* used when mock_vposr_override */
extern int			mock_vposr_override;
extern unsigned long		mock_accesses;		/* every register access */
extern unsigned long		mock_bad_accesses;	/* accesses the board does not decode */
extern unsigned long		mock_taskfile_accesses;	/* task-file accesses on the selected board */
extern unsigned long		mock_slave_selects;	/* SDH writes with DRV1 set */
extern unsigned long		mock_cmd_count[256];	/* commands issued, by opcode */
extern int			mock_last_cmd;
extern int			mock_cmd_log[64];	/* opcodes in issue order */
extern int			mock_cmd_log_n;
extern int			mock_nien;		/* drive's nIEN (1 = interrupts off) */
extern int			mock_intrq;		/* drive asserting INTRQ */
extern int			mock_irq_latch;		/* A1200/A600 INTREQ bit 7 */
extern unsigned char		mock_intena;		/* A1200/A600 INTENA register */
extern unsigned long		mock_irq_acks;		/* acknowledge writes seen */
extern int			mock_latch_stuck;	/* A1200/A600: an acknowledge does not clear the latch */
extern unsigned long		mock_id_reads;		/* Gayle ID register reads */
extern unsigned long		mock_id_writes;		/* Gayle ID register writes */
extern unsigned long		mock_srst_count;	/* software resets seen (SRST set) */

/* The Kickstart ROM window 0xF80000-0xFFFFFF (the module's board choice).
 * MOCK_ROM_AUTO gives the board's own: an A1200/A600 ROM (card.resource
 * among its resident tags) on those boards, an A4000-style ROM otherwise,
 * which carries the string "card.resource" and a tag-shaped decoy with that
 * name but no resident of it. */
#define MOCK_ROM_AUTO	0
#define MOCK_ROM_PLAIN	1	/* A3000/A4000-style: no card.resource resident */
#define MOCK_ROM_CARD	2	/* A600/A1200-style: card.resource is resident */
#define MOCK_ROM_BLANK	3	/* all 0xFF: no resident tag at all */
#define MOCK_ROM_TAGS	5	/* resident tags in either image */
extern int			mock_rom;		/* MOCK_ROM_*; mock_reset() sets AUTO */
extern unsigned long		mock_rom_reads;		/* ROM-window byte reads (not in mock_accesses) */
extern unsigned long		mock_dd_accesses;	/* any access in 0xDD0000-0xDDFFFF */
extern unsigned long		mock_da_accesses;	/* any access in 0xDA0000-0xDAFFFF */

/* CPU-side model (IDE_CACHE build): see mock_gayle.c */
#define MOCK_LINEOPS	8192
#define MOCK_CPUSHL	1
#define MOCK_CINVL	2
struct mock_lineop {
	unsigned long	addr, seq;
	int		op;
};
extern unsigned long		mock_bsy_reads;		/* next STATUS reads that show BSY */
extern unsigned long		mock_seq, mock_data_first, mock_data_last;
extern unsigned long		mock_order_violations;	/* task-file read with a write not yet synced */
extern unsigned long		mock_dtt0, mock_dtt1, mock_cacr;
extern struct mock_lineop	mock_lineops[MOCK_LINEOPS];
extern int			mock_nlineops;
extern unsigned long		mock_lineops_lost;

/* ATAPI: the packets the drive executed, the last byte count limit, MODE SELECT data */
extern unsigned char		mock_pkt_log[64];	/* opcodes in order */
extern int			mock_pkt_log_n;
extern unsigned			mock_pkt_bc;		/* byte count limit of the last PACKET */
extern unsigned char		mock_pkt_last[16];	/* the last packet as received */
extern unsigned char		mock_atapi_out[256];	/* data-out bytes of the last command */
extern unsigned			mock_atapi_out_n;

void mock_reset();
void mock_add_cdrom();			/* (unit, blocks, model) */
void mock_add_drive();			/* (unit, nsectors, model, flush) */
void mock_free();
unsigned long mock_cmds();		/* total commands issued */
int  mock_int2_asserted();		/* would p2int run now? */

#endif
