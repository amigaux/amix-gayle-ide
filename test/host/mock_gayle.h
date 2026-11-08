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
};

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
extern unsigned long		mock_id_reads;		/* Gayle ID register reads */
extern unsigned long		mock_srst_count;	/* software resets seen (SRST set) */

void mock_reset();
void mock_add_drive();			/* (unit, nsectors, model, flush) */
void mock_free();
unsigned long mock_cmds();		/* total commands issued */
int  mock_int2_asserted();		/* would p2int run now? */

#endif
