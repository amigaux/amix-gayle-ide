/*
 * Host-test stand-in for the ABI kit's sys/sdseg.h (kernel/dlm
 * docs/contracts/C4-7-sd-segments.md, kits k30.1.6 / k46.1.7): the same
 * structures and flags, and DDI_SD_LIMITS as a plain call of the harness's
 * mock sd_limits (ide_modtest.c), which can answer ENOSYS like a kernel
 * before the segment minor.  The kit's header refers to sd_limits weakly.
 */
#ifndef _SYS_SDSEG_H
#define _SYS_SDSEG_H

struct sdseg {
	unsigned long	ss_pa;
	unsigned long	ss_len;
};

struct sdcomx {
	struct sdcom	sx_com;
	unsigned short	sx_flags;
	unsigned short	sx_nseg;
	struct sdseg	*sx_seg;
	unsigned int	sx_resid;
};

#define	SDX_SEG		0x0001

struct sdlim {
	unsigned short	sl_ver;
	unsigned short	sl_flags;
	unsigned long	sl_maxbytes;
	unsigned short	sl_maxseg;
	unsigned short	sl_align;
	unsigned short	sl_gran;
	unsigned short	sl_pad;
	unsigned long	sl_reach;
	bool		(*sl_queue)();
};

#define	SDLIM_VER	1
#define	SDL_SEG		0x0001
#define	SDL_ALLRAM	0x0002

extern int	sd_limits();
#define	DDI_SD_LIMITS(queuefn, c, l)	sd_limits((queuefn), (unsigned int)(c), (l))

#endif /* _SYS_SDSEG_H */
