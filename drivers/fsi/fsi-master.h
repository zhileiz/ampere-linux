/*
 * FSI master definitions. These comprise the core <--> master interface,
 * to allow the core to interact with the (hardware-specific) masters.
 *
 * Copyright (C) IBM Corporation 2016
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef DRIVERS_FSI_MASTER_H
#define DRIVERS_FSI_MASTER_H

#include <linux/device.h>

/* Control Registers */
#define FSI_MMODE		0x0		/* R/W: mode */
#define FSI_MDLYR		0x4		/* R/W: delay */
#define FSI_MCRSP		0x8		/* R/W: clock rate */
#define FSI_MENP0		0x10		/* R/W: enable */
#define FSI_MLEVP0		0x18		/* R: plug detect */
#define FSI_MSENP0		0x18		/* S: Set enable */
#define FSI_MCENP0		0x20		/* C: Clear enable */
#define FSI_MAEB		0x70		/* R: Error address */
#define FSI_MVER		0x74		/* R: master version/type */
#define FSI_MRESP0		0xd0		/* W: Port reset */
#define FSI_MESRB0		0x1d0		/* R: Master error status */
#define FSI_MRESB0		0x1d0		/* W: Reset bridge */
#define FSI_MECTRL		0x2e0		/* W: Error control */

/* MMODE: Mode control */
#define FSI_MMODE_EIP		0x80000000	/* Enable interrupt polling */
#define FSI_MMODE_ECRC		0x40000000	/* Enable error recovery */
#define FSI_MMODE_EPC		0x10000000	/* Enable parity checking */
#define FSI_MMODE_P8_TO_LSB	0x00000010	/* Timeout value LSB */
						/* Rolf Fritz Nov 20, 2013: */
						/*   MSB=1, LSB=0 is 0.8 ms */
						/*   MSB=0, LSB=1 is 0.9 ms */
#define FSI_MMODE_CRS0SHFT	18		/* Clk rate selection 0 shift */
#define FSI_MMODE_CRS0MASK	0x3ff		/* Clk rate selection 0 mask */
#define FSI_MMODE_CRS1SHFT	8		/* Clk rate selection 1 shift */
#define FSI_MMODE_CRS1MASK	0x3ff		/* Clk rate selection 1 mask */

/* MRESB: Reset brindge */
#define FSI_MRESB_RST_GEN	0x80000000	/* General reset */
#define FSI_MRESB_RST_ERR	0x40000000	/* Error Reset */

/* MRESB: Reset port */
#define FSI_MRESP_RST_ALL_MASTER 0x20000000	/* Reset all FSI masters */
#define FSI_MRESP_RST_ALL_LINK	0x10000000	/* Reset all FSI port contr. */
#define FSI_MRESP_RST_MCR	0x08000000	/* Reset FSI master reg. */
#define FSI_MRESP_RST_PYE	0x04000000	/* Reset FSI parity error */
#define FSI_MRESP_RST_ALL	0xfc000000	/* Reset any error */

/* MECTRL: Error control */
#define FSI_MECTRL_EOAE		0x8000		/* Enable machine check when */
						/* master 0 in error */
#define FSI_MECTRL_P8_AUTO_TERM	0x4000		/* Auto terminate */

#define L_MSB_MASK(x)		(0x80000000 >> (x))

struct fsi_master {
	struct list_head my_slaves;
	bool		slave_list;
	struct device	*dev;
	int		idx;
	int		n_links;
	uint32_t	ipoll;
	int		(*read)(struct fsi_master *, int link,
				uint8_t slave, uint32_t addr,
				void *val, size_t size);
	int		(*write)(struct fsi_master *, int link,
				uint8_t slave, uint32_t addr,
				const void *val, size_t size);
	int		(*send_break)(struct fsi_master *, int link);
	int		(*link_enable)(struct fsi_master *, int link);
};

extern int fsi_master_register(struct fsi_master *master);
extern void fsi_master_unregister(struct fsi_master *master);
extern int fsi_master_start_ipoll(struct fsi_master *master);
extern void fsi_master_handle_error(struct fsi_master *master, uint32_t addr);

/**
 * crc4 helper: Given a starting crc4 state @c, calculate the crc4 vaue of @x,
 * which is @bits in length. This may be required by master implementations
 * that do not provide their own hardware checksums.
 *
 * The crc4 is performed on 4-bit chunks (which is all we need for FSI
 * calculations). Typically, we'll want a starting state of 0:
 *
 *  c = fsi_crc4(0, msg, len);
 *
 * To crc4 a message that includes a single start bit, initialise crc4 state
 * with:
 *
 *  c = fsi_crc4(0, 1, 1);
 *
 * Then update with message data:
 *
 *  c = fsi_crc4(c, msg, len);
 */
uint8_t fsi_crc4(uint8_t c, uint64_t x, int bits);

/* mmode encoders */
static inline u32 fsi_mmode_crs0(u32 x)
{
	return (x & FSI_MMODE_CRS0MASK) << FSI_MMODE_CRS0SHFT;
}

static inline u32 fsi_mmode_crs1(u32 x)
{
	return (x & FSI_MMODE_CRS1MASK) << FSI_MMODE_CRS1SHFT;
}

#endif /* DRIVERS_FSI_MASTER_H */
