/*
 *   Copyright (c) International Business Machines Corp., 2006, 2009
 *
 *   This program is free software;  you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY;  without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
 *   the GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program;  if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef _UAPI_I2CFSI_H
#define _UAPI_I2CFSI_H

#define  I2C_RDRW_IOCTL_MAX_MSGS        42

typedef struct iic_rec_pol
{
	unsigned long redo_pol;
#define IIC_VAL_ADDR_NOACK	0x00010000
#define IIC_VAL_DATA_NOACK	0x00020000
#define IIC_VAL_TIMEOUT		0x00040000
#define IIC_VAL_LOST_ARB	0x00080000
#define IIC_VAL_BUS_ERR		0x00100000
#define IIC_VAL_ALL_ERRS	0xffff0000
	unsigned long rsvd;
	unsigned long redo_delay;
} iic_rec_pol_t;

#define IIC_VAL_100KHZ  100
#define IIC_VAL_400KHZ	400
typedef struct iic_xfr_opts
{
	unsigned short rsvd;
	unsigned short dev_addr;	// address of end device
	unsigned short dev_width;	// number of bytes for offset (1-4)
	unsigned long inc_addr;		// mask of address bits to increment
					// for devices that span multiple
					// addresses.
	unsigned long timeout;		// operation timeout (msec)
	unsigned short wdelay;		// delay between write xfrs (msec)
	unsigned short rdelay;		// delay between read xfrs (msec)
	unsigned short wsplit;		// splits writes into smaller chunks
	unsigned short rsplit;		// splits reads into smaller chunks
	unsigned long offset;		// offset from beginning of device
	unsigned long flags;		// flags defined below
} iic_xfr_opts_t;

enum 
{
	IIC_FORCE_DMA = 0x01,		// use dma regardless of xfr size
	IIC_NO_DMA = 0x02,		// disallow dma
	IIC_SPECIAL_RD = 0x04,		// workaround for PLL/CRC chips
	IIC_REPEATED_START = 0x08,      // repeated start
};

typedef struct iic_opts
{
	iic_xfr_opts_t xfr_opts;
	iic_rec_pol_t recovery;
} iic_opts_t;

typedef struct iic_lock
{
	unsigned short mask;
	unsigned short addr;
	unsigned long timeout;
} iic_lock_t;

typedef struct iicslv_opts
{
	unsigned long addr;
	unsigned long timeout;
} iicslv_opts_t;

#define IICSLV_ZBUF_MAX_SZ	256

/* external master access mode of local slave shared buffer */
enum 
{
	IICSLV_BUF_MODE_EXT_R = 1,      
	IICSLV_BUF_MODE_EXT_RW = 2,  
};

/* Master IOCTL Ordinal Numbers */
#define IIC_IOC_MAGIC 		0x07
enum
{
	/* 0 bytes */
	IIC_IOC_RESET_LIGHT,
	IIC_IOC_RESET_FULL,

	IIC_IOC_0_BYTES = IIC_IOC_RESET_FULL,

	/* 4 bytes */
	IIC_IOC_SPEED,
	IIC_IOC_DEV_ADDR,
	IIC_IOC_DEV_WIDTH,
	IIC_IOC_OFFSET,
	IIC_IOC_INC_ADDR,
	IIC_IOC_TIMEOUT,
	IIC_IOC_RDELAY,
	IIC_IOC_WDELAY,
	IIC_IOC_RSPLIT,
	IIC_IOC_WSPLIT,
	IIC_IOC_REDO_POL,
	IIC_IOC_SPD_POL,
	IIC_IOC_REDO_DELAY,
	IIC_IOC_BUS_STATE,
#define IIC_VAL_BOTH_LO 0x00
#define IIC_VAL_SDA_LO  0x01
#define IIC_VAL_SCL_LO  0x02
#define IIC_VAL_BOTH_HI 0x03
	IIC_IOC_FLAGS,

	IIC_IOC_4_BYTES = IIC_IOC_FLAGS,

	/* Objects */
	IIC_IOC_LCK_ADDR,
	IIC_IOC_ULCK_ADDR,
	IIC_IOC_LCK_ENG,
	IIC_IOC_ULCK_ENG,
	IIC_IOC_ALL,
	IIC_IOC_DISPLAY_REGS,
	IIC_IOC_REPEATED_IO,
	IIC_IOC_MAXNR = IIC_IOC_REPEATED_IO,
};

#endif
