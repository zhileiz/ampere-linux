/*
 *   Copyright (c) International Business Machines Corp., 2006
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

/************************ BOE Engine offsets *********************************/
#define IIC_BOE_FIFO 		0x00
#define IIC_BOE_CMD		0x01
#define IIC_BOE_MODE		0x02
#define IIC_BOE_WATER_MARK	0x03
#define IIC_BOE_INT_MASK	0x04
#define IIC_BOE_INT_COND	0x05
#define IIC_BOE_OR_INT_MASK	0x05
#define IIC_BOE_INTS		0x06
#define IIC_BOE_AND_INT_MASK	0x06
#define IIC_BOE_STAT		0x07
#define IIC_BOE_RESET_I2C	0x07
#define IIC_BOE_ESTAT		0x08
#define IIC_BOE_RESET_ERR	0x08
#define IIC_BOE_RESID_LEN	0x09
#define IIC_BOE_SET_SCL		0x09
#define IIC_BOE_RESERVED        0x0a
#define IIC_BOE_PORTBUSY        0x0a
#define IIC_BOE_RESET_SCL	0x0b
#define IIC_BOE_SET_SDA		0x0c
#define IIC_BOE_RESET_SDA	0x0d
#define IIC_BOE_MAX_OFFSET	IIC_BOE_RESET_SDA

/************************ BOE Bit Definitions ********************************/

#define IIC_BOE_MAX_BUSES	16
#define IIC_BOE_MAX_FIFO_SIZE	32
#define IIC_BOE_DFLT_FIFO_SIZE	8

/* cmd register bits */
#define IIC_BOE_WITH_START	0x80000000
#define IIC_BOE_WITH_ADDR	0x40000000
#define IIC_BOE_RD_CONT		0x20000000
#define IIC_BOE_WITH_STOP	0x10000000
#define IIC_BOE_FORCE_LAUNCH	0x08000000
#define IIC_BOE_ADDR		0x00fe0000
#define IIC_BOE_ADDR_SHIFT	16
#define IIC_BOE_READ		0x00010000
#define IIC_BOE_XFR_LEN		0x0000ffff

#define IIC_BOE_MK_ADDR(addr) ((addr << IIC_BOE_ADDR_SHIFT) & IIC_BOE_ADDR)
#define IIC_BOE_GET_ADDR(r) ((r & IIC_BOE_ADDR) >> IIC_BOE_ADDR_SHIFT)

/* mode register bits */
#define IIC_BOE_CLKDIV_LO	0xff000000
#define IIC_BOE_CLKDIV_LO_SHIFT 24
#define IIC_BOE_PORT		0x00ff0000
#define IIC_BOE_PORT_SHIFT	16
#define IIC_BOE_MODE_RSVD	0x0000ff00
#define IIC_BOE_CLKDIV_HI	0x000000f0
#define IIC_BOE_CLKDIV_HI_SHIFT 4
#define IIC_BOE_ENHANCED	0x00000008
#define IIC_BOE_DIAG		0x00000004
#define IIC_BOE_PACE_ALLOW	0x00000002
#define IIC_BOE_WRAP		0x00000001

#define IIC_BOE_GET_PORT(r) ((r & IIC_BOE_PORT) >> IIC_BOE_PORT_SHIFT)
#define IIC_BOE_MK_PORT(p) ((p << IIC_BOE_PORT_SHIFT) & IIC_BOE_PORT)

#define IIC_BOE_MK_CLKDIV(val) \
((val << IIC_BOE_CLKDIV_LO_SHIFT) | \
((val >> IIC_BOE_CLKDIV_HI_SHIFT) & IIC_BOE_CLKDIV_HI))

#define IIC_BOE_GET_CLKDIV(r) \
(((r & IIC_BOE_CLKDIV_LO) >> IIC_BOE_CLKDIV_LO_SHIFT) | \
((r & IIC_BOE_CLKDIV_HI) << IIC_BOE_CLKDIV_HI_SHIFT))

/* Z7 plus mode register */
#define IIC_BOE_Z7_CLKDIV	0xffff0000
#define IIC_BOE_Z7_CLKDIV_SHIFT	16
#define IIC_BOE_Z7_PORT		0x0000fc00
#define IIC_BOE_Z7_PORT_SHIFT	10

#define IIC_BOE_Z7_GET_PORT(r) ((r & IIC_BOE_Z7_PORT) >> IIC_BOE_Z7_PORT_SHIFT)
#define IIC_BOE_Z7_MK_PORT(p) ((p << IIC_BOE_Z7_PORT_SHIFT) & IIC_BOE_Z7_PORT)

#define IIC_BOE_Z7_MK_CLKDIV(val) ((val << IIC_BOE_Z7_CLKDIV_SHIFT))
#define IIC_BOE_Z7_GET_CLKDIV(r) ((r & IIC_BOE_Z7_CLKDIV) >> IIC_BOE_Z7_CLKDIV_SHIFT)

/* water mark register bits */
#define IIC_BOE_HI_MRK		0x0000ff00
#define IIC_BOE_HI_MRK_SHIFT	8
#define IIC_BOE_LO_MRK		0x000000ff
#define IIC_BOE_LO_MRK_SHIFT	0
#define IIC_BOE_GET_LO_MRK(e) ((e->regs[IIC_BOE_WATER_MARK] & \
			        IIC_BOE_LO_MRK) >> \
                               IIC_BOE_LO_MRK_SHIFT)
#define IIC_BOE_MK_WATER_MRK(hi, lo)\
			((((hi) << IIC_BOE_HI_MRK_SHIFT) & IIC_BOE_HI_MRK) |\
			(((lo) << IIC_BOE_LO_MRK_SHIFT) & IIC_BOE_LO_MRK))
#define IIC_BOE_GET_HI_MRK(e) ((e->regs[IIC_BOE_WATER_MARK] & \
			        IIC_BOE_HI_MRK) >> \
                               IIC_BOE_HI_MRK_SHIFT)

/* interrupt, interrupt mask, interrupt condition bits */
#define IIC_BOE_INVALID_CMD	0x00008000
#define IIC_BOE_PARITY		0x00004000
#define IIC_BOE_BE_OVERRUN	0x00002000
#define IIC_BOE_BE_ACCESS	0x00001000
#define IIC_BOE_LOST_ARB	0x00000800
#define IIC_BOE_NACK		0x00000400
#define IIC_BOE_DAT_REQ		0x00000200
#define IIC_BOE_CMD_COMP	0x00000100
#define IIC_BOE_STOP_ERR	0x00000080
#define IIC_BOE_BUSY		0x00000040
#define IIC_BOE_IDLE		0x00000020
#define IIC_BOE_ALL_INTS	0x0000ffff
#define IIC_BOE_FE_ERRS		(IIC_BOE_LOST_ARB | IIC_BOE_NACK | \
				 IIC_BOE_STOP_ERR)
#define IIC_BOE_BE_ERRS		(IIC_BOE_INVALID_CMD | IIC_BOE_PARITY | \
				 IIC_BOE_BE_OVERRUN | IIC_BOE_BE_ACCESS)
#define IIC_BOE_ANY_ERR (IIC_BOE_BE_ERRS | IIC_BOE_FE_ERRS)

/* status register bits */
#define IIC_BOE_S_INVALID_CMD     0x80000000
#define IIC_BOE_S_PARITY          0x40000000
#define IIC_BOE_S_BE_OVERRUN      0x20000000
#define IIC_BOE_S_BE_ACCESS       0x10000000
#define IIC_BOE_S_LOST_ARB        0x08000000
#define IIC_BOE_S_NACK            0x04000000
#define IIC_BOE_S_DAT_REQ         0x02000000
#define IIC_BOE_S_CMD_COMP        0x01000000
#define IIC_BOE_S_STOP_ERR        0x00800000
#define IIC_BOE_STAT_SHIFT	  16
#define IIC_BOE_MAX_PORT	  0x000f0000
#define IIC_BOE_MAX_PORT_SHIFT	  16
#define IIC_BOE_ANY_INT		  0x00008000
#define IIC_BOE_SCL_IN		  0x00000800
#define IIC_BOE_SDA_IN		  0x00000400
#define IIC_BOE_PORT_BUSY	  0x00000200
#define IIC_BOE_SELF_BUSY	  0x00000100
#define IIC_BOE_FIFO_COUNT	  0x000000ff

#define IIC_BOE_S_BE_ERRS (IIC_BOE_BE_ERRS << IIC_BOE_STAT_SHIFT)
#define IIC_BOE_S_FE_ERRS (IIC_BOE_FE_ERRS << IIC_BOE_STAT_SHIFT)
#define IIC_BOE_S_ANY_ERR (IIC_BOE_ANY_ERR << IIC_BOE_STAT_SHIFT)

/* extended status register bits */
#define IIC_BOE_FIFO_SZ		0xff000000
#define IIC_BOE_FIFO_SZ_SHIFT	24
#define IIC_BOE_SCL_IN_SYN	0x00008000
#define IIC_BOE_SDA_IN_SYN	0x00004000
#define IIC_BOE_S_SCL		0x00002000
#define IIC_BOE_S_SDA		0x00001000
#define IIC_BOE_M_SCL		0x00000800
#define IIC_BOE_M_SDA		0x00000400
#define IIC_BOE_HI_WATER	0x00000200
#define IIC_BOE_LO_WATER	0x00000100
#define IIC_BOE_ES_PORT_BUSY	0x00000080
#define IIC_BOE_ES_SELF_BUSY	0x00000040
#define IIC_BOE_VERSION		0x0000001f

/* residual length register bits */
#define IIC_BOE_FE_LEN		0xffff0000
#define IIC_BOE_FE_LEN_SHIFT	16
#define IIC_BOE_BE_LEN		0x0000ffff
#define IIC_BOE_GET_BE(r)	(r & IIC_BOE_BE_LEN)
#define IIC_BOE_GET_FE(r) (r >> IIC_BOE_FE_LEN_SHIFT)

/* Murano/Centaur port busy register */
#define IIC_BOE_PORT_BUSY_REST  0x80000000

