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

#ifndef IIC_FSI_H
#define IIC_FSI_H

#define IIC_BUS_BITS 6
#define IIC_ENG_BITS 5
#define IIC_CFAM_BITS 2
#define IIC_LINK_BITS 6
#define IIC_PCFAM_BITS 2
#define IIC_PLINK_BITS 6
#define IIC_MAX_BUS (1U << IIC_BUS_BITS)
#define IIC_MAX_ENG (1U << IIC_ENG_BITS)
#define IIC_MAX_CFAM (1U << IIC_CFAM_BITS)
#define IIC_MAX_LINK (1U << IIC_LINK_BITS)
#define IIC_MAX_PCFAM (1U << IIC_PCFAM_BITS)
#define IIC_MAX_PLINK (1U << IIC_PLINK_BITS)
#define IIC_BUS_MASK ((1U << IIC_BUS_BITS) - 1U)
#define IIC_ENG_MASK (((1U << IIC_ENG_BITS) - 1U) << IIC_BUS_BITS)
#define IIC_CFAM_MASK (((1U << IIC_CFAM_BITS) - 1U) << \
			(IIC_BUS_BITS + IIC_ENG_BITS))
#define IIC_LINK_MASK (((1U << IIC_LINK_BITS) - 1U) << \
			(IIC_BUS_BITS + IIC_ENG_BITS + IIC_CFAM_BITS))
#define IIC_PCFAM_MASK (((1U << IIC_PCFAM_BITS) - 1U) << \
			(IIC_BUS_BITS + IIC_ENG_BITS + \
			IIC_CFAM_BITS + IIC_LINK_BITS))
#define IIC_PLINK_MASK (((1U << IIC_PLINK_BITS) - 1U) << \
			(IIC_BUS_BITS + IIC_ENG_BITS + IIC_CFAM_BITS + \
			 IIC_LINK_BITS + IIC_PCFAM_BITS))

/*
 * we don't know ahead of time how many minors will be needed as this is based 
 * on platform type and the possibility of sparsely populated FRU's on special 
 * systems so picking a safely large number
 */
#define IIC_FSI_MAX_DEVS	1024

#define IIC_SET_BUS(id, bus)  \
( \
 (id & ~IIC_BUS_MASK) | ((unsigned long)bus & IIC_BUS_MASK) \
)

#define IIC_SET_ENG(id, eng)  \
( \
 (id & ~IIC_ENG_MASK) | \
 (((unsigned long)eng << IIC_BUS_BITS) & IIC_ENG_MASK) \
)

#define IIC_SET_CFAM(id, cfam)  \
( \
 (id & ~IIC_CFAM_MASK) | \
 (((unsigned long)cfam << (IIC_BUS_BITS + IIC_ENG_BITS)) & IIC_CFAM_MASK) \
)

#define IIC_SET_LINK(id, link)  \
( \
 (id & ~IIC_LINK_MASK) | \
 (((unsigned long)link << (IIC_BUS_BITS + IIC_ENG_BITS + IIC_CFAM_BITS)) & \
  IIC_LINK_MASK) \
)

#define IIC_SET_PCFAM(id, pcfam) \
( \
 (id & ~IIC_PCFAM_MASK) | \
 (((unsigned long)pcfam << \
 (IIC_BUS_BITS + IIC_ENG_BITS + IIC_CFAM_BITS + IIC_LINK_BITS)) & \
 IIC_PCFAM_MASK) \
)

#define IIC_SET_PLINK(id, plink) \
( \
 (id & ~IIC_PLINK_MASK) | \
 (((unsigned long)plink << \
 (IIC_BUS_BITS + IIC_ENG_BITS + IIC_CFAM_BITS + IIC_LINK_BITS + IIC_PCFAM_BITS)) & \
 IIC_PLINK_MASK) \
)

#define IIC_MK_ENG_ID(plink, pcfam, link, cfam, eng) \
(\
	IIC_SET_BUS(IIC_SET_ENG(IIC_SET_CFAM(IIC_SET_LINK( \
			      IIC_SET_PCFAM(IIC_SET_PLINK(0U, plink), \
							  pcfam), \
							  link), \
							  cfam), \
			                                  eng), \
		                                          0U) \
)

#define IIC_GET_BUS(id) (id & IIC_BUS_MASK)
#define IIC_GET_ENG(id) ((id & IIC_ENG_MASK) >> IIC_BUS_BITS)
#define IIC_GET_CFAM(id) ((id & IIC_CFAM_MASK) >> (IIC_BUS_BITS + IIC_ENG_BITS))
#define IIC_GET_LINK(id) \
((id & IIC_LINK_MASK) >> (IIC_BUS_BITS + IIC_ENG_BITS + IIC_CFAM_BITS))
#define IIC_GET_PCFAM(id) \
((id & IIC_PCFAM_MASK) >> (IIC_BUS_BITS + IIC_ENG_BITS + \
	                   IIC_CFAM_BITS + IIC_LINK_BITS))
#define IIC_GET_PLINK(id) \
((id & IIC_PLINK_MASK) >> (IIC_BUS_BITS + IIC_ENG_BITS + \
			   IIC_CFAM_BITS + IIC_LINK_BITS + IIC_PCFAM_BITS))
#endif
