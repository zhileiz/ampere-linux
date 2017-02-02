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

/*
 * we don't know ahead of time how many minors will be needed as this is based 
 * on platform type and the possibility of sparsely populated FRU's on special 
 * systems so picking a safely large number
 */
#define IIC_FSI_MAX_DEVS	1024

#endif
