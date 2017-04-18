/*
 * Copyright (C) IBM Corporation 2017
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERGCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __FSI_SBEFIFO_H__
#define __FSI_SBEFIFO_H__

#include <linux/types.h>

struct device_node;
struct sbefifo;
struct sbefifo_client;

struct sbefifo_drv_ref {
	struct list_head link;
	void (*notify)(struct sbefifo_drv_ref *ref);
};

extern struct sbefifo *sbefifo_drv_reference(struct device_node *node,
					     struct sbefifo_drv_ref *ref);

extern struct sbefifo_client *sbefifo_drv_open(struct sbefifo *sbefifo,
					       unsigned long flags);
extern int sbefifo_drv_read(struct sbefifo_client *client, char *buf,
			    size_t len);
extern int sbefifo_drv_write(struct sbefifo_client *client, const char *buf,
			     size_t len);
extern void sbefifo_drv_release(struct sbefifo_client *client);

extern int sbefifo_drv_get_idx(struct sbefifo *sbefifo);

#endif /* __FSI_SBEFIFO_H__ */
