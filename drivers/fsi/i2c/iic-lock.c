/*
 *   Copyright (c) International Business Machines Corp., 2006, 2010
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

#include <linux/sched.h>
#include <linux/slab.h>

#include "iic-int.h"

#define IIC_NO_MATCH 0
#define IIC_PARTIAL_MATCH 1
#define IIC_EXACT_MATCH 2
#define IIC_ADDR_MAX_BITS 10


void iic_lck_mgr_init(iic_lck_mgr_t* mgr)
{
	IENTER();
	INIT_LIST_HEAD(&mgr->locked);
	INIT_LIST_HEAD(&mgr->reqs);
	IEXIT(0);
}
EXPORT_SYMBOL(iic_lck_mgr_init);

int iic_get_match_type(short a_addr, short a_mask, short b_addr, short b_mask)
{
	int rc = IIC_PARTIAL_MATCH;
	short mask = a_mask | b_mask;

	IENTER();

	/* no match if the fixed bits aren't the same */
	if((a_addr & ~mask) != (b_addr & ~mask))
	{
		rc = IIC_NO_MATCH;
	}

	/* exact match if the fixed bits and mask bits are the same */
	else if(a_mask == b_mask)
	{
		rc = IIC_EXACT_MATCH;
	}

	/* everything else is partial */

	IEXIT(rc);
	return rc;
}


/* to must be in jiffies! */ 
int iic_wait_lock(iic_lck_mgr_t *lm, short addr, short mask, 
		  iic_client_t *client, unsigned long to)
{
	iic_lck_t *handle = 0;
	int rc = 0;
	unsigned long flags;

	IENTER();

	spin_lock_irqsave(&client->bus->eng->lock, flags);

	/* try the lock and enqueue our request if locked by someone else */
	rc = iic_req_lock(lm, addr, mask, client, &handle);

	spin_unlock_irqrestore(&client->bus->eng->lock, flags);

	if(rc == IIC_REQ_QUEUED)
	{
		/* wait for timeout, signal, or lock to be granted */
		rc = wait_event_interruptible_timeout(
				client->wait,
				handle->count > 0,
				to);
		if(rc > 0)
		{
			rc = 0;
		}
		else
		{
			if(!rc)
			{
				rc = -ETIME;
				IFLDe(4, "lock req timed out: client[%p] bus[%08lx] lock[%04x:%04x]\n", 
				      client, client->bus->bus_id,
				      handle->addr, handle->mask);
			}
			else if(rc == -ERESTARTSYS)
			{
				rc = -EINTR;
				IFLDe(4, "lock req interrupted: client[%p] bus[%08lx] lock[%04x:%04x]\n", 
				      client, client->bus->bus_id,
				      handle->addr, handle->mask);
			}

			/* interrupted or timed out.  delete request and
			 * return to caller.
			 */
			spin_lock_irqsave(&client->bus->eng->lock, flags);
			iic_unlock(lm, handle);
			spin_unlock_irqrestore(&client->bus->eng->lock, flags);
		}
	}
	IEXIT(rc);
	return rc;
}

int iic_sideways_lock_bus(iic_client_t * client, unsigned short addr,
			  unsigned short mask, unsigned long timeout)
{
	int rc = 0;
	iic_eng_t * eng = client->bus->eng;

	rc = iic_wait_lock(&eng->lck_mgr, addr, mask >> 1, client,
			   msecs_to_jiffies(timeout));

	return rc;
}
EXPORT_SYMBOL(iic_sideways_lock_bus);

int iic_sideways_unlock_bus(iic_client_t * client, unsigned short addr,
			    unsigned short mask)
{
	int rc = 0;
	iic_lck_t * klck;
	iic_eng_t * eng = client->bus->eng;

	spin_lock_irq(&eng->lock);
	klck = iic_find_handle(&eng->lck_mgr, client, addr, mask >> 1);

	if(klck)
	{
		rc = iic_unlock(&eng->lck_mgr, klck);
		if(!rc)
			iic_start_next_xfr(eng);
	}
	spin_unlock_irq(&eng->lock);

	return rc;
}
EXPORT_SYMBOL(iic_sideways_unlock_bus);

/* Engine must be locked before using this function!
 *
 * This function only requests a lock.  It doesn't block waiting for the
 * lock to be granted.  Instead, a handle is returned and the caller can
 * query this handle to determine if the lock was granted.  If the lock
 * count is not zero, the lock has been granted.
 *
 * returns IIC_REQ_QUEUED on success, negative value on failure.
 *
 * Locks from the same client can overlap each other, so a client could
 * lock all addresses on a bus and still request a subset of locks within
 * that range.
 * 
 * Here's the algorithm in a nutshell:
 *
 * Iterate through lock list
 *     -If exact match and same client, stop iterating and increment the
 * 	lock count of the existing lock.
 *     -If partial or exact match but different client, stop iterating and
 *     	queue a new request on the request queue without granting the lock.
 *     -If partial and same client, keep iterating.
 *     -If we reach the end and none of the above apply,
 *       create a new lock and increment the count.
 */
int iic_req_lock(iic_lck_mgr_t *lm, short addr, unsigned short mask,
		 iic_client_t *client, iic_lck_t **handle)
{
	iic_lck_t *iterator, *found;
	int mtype = IIC_NO_MATCH;
	int rc = IIC_REQ_QUEUED;
	iic_lck_t *new_req = 0;

	IENTER();
	found = 0;
	addr =  (client->bus->port << IIC_ADDR_MAX_BITS) | (addr >> 1);

	IFLDs(4, "REQLOCK  client[%p] bus[%08lx] lock[%04x:%04x]\n",
			client, client->bus->bus_id, addr, mask);

	/* Look for a partial or exact address match in the lock list */
	list_for_each_entry(iterator, &lm->locked, list)
	{
		mtype = iic_get_match_type(iterator->addr, iterator->mask,
				           addr, mask);
		if(mtype != IIC_NO_MATCH)
		{
			found = iterator;
			if((found->client == client) && 
					(mtype == IIC_EXACT_MATCH))
			{
				found->count++;
				IDBGd(4, "bus[%08lx] lock[%04x:%04x] count=%ld\n",
					client->bus->bus_id, addr, mask, 
					found->count);
				*handle = found;
				goto exit;
			}
			break;
		}
	}

	/* Exact matches with same client id have already exited.  Deal
	 * with the rest as follows:
	 * 	exact/partial matches w/diff client -> new lock requested
	 * 	no matches or partial w/same client -> new lock granted
	 */
	new_req = (iic_lck_t*)kmalloc(sizeof(iic_lck_t), GFP_KERNEL);
	if(!new_req)
	{
		rc = -ENOMEM;
		goto exit;
	}
	new_req->client = client;
	new_req->addr = addr;
	new_req->mask = mask;
	new_req->cur_xfr = 0;
	*handle = new_req;

	/* queue the request if someone else owns the lock */
	if((mtype != IIC_NO_MATCH) && (found->client != client)){
		IFLDi(5, "BLOCKED  client[%p] bus[%08lx] lock[%04x:%04x] blocked by client[%p]\n", 
		      client, client->bus->bus_id, addr, mask, found->client);
		new_req->count = 0;
		list_add_tail(&new_req->list, &lm->reqs);
	}
	/* if nobody owns the lock, then grant it to the requestor */
	else
	{
		IDBGd(4, "bus[%08lx] lock[%04x:%04x] count=%d\n", 
			client->bus->bus_id, addr, mask, 1);
		new_req->count = 1;
		list_add_tail(&new_req->list, &lm->locked);
	}

exit:
	IEXIT(rc);
	return rc;
}

/* Engine must be locked before calling this function!
 *
 * If the lock pointed to by lck has been granted, the lock count is decremented
 * and if it reaches zero, the lock is freed up, and the next in line (if any)
 * is granted the lock.  If lck points to a queued up lock request, the
 * request is removed from the request queue and freed up.
 *
 * Here's the algorithm for finding the "next in line":
 *
 * Iterate through the request list
 * 	-if exact or partial match of unlocked range found, search the 
 * 	 lock list for partial matches to the requested lock that have
 * 	 different clients.
 * 		-If not found, grant the lock request.
 * 		-If found, don't grant the lock, continue iterating the
 * 		 request list for other potential candidates.
 */
int iic_unlock(iic_lck_mgr_t *lm, iic_lck_t *lck)
{
	iic_lck_t *req, *temp, *locked, *candidate;
	int mtype = IIC_NO_MATCH;

	IENTER();

	IFLDs(5, "UNLOCK   client[%p] bus[%08lx] lock[%04x:%04x] count=%ld\n", 
		lck->client, lck->client->bus->bus_id, lck->addr, lck->mask, 
		lck->count);
	switch(lck->count)
	{
	/* lock count will decrement to 0, lock is to be unlocked and 
	 * next in line gets the lock
	 */
	case 1: 
		/*remove the lock from the locked list*/
		list_del(&lck->list);

		/* Look for a partial or exact address match in the 
		 * request queue as a candidate for being granted
		 * the lock.
		 */
		candidate = 0;
		list_for_each_entry_safe(req, temp, &lm->reqs, list)
		{
			mtype = iic_get_match_type(req->addr, req->mask,
						   lck->addr, lck->mask);
			if(mtype == IIC_NO_MATCH)
			{
				continue;
			}

			/* found someone waiting for this lock!
			 * Check if the requestors lock range
			 * is free
			 */
			candidate = req;
			list_for_each_entry(locked, &lm->locked, list)
			{
				mtype = iic_get_match_type(
						req->addr, 
						req->mask, 
						locked->addr, 
						locked->mask);
				if(mtype != IIC_NO_MATCH &&
				   req->client != locked->client)
				{
					candidate = 0;
					break;
				}
			}
			if(candidate)
			{
				/* if the candidate survived the test,
				 * grant the lock and discontinue search.
				 * Also, wakeup the client waiting for
				 * this lock to be granted.
				 */
				candidate->count++;
				list_del(&candidate->list);
				list_add_tail(&candidate->list, &lm->locked);
				wake_up_interruptible(&candidate->client->wait);
				IFLDi(4, "UNBLOCK  client[%p] bus[%08lx] lock[%04x:%04x]\n",
				     candidate->client, 
				     candidate->client->bus->bus_id,
				     candidate->addr, candidate->mask);
			}
		}
		kfree(lck);
		break;

	/* lock is unlocked (above case) or was waiting on the request q */
	case 0: 
		/* Remove the old lock from the locked or request
		 * q and free it.
		 */
		list_del(&lck->list);
		kfree(lck);
		break;

	default: /* lock count is >= 2 */
		lck->count--;
	}
	IEXIT(0);
	return 0;
}

/* Find all locks and lock requests associated with the client_id and remove
 * them all regardless of lock count.  All associated transfers should have
 * already been aborted before calling this function.
 */
int iic_unlock_all(iic_lck_mgr_t *lm, iic_client_t *client)
{
	iic_lck_t* iterator, *temp;

	IENTER();

	if(!lm || !client)
	{
		return 0;
	}
	/* first, search the request q */
	list_for_each_entry_safe(iterator, temp, &lm->reqs, list)
	{
		if(iterator->client == client)
		{
			iic_unlock(lm, iterator);
		}
	}

	/* then, search the lock q */
	list_for_each_entry_safe(iterator, temp, &lm->locked, list)
	{
		if(iterator->client == client)
		{
			/* This removes all explicit locks and leaves a single
			 * implicit lock in case there is an active transfer
			 * associated with the lock.
			 */
			iterator->count = 1;
			/* if cur_xfr is set, it means that the xfr has
			 * been started and not completed.  It is bad to
			 * forcefully unlock a lock associated with a
			 * xfr that hasn't completed because the implicit
			 * unlock will fail.
			 */
			if(!iterator->cur_xfr) 
				iic_unlock(lm, iterator); //unlock now
		}
	}
	IEXIT(0);
	return 0;
}

/* Find the lock handle for a given client and address range */
iic_lck_t* iic_find_handle(iic_lck_mgr_t *lm, iic_client_t *client, 
		                short addr, short mask)
{
	iic_lck_t *iterator, *found;
	IENTER();
	addr =  (client->bus->port << IIC_ADDR_MAX_BITS) | (addr >> 1);
        found = 0;
	list_for_each_entry(iterator, &lm->locked, list)
	{
		if((iterator->client == client) &&
				(iic_get_match_type(addr, mask, iterator->addr,
					iterator->mask) == IIC_EXACT_MATCH))
		{
			found = iterator;
			goto exit;
		}
	}
	list_for_each_entry(iterator, &lm->reqs, list)
	{
		if((iterator->client == client) &&
				(iic_get_match_type(addr, mask, iterator->addr,
					iterator->mask) == IIC_EXACT_MATCH))
		{
			found = iterator;
			break;
		}
	}
exit:
	IEXIT((int)found);
	return found;
}
