/*
 *   Copyright (c) International Business Machines Corp., 2006, 2012
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

#ifndef IIC_INT_H
#define IIC_INT_H
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/fs.h>
#include <linux/kobject.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/wait.h>
#include <linux/i2cfsi.h>
#include <linux/semaphore.h>
#include <linux/fsi.h>
#include <asm/atomic.h>

#define FSI_ENGID_I2C		0x7
#define FSI_ENGID_I2C_BB	0x17

#ifdef FSI_I2C_DEBUG
#define IDBGs(num, msg, args...) printk(msg, ## args)
#define IDBGd(num, msg, args...) printk(msg, ## args)
#define IDBGf(num, msg, args...) printk(msg, ## args)
#define IDBGl(num, msg, args...) printk(msg, ## args)
#else
#define IDBGs(num, msg, args...)
#define IDBGd(num, msg, args...)
#define IDBGf(num, msg, args...)
#define IDBGl(num, msg, args...)
#endif

#define IENTER()
#define IEXIT(RC)

/* IFLDx traces will not get compiled out */
#define IFLDe(num, msg, args...)\
	printk("ERR: "msg, ## args)
#define IFLDi(num, msg, args...)\
	printk(msg, ## args)

#ifdef FSI_I2C_DEBUG
#define IFLDs(num, msg, args...) printk(msg, ## args)
#define IFLDd(num, msg, args...) printk(msg, ## args)
#define IFLDf(num, msg, args...) printk(msg, ## args)
#define IFLDl(num, msg, args...) printk(msg, ## args)
#else
#define IFLDs(num, msg, args...)
#define IFLDd(num, msg, args...)
#define IFLDf(num, msg, args...)
#define IFLDl(num, msg, args...)
#endif

#define IIC_RESET_DELAY ( msecs_to_jiffies(100) ) /* 100ms in jiffies */

struct iic_client;
struct iic_bus;
struct iic_eng;
struct iic_xfr;
struct iic_eng_ops;
struct iic_reg_access;
struct iic_lck;
struct iic_lck_mgr;
struct iic_slv;
struct iicslv_client;
struct iicslv_xfr;
struct iic_ffdc {};
struct iic_slv_ffdc;
struct iic_mstr_ffdc;
struct iic_eng_ffdc;
struct dd_ffdc {}; //dummy to compile

typedef struct iic_client iic_client_t;
typedef struct iic_bus iic_bus_t;
typedef struct iic_eng iic_eng_t;
typedef struct iic_eng_ops iic_eng_ops_t;
typedef struct iic_xfr iic_xfr_t;
typedef struct iic_reg_access iic_reg_access_t;
typedef struct iic_lck iic_lck_t;
typedef struct iic_lck_mgr iic_lck_mgr_t;
typedef struct iic_slv iicslv_t;
typedef struct iicslv_client iicslv_client_t;
typedef struct iicslv_xfr iicslv_xfr_t;
typedef struct iic_ffdc iic_ffdc_t;
typedef struct iic_slv_ffdc iic_slv_ffdc_t;
typedef struct iic_mstr_ffdc iic_mstr_ffdc_t;
typedef struct iic_eng_ffdc iic_eng_ffdc_t;
typedef struct iic_trace_entry iic_trace_entry_t;
typedef struct iicslv_zbuf_data iicslv_zbuf_data_t;
typedef struct dd_ffdc dd_ffdc_t;

struct iic_lck
{
	short addr;			//bus & address
	unsigned short mask;            //bus & address mask
	iic_client_t *client;		//requesting/owning client
	iic_xfr_t *cur_xfr;		//transfer currently using this lock
	unsigned long count;            //lock count
	struct list_head list;          //node on locked or reqs list (below)
};

struct iic_lck_mgr
{
	struct list_head locked;        //list of granted locks
	struct list_head reqs;          //list of yet-to-be granted locks
};

struct iic_xfr
{
    iic_client_t* client;
    iic_lck_t* addr_lck;		// lock handle for this xfr
    struct timer_list delay;		// pops when a write delay ends
    struct list_head q_entry;		// entry on engine q
    struct kiocb *iocb;
    char* buf;				// Kernel space buffer
    unsigned long num_pages;
    unsigned long size;			// size of the transfer
    unsigned long bytes_xfrd;		// bytes transfered so far
    int status;				// return status of this transfer
    unsigned long flags;		// miscellaneous transfer flags
    iic_opts_t opts;			// client defined attributes
    struct timer_list timeout;		// for xfr timeouts
    unsigned short retry_count;		// # of attempted retries
    iic_ffdc_t* ffdc;			// chain of ffdc
    pid_t pid;
    unsigned long offset_ffdc;		// 1st 4 bytes of write data
};

/* transfer flags definitions */
enum {
	IIC_XFR_RD,		// 1 = READ, 0 = WRITE
	IIC_XFR_ASYNC,		// transfer is asynchronous
	IIC_XFR_FAST,           // use 400khz for this transfer
	IIC_XFR_CANCELLED,	// transfer has been cancelled
	IIC_XFR_ABORT,		// transfer needs to be aborted
	IIC_XFR_RECOVERY,	// transfer is stuck, recovery in progress
	IIC_XFR_STARTED,	// transfer has been started at least once
	IIC_XFR_DELAYED,	// delay next transfer to xfr address
	IIC_XFR_ENDED,		// all processing  for xfr has completed 
	IIC_XFR_ENG_COMPLETED,	// xfr completed on the engine
	IIC_XFR_DMA,		// DMA will be used for this transfer
	IIC_XFR_DMA_SUBMITTED,	// DMA xfr has been submitted
	IIC_XFR_DMA_COMPLETED,	// DMA xfr has been completed 
	IIC_XFR_OFFSET_PHASE,	// xfr is in offset phase
	IIC_XFR_QUEUED,		// xfr has been queued at least once
	IIC_XFR_SPECIAL_PHASE,  // xfr is in special phase (for pll/crc chips)
	IIC_XFR_RETRY_IN_PROGRESS, // retry is in progress, (don't complete xfr)
};

struct iic_reg_access
{
    int (*bus_readb)(iic_eng_t*, unsigned int, unsigned char*, iic_ffdc_t**);
    int (*bus_readh)(iic_eng_t*, unsigned int, unsigned short*, iic_ffdc_t**);
    int (*bus_readw)(iic_eng_t*, unsigned int, unsigned long*, iic_ffdc_t**);
    int (*bus_writeb)(iic_eng_t*, unsigned int, unsigned char, iic_ffdc_t**);
    int (*bus_writeh)(iic_eng_t*, unsigned int, unsigned short, iic_ffdc_t**);
    int (*bus_writew)(iic_eng_t*, unsigned int, unsigned long, iic_ffdc_t**);
    int (*bus_enable_irq)(iic_eng_t *);
    void (*bus_disable_irq)(iic_eng_t *);
};

struct iic_eng_ops
{
    int (*use_dma)(iic_xfr_t*);
    int (*start)(iic_xfr_t*);
    int (*start_abort)(iic_eng_t*, iic_ffdc_t**);
    int (*finish_abort)(iic_eng_t*, iic_ffdc_t**);
    int (*start_rescue_timeout)(iic_eng_t*, iic_xfr_t*);
    int (*finish_rescue_timeout)(iic_eng_t*, iic_xfr_t*);
    int (*reset_bus)(iic_bus_t*, iic_ffdc_t**);
    int (*reset_eng)(iic_eng_t*, iic_ffdc_t**);
    int (*run_bat)(iic_eng_t*, iic_ffdc_t**);
    int (*init)(iic_eng_t*, iic_ffdc_t**);
    int (*enable_int)(iic_eng_t*, iic_ffdc_t**);
    int (*disable_int)(iic_eng_t*, iic_ffdc_t**);
    int (*int_handler)(int, void*);
    int (*cleanup_eng)(iic_eng_t*);
    int (*wait_for_idle)(iic_eng_t*, int, iic_ffdc_t**);
    int (*slv_on)(iic_eng_t*, iic_ffdc_t**);
    int (*slv_off)(iic_eng_t*, iic_ffdc_t**);
    int (*slv_recv)(iic_eng_t*, char*, int*, iic_ffdc_t**);
    int (*slv_cont)(iic_eng_t*, iic_ffdc_t**);
    int (*slv_set_addr)(iic_eng_t*, unsigned long, iic_ffdc_t**);
    int (*slv_get_addr)(iic_eng_t*, unsigned long*, iic_ffdc_t**);
    void (*display_regs)(iic_eng_t*, iic_ffdc_t**);
    int (*get_bus_state)(iic_bus_t*, unsigned long*, iic_ffdc_t**);
    void (*get_ffdc)(iic_eng_t* eng, iic_ffdc_t* element);
    int (*set_speed)(iic_bus_t* bus, int speed);
    int (*get_speed)(iic_bus_t* bus);
    int (*send)(iic_eng_t*, iic_ffdc_t**);
};

/*engine flags definitions */
enum
{
	IIC_ENG_ABORT,
	IIC_ENG_RESET,
	IIC_ENG_BLOCK,			
	IIC_ENG_THROTTLED, 
	IIC_ENG_NEW_SEND_DATA,
	IIC_ENG_INIT_FAILED,		//engine initialization failed 
	IIC_ENG_CLEANUP_FAILED,		//cleanup of previous xfr failed
	IIC_NO_ACCESS,
	IIC_ENG_RESUMED, 		// off = suspended
	IIC_ENG_REMOVED,
	IIC_ENG_Z7PLUS,
	IIC_ENG_P8_Z8_CENTAUR,
	IIC_ENG_OPB,			// which parent bus we're on: opb/!fsi
};

struct iic_eng
{
    unsigned int id;			//unique id for this engine
    iic_reg_access_t* ra;		//bus specific reg access methods
    unsigned long base;			//ioremapped base address of registers
    int irq;
    struct device* dev;			//ldm structure from bus driver
    iic_eng_ops_t* ops;			//engine specific operations
    spinlock_t lock;
    struct list_head xfrq;		//queue for xfrs waiting to run
    iic_xfr_t* cur_xfr;			//The currently running xfr
    iic_bus_t* busses;			//all busses attached to this engine
    iic_bus_t* cur_bus;
    struct iic_lck_mgr lck_mgr;		//address lock management
    struct work_struct work;		//for freeing the bus
    unsigned long flags;		//misc engine flags
    wait_queue_head_t waitq;		//wait for engine events to occur
    struct semaphore sem;
    iicslv_t* slv;			//pointer to a slave object
    unsigned long type;			//currently not used
    void* private_data;			//currently not used
    unsigned long bus_speed;		//parent bus speed in MHz
    unsigned long fifo_size;		//size of the engine fifo
    iic_ffdc_t* ffdc;			//ffdc data not associated with
    					//a process or bus(i.e., init errors)
    struct kobject kobj;		//for reference counting
    unsigned long trace_sz;		//number of trace entries
    atomic_t xfr_num;			//index to current trace entry
    uint64_t enabled;
    int idx;				// ida number
};

struct iic_bus
{
    unsigned char port;			//the port number of this bus
    unsigned long bus_id;		//Unique ID for this bus
    int idx;				// ida number
    struct cdev cdev;
    struct device* class_dev;
    dev_t devnum;
    iic_eng_t* eng;			//the engine this bus is connected to
    iic_bus_t* next;			//the next bus connected to this engine
    long i2c_hz;			//the i2c clock speed for this bus.
};

struct iic_client
{
    iic_bus_t* bus;
    wait_queue_head_t wait;		//wait for locks and sync xfrs
    iic_opts_t opts;
    struct semaphore sem;
    pid_t tgid;
    char proc_name[16];
    unsigned long flags;
#define IIC_CLIENT_SOURCE_USER	(1 << 0)
#define IIC_CLIENT_SOURCE_KERN  (1 << 1)
#define IIC_CLIENT_EOD          (1 << 2)		/* End Of Data */
};

//--------------------------------------------------------------------
// Specific header for repeated I/O
//--------------------------------------------------------------------
/*
 * I2C Message - used for i2c transaction
 */
struct i2c_msg {
        unsigned short addr;     /* slave address                        */
        unsigned short flags;
#define I2C_M_TEN       0x10    /* we have a ten bit chip address       */
#define I2C_M_RD        0x01
#define I2C_M_NOSTART   0x4000
#define I2C_M_REV_DIR_ADDR      0x2000
#define I2C_M_IGNORE_NAK        0x1000
#define I2C_M_NO_RD_ACK         0x0800
        unsigned short len;              /* msg length                           */
        unsigned char *buf;              /* pointer to msg data                  */
};
/* This is the structure as used in the I2C_RDWR ioctl call */
struct i2c_rdwr_ioctl_data {
        struct i2c_msg  *msgs;    /* pointers to i2c_msgs */
        unsigned int nmsgs;                    /* number of i2c_msgs */
};

int iic_register_eng_ops(iic_eng_ops_t* new_ops, unsigned long type);
int iic_unregister_eng_ops(unsigned long type);
iic_eng_ops_t* iic_get_eng_ops(unsigned long type);
int iic_eng_ops_is_vaild(struct iic_eng_ops *ops);
iic_bus_t*  iic_create_bus(struct class* classp, iic_eng_t* eng,
		                           dev_t devnum, char* name, 
					   unsigned char port,
					   int bus_id);
void iic_delete_bus(struct class* classp, iic_bus_t* bus);
void iic_register_bus(iic_bus_t * bus, unsigned long type);
void iic_unregister_bus(iic_bus_t * bus, unsigned long type);
void iic_init_eng(iic_eng_t* eng);
int iic_reset(iic_bus_t* bus, int timeout, iic_ffdc_t** ffdc);  /* timeout is in milliseconds! */

/* Engine specific code must call this function when a transfer completes
 * and the engine is ready to start a new transfer
 */
void iic_xfr_complete(iic_xfr_t* xfr);

void iic_delay_xfr(iic_xfr_t* xfr, unsigned long delay);
void iic_abort_xfr(iic_xfr_t* xfr);
int iic_abort_all(iic_eng_t* eng, iic_client_t* client, int status);
int iic_start_next_xfr(iic_eng_t* eng);

/* Name says it all!
 */
void iic_lck_mgr_init(iic_lck_mgr_t* mgr);

/* Block waiting for an address lock to be granted until the timeout is
 * reached.
 *
 * Return Values:
 * SUCCESS: The requester owns the lock and the lock count is incremented.
 * EINTR:   The request was interrupted.
 * ETIME:   The request timed out, in jiffies.
 */
int iic_wait_lock(iic_lck_mgr_t *lm, short addr, short mask,
		iic_client_t *client, unsigned long to);

/* Try locking an address.  Never block.  If the address is already locked
 * by another client, queue this request.  After a request is queued,
 * handle->count can be checked to determine when the lock has been granted.
 * If handle->count > 0, the lock has been granted.
 *
 * Return Values:
 * SUCCESS: The requester owns the lock and the lock count is incremented.
 * QUEUED:  The requester doesn't own the lock, but a lock request has been
 *          queued.
 */
#define IIC_REQ_QUEUED 1
int iic_req_lock(iic_lck_mgr_t *lm, short addr, unsigned short mask,
		iic_client_t *client, iic_lck_t **handle);

iic_lck_t* iic_find_handle(iic_lck_mgr_t *lm, iic_client_t *client, 
		           short addr, short mask);

/* Unlock the specified address lock.  If others are waiting for a lock, grant
 * the next person in line.
 */
int iic_unlock(iic_lck_mgr_t *lm, iic_lck_t *lck);

/* Unlock all locks associated with the specified client.
 */
int iic_unlock_all(iic_lck_mgr_t *lm, iic_client_t *client);


#define iic_addr_is_locked(lck, xfr)\
        (lck->count > 0 && (lck->cur_xfr == 0 || lck->cur_xfr == xfr))
#define cdev_to_iic_bus(cdevp) container_of(cdevp, iic_bus_t, cdev)

/*
 * Exported for drivers needing 'sideways' kernel call access to IIC
 * Currently only supports local OPB type IIC masters
 */
#define IIC_RESET_BUS   0
#define IIC_RESET_ENG   1

int iic_sideways_open(iic_client_t ** client, iic_bus_t * bus, int bus_num);
int iic_sideways_read(iic_client_t * client, void * buf, size_t count,
		    loff_t * offset, dd_ffdc_t ** ffdc);
int iic_sideways_write(iic_client_t * client, void * buf, size_t count,
		     loff_t * offset, dd_ffdc_t ** ffdc);
int iic_sideways_release(iic_client_t * client);

#define IIC_LOCK_ALL	0x7FF
int iic_sideways_lock_bus(iic_client_t * client, unsigned short mask,
			  unsigned short addr, unsigned long timeout);
int iic_sideways_unlock_bus(iic_client_t * client, unsigned short addr,
			    unsigned short mask);
#endif
