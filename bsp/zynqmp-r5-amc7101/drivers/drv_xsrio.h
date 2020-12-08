#ifndef __DRV_XSRIO_H__
#define __DRV_XSRIO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <rthw.h>
#include <drivers/rio_drv.h>
#include <drivers/rio.h>
#include <ipc/workqueue.h>
#include <ipc/ringbuffer.h>
#include "zynqmp-r5.h"

#define __iomem

#define DBELL_SID(buf)		(((rt_uint8_t)buf[2] << 8) | (rt_uint8_t)buf[3])      /* waiting */
#define DBELL_TID(buf)		(((rt_uint8_t)buf[4] << 8) | (rt_uint8_t)buf[5])
#define DBELL_INF(buf)		(((rt_uint8_t)buf[0] << 8) | (rt_uint8_t)buf[1])

/*
 * Inbound Doorbells
 */

#define XSRIO_IDB_ENTRY_SIZE	  64    /* waiting */

/*
 * Messaging definitions
 */
#define XSRIO_IMSG_CHNUM		4
#define XSRIO_IMSGD_MIN_RING_SIZE	2
#define XSRIO_IMSGD_MAX_RING_SIZE	2048

#define XSRIO_OMSG_CHNUM		4 
#define XSRIO_OMSGD_MIN_RING_SIZE	2
#define XSRIO_OMSGD_MAX_RING_SIZE	2048



struct xsrio_imsg_ring {
	rt_uint32_t		size;
	/* VA/PA of data buffers for incoming messages */
	void		*buf_base;
	rio_dma_addr_t	buf_phys;

	 /* Inbound Queue buffer pointers */
	void		*imq_base[XSRIO_IMSGD_MAX_RING_SIZE];

	rt_uint32_t		rx_slot;
	void		*dev_id;
#ifdef RT_USING_SMP
	struct rt_spinlock	lock;
#endif
};

struct xsrio_omsg_ring {
	rt_uint32_t		size;
	/* VA/PA of OB Msg descriptors */
	void		*omd_base;
	rio_dma_addr_t	omd_phys;
	/* VA/PA of OB Msg data buffers */
	void		*omq_base[XSRIO_OMSGD_MAX_RING_SIZE];
	rio_dma_addr_t	omq_phys[XSRIO_OMSGD_MAX_RING_SIZE];

	rt_uint32_t		tx_slot;
	void		*dev_id;
#ifdef RT_USING_SMP
	struct rt_spinlock	lock;
#endif
};

struct xsrio_mport {
	struct rio_mport mport;
	void __iomem *regs;
	int irq;

	/* Doorbells */
	void __iomem	*odb_base;
	void		*idb_base;
	rio_dma_addr_t	idb_dma;
	struct rt_work idb_work;
	rt_uint32_t		db_discard_count;

	/* Inbound Port-Write */
	struct rt_work pw_work;
	struct rt_ringbuffer	*pw_fifo;
	rt_uint32_t		pw_discard_count;

	/* Inbound Messaging */
	int		imsg_init[XSRIO_IMSG_CHNUM];
	struct xsrio_imsg_ring imsg_ring[XSRIO_IMSG_CHNUM];

	/* Outbound Messaging */
	int		omsg_init[XSRIO_OMSG_CHNUM];
	struct xsrio_omsg_ring	omsg_ring[XSRIO_OMSG_CHNUM];
};

static inline rt_uint32_t xsrio_read32(rt_uint32_t addr)
{
	return __REG32(addr);
}

static inline void xsrio_write32(rt_uint32_t addr, rt_uint32_t value)
{
	__REG32(addr) = value;
}


#ifdef __cplusplus
}
#endif

#endif		/* __DRV_XSRIO_H__ */
