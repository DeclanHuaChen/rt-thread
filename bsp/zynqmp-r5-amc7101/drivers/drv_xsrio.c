/*
 * Xilinx Serial RapidIO driver support
 *
 * Copyright 2020 Harbin Institute of Technology, Inc.
 * Wang Huachen <oisun@qq.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include "drv_xsrio.h"


#define XSRIO_REG_START        0x40000000
#define XSRIO_REG_SIZE         0x40000000

#define DRIVER_NAME	"xmport"

static int xsrio_lcread(struct rio_mport *mport, int index, rt_uint32_t offset, int len,
        rt_uint32_t *data)
{
    struct xsrio_mport *priv = (struct xsrio_mport *)mport->priv;

    if (len != 4)
        return -RT_EINVAL;

    *data = xsrio_read32(priv->regs + offset);

    return RT_EOK;
}

static int xsrio_lcwrite(struct rio_mport *mport, int index, rt_uint32_t offset, int len,
        rt_uint32_t data)
{
    struct xsrio_mport *priv = (struct xsrio_mport *)mport->priv;

    if (len != 4)
        return -RT_EINVAL;

    xsrio_write32(priv->regs + offset, data);

    return RT_EOK;
}

static int xsrio_cread(struct rio_mport *mport, int index, rt_uint16_t destid,
            rt_uint8_t hopcount, rt_uint32_t offset, int len, rt_uint32_t *data)
{
    return -RT_ENOSYS;
}

static int xsrio_cwrite(struct rio_mport *mport, int index, rt_uint16_t destid,
        rt_uint8_t hopcount, rt_uint32_t offset, int len, rt_uint32_t data)
{
    return -RT_ENOSYS;
}

static int xsrio_dsend(struct rio_mport *mport, int index, rt_uint16_t destid, rt_uint16_t data)
{
    return -RT_ENOSYS;
}

static int xsrio_dbell_handler(struct xsrio_mport *priv)
{

    /* waiting */

    rt_work_submit(&priv->idb_work, 0);

    return 0;
}

/* waiting */
static void xsrio_db_dpc(struct rt_work *work, void *work_data)
{
    struct xsrio_mport *priv = (struct xsrio_mport *)work_data;
    struct rio_mport *mport;
    struct rio_dbell *dbell;
    int found = 0;
    union {
        rt_uint64_t msg;
        rt_uint8_t  bytes[8];
    } idb;
    rt_memset(&idb, 0, sizeof(idb));
    /*
     * Process queued inbound doorbells
     */
    mport = &priv->mport;


    /* Process one doorbell */
    rt_list_for_each_entry(dbell, &mport->dbells, node) {
        if ((dbell->res->start <= DBELL_INF(idb.bytes)) &&
            (dbell->res->end >= DBELL_INF(idb.bytes))) {
            found = 1;
            break;
        }
    }

    if (found) {
        dbell->dinb(mport, dbell->dev_id, DBELL_SID(idb.bytes),
                DBELL_TID(idb.bytes), DBELL_INF(idb.bytes));
    } else {
        rt_kprintf("spurious IDB sid %2.2x tid %2.2x info %4.4x",
                DBELL_SID(idb.bytes), DBELL_TID(idb.bytes),
                DBELL_INF(idb.bytes));
    }

}

static int xsrio_doorbell_init(struct xsrio_mport *priv)
{
    priv->db_discard_count = 0;
    rt_work_init(&priv->idb_work, xsrio_db_dpc, priv);

    /* Allocate buffer for inbound doorbells queue */
    // priv->idb_base = dma_zalloc_coherent(&priv->pdev->dev,
    // 			512 * XSRIO_IDB_ENTRY_SIZE,
    // 			&priv->idb_dma, GFP_KERNEL);
    if (!priv->idb_base)
        return -ENOMEM;

    rt_kprintf("Allocated IDB buffer @ %p (phys = %pad)",
          priv->idb_base, &priv->idb_dma);

    /* waiting */

    return 0;
}

static void xsrio_doorbell_free(struct xsrio_mport *priv)
{
    if (priv->idb_base == NULL)
        return;

    /* Free buffer allocated for inbound doorbell queue */
    // dma_free_coherent(&priv->pdev->dev, 512 * XSRIO_IDB_ENTRY_SIZE,
    // 		  priv->idb_base, priv->idb_dma);
    priv->idb_base = NULL;
}

static int xsrio_pw_enable(struct rio_mport *mport, int enable)
{
    return -RT_ENOSYS;
}

static int xsrio_pw_handler(struct xsrio_mport *priv)
{
    rt_uint32_t pw_buf[RIO_PW_MSG_SIZE/sizeof(rt_uint32_t)];

    /* waiting */

    /* Queue PW message (if there is room in FIFO),
        * otherwise discard it.
        */
    if ((rt_ringbuffer_get_size(priv->pw_fifo) - rt_ringbuffer_data_len(priv->pw_fifo)) >= 
                RIO_PW_MSG_SIZE)
        rt_ringbuffer_put(priv->pw_fifo, (rt_uint8_t*)pw_buf, RIO_PW_MSG_SIZE);
    else
        priv->pw_discard_count++;

    rt_work_submit(&priv->pw_work, 0);

    return 0;
}

static void xsrio_pw_dpc(struct rt_work *work, void *work_data)
{
    struct xsrio_mport *priv = (struct xsrio_mport *)work_data;
    union rio_pw_msg pwmsg;

    /*
     * Process port-write messages
     */
    while (rt_ringbuffer_data_len(priv->pw_fifo) >= RIO_PW_MSG_SIZE) {
        rt_ringbuffer_get(priv->pw_fifo, (unsigned char *)&pwmsg, RIO_PW_MSG_SIZE);
        /* Pass the port-write message to RIO core for processing */
        rio_inb_pwrite_handler(&priv->mport, &pwmsg);
    }
}

static int xsrio_port_write_init(struct xsrio_mport *priv)
{
    priv->pw_discard_count = 0;
    rt_work_init(&priv->pw_work, xsrio_pw_dpc, priv);
    priv->pw_fifo = rt_ringbuffer_create(RIO_PW_MSG_SIZE * 32);
    if (priv->pw_fifo == RT_NULL) {
        rt_kprintf("PW FIFO allocation failed");
        return -ENOMEM;
    }

    /* waiting */

    return 0;
}

static void xsrio_port_write_free(struct xsrio_mport *priv)
{
    rt_ringbuffer_destroy(priv->pw_fifo);
}

static int xsrio_open_outb_mbox(struct rio_mport *mport, void *dev_id,
                int mbox, int entries)
{
    return -RT_ENOSYS;
}

static void xsrio_close_outb_mbox(struct rio_mport *mport, int mbox)
{
}

static int xsrio_open_inb_mbox(struct rio_mport *mport, void *dev_id,
                int mbox, int entries)
{
    return -RT_ENOSYS;
}

static void xsrio_close_inb_mbox(struct rio_mport *mport, int mbox)
{
}

static int xsrio_add_outb_message(struct rio_mport *mport, struct rio_dev *rdev,
                int mbox, void *buffer, rt_size_t len)
{
    return -RT_ENOSYS;
}

static int xsrio_add_inb_buffer(struct rio_mport *mport, int mbox, void *buf)
{
    return -RT_ENOSYS;
}

static void *xsrio_get_inb_message(struct rio_mport *mport, int mbox)
{
    return RT_NULL;
}

static int xsrio_messages_init(struct xsrio_mport *priv)
{
    /* waiting */
    return -RT_ENOSYS;
}

static int xsrio_map_inb(struct rio_mport *mport, rio_dma_addr_t lstart,
        rt_uint64_t rstart, rt_uint64_t size, rt_uint32_t flags)
{
    return -RT_ENOSYS;
}

static void xsrio_unmap_inb(struct rio_mport *mport, rio_dma_addr_t lstart)
{
}

static int xsrio_query_mport(struct rio_mport *mport,
            struct rio_mport_attr *attr)
{
    return -RT_ENOSYS;
}

static int xsrio_map_outb(struct rio_mport *mport, rt_uint16_t destid, rt_uint64_t rstart,
        rt_uint32_t size, rt_uint32_t flags, rio_dma_addr_t *laddr)
{
    return -RT_ENOSYS;
}

static void xsrio_unmap_outb(struct rio_mport *mport, rt_uint16_t destid, rt_uint64_t rstart)
{
}


static struct rio_ops xsrio_ops = {
    .lcread			= xsrio_lcread,
    .lcwrite		= xsrio_lcwrite,
    .cread			= xsrio_cread,
    .cwrite			= xsrio_cwrite,
    .dsend			= xsrio_dsend,
    .open_inb_mbox		= xsrio_open_inb_mbox,
    .close_inb_mbox		= xsrio_close_inb_mbox,
    .open_outb_mbox		= xsrio_open_outb_mbox,
    .close_outb_mbox	= xsrio_close_outb_mbox,
    .add_outb_message	= xsrio_add_outb_message,
    .add_inb_buffer		= xsrio_add_inb_buffer,
    .get_inb_message	= xsrio_get_inb_message,
    .map_inb		= xsrio_map_inb,
    .unmap_inb		= xsrio_unmap_inb,
    .pwenable		= xsrio_pw_enable,
    .query_mport		= xsrio_query_mport,
    .map_outb		= xsrio_map_outb,
    .unmap_outb		= xsrio_unmap_outb,
};

static int xsrio_request_irq(struct xsrio_mport *priv)
{
    int err = 0;

    /* waiting */

    return err;
}

static void xsrio_free_irq(struct xsrio_mport *priv)
{
    /* waiting */
}

static void xsrio_irqhandler(int vector, void *param)
{
    rt_interrupt_enter();
    /* waiting */
    rt_interrupt_leave();
}

static int xsrio_setup_mport(struct xsrio_mport *priv)
{
    int err = 0;
    struct rio_mport *mport = &priv->mport;

    err = rio_mport_initialize(mport);
    if (err)
        return err;
    
    mport->ops = &xsrio_ops;
    mport->host_deviceid = 0;
    mport->index = 0;
    mport->sys_size = 0; /* small system */
    mport->priv = (void *)priv;
    mport->phys_efptr = 0x100;
    mport->phys_rmap = 1;
    rt_list_init(&mport->dbells);

    rio_init_dbell_res(&mport->riores[RIO_DOORBELL_RESOURCE], 0, 0xffff);
    rio_init_mbox_res(&mport->riores[RIO_INB_MBOX_RESOURCE], 0, 3);
    rio_init_mbox_res(&mport->riores[RIO_OUTB_MBOX_RESOURCE], 0, 3);
    rt_snprintf(mport->name, RIO_MAX_MPORT_NAME, "%s", DRIVER_NAME);

    err = rio_register_mport(mport);
    if (err) 
        return err;

    err = xsrio_request_irq(priv);
    if (err) 
        return err;

    return 0;

err_exit:   /* waiting */
    xsrio_free_irq(priv);
    return err;
}

static int xsrio_init(void)
{
    void __iomem *addr = (void *)XSRIO_REG_START;
    struct xsrio_mport *priv;
    int err = 0;

    priv = rio_zalloc(sizeof(struct xsrio_mport));
    if (!priv) {
        err = -RT_ENOMEM;
        goto err_exit;
    }
    priv->regs = addr;

    err = xsrio_doorbell_init(priv);
    if (err)
        goto err_clean;

    xsrio_port_write_init(priv);

    err = xsrio_messages_init(priv);
    if (err)
        goto err_free_consistent;

    err = xsrio_setup_mport(priv);
    if (err)
        goto err_free_consistent;

    return 0;

err_free_consistent:
    xsrio_port_write_free(priv);
    xsrio_doorbell_free(priv);
err_clean:
    rt_free(priv);
err_exit:
    rt_kprintf("Err init xsrio mport\n");
    return err;
}
INIT_DEVICE_EXPORT(xsrio_init);