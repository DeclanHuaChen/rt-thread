/*
 * RapidIO driver services
 * 
 * Copyright 2005 MontaVista Software, Inc.
 * Matt Porter <mporter@kernel.crashing.org>
 * 
 * Copyright 2020 Automatic Test and Control Institute, Harbin Institute of Technology
 * Wang Huachen <oisun@qq.com>
 * - Ported code from Linux kernel
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 * 
 * Change Logs:
 * Date           Author            Notes
 */

#ifndef __RIO_DRV_H__
#define __RIO_DRV_H__

#include <drivers/rio.h>

#ifdef __cplusplus
extern "C"
{
#endif

extern int __rio_local_read_config_32(struct rio_mport *port, rt_uint32_t offset,
                      rt_uint32_t * data);
extern int __rio_local_write_config_32(struct rio_mport *port, rt_uint32_t offset,
                       rt_uint32_t data);
extern int __rio_local_read_config_16(struct rio_mport *port, rt_uint32_t offset,
                      rt_uint16_t * data);
extern int __rio_local_write_config_16(struct rio_mport *port, rt_uint32_t offset,
                       rt_uint16_t data);
extern int __rio_local_read_config_8(struct rio_mport *port, rt_uint32_t offset,
                     rt_uint8_t * data);
extern int __rio_local_write_config_8(struct rio_mport *port, rt_uint32_t offset,
                      rt_uint8_t data);

extern int rio_mport_read_config_32(struct rio_mport *port, rt_uint16_t destid,
                    rt_uint8_t hopcount, rt_uint32_t offset, rt_uint32_t * data);
extern int rio_mport_write_config_32(struct rio_mport *port, rt_uint16_t destid,
                     rt_uint8_t hopcount, rt_uint32_t offset, rt_uint32_t data);
extern int rio_mport_read_config_16(struct rio_mport *port, rt_uint16_t destid,
                    rt_uint8_t hopcount, rt_uint32_t offset, rt_uint16_t * data);
extern int rio_mport_write_config_16(struct rio_mport *port, rt_uint16_t destid,
                     rt_uint8_t hopcount, rt_uint32_t offset, rt_uint16_t data);
extern int rio_mport_read_config_8(struct rio_mport *port, rt_uint16_t destid,
                   rt_uint8_t hopcount, rt_uint32_t offset, rt_uint8_t * data);
extern int rio_mport_write_config_8(struct rio_mport *port, rt_uint16_t destid,
                    rt_uint8_t hopcount, rt_uint32_t offset, rt_uint8_t data);

/**
 * rio_local_read_config_32 - Read 32 bits from local configuration space
 * @port: Master port
 * @offset: Offset into local configuration space
 * @data: Pointer to read data into
 *
 * Reads 32 bits of data from the specified offset within the local
 * device's configuration space.
 */
static inline int rio_local_read_config_32(struct rio_mport *port, rt_uint32_t offset,
                       rt_uint32_t * data)
{
    return __rio_local_read_config_32(port, offset, data);
}

/**
 * rio_local_write_config_32 - Write 32 bits to local configuration space
 * @port: Master port
 * @offset: Offset into local configuration space
 * @data: Data to be written
 *
 * Writes 32 bits of data to the specified offset within the local
 * device's configuration space.
 */
static inline int rio_local_write_config_32(struct rio_mport *port, rt_uint32_t offset,
                        rt_uint32_t data)
{
    return __rio_local_write_config_32(port, offset, data);
}

/**
 * rio_local_read_config_16 - Read 16 bits from local configuration space
 * @port: Master port
 * @offset: Offset into local configuration space
 * @data: Pointer to read data into
 *
 * Reads 16 bits of data from the specified offset within the local
 * device's configuration space.
 */
static inline int rio_local_read_config_16(struct rio_mport *port, rt_uint32_t offset,
                       rt_uint16_t * data)
{
    return __rio_local_read_config_16(port, offset, data);
}

/**
 * rio_local_write_config_16 - Write 16 bits to local configuration space
 * @port: Master port
 * @offset: Offset into local configuration space
 * @data: Data to be written
 *
 * Writes 16 bits of data to the specified offset within the local
 * device's configuration space.
 */

static inline int rio_local_write_config_16(struct rio_mport *port, rt_uint32_t offset,
                        rt_uint16_t data)
{
    return __rio_local_write_config_16(port, offset, data);
}

/**
 * rio_local_read_config_8 - Read 8 bits from local configuration space
 * @port: Master port
 * @offset: Offset into local configuration space
 * @data: Pointer to read data into
 *
 * Reads 8 bits of data from the specified offset within the local
 * device's configuration space.
 */
static inline int rio_local_read_config_8(struct rio_mport *port, rt_uint32_t offset,
                      rt_uint8_t * data)
{
    return __rio_local_read_config_8(port, offset, data);
}

/**
 * rio_local_write_config_8 - Write 8 bits to local configuration space
 * @port: Master port
 * @offset: Offset into local configuration space
 * @data: Data to be written
 *
 * Writes 8 bits of data to the specified offset within the local
 * device's configuration space.
 */
static inline int rio_local_write_config_8(struct rio_mport *port, rt_uint32_t offset,
                       rt_uint8_t data)
{
    return __rio_local_write_config_8(port, offset, data);
}

/**
 * rio_read_config_32 - Read 32 bits from configuration space
 * @rdev: RIO device
 * @offset: Offset into device configuration space
 * @data: Pointer to read data into
 *
 * Reads 32 bits of data from the specified offset within the
 * RIO device's configuration space.
 */
static inline int rio_read_config_32(struct rio_dev *rdev, rt_uint32_t offset,
                     rt_uint32_t * data)
{
    return rio_mport_read_config_32(rdev->net->hport, rdev->destid,
                    rdev->hopcount, offset, data);
};

/**
 * rio_write_config_32 - Write 32 bits to configuration space
 * @rdev: RIO device
 * @offset: Offset into device configuration space
 * @data: Data to be written
 *
 * Writes 32 bits of data to the specified offset within the
 * RIO device's configuration space.
 */
static inline int rio_write_config_32(struct rio_dev *rdev, rt_uint32_t offset,
                      rt_uint32_t data)
{
    return rio_mport_write_config_32(rdev->net->hport, rdev->destid,
                     rdev->hopcount, offset, data);
};

/**
 * rio_read_config_16 - Read 16 bits from configuration space
 * @rdev: RIO device
 * @offset: Offset into device configuration space
 * @data: Pointer to read data into
 *
 * Reads 16 bits of data from the specified offset within the
 * RIO device's configuration space.
 */
static inline int rio_read_config_16(struct rio_dev *rdev, rt_uint32_t offset,
                     rt_uint16_t * data)
{
    return rio_mport_read_config_16(rdev->net->hport, rdev->destid,
                    rdev->hopcount, offset, data);
};

/**
 * rio_write_config_16 - Write 16 bits to configuration space
 * @rdev: RIO device
 * @offset: Offset into device configuration space
 * @data: Data to be written
 *
 * Writes 16 bits of data to the specified offset within the
 * RIO device's configuration space.
 */
static inline int rio_write_config_16(struct rio_dev *rdev, rt_uint32_t offset,
                      rt_uint16_t data)
{
    return rio_mport_write_config_16(rdev->net->hport, rdev->destid,
                     rdev->hopcount, offset, data);
};

/**
 * rio_read_config_8 - Read 8 bits from configuration space
 * @rdev: RIO device
 * @offset: Offset into device configuration space
 * @data: Pointer to read data into
 *
 * Reads 8 bits of data from the specified offset within the
 * RIO device's configuration space.
 */
static inline int rio_read_config_8(struct rio_dev *rdev, rt_uint32_t offset, rt_uint8_t * data)
{
    return rio_mport_read_config_8(rdev->net->hport, rdev->destid,
                       rdev->hopcount, offset, data);
};

/**
 * rio_write_config_8 - Write 8 bits to configuration space
 * @rdev: RIO device
 * @offset: Offset into device configuration space
 * @data: Data to be written
 *
 * Writes 8 bits of data to the specified offset within the
 * RIO device's configuration space.
 */
static inline int rio_write_config_8(struct rio_dev *rdev, rt_uint32_t offset, rt_uint8_t data)
{
    return rio_mport_write_config_8(rdev->net->hport, rdev->destid,
                    rdev->hopcount, offset, data);
};

extern int rio_mport_send_doorbell(struct rio_mport *mport, rt_uint16_t destid,
                   rt_uint16_t data);

/**
 * rio_send_doorbell - Send a doorbell message to a device
 * @rdev: RIO device
 * @data: Doorbell message data
 *
 * Send a doorbell message to a RIO device. The doorbell message
 * has a 16-bit info field provided by the @data argument.
 */
static inline int rio_send_doorbell(struct rio_dev *rdev, rt_uint16_t data)
{
    return rio_mport_send_doorbell(rdev->net->hport, rdev->destid, data);
};

/**
 * rio_init_mbox_res - Initialize a RIO mailbox rio_resource
 * @res: rio_resource struct
 * @start: start of mailbox range
 * @end: end of mailbox range
 *
 * This function is used to initialize the fields of a rio_resource
 * for use as a mailbox rio_resource.  It initializes a range of
 * mailboxes using the start and end arguments.
 */
static inline void rio_init_mbox_res(struct rio_resource *res, int start, int end)
{
    rt_memset(res, 0, sizeof(struct rio_resource));
    rt_list_init(&res->node);
    res->start = start;
    res->end = end;
    res->flags = RIO_RESOURCE_MAILBOX;
}

/**
 * rio_init_dbell_res - Initialize a RIO doorbell rio_resource
 * @res: rio_resource struct
 * @start: start of doorbell range
 * @end: end of doorbell range
 *
 * This function is used to initialize the fields of a rio_resource
 * for use as a doorbell rio_resource.  It initializes a range of
 * doorbell messages using the start and end arguments.
 */
static inline void rio_init_dbell_res(struct rio_resource *res, rt_uint16_t start, rt_uint16_t end)
{
    rt_memset(res, 0, sizeof(struct rio_resource));
    rt_list_init(&res->node);
    res->start = start;
    res->end = end;
    res->flags = RIO_RESOURCE_DOORBELL;
}

struct rio_device_id {
	rt_uint16_t did, vid;
	rt_uint16_t asm_did, asm_vid;
};

/**
 * RIO_DEVICE - macro used to describe a specific RIO device
 * @dev: the 16 bit RIO device ID
 * @ven: the 16 bit RIO vendor ID
 *
 * This macro is used to create a struct rio_deviceice_id that matches a
 * specific device.  The assembly vendor and assembly device fields
 * will be set to %RIO_ANY_ID.
 */
#define RIO_DEVICE(dev,ven) \
    .did = (dev), .vid = (ven), \
    .asm_did = RIO_ANY_ID, .asm_vid = RIO_ANY_ID

/* Mailbox management */
extern int rio_request_outb_mbox(struct rio_mport *, void *, int, int,
                 void (*)(struct rio_mport *, void *,int, int));
extern int rio_release_outb_mbox(struct rio_mport *, int);

/**
 * rio_add_outb_message - Add RIO message to an outbound mailbox queue
 * @mport: RIO master port containing the outbound queue
 * @rdev: RIO device the message is be sent to
 * @mbox: The outbound mailbox queue
 * @buffer: Pointer to the message buffer
 * @len: Length of the message buffer
 *
 * Adds a RIO message buffer to an outbound mailbox queue for
 * transmission. Returns 0 on success.
 */
static inline int rio_add_outb_message(struct rio_mport *mport,
                       struct rio_dev *rdev, int mbox,
                       void *buffer, size_t len)
{
    return mport->ops->add_outb_message(mport, rdev, mbox,
                           buffer, len);
}

extern int rio_request_inb_mbox(struct rio_mport *, void *, int, int,
                void (*)(struct rio_mport *, void *, int, int));
extern int rio_release_inb_mbox(struct rio_mport *, int);

/**
 * rio_add_inb_buffer - Add buffer to an inbound mailbox queue
 * @mport: Master port containing the inbound mailbox
 * @mbox: The inbound mailbox number
 * @buffer: Pointer to the message buffer
 *
 * Adds a buffer to an inbound mailbox queue for reception. Returns
 * 0 on success.
 */
static inline int rio_add_inb_buffer(struct rio_mport *mport, int mbox,
                     void *buffer)
{
    return mport->ops->add_inb_buffer(mport, mbox, buffer);
}

/**
 * rio_get_inb_message - Get A RIO message from an inbound mailbox queue
 * @mport: Master port containing the inbound mailbox
 * @mbox: The inbound mailbox number
 *
 * Get a RIO message from an inbound mailbox queue. Returns 0 on success.
 */
static inline void *rio_get_inb_message(struct rio_mport *mport, int mbox)
{
    return mport->ops->get_inb_message(mport, mbox);
}

/* Doorbell management */
extern int rio_request_inb_dbell(struct rio_mport *, void *, rt_uint16_t, rt_uint16_t,
                 void (*)(struct rio_mport *, void *, rt_uint16_t, rt_uint16_t, rt_uint16_t));
extern int rio_release_inb_dbell(struct rio_mport *, rt_uint16_t, rt_uint16_t);
extern struct rio_resource *rio_request_outb_dbell(struct rio_dev *, rt_uint16_t, rt_uint16_t);
extern int rio_release_outb_dbell(struct rio_dev *, struct rio_resource *);

/* Memory mapping functions */
extern int rio_map_inb_region(struct rio_mport *mport, rio_dma_addr_t local,
            rt_uint64_t rbase, rt_uint32_t size, rt_uint32_t rflags);
extern void rio_unmap_inb_region(struct rio_mport *mport, rio_dma_addr_t lstart);
extern int rio_map_outb_region(struct rio_mport *mport, rt_uint16_t destid, rt_uint64_t rbase,
            rt_uint32_t size, rt_uint32_t rflags, rio_dma_addr_t *local);
extern void rio_unmap_outb_region(struct rio_mport *mport,
                  rt_uint16_t destid, rt_uint64_t rstart);

/* Port-Write management */
extern int rio_request_inb_pwrite(struct rio_dev *,
            int (*)(struct rio_dev *, union rio_pw_msg*, int));
extern int rio_release_inb_pwrite(struct rio_dev *);
extern int rio_add_mport_pw_handler(struct rio_mport *mport, void *dev_id,
            int (*pwcback)(struct rio_mport *mport, void *dev_id,
            union rio_pw_msg *msg, int step));
extern int rio_del_mport_pw_handler(struct rio_mport *mport, void *dev_id,
            int (*pwcback)(struct rio_mport *mport, void *dev_id,
            union rio_pw_msg *msg, int step));
extern int rio_inb_pwrite_handler(struct rio_mport *mport,
                  union rio_pw_msg *pw_msg);
extern void rio_pw_enable(struct rio_mport *mport, int enable);


/**
 * rio_name - Get the unique RIO device identifier
 * @rdev: RIO device
 *
 * Get the unique RIO device identifier. Returns the device
 * identifier string.
 */
static inline const char *rio_name(struct rio_dev *rdev)
{
    return rdev->name;
}

struct rio_probe_callback
{
    rt_list_t node;
    void (*callback)(struct rio_dev *dev, const struct rio_device_id *id);
    const struct rio_device_id *id;
};

/* Misc driver helpers */
extern void rio_dev_probe_add(struct rio_probe_callback* rdev_cb);
extern rt_uint16_t rio_local_get_device_id(struct rio_mport *port);
extern void rio_local_set_device_id(struct rio_mport *port, rt_uint16_t did);
extern struct rio_dev *rio_get_device(rt_uint16_t vid, rt_uint16_t did, struct rio_dev *from);
extern struct rio_dev *rio_get_asm(rt_uint16_t vid, rt_uint16_t did, rt_uint16_t asm_vid, rt_uint16_t asm_did,
                   struct rio_dev *from);
extern int rio_init_mports(void);


#ifdef __cplusplus
}
#endif

#endif          /* __RIO_DRV_H__ */
