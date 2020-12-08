/*
 * RapidIO interconnect services
 * (RapidIO Interconnect Specification, http://www.rapidio.org)
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
#ifndef __RIO_H__
#define __RIO_H__

#include <stdlib.h>
#include <rthw.h>
#include <rtthread.h>
#include <drivers/rio_dep.h>
#include <drivers/rio_regs.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define RIO_RESOURCE_MEM	0x00000100
#define RIO_RESOURCE_DOORBELL	0x00000200
#define RIO_RESOURCE_MAILBOX	0x00000400

#define RIO_RESOURCE_CACHEABLE	0x00010000
#define RIO_RESOURCE_PCI	0x00020000

#define RIO_RESOURCE_BUSY	0x80000000


#define RIO_NO_HOPCOUNT		-1
#define RIO_INVALID_DESTID	0xffff

#define RIO_MAX_MPORTS		8
#define RIO_MAX_MPORT_RESOURCES	16
#define RIO_MAX_DEV_RESOURCES	16
#define RIO_MAX_MPORT_NAME	40
#define RIO_MAX_DEV_NAME	12

#define RIO_GLOBAL_TABLE	0xff	/* Indicates access of a switch's
                       global routing table if it
                       has multiple (or per port)
                       tables */

#define RIO_INVALID_ROUTE	0xff	/* Indicates that a route table
                       entry is invalid (no route
                       exists for the device ID) */

#define RIO_MAX_ROUTE_ENTRIES(size)	(size ? (1 << 16) : (1 << 8))
#define RIO_ANY_DESTID(size)		(size ? 0xffff : 0xff)

#define RIO_MAX_MBOX		4
#define RIO_MAX_MSG_SIZE	0x1000

/*
 * Error values that may be returned by RIO functions.
 */
#define RIO_SUCCESSFUL			0x00
#define RIO_BAD_SIZE			0x81

/*
 * For RIO devices, the region numbers are assigned this way:
 *
 *	0	RapidIO outbound doorbells
 *      1-15	RapidIO memory regions
 *
 * For RIO master ports, the region number are assigned this way:
 *
 *	0	RapidIO inbound doorbells
 *	1	RapidIO inbound mailboxes
 *	2	RapidIO outbound mailboxes
 */
#define RIO_DOORBELL_RESOURCE	0
#define RIO_INB_MBOX_RESOURCE	1
#define RIO_OUTB_MBOX_RESOURCE	2

#define RIO_PW_MSG_SIZE		64

/*
 * A component tag value (stored in the component tag CSR) is used as device's
 * unique identifier assigned during enumeration. Besides being used for
 * identifying switches (which do not have device ID register), it also is used
 * by error management notification and therefore has to be assigned
 * to endpoints as well.
 */
#define RIO_CTAG_RESRVD	0xfffe0000 /* Reserved */
#define RIO_CTAG_UDEVID	0x0001ffff /* Unique device identifier */

#define RIO_ANY_ID	0xffff


struct rio_mport;
struct rio_dev;
union rio_pw_msg;

/**
 * struct rio_switch - RIO switch info
 * @node: Node in global list of switches
 * @route_table: Copy of switch routing table
 * @port_ok: Status of each port (one bit per port) - OK=1 or UNINIT=0
 * @ops: pointer to switch-specific operations
 * @lock: lock to serialize operations updates
 * @nextdev: Array of per-port pointers to the next attached device
 */
struct rio_switch {
    rt_list_t node;
    rt_uint8_t *route_table;
    rt_uint32_t port_ok;
    struct rio_switch_ops *ops;
#ifdef RT_USING_SMP
    struct rt_spinlock lock;
#endif
    struct rio_dev *nextdev[0];
};

/**
 * struct rio_switch_ops - Per-switch operations
 * @owner: The module owner of this structure
 * @add_entry: Callback for switch-specific route add function
 * @get_entry: Callback for switch-specific route get function
 * @clr_table: Callback for switch-specific clear route table function
 * @set_domain: Callback for switch-specific domain setting function
 * @get_domain: Callback for switch-specific domain get function
 * @em_init: Callback for switch-specific error management init function
 * @em_handle: Callback for switch-specific error management handler function
 *
 * Defines the operations that are necessary to initialize/control
 * a particular RIO switch device.
 */
struct rio_switch_ops {
    int (*add_entry) (struct rio_mport *mport, rt_uint16_t destid, rt_uint8_t hopcount,
              rt_uint16_t table, rt_uint16_t route_destid, rt_uint8_t route_port);
    int (*get_entry) (struct rio_mport *mport, rt_uint16_t destid, rt_uint8_t hopcount,
              rt_uint16_t table, rt_uint16_t route_destid, rt_uint8_t *route_port);
    int (*clr_table) (struct rio_mport *mport, rt_uint16_t destid, rt_uint8_t hopcount,
              rt_uint16_t table);
    int (*set_domain) (struct rio_mport *mport, rt_uint16_t destid, rt_uint8_t hopcount,
               rt_uint8_t sw_domain);
    int (*get_domain) (struct rio_mport *mport, rt_uint16_t destid, rt_uint8_t hopcount,
               rt_uint8_t *sw_domain);
    int (*em_init) (struct rio_dev *dev);
    int (*em_handle) (struct rio_dev *dev, rt_uint8_t swport);
};

enum rio_device_state {
    RIO_DEVICE_INITIALIZING,
    RIO_DEVICE_RUNNING,
    RIO_DEVICE_GONE,
    RIO_DEVICE_SHUTDOWN,
};

/**
 * struct rio_dev - RIO device info
 * @parent: Device model device
 * @name: Device Port name string
 * @global_list: Node in list of all RIO devices
 * @net_list: Node in list of RIO devices in a network
 * @net: Network this device is a part of
 * @do_enum: Enumeration flag
 * @did: Device ID
 * @vid: Vendor ID
 * @device_rev: Device revision
 * @asm_did: Assembly device ID
 * @asm_vid: Assembly vendor ID
 * @asm_rev: Assembly revision
 * @efptr: Extended feature pointer
 * @pef: Processing element features
 * @swpinfo: Switch port info
 * @src_ops: Source operation capabilities
 * @dst_ops: Destination operation capabilities
 * @comp_tag: RIO component tag
 * @phys_efptr: RIO device extended features pointer
 * @phys_rmap: LP-Serial Register Map Type (1 or 2)
 * @em_efptr: RIO Error Management features pointer
 * @dma_mask: Mask of bits of RIO address this device implements
 * @riores: RIO resources this device owns
 * @pwcback: port-write callback function for this device
 * @destid: Network destination ID (or associated destid for switch)
 * @hopcount: Hopcount to this device
 * @prev: Previous RIO device connected to the current one
 * @state: device state
 * @rswitch: struct rio_switch (if valid for this device)
 */
struct rio_dev
{
    struct rt_device parent;
    char name[RIO_MAX_DEV_NAME];
    rt_list_t global_list; /* node in list of all RIO devices */
    rt_list_t net_list;	/* node in per net list */
    struct rio_net *net;	/* RIO net this device resides in */    
    rt_bool_t do_enum;
    rt_uint16_t did;
    rt_uint16_t vid;
    rt_uint32_t device_rev;
    rt_uint16_t asm_did;
    rt_uint16_t asm_vid;
    rt_uint16_t asm_rev;
    rt_uint16_t efptr;
    rt_uint32_t pef;
    rt_uint32_t swpinfo;
    rt_uint32_t src_ops;
    rt_uint32_t dst_ops;
    rt_uint32_t comp_tag;
    rt_uint32_t phys_efptr;
    rt_uint32_t phys_rmap;
    rt_uint32_t em_efptr;
    rt_uint64_t dma_mask;

    struct rio_resource riores[RIO_MAX_DEV_RESOURCES];
    int (*pwcback) (struct rio_dev *rdev, union rio_pw_msg *msg, int step);
    rt_uint16_t destid;
    rt_uint8_t hopcount;
    struct rio_dev *prev;
    rio_atomic_t state;
    struct rio_switch rswitch[0];	/* RIO switch info */
};

#define rio_dev_g(n) rt_list_entry(n, struct rio_dev, global_list)
#define rio_dev_f(n) rt_list_entry(n, struct rio_dev, net_list)
#define sw_to_rio_dev(n) rt_container_of(n, struct rio_dev, rswitch[0])

/**
 * struct rio_msg - RIO message event
 * @res: Mailbox rio_resource
 * @mcback: Message event callback
 */
struct rio_msg {
    struct rio_resource *res;
    void (*mcback) (struct rio_mport * mport, void *dev_id, int mbox, int slot);
};

/**
 * struct rio_dbell - RIO doorbell event
 * @node: Node in list of doorbell events
 * @res: Doorbell rio_resource
 * @dinb: Doorbell event callback
 * @dev_id: Device specific pointer to pass on event
 */
struct rio_dbell {
    rt_list_t node;
    struct rio_resource *res;
    void (*dinb) (struct rio_mport *mport, void *dev_id, 
            rt_uint16_t src, rt_uint16_t dst, rt_uint16_t info);
    void *dev_id;
};

/**
 * struct rio_mport - RIO master port info
 * @parent: Device model device
 * @dbells: List of doorbell events
 * @pwrites: List of portwrite events
 * @node: Node in global list of master ports
 * @nnode: Node in network list of master ports
 * @net: RIO net this mport is attached to
 * @lock: lock to synchronize lists manipulations
 * @riores: RIO resources that this master port interfaces owns
 * @inb_msg: RIO inbound message event descriptors
 * @outb_msg: RIO outbound message event descriptors
 * @host_deviceid: Host device ID associated with this master port
 * @ops: configuration space functions
 * @id: Port ID, unique among all ports
 * @index: Port index, unique among all port interfaces of the same type
 * @sys_size: RapidIO common transport system size
 * @phys_efptr: RIO port extended features pointer
 * @phys_rmap: LP-Serial EFB Register Mapping type (1 or 2).
 * @name: Port name string
 * @priv: Master port private data
 * @nscan: RapidIO network enumeration/discovery operations
 * @state: mport device state
 * @pwe_refcnt: port-write enable ref counter to track enable/disable requests
 */
struct rio_mport
{
    struct rt_device parent;
    rt_list_t dbells;	/* list of doorbell events */
    rt_list_t pwrites;	/* list of portwrite events */
    rt_list_t node;	/* node in global list of ports */
    rt_list_t nnode;	/* node in net list of ports */
    struct rio_net *net;	/* RIO net this mport is attached to */
    struct rt_mutex lock;
    struct rio_resource riores[RIO_MAX_MPORT_RESOURCES];
    struct rio_msg inb_msg[RIO_MAX_MBOX];
    struct rio_msg outb_msg[RIO_MAX_MBOX];
    int host_deviceid;	/* Host device ID */
    struct rio_ops *ops;	/* low-level architecture-dependent routines */
    rt_uint8_t id;	/* port ID, unique among all ports */
    rt_uint8_t index;	/* port index, unique among all port
                   interfaces of the same type */
    rt_uint32_t sys_size;	/* RapidIO common transport system size.
                 * 0 - Small size. 256 devices.
                 * 1 - Large size, 65536 devices.
                 */
    rt_uint32_t phys_efptr;
    rt_uint32_t phys_rmap;
    char name[RIO_MAX_MPORT_NAME];
    void *priv;		/* Master port private data */

    struct rio_scan *nscan;
    rio_atomic_t state;
    rt_uint32_t pwe_refcnt;
};

static inline int rio_mport_is_running(struct rio_mport *mport)
{
    return rio_atomic_read(&mport->state) == RIO_DEVICE_RUNNING;
}

/*
 * Enumeration/discovery control flags
 */
#define RIO_SCAN_ENUM_NO_WAIT	0x00000001 /* Do not wait for enum completed */

/**
 * struct rio_net - RIO network info
 * @node: Node in global list of RIO networks
 * @devices: List of devices in this network
 * @switches: List of switches in this network
 * @mports: List of master ports accessing this network
 * @hport: Default port for accessing this network
 * @id: RIO network ID
 * @enum_data: private data specific to a network enumerator
 * @release: enumerator-specific release callback
 */
struct rio_net {
    rt_list_t node;	/* node in list of networks */
    rt_list_t devices;	/* list of devices in this net */
    rt_list_t switches;	/* list of switches in this net */
    rt_list_t mports;	/* list of ports accessing net */
    struct rio_mport *hport;	/* primary port for accessing net */
    unsigned char id;	/* RIO network ID */
    void *enum_data;	/* private data for enumerator of the network */
    void (*release)(struct rio_net *net);
};

enum rio_link_speed {
    RIO_LINK_DOWN = 0, /* SRIO Link not initialized */
    RIO_LINK_125 = 1, /* 1.25 GBaud  */
    RIO_LINK_250 = 2, /* 2.5 GBaud   */
    RIO_LINK_312 = 3, /* 3.125 GBaud */
    RIO_LINK_500 = 4, /* 5.0 GBaud   */
    RIO_LINK_625 = 5  /* 6.25 GBaud  */
};

enum rio_link_width {
    RIO_LINK_1X  = 0,
    RIO_LINK_1XR = 1,
    RIO_LINK_2X  = 3,
    RIO_LINK_4X  = 2,
    RIO_LINK_8X  = 4,
    RIO_LINK_16X = 5
};

enum rio_mport_flags {
    RIO_MPORT_DMA	 = (1 << 0), /* supports DMA data transfers */
    RIO_MPORT_DMA_SG = (1 << 1), /* DMA supports HW SG mode */
    RIO_MPORT_IBSG	 = (1 << 2), /* inbound mapping supports SG */
};

/**
 * struct rio_mport_attr - RIO mport device attributes
 * @flags: mport device capability flags
 * @link_speed: SRIO link speed value (as defined by RapidIO specification)
 * @link_width:	SRIO link width value (as defined by RapidIO specification)
 * @dma_max_sge: number of SG list entries that can be handled by DMA channel(s)
 * @dma_max_size: max number of bytes in single DMA transfer (SG entry)
 * @dma_align: alignment shift for DMA operations (as for other DMA operations)
 */
struct rio_mport_attr {
    int flags;
    int link_speed;
    int link_width;

    /* DMA capability info: valid only if RIO_MPORT_DMA flag is set */
    int dma_max_sge;
    int dma_max_size;
    int dma_align;
};

/**
 * struct rio_ops - Low-level RIO configuration space operations
 * @lcread: Callback to perform local (master port) read of config space.
 * @lcwrite: Callback to perform local (master port) write of config space.
 * @cread: Callback to perform network read of config space.
 * @cwrite: Callback to perform network write of config space.
 * @dsend: Callback to send a doorbell message.
 * @pwenable: Callback to enable/disable port-write message handling.
 * @open_outb_mbox: Callback to initialize outbound mailbox.
 * @close_outb_mbox: Callback to shut down outbound mailbox.
 * @open_inb_mbox: Callback to initialize inbound mailbox.
 * @close_inb_mbox: Callback to	shut down inbound mailbox.
 * @add_outb_message: Callback to add a message to an outbound mailbox queue.
 * @add_inb_buffer: Callback to	add a buffer to an inbound mailbox queue.
 * @get_inb_message: Callback to get a message from an inbound mailbox queue.
 * @map_inb: Callback to map RapidIO address region into local memory space.
 * @unmap_inb: Callback to unmap RapidIO address region mapped with map_inb().
 * @query_mport: Callback to query mport device attributes.
 * @map_outb: Callback to map outbound address region into local memory space.
 * @unmap_outb: Callback to unmap outbound RapidIO address region.
 */
struct rio_ops {
    int (*lcread) (struct rio_mport *mport, int index, rt_uint32_t offset, int len,
            rt_uint32_t *data);
    int (*lcwrite) (struct rio_mport *mport, int index, rt_uint32_t offset, int len,
            rt_uint32_t data);
    int (*cread) (struct rio_mport *mport, int index, rt_uint16_t destid,
            rt_uint8_t hopcount, rt_uint32_t offset, int len, rt_uint32_t *data);
    int (*cwrite) (struct rio_mport *mport, int index, rt_uint16_t destid,
            rt_uint8_t hopcount, rt_uint32_t offset, int len, rt_uint32_t data);
    int (*dsend) (struct rio_mport *mport, int index, rt_uint16_t destid, rt_uint16_t data);
    int (*pwenable) (struct rio_mport *mport, int enable);
    int (*open_outb_mbox)(struct rio_mport *mport, void *dev_id,
                  int mbox, int entries);
    void (*close_outb_mbox)(struct rio_mport *mport, int mbox);
    int  (*open_inb_mbox)(struct rio_mport *mport, void *dev_id,
                 int mbox, int entries);
    void (*close_inb_mbox)(struct rio_mport *mport, int mbox);
    int  (*add_outb_message)(struct rio_mport *mport, struct rio_dev *rdev,
                 int mbox, void *buffer, rt_size_t len);
    int (*add_inb_buffer)(struct rio_mport *mport, int mbox, void *buf);
    void *(*get_inb_message)(struct rio_mport *mport, int mbox);
    int (*map_inb)(struct rio_mport *mport, rio_dma_addr_t lstart,
            rt_uint64_t rstart, rt_uint64_t size, rt_uint32_t flags);
    void (*unmap_inb)(struct rio_mport *mport, rio_dma_addr_t lstart);
    int (*query_mport)(struct rio_mport *mport,
               struct rio_mport_attr *attr);
    int (*map_outb)(struct rio_mport *mport, rt_uint16_t destid, rt_uint64_t rstart,
            rt_uint32_t size, rt_uint32_t flags, rio_dma_addr_t *laddr);
    void (*unmap_outb)(struct rio_mport *mport, rt_uint16_t destid, rt_uint64_t rstart);
};

union rio_pw_msg {
    struct {
        rt_uint32_t comptag;	/* Component Tag CSR */
        rt_uint32_t errdetect;	/* Port N Error Detect CSR */
        rt_uint32_t is_port;	/* Implementation specific + PortID */
        rt_uint32_t ltlerrdet;	/* LTL Error Detect CSR */
        rt_uint32_t padding[12];
    } em;
    rt_uint32_t raw[RIO_PW_MSG_SIZE/sizeof(rt_uint32_t)];
};

/**
 * struct rio_scan - RIO enumeration and discovery operations
 * @owner: The module owner of this structure
 * @enumerate: Callback to perform RapidIO fabric enumeration.
 * @discover: Callback to perform RapidIO fabric discovery.
 */
struct rio_scan {
    int (*enumerate)(struct rio_mport *mport, rt_uint32_t flags);
    int (*discover)(struct rio_mport *mport, rt_uint32_t flags);
};

/**
 * struct rio_scan_node - list node to register RapidIO enumeration and
 * discovery methods with RapidIO core.
 * @mport_id: ID of an mport (net) serviced by this enumerator
 * @node: node in global list of registered enumerators
 * @ops: RIO enumeration and discovery operations
 */
struct rio_scan_node {
    int mport_id;
    rt_list_t node;
    struct rio_scan *ops;
};

/* Architecture and hardware-specific functions */
extern int rio_mport_initialize(struct rio_mport *);
extern int rio_register_mport(struct rio_mport *);
extern int rio_unregister_mport(struct rio_mport *);
extern int rio_open_inb_mbox(struct rio_mport *, void *, int, int);
extern void rio_close_inb_mbox(struct rio_mport *, int);
extern int rio_open_outb_mbox(struct rio_mport *, void *, int, int);
extern void rio_close_outb_mbox(struct rio_mport *, int);
extern int rio_query_mport(struct rio_mport *port,
               struct rio_mport_attr *mport_attr);

#ifdef __cplusplus
}
#endif

#endif /* __RIO_H__ */
