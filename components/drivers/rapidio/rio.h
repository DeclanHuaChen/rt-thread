/*
 * RapidIO interconnect services
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

#include <drivers/rio.h>

#define RIO_MAX_CHK_RETRY	3
#define RIO_MPORT_ANY		(-1)

/* Functions internal to the RIO core code */

extern rt_uint32_t rio_mport_get_feature(struct rio_mport *mport, int local, rt_uint16_t destid,
                 rt_uint8_t hopcount, int ftr);
extern rt_uint32_t rio_mport_get_physefb(struct rio_mport *port, int local,
                 rt_uint16_t destid, rt_uint8_t hopcount, rt_uint32_t *rmap);
extern rt_uint32_t rio_mport_get_efb(struct rio_mport *port, int local, rt_uint16_t destid,
                 rt_uint8_t hopcount, rt_uint32_t from);
extern int rio_mport_chk_dev_access(struct rio_mport *mport, rt_uint16_t destid,
                    rt_uint8_t hopcount);
extern int rio_lock_device(struct rio_mport *port, rt_uint16_t destid,
            rt_uint8_t hopcount, int wait_ms);
extern int rio_unlock_device(struct rio_mport *port, rt_uint16_t destid, rt_uint8_t hopcount);
extern int rio_route_add_entry(struct rio_dev *rdev,
            rt_uint16_t table, rt_uint16_t route_destid, rt_uint8_t route_port, int lock);
extern int rio_route_get_entry(struct rio_dev *rdev, rt_uint16_t table,
            rt_uint16_t route_destid, rt_uint8_t *route_port, int lock);
extern int rio_route_clr_table(struct rio_dev *rdev, rt_uint16_t table, int lock);
extern int rio_set_port_lockout(struct rio_dev *rdev, rt_uint32_t pnum, int lock);
extern struct rio_dev *rio_get_comptag(rt_uint32_t comp_tag, struct rio_dev *from);
extern struct rio_net *rio_alloc_net(struct rio_mport *mport);
extern int rio_add_net(struct rio_net *net);
extern void rio_free_net(struct rio_net *net);
extern int rio_add_device(struct rio_dev *rdev);
extern void rio_del_device(struct rio_dev *rdev, enum rio_device_state state);
extern int rio_enable_rx_tx_port(struct rio_mport *port, int local, rt_uint16_t destid,
                 rt_uint8_t hopcount, rt_uint8_t port_num);
extern int rio_register_scan(int mport_id, struct rio_scan *scan_ops);
extern int rio_unregister_scan(int mport_id, struct rio_scan *scan_ops);
extern struct rio_mport *rio_find_mport(int mport_id);
extern int rio_mport_scan(int mport_id);


#define RIO_GET_DID(size, x)	(size ? (x & 0xffff) : ((x & 0x00ff0000) >> 16))
#define RIO_SET_DID(size, x)	(size ? (x & 0xffff) : ((x & 0x000000ff) << 16))