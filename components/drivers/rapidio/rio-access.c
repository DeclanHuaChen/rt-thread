/*
 * RapidIO configuration space access support
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
#include <drivers/rio_drv.h>

/*
 *  Wrappers for all RIO configuration access functions.  They just check
 *  alignment and call the low-level functions pointed to by rio_mport->ops.
 */

#define RIO_8_BAD 0
#define RIO_16_BAD (offset & 1)
#define RIO_32_BAD (offset & 3)

/**
 * RIO_LOP_READ - Generate rio_local_read_config_* functions
 * @size: Size of configuration space read (8, 16, 32 bits)
 * @type: C type of value argument
 * @len: Length of configuration space read (1, 2, 4 bytes)
 *
 * Generates rio_local_read_config_* functions used to access
 * configuration space registers on the local device.
 */
#define RIO_LOP_READ(size,type,len) \
int __rio_local_read_config_##size \
    (struct rio_mport *mport, rt_uint32_t offset, type *value)		\
{									\
    int res;							\
    rt_uint32_t data = 0;							\
    if (RIO_##size##_BAD) return RIO_BAD_SIZE;			\
    res = mport->ops->lcread(mport, mport->id, offset, len, &data);	\
    *value = (type)data;						\
    return res;							\
}

/**
 * RIO_LOP_WRITE - Generate rio_local_write_config_* functions
 * @size: Size of configuration space write (8, 16, 32 bits)
 * @type: C type of value argument
 * @len: Length of configuration space write (1, 2, 4 bytes)
 *
 * Generates rio_local_write_config_* functions used to access
 * configuration space registers on the local device.
 */
#define RIO_LOP_WRITE(size,type,len) \
int __rio_local_write_config_##size \
    (struct rio_mport *mport, rt_uint32_t offset, type value)		\
{									\
    if (RIO_##size##_BAD) return RIO_BAD_SIZE;			\
    return mport->ops->lcwrite(mport, mport->id, offset, len, value);\
}

RIO_LOP_READ(8, rt_uint8_t, 1)
RIO_LOP_READ(16, rt_uint16_t, 2)
RIO_LOP_READ(32, rt_uint32_t, 4)
RIO_LOP_WRITE(8, rt_uint8_t, 1)
RIO_LOP_WRITE(16, rt_uint16_t, 2)
RIO_LOP_WRITE(32, rt_uint32_t, 4)

RTM_EXPORT(__rio_local_read_config_8);
RTM_EXPORT(__rio_local_read_config_16);
RTM_EXPORT(__rio_local_read_config_32);
RTM_EXPORT(__rio_local_write_config_8);
RTM_EXPORT(__rio_local_write_config_16);
RTM_EXPORT(__rio_local_write_config_32);

/**
 * RIO_OP_READ - Generate rio_mport_read_config_* functions
 * @size: Size of configuration space read (8, 16, 32 bits)
 * @type: C type of value argument
 * @len: Length of configuration space read (1, 2, 4 bytes)
 *
 * Generates rio_mport_read_config_* functions used to access
 * configuration space registers on the local device.
 */
#define RIO_OP_READ(size,type,len) \
int rio_mport_read_config_##size \
    (struct rio_mport *mport, rt_uint16_t destid, rt_uint8_t hopcount, rt_uint32_t offset, type *value)	\
{									\
    int res;							\
    rt_uint32_t data = 0;							\
    if (RIO_##size##_BAD) return RIO_BAD_SIZE;			\
    res = mport->ops->cread(mport, mport->id, destid, hopcount, offset, len, &data); \
    *value = (type)data;						\
    return res;							\
}

/**
 * RIO_OP_WRITE - Generate rio_mport_write_config_* functions
 * @size: Size of configuration space write (8, 16, 32 bits)
 * @type: C type of value argument
 * @len: Length of configuration space write (1, 2, 4 bytes)
 *
 * Generates rio_mport_write_config_* functions used to access
 * configuration space registers on the local device.
 */
#define RIO_OP_WRITE(size,type,len) \
int rio_mport_write_config_##size \
    (struct rio_mport *mport, rt_uint16_t destid, rt_uint8_t hopcount, rt_uint32_t offset, type value)	\
{									\
    if (RIO_##size##_BAD) return RIO_BAD_SIZE;			\
    return mport->ops->cwrite(mport, mport->id, destid, hopcount,	\
            offset, len, value);				\
}

RIO_OP_READ(8, rt_uint8_t, 1)
RIO_OP_READ(16, rt_uint16_t, 2)
RIO_OP_READ(32, rt_uint32_t, 4)
RIO_OP_WRITE(8, rt_uint8_t, 1)
RIO_OP_WRITE(16, rt_uint16_t, 2)
RIO_OP_WRITE(32, rt_uint32_t, 4)

RTM_EXPORT(rio_mport_read_config_8);
RTM_EXPORT(rio_mport_read_config_16);
RTM_EXPORT(rio_mport_read_config_32);
RTM_EXPORT(rio_mport_write_config_8);
RTM_EXPORT(rio_mport_write_config_16);
RTM_EXPORT(rio_mport_write_config_32);

/**
 * rio_mport_send_doorbell - Send a doorbell message
 *
 * @mport: RIO master port
 * @destid: RIO device destination ID
 * @data: Doorbell message data
 *
 * Send a doorbell message to a RIO device. The doorbell message
 * has a 16-bit info field provided by the data argument.
 */
int rio_mport_send_doorbell(struct rio_mport *mport, rt_uint16_t destid, rt_uint16_t data)
{
    return mport->ops->dsend(mport, mport->id, destid, data);
}

RTM_EXPORT(rio_mport_send_doorbell);