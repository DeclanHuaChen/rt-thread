/*
 * RapidIO driver support
 *
 * Copyright 2020 Automatic Test and Control Institute, Harbin Institute of Technology
 * Wang Huachen <oisun@qq.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 * 
 * Change Logs:
 * Date           Author            Notes
 */

#include <rtthread.h>
#include <drivers/rio.h>
#include <drivers/rio_drv.h>
#include <stdlib.h>

#include "rio.h"

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops rio_mport_ops = 
{
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL
};
#endif

rt_err_t rio_mport_init(struct rio_mport *mport)
{
    struct rt_device *device;
    char name[RIO_MAX_MPORT_NAME+1];
    RT_ASSERT(mport != RT_NULL);

    device = &mport->parent;

    /* set device type */
    device->type    = RT_Device_Class_Unknown;
    /* initialize device interface */
#ifdef RT_USING_DEVICE_OPS
    device->ops     = &rio_mport_ops;
#else
    device->init    = RT_NULL;
    device->open    = RT_NULL;
    device->close   = RT_NULL;
    device->read    = RT_NULL;
    device->write   = RT_NULL;
    device->control = RT_NULL;
#endif

    rt_snprintf(name, RIO_MAX_MPORT_NAME+1, "%s/%d", mport->name, mport->id);
    /* register to device manager */
    return rt_device_register(device, name, RT_DEVICE_FLAG_DEACTIVATE);
}


static rt_list_t rio_probe_callbacks = RT_LIST_OBJECT_INIT(rio_probe_callbacks);

/**
 * rio_dev_probe_register - Add a callback function that will be executed 
 *                             before a device is registered
 * @rdev_cb: struct rio_probe_callback to be added
 *
 * It should be called before mports scan. The callback function can 
 * be used to add rio_switch_ops for switch device. 
 */
void rio_dev_probe_register(struct rio_probe_callback* rdev_cb)
{
    rt_list_insert_before(&rio_probe_callbacks, &rdev_cb->node);
}

/**
 *  rio_match_device - Tell if a RIO device has a matching RIO device id structure
 *  @id: the RIO device id structure to match against
 *  @rdev: the RIO device structure to match against
 *
 *  Returns the matching &struct rio_device_id or %NULL if there is no match.
 */
static const struct rio_device_id *rio_match_device(const struct rio_device_id
						    *id, const struct rio_dev *rdev)
{
	while (id->vid || id->asm_vid) {
		if (((id->vid == RIO_ANY_ID) || (id->vid == rdev->vid)) &&
		    ((id->did == RIO_ANY_ID) || (id->did == rdev->did)) &&
		    ((id->asm_vid == RIO_ANY_ID)
		     || (id->asm_vid == rdev->asm_vid))
		    && ((id->asm_did == RIO_ANY_ID)
			|| (id->asm_did == rdev->asm_did)))
			return id;
		id++;
	}
	return NULL;
}


#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops rio_device_ops = 
{
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL
};
#endif

rt_err_t rio_device_init(struct rio_dev *dev)
{
    struct rt_device *device;
    struct rio_probe_callback *callback;
    RT_ASSERT(dev != RT_NULL);
    const struct rio_device_id *id;

    device = &(dev->parent);

    /* set device type */
    device->type    = RT_Device_Class_Unknown;
#ifdef RT_USING_DEVICE_OPS
    device->ops     = &rio_device_ops;
#else
    device->init    = RT_NULL;
    device->open    = RT_NULL;
    device->close   = RT_NULL;
    device->read    = RT_NULL;
    device->write   = RT_NULL;
    device->control = RT_NULL;
#endif

    rt_list_for_each_entry(callback, &rio_probe_callbacks, node)
    {
        if ((id = rio_match_device(callback->id, dev)))
        {
            callback->callback(dev, id);
            break;
        }
    }

    /* register to device manager */
    return rt_device_register(device, dev->name, RT_DEVICE_FLAG_DEACTIVATE);
}

static int mport_scan(int argc, char**argv)
{
    long val;
    char *endptr;
    int rc;

    if (argc != 2)
        goto exit;
    if ((val = strtol(argv[1], &endptr, 0)) < 0)
        goto exit;

    if (val == RIO_MPORT_ANY)
    {
        rc = rio_init_mports();
        return rc;
    }

    if (val < 0 || val >= RIO_MAX_MPORTS)
        goto exit;

    rc = rio_mport_scan((int)val);
    return rc;

exit:
    rt_kprintf("mport_scan <mport_id|%d>\n", RIO_MPORT_ANY);
    rt_kprintf("execute enumeration/discovery on the specified mport_id, %d for all morts\n",
                    RIO_MPORT_ANY);
    return -RT_EINVAL;
}
MSH_CMD_EXPORT(mport_scan, mport_scan <mport_id>);