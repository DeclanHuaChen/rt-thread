#include <rtthread.h>
#include <stdio.h>
#include "xil_io.h"
#include "drv_rio.h"

#define DBG_TAG "drv.rio"
#define DBG_LVL DBG_LOG
#include "rtdbg.h"

// 读取寄存器命令的起始地址，可根据需要修改
static uintptr_t const global_Handle = 0xb0000000;

static int devmemin(rt_off_t startAddress, uint32_t *val)
{
    *val = *((volatile uint32_t *)startAddress);
    return 0;
}
static int devmemout(rt_off_t startAddress, uint32_t val)
{
    *((volatile uint32_t *)startAddress) = val;
    return 0;
}

/*
num：板卡数量，输出
*/
int switch_port_query(int *num)
{
    int ret = 0;
    int i = 0;
    uint32_t val;

    for (i = 0; i < 9; i++)
    {
        ret = devmemin(0xb9000158 + i * 0x20, &val);
        if (ret < 0)
            return -1;
        LOG_D("switch_port_query port %d, addr %x , val %x", i, 0xb9000158 + i * 0x20, val);
        if ((val & 0x2) == 0)
            break;
    }
    *num = i - 1;
    LOG_D("switch_port_query port is %d", *num);

    switch (*num)
    {
    case 0:
        ret = devmemout(0xb900015C, 0x00600001);
        ret = devmemout(0xb900017C, 0x00600001);
        ret = devmemout(0xb900019C, 0x00600001);
        ret = devmemout(0xb90001BC, 0x00600001);
        ret = devmemout(0xb90001DC, 0x00600001);
        ret = devmemout(0xb90001FC, 0x00600001);
        ret = devmemout(0xb900021C, 0x00600001);
        ret = devmemout(0xb900023C, 0x00600001);
        break;
    case 1:
        ret = devmemout(0xb900015C, 0x00600001);
        ret = devmemout(0xb900017C, 0x00600001);
        ret = devmemout(0xb900019C, 0x00600001);
        ret = devmemout(0xb90001BC, 0x00600001);
        ret = devmemout(0xb90001DC, 0x00600001);
        ret = devmemout(0xb90001FC, 0x00600001);
        ret = devmemout(0xb900021C, 0x00600001);
        ret = devmemout(0xb900023C, 0x00600001);

        ret = devmemout(0xb9E10020, 0x00000001);

        ret = devmemout(0xb9E11004, 0x00000000);

        ret = devmemout(0xb8010100, 0x01020001);
        ret = devmemout(0xba000060, 0x00000001);
        break;
    case 2:
        ret = devmemout(0xb900015C, 0x00600001);
        ret = devmemout(0xb900017C, 0x00600001);
        ret = devmemout(0xb900019C, 0x00600001);
        ret = devmemout(0xb90001BC, 0x00600001);
        ret = devmemout(0xb90001DC, 0x00600001);
        ret = devmemout(0xb90001FC, 0x00600001);
        ret = devmemout(0xb900021C, 0x00600001);
        ret = devmemout(0xb900023C, 0x00600001);

        ret = devmemout(0xb9E10020, 0x00000002);
        ret = devmemout(0xb9E11020, 0x00000002);

        ret = devmemout(0xb9E12004, 0x00000000);
        ret = devmemout(0xb9E12008, 0x00000001);

        ret = devmemout(0xb8010100, 0x01020001);
        ret = devmemout(0xba000060, 0x00000001);
        ret = devmemout(0xb8010100, 0x02020002);
        ret = devmemout(0xba000060, 0x00000002);
        break;
    case 3:
        ret = devmemout(0xb900015C, 0x00600001);
        ret = devmemout(0xb900017C, 0x00600001);
        ret = devmemout(0xb900019C, 0x00600001);
        ret = devmemout(0xb90001BC, 0x00600001);
        ret = devmemout(0xb90001DC, 0x00600001);
        ret = devmemout(0xb90001FC, 0x00600001);
        ret = devmemout(0xb900021C, 0x00600001);
        ret = devmemout(0xb900023C, 0x00600001);

        ret = devmemout(0xb9E10020, 0x00000003);
        ret = devmemout(0xb9E11020, 0x00000003);
        ret = devmemout(0xb9E12020, 0x00000003);

        ret = devmemout(0xb9E13004, 0x00000000);
        ret = devmemout(0xb9E13008, 0x00000001);
        ret = devmemout(0xb9E1300C, 0x00000002);

        ret = devmemout(0xb8010100, 0x01020001);
        ret = devmemout(0xba000060, 0x00000001);
        ret = devmemout(0xb8010100, 0x02020002);
        ret = devmemout(0xba000060, 0x00000002);
        ret = devmemout(0xb8010100, 0x03020003);
        ret = devmemout(0xba000060, 0x00000003);
        break;
    case 4:
        ret = devmemout(0xb900015C, 0x00600001);
        ret = devmemout(0xb900017C, 0x00600001);
        ret = devmemout(0xb900019C, 0x00600001);
        ret = devmemout(0xb90001BC, 0x00600001);
        ret = devmemout(0xb90001DC, 0x00600001);
        ret = devmemout(0xb90001FC, 0x00600001);
        ret = devmemout(0xb900021C, 0x00600001);
        ret = devmemout(0xb900023C, 0x00600001);

        ret = devmemout(0xb9E10020, 0x00000004);
        ret = devmemout(0xb9E11020, 0x00000004);
        ret = devmemout(0xb9E12020, 0x00000004);
        ret = devmemout(0xb9E13020, 0x00000004);

        ret = devmemout(0xb9E14004, 0x00000000);
        ret = devmemout(0xb9E14008, 0x00000001);
        ret = devmemout(0xb9E1400C, 0x00000002);
        ret = devmemout(0xb9E14010, 0x00000003);

        ret = devmemout(0xb8010100, 0x01020001);
        ret = devmemout(0xba000060, 0x00000001);
        ret = devmemout(0xb8010100, 0x02020002);
        ret = devmemout(0xba000060, 0x00000002);
        ret = devmemout(0xb8010100, 0x03020003);
        ret = devmemout(0xba000060, 0x00000003);
        ret = devmemout(0xb8010100, 0x04020004);
        ret = devmemout(0xba000060, 0x00000004);
        break;
    case 5:
        ret = devmemout(0xb900015C, 0x00600001);
        ret = devmemout(0xb900017C, 0x00600001);
        ret = devmemout(0xb900019C, 0x00600001);
        ret = devmemout(0xb90001BC, 0x00600001);
        ret = devmemout(0xb90001DC, 0x00600001);
        ret = devmemout(0xb90001FC, 0x00600001);
        ret = devmemout(0xb900021C, 0x00600001);
        ret = devmemout(0xb900023C, 0x00600001);

        ret = devmemout(0xb9E10020, 0x00000005);
        ret = devmemout(0xb9E11020, 0x00000005);
        ret = devmemout(0xb9E12020, 0x00000005);
        ret = devmemout(0xb9E13020, 0x00000005);
        ret = devmemout(0xb9E14020, 0x00000005);

        ret = devmemout(0xb9E15004, 0x00000000);
        ret = devmemout(0xb9E15008, 0x00000001);
        ret = devmemout(0xb9E1500C, 0x00000002);
        ret = devmemout(0xb9E15010, 0x00000003);
        ret = devmemout(0xb9E15014, 0x00000004);

        ret = devmemout(0xb8010100, 0x01020001);
        ret = devmemout(0xba000060, 0x00000001);
        ret = devmemout(0xb8010100, 0x02020002);
        ret = devmemout(0xba000060, 0x00000002);
        ret = devmemout(0xb8010100, 0x03020003);
        ret = devmemout(0xba000060, 0x00000003);
        ret = devmemout(0xb8010100, 0x04020004);
        ret = devmemout(0xba000060, 0x00000004);
        ret = devmemout(0xb8010100, 0x05020005);
        ret = devmemout(0xba000060, 0x00000005);
        break;
    case 6:
        ret = devmemout(0xb900015C, 0x00600001);
        ret = devmemout(0xb900017C, 0x00600001);
        ret = devmemout(0xb900019C, 0x00600001);
        ret = devmemout(0xb90001BC, 0x00600001);
        ret = devmemout(0xb90001DC, 0x00600001);
        ret = devmemout(0xb90001FC, 0x00600001);
        ret = devmemout(0xb900021C, 0x00600001);
        ret = devmemout(0xb900023C, 0x00600001);

        ret = devmemout(0xb9E10020, 0x00000006);
        ret = devmemout(0xb9E11020, 0x00000006);
        ret = devmemout(0xb9E12020, 0x00000006);
        ret = devmemout(0xb9E13020, 0x00000006);
        ret = devmemout(0xb9E14020, 0x00000006);
        ret = devmemout(0xb9E15020, 0x00000006);

        ret = devmemout(0xb9E16004, 0x00000000);
        ret = devmemout(0xb9E16008, 0x00000001);
        ret = devmemout(0xb9E1600C, 0x00000002);
        ret = devmemout(0xb9E16010, 0x00000003);
        ret = devmemout(0xb9E16014, 0x00000004);
        ret = devmemout(0xb9E16018, 0x00000005);

        ret = devmemout(0xb8010100, 0x01020001);
        ret = devmemout(0xba000060, 0x00000001);
        ret = devmemout(0xb8010100, 0x02020002);
        ret = devmemout(0xba000060, 0x00000002);
        ret = devmemout(0xb8010100, 0x03020003);
        ret = devmemout(0xba000060, 0x00000003);
        ret = devmemout(0xb8010100, 0x04020004);
        ret = devmemout(0xba000060, 0x00000004);
        ret = devmemout(0xb8010100, 0x05020005);
        ret = devmemout(0xba000060, 0x00000005);
        ret = devmemout(0xb8010100, 0x06020006);
        ret = devmemout(0xba000060, 0x00000006);
        break;
    case 7:
        ret = devmemout(0xb900015C, 0x00600001);
        ret = devmemout(0xb900017C, 0x00600001);
        ret = devmemout(0xb900019C, 0x00600001);
        ret = devmemout(0xb90001BC, 0x00600001);
        ret = devmemout(0xb90001DC, 0x00600001);
        ret = devmemout(0xb90001FC, 0x00600001);
        ret = devmemout(0xb900021C, 0x00600001);
        ret = devmemout(0xb900023C, 0x00600001);

        ret = devmemout(0xb9E10020, 0x00000007);
        ret = devmemout(0xb9E11020, 0x00000007);
        ret = devmemout(0xb9E12020, 0x00000007);
        ret = devmemout(0xb9E13020, 0x00000007);
        ret = devmemout(0xb9E14020, 0x00000007);
        ret = devmemout(0xb9E15020, 0x00000007);
        ret = devmemout(0xb9E16020, 0x00000007);

        ret = devmemout(0xb9E17004, 0x00000000);
        ret = devmemout(0xb9E17008, 0x00000001);
        ret = devmemout(0xb9E1700C, 0x00000002);
        ret = devmemout(0xb9E17010, 0x00000003);
        ret = devmemout(0xb9E17014, 0x00000004);
        ret = devmemout(0xb9E17018, 0x00000005);
        ret = devmemout(0xb9E1701C, 0x00000006);

        ret = devmemout(0xb8010100, 0x01020001);
        ret = devmemout(0xba000060, 0x00000001);
        ret = devmemout(0xb8010100, 0x02020002);
        ret = devmemout(0xba000060, 0x00000002);
        ret = devmemout(0xb8010100, 0x03020003);
        ret = devmemout(0xba000060, 0x00000003);
        ret = devmemout(0xb8010100, 0x04020004);
        ret = devmemout(0xba000060, 0x00000004);
        ret = devmemout(0xb8010100, 0x05020005);
        ret = devmemout(0xba000060, 0x00000005);
        ret = devmemout(0xb8010100, 0x06020006);
        ret = devmemout(0xba000060, 0x00000006);
        ret = devmemout(0xb8010100, 0x07020007);
        ret = devmemout(0xba000060, 0x00000007);
        break;
    default:
        ret = -10;
        break;
    }
    return ret;
}

static void rioRegRead(int argc, char **argv)
{
    uint32_t regAddr, regValue;
    if (argc != 2)
    {
        rt_kprintf("rioRegRead: format error\n");
        return;
    }

    if (sscanf(argv[1], "0x%lx", &regAddr) != 1)
    {
        rt_kprintf("rioRegRead: address error\n");
        return;
    }

    regValue = Xil_In32(global_Handle + regAddr);
    rt_kprintf("reg[0x%08lX]=0x%08lX\n", regAddr, regValue);
}
MSH_CMD_EXPORT(rioRegRead, rioRegRead sample: rioRegRead <reg address(Hex)>);

static void rioRegWrite(int argc, char **argv)
{
    uint32_t regAddr, regValue;
    if (argc != 3)
    {
        rt_kprintf("rioRegWrite: format error\n");
        return;
    }
    if (sscanf(argv[1], "0x%lx", &regAddr) != 1)
    {
        rt_kprintf("rioRegWrite: address error\n");
        return;
    }
    if (sscanf(argv[2], "0x%lx", &regValue) != 1)
    {
        rt_kprintf("rioRegWrite: value error\n");
        return;
    }
    Xil_Out32(global_Handle + regAddr, regValue);
    rt_kprintf("reg[0x%08lX]=0x%08lX\n", regAddr, regValue);
}
MSH_CMD_EXPORT(rioRegWrite, rioRegWrite sample: rioRegWrite <reg address(Hex)> <data(Hex)>);

extern int ADC_CDMA_Init(void);
int rt_hw_rio_init(void)
{
    rt_err_t res = RT_EOK;
    int ret = 0;
    int modnum;
    ret = switch_port_query(&modnum);
    LOG_D("switch ret is: %d,modnum is :%d", ret, modnum);

    res = ADC_CDMA_Init();
    return res;
}
INIT_DEVICE_EXPORT(rt_hw_rio_init);