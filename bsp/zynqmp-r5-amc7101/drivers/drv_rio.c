#include <rtthread.h>
#include <stdio.h>
#include "xil_io.h"
#include "drv_rio.h"

static uintptr_t const global_Handle = 0xb0010000;

static void rioRegRead(int argc, char** argv)
{
    uint32_t regAddr, regValue;
    if (argc != 2)
    {
        printf("rioRegRead: format error\n");
        return;
    }

    if (sscanf(argv[1], "0x%lx", &regAddr) != 1)
    {
        printf("rioRegRead: address error\n");
        return;
    }

    regValue = Xil_In32(global_Handle + regAddr * 4);
    printf("reg[0x%08lX]=0x%08lX\n", regAddr, regValue);
}
MSH_CMD_EXPORT(rioRegRead, rioRegRead sample: rioRegRead <reg address(Hex)>);

static void rioRegWrite(int argc, char** argv)
{
    uint32_t regAddr, regValue;
    if (argc != 3)
    {
        printf("rioRegWrite: format error\n");
        return;
    }
    if (sscanf(argv[1], "0x%lx", &regAddr) != 1)
    {
        printf("rioRegWrite: address error\n");
        return;
    }
    if (sscanf(argv[2], "0x%lx", &regValue) != 1)
    {
        printf("rioRegWrite: value error\n");
        return;
    }
    Xil_Out32(global_Handle + regAddr*4, regValue);
    printf("reg[0x%08lX]=0x%08lX\n", regAddr, regValue);
}
MSH_CMD_EXPORT(rioRegWrite, rioRegWrite sample: rioRegWrite <reg address(Hex)> <data(Hex)>);