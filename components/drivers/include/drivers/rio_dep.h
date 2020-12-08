/*
 * Since the RapidIO driver is migrated from the Linux kernel, 
 * some of the Linux APIs that we depend on are implemented here
 * 
 * Copyright 2020 Automatic Test and Control Institute, Harbin Institute of Technology
 * Wang Huachen <oisun@qq.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 * 
 * Change Logs:
 * Date           Author            Notes
 */
#ifndef __RIO_DEP_H__
#define __RIO_DEP_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* struct resource */
struct rio_resource
{
    rt_uint32_t start;
    rt_uint32_t end;
    rt_uint32_t flags;
    rt_list_t node;
};

static inline int rio_request_resource(struct rio_resource *root,
                                                        struct rio_resource *new)
{
    rt_uint32_t start = new->start;
    rt_uint32_t end = new->end;
    struct rio_resource *pos, *next;
    rt_uint32_t find;

    if (end < start)
        return -RT_EFULL;
    if (start < root->start)
        return -RT_EFULL;
    if (end > root->end)
        return -RT_EFULL;

    rt_enter_critical();
    find = 0;
    next = rt_list_entry(root->node.next, typeof(struct rio_resource), node);
    if (rt_list_isempty(&root->node) || (end < next->start))
    {
        find = 1;
        rt_list_insert_after(&root->node, &new->node);
    }
    else
    {
        rt_list_for_each_entry_safe(pos, next, &root->node, node)
        {
            if (start <= pos->end)
            {
                find = 0;
                break;
            }
            else if ((next == root) || (end < next->start))
            {
                find = 1;
                rt_list_insert_after(&pos->node, &new->node);
                break;
            }
        }
    }
    rt_exit_critical();

    if (find)
        return 0;
    else
        return -RT_EFULL;
}

static inline int rio_release_resource(struct rio_resource *old)
{
    int ret;
    rt_enter_critical();
    if (rt_list_isempty(&old->node))
        ret = -RT_EINVAL;
    else
    {
        rt_list_remove(&old->node);
        ret = 0;
    }
    rt_exit_critical();
    return ret;
}

/* atomic_t */
typedef struct 
{ 
     volatile rt_base_t counter; 
} rio_atomic_t;

#define rio_atomic_set(v,i)    (((v)->counter) = (i))
#define rio_atomic_read(v)   (*(volatile rt_base_t *)&(v)->counter)
static inline rt_base_t rio_atomic_cmpxchg(rio_atomic_t *v, 
                                rt_base_t old, rt_base_t new)
{
    rt_base_t ret;
    rt_enter_critical();
    ret = rio_atomic_read(v);
    if (ret == old)
        rio_atomic_set(v, new);
    rt_exit_critical();
    return ret;
}


/* dma_addr_t */
typedef volatile void *rio_dma_addr_t;
#define RIO_DMA_BIT_MASK(n)	(((n) == 64) ? ~0ULL : ((1ULL<<(n))-1))

/* bitmap */
#define RIO_BITS_PER_BYTE        8
#define RIO_BITS_PER_LONG        (RIO_BITS_PER_BYTE * sizeof(long))
#define RIO_DIV_ROUND_UP(n,d)    (((n) + (d) - 1) / (d))
#define RIO_BITS_TO_LONGS(nr)    RIO_DIV_ROUND_UP(nr, RIO_BITS_PER_LONG)

#define RIO_BIT(nr)         (1UL << (nr))
#define RIO_BIT_MASK(nr)    (1UL << ((nr) % RIO_BITS_PER_LONG))
#define RIO_BIT_WORD(nr)    ((nr) / RIO_BITS_PER_LONG)
#define RIO_BITMAP_FIRST_WORD_MASK(start) (~0UL << ((start) & (RIO_BITS_PER_LONG - 1)))

#define __rio_round_mask(x, y) ((__typeof__(x))((y)-1))
#define rio_round_down(x, y)   ((x) & ~__rio_round_mask(x, y))

#define __rio_ffs(x)  __rt_ffs(x)
#define __rio_ffz(x)  __rio_ffs(~(x))

#define rio_min(a, b)  ((a < b) ? a: b)

static inline void rio_set_bit(int nr, volatile unsigned long *addr)
{
    unsigned long mask = RIO_BIT_MASK(nr);
    unsigned long *p = ((unsigned long *)addr) + RIO_BIT_WORD(nr);

    *p |= mask;
}

static inline void rio_clear_bit(int nr, volatile unsigned long *addr)
{
    unsigned long mask = RIO_BIT_MASK(nr);
    unsigned long *p = ((unsigned long *)addr) + RIO_BIT_WORD(nr);

    *p &= ~mask;
}

static inline int rio_test_and_set_bit(int nr, volatile unsigned long *addr)
{
    unsigned long mask = RIO_BIT_MASK(nr);
    unsigned long *p = ((unsigned long *)addr) + RIO_BIT_WORD(nr);
    unsigned long old = *p;

    *p = old | mask;
    return (old & mask) != 0;
}

static inline unsigned long _rio_find_next_bit(const unsigned long *addr1,
        const unsigned long *addr2, unsigned long nbits,
        unsigned long start, unsigned long invert)
{
    unsigned long tmp;

    if (start >= nbits)
        return nbits;

    tmp = addr1[start / RIO_BITS_PER_LONG];
    if (addr2)
        tmp &= addr2[start / RIO_BITS_PER_LONG];
    tmp ^= invert;

    /* Handle 1st word. */
    tmp &= RIO_BITMAP_FIRST_WORD_MASK(start);
    start = rio_round_down(start, RIO_BITS_PER_LONG);

    while (!tmp) {
        start += RIO_BITS_PER_LONG;
        if (start >= nbits)
            return nbits;

        tmp = addr1[start / RIO_BITS_PER_LONG];
        if (addr2)
            tmp &= addr2[start / RIO_BITS_PER_LONG];
        tmp ^= invert;
    }

    return rio_min(start + __rt_ffs(tmp), nbits);
}

static inline unsigned long rio_find_next_bit(const unsigned long *addr, unsigned long size,
                unsigned long offset)
{
    return _rio_find_next_bit(addr, NULL, size, offset, 0UL);
}

static inline unsigned long rio_find_first_bit(const unsigned long *addr, unsigned long size)
{
    unsigned long idx;

    for (idx = 0; idx * RIO_BITS_PER_LONG < size; idx++) {
        if (addr[idx])
            return rio_min(idx * RIO_BITS_PER_LONG + __rio_ffs(addr[idx]), size);
    }

    return size;
}

static inline unsigned long rio_find_first_zero_bit(const unsigned long *addr, unsigned long size)
{
    unsigned long idx;

    for (idx = 0; idx * RIO_BITS_PER_LONG < size; idx++) {
        if (addr[idx] != ~0UL)
            return rio_min(idx * RIO_BITS_PER_LONG + __rio_ffz(addr[idx]), size);
    }

    return size;
}

/* memory alloc */
static inline void *rio_zalloc(rt_size_t size)
{
    void *ret;
    ret = rt_malloc(size);
    if(ret)
        rt_memset(ret, 0, size);

    return ret;
}

/* time delay */
#define rio_typecheck(type,x) \
({        type __dummy; \
        typeof(x) __dummy2; \
        (void)(&__dummy == &__dummy2); \
        1; \
})

#define rio_time_after(a,b)        \
    (rio_typecheck(unsigned long, a) && \
     rio_typecheck(unsigned long, b) && \
     ((long)(b) - (long)(a) < 0))
#define rio_time_before(a,b)    rio_time_after(b,a)

static inline void rio_udelay(rt_uint32_t us)
{
    unsigned long tick, delay, now;

    delay = (us * RT_TICK_PER_SECOND) / 1000000;
    delay = (delay == 0)?1:delay;
    tick = rt_tick_get() + delay;
    do
    {
        now = rt_tick_get();
    } while (rio_time_before(now, tick));
}
static inline void rio_mdelay(rt_uint32_t ms)
{
    unsigned long tick, delay, now;

    delay = (ms * RT_TICK_PER_SECOND) / 1000;
    delay = (delay == 0)?1:delay;
    tick = rt_tick_get() + delay;
    do
    {
        now = rt_tick_get();
    } while (rio_time_before(now, tick));
}


/* debug */
#ifdef  RT_RIO_DEBUG
#define rio_pr_debug(fmt, ...)  rt_kprintf(fmt, ##__VA_ARGS__)
#define rio_pr_err(fmt, ...)    rt_kprintf(fmt, ##__VA_ARGS__)
#else
static inline void rio_no_printf(const char *fmt, ...){}
#define rio_pr_debug(fmt, ...) rio_no_printf(fmt, ##__VA_ARGS__)
#define rio_pr_err(fmt, ...)   rio_no_printf(fmt, ##__VA_ARGS__)
#endif  /* RT_RIO_DEBUG */

#ifdef __cplusplus
}
#endif

#endif /* __RIO_DEP_H__ */