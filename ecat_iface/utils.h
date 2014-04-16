#ifndef __ECAT_UTILS_H__
#define __ECAT_UTILS_H__

#include <stdint.h>

#ifdef __XENO__
    #include <rtdk.h>
    #define DPRINTF rt_printf
#else
    #include <stdio.h>
    #define DPRINTF printf
#endif

#define NSEC_PER_SEC	1000000000ULL

inline uint64_t get_time_ns(clockid_t clock_id=CLOCK_MONOTONIC)
{
    uint64_t time_ns;
    struct timespec ts;
    clock_gettime(clock_id, &ts);
    time_ns = ts.tv_sec * NSEC_PER_SEC + ts.tv_nsec;
    return time_ns;
}


/* add ns to timespec */
inline void add_timespec(struct timespec *ts, int64 addtime)
{
    int64 sec, nsec;

    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;
    if ( ts->tv_nsec > NSEC_PER_SEC ) {
        nsec = ts->tv_nsec % NSEC_PER_SEC;
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
        ts->tv_nsec = nsec;
    }
}   


#endif
