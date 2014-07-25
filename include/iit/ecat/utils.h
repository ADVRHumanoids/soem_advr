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

#include <sstream>
#include <boost/format.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/error_of.hpp>
#include <boost/accumulators/statistics/error_of_mean.hpp>

using namespace boost::accumulators;

typedef accumulator_set<uint64_t,
        features<
             tag::count
            ,tag::mean
            ,tag::min
            ,tag::max
            ,tag::variance(lazy)
            ,tag::error_of<tag::mean>
        >
    > stat_t;

inline void print_stat(stat_t &s) {

    std::ostringstream oss;

    if (count(s) > 0) {
        oss << "\tCount " << count(s) << std::endl;
        oss << "\tMean " << (uint64_t)mean(s);
        oss << "\tMin " << min(s);
        oss << "\tMax " << max(s);
        oss << "\tVar " << variance(s);
        oss << "\tErrOfMean " << error_of<tag::mean>(s);
        oss << std::endl;
    } else {
        oss << "No data ..." << std::endl;
    }
    DPRINTF("%s", oss.str().c_str());
}

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
inline void add_timespec(struct timespec *ts, int64_t addtime)
{
    int64_t sec, nsec;

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
