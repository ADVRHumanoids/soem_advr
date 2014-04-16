#ifndef __ECAT_MASTER_IFACE_H__
#define __ECAT_MASTER_IFACE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"
}
#endif

#include "utils.h"
#include "ec_slave_type.h"


#define EC_TIMEOUT_US      500

typedef struct {

    int64_t     recv_dc_time;
    int64_t     offset;
    uint64_t    loop_time;

} ec_timing_t;

/**
 * 
 * @param cycle_time_ns 
 */
void start_ecat_thread(uint64_t cycle_time_ns);

/**
 * 
 * @param ifname 
 * @param ecat_cycle_ns 
 * @param ecat_cycle_shift_ns 
 * 
 * @return int expectedWKC
 */
int initialize(const char * ifname, uint64_t ecat_cycle_ns, uint64_t ecat_cycle_shift_ns);

/**
 * 
 */
void finalize(void);


int recv_from_slaves(output_slave_t *, ec_timing_t *);
 
int send_to_slaves(input_slave_t *);


#endif
