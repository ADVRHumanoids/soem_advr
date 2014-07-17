#ifndef __ECAT_MASTER_IFACE_H__
#define __ECAT_MASTER_IFACE_H__

#include <stdint.h>
#include <string>

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

namespace ec_master_iface {

    typedef struct {

        int64_t     recv_dc_time;
        int64_t     offset;
        uint64_t    loop_time;

    } ec_timing_t;


/**
 * 
 * @param ifname 
 * @param ecat_cycle_ns 
 * @param ecat_cycle_shift_ns 
 * 
 * @return int expectedWKC
 */
    int initialize(const char* ifname,
                   const uint64_t* ecat_cycle_ns,
                   const uint64_t* ecat_cycle_shift_ns);

/**
 * 
 */
    void finalize(void);

    bool req_state_check(uint16 slave, uint16_t req_state);

    int recv_from_slaves(output_slave_t*, ec_timing_t *);

    int send_to_slaves(input_slave_t*);

    int update_slave_firmware(uint16_t slave, std::string firmware, uint32_t passwd_firm);
}

#endif
