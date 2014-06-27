#ifndef __ECAT_MASTER_IFACE_H__
#define __ECAT_MASTER_IFACE_H__

#include <stdint.h>
#include <memory> // shared_ptr
#include <map>

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
#include "slave_wrapper.h"


#define EC_TIMEOUT_US      500

namespace iit {
namespace io {
namespace ecat {


struct ec_timing_t {

    int64_t     recv_dc_time;
    int64_t     offset;
    uint64_t    loop_time;

};

typedef std::shared_ptr<EscWrapper>  ESCPtr;
typedef std::map<int, ESCPtr>  SlavesMap;


/**
 *
 * @param ifname
 * @param ecat_cycle_ns
 * @param ecat_cycle_shift_ns
 *
 * @return int expectedWKC
 */
int initialize(
        const char* ifname,
        const uint64_t* ecat_cycle_ns,
        const uint64_t* ecat_cycle_shift_ns);

/**
 *
 */
void finalize(void);

int setExpectedSlaves(const SlavesMap& expectedSlaves);

int recv_from_slaves(ec_timing_t *);

int send_to_slaves(input_slave_t*);

}
}
}

#endif
