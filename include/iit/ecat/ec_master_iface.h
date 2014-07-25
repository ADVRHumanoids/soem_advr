#ifndef __ECAT_MASTER_IFACE_H__
#define __ECAT_MASTER_IFACE_H__

#include <stdint.h>
#include <memory> // shared_ptr
#include <map>

#ifdef __cplusplus
extern "C" {
#endif

#include <soem-1.3.0/ethercattype.h>
#include <soem-1.3.0/nicdrv.h>
#include <soem-1.3.0/ethercatbase.h>
#include <soem-1.3.0/ethercatmain.h>
#include <soem-1.3.0/ethercatdc.h>
#include <soem-1.3.0/ethercatcoe.h>
#include <soem-1.3.0/ethercatfoe.h>
#include <soem-1.3.0/ethercatconfig.h>
#include <soem-1.3.0/ethercatprint.h>

#ifdef __cplusplus
}
#endif


#include "utils.h"
//#include "ec_slave_type.h"
#include "slave_wrapper.h"


#define EC_TIMEOUT_US      500

namespace iit {
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

int send_to_slaves(void);

int update_slave_firmware(uint16_t slave, std::string firmware, uint32_t passwd_firm);
 
}
}

#endif
