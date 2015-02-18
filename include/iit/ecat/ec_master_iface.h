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
#include "slave_wrapper.h"


#define EC_TIMEOUT_US      500

namespace iit {
namespace ecat {


struct ec_timing_t {

    int64_t     recv_dc_time; ///< reception timestamp from 1-1-2000 (in [ns])
    int64_t     offset;       ///< sleep time of the master until next "DC tick"
    uint64_t    loop_time;    ///< actual measure of the DC period

};

typedef std::shared_ptr<EscWrapper>  ESCPtr;
typedef std::map<int, ESCPtr>  SlavesMap;


/**
 *
 * \param ifname
 *
 * \return int ec_slavexount
 */
int initialize(const char* ifname);
/**
 *
 * \param ecat_cycle_ns desired period of the DC (in nanoseconds)
 * \param ecat_cycle_shift_ns initial delay before using DC (in nanoseconds)
 *
 * \return int expectedWKC
 */
int operational(
        const uint64_t* ecat_cycle_ns,
        const uint64_t* ecat_cycle_shift_ns);

/**
 *
 */
void finalize(void);

// return actual state
int req_state_check(uint16 slave, uint16_t req_state);

int setExpectedSlaves(const SlavesMap& expectedSlaves);

int recv_from_slaves(ec_timing_t *);

int send_to_slaves(void);

int send_file(uint16_t slave, std::string filename, uint32_t passwd_firm);
 
void power_off(void);

//int write_alias(uint16_t slave, int alias);
 
}
}

#endif
