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


#define EC_TIMEOUT_US      500


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

/**
 * 
 * @return int 
 */
int ecat_cycle(void);

int recv_from_slaves(uint8_t * , int);
 
int send_to_slaves(uint8_t * , int);

void print_ecat_IOmap(void);

#endif
