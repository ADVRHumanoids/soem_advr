
#include <pthread.h>
#include <string.h>

#include "ec_master_iface.h"

/**
 * 
 */
static uint8_t IOmap[4096];

static int expectedWKC; 


int ecat_cycle(void) {

    int wkc;
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUT_US);

    return wkc;
}

/**
 * 
 * 
 * @author amargan (3/31/2014)
 * 
 * @param ifname 
 * @param ecat_cycle_ns 
 * @param ecat_cycle_shift_ns 
 * 
 * @return int expectedWKC
 */
int initialize(const char * ifname, uint64_t ecat_cycle_ns, uint64_t ecat_cycle_shift_ns) {


    DPRINTF("[ECat_master] Using %s\n", ifname);
    if ( ! ec_init((char*)ifname) ) {
        DPRINTF("[ECat_master] ECat_rt_soem_Master: ec_init(%s) failed!\n", ifname);
        return 0;
    }

    ec_config_init(FALSE);

    DPRINTF("[ECat_master] %d EtherCAT slaves identified.\n", ec_slavecount);
    if ( ec_slavecount < 1 ) {
        DPRINTF("[ECat_master] Failed to identify any slaves! Failing to init.\n");
        return 0;
    }

    if ( ! ec_configdc() ) {
        DPRINTF("[ECat_master] Failed to config DC\n");
    }

    ec_config_map(IOmap);

    // Wait for SAFE-OP
    ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

    // Configure DC
    if ( ecat_cycle_ns > 0 ) {
        // Configure the distributed clocks for each slave.
        for ( int i = 1; i <= ec_slavecount; i++ ) {
            ec_dcsync0(i, true, ecat_cycle_ns, ecat_cycle_shift_ns);
        }
    }

    // Send the EtherCAT slaves into OP
    DPRINTF("[ECat_master] Request OP\n");
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_writestate(0);
    if ( EC_STATE_OPERATIONAL != ec_statecheck(0, EC_STATE_OPERATIONAL,  EC_TIMEOUTSTATE * 2) ) {
        DPRINTF("[ECat_master] NOT all slaves OP\n");
        return 0;
    } else {
        DPRINTF("[ECat_master] All slaves OP\n");
    }

    // We are now in OP.


    // Update ec_DCtime so we can calculate stop time below.
    ecat_cycle();
    // Send a barrage of packets to set up the DC clock.
    int64_t stoptime = ec_DCtime + 200000000;
    // SOEM automatically updates ec_DCtime.
    while ( ec_DCtime < stoptime ) {
        ecat_cycle();
    }

    // We now have data.

    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    DPRINTF("[ECat_master] Calculated workcounter %d\n", expectedWKC);

    return expectedWKC;

}



void finalize(void) {

    DPRINTF("[ECat_master] Request preop state for all slaves\n");
    ec_slave[0].state = EC_STATE_PRE_OP;
    /* request PRE_OP state for all slaves */
    ec_writestate(0);
    ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE * 4);

    ec_close();
    DPRINTF("[ECat_master] close\n");

}


int recv_from_slaves(uint8_t * dest_buffer, int buff_size) {

    int wkc = ecat_cycle();

    if ( wkc < expectedWKC ) {
        return -wkc;
    }

    if ( buff_size >= ec_slave[0].Ibytes ) {
        memcpy((void*)dest_buffer, ec_slave[0].inputs, ec_slave[0].Ibytes);
    }

    return ec_slave[0].Ibytes;

}

int send_to_slaves(uint8_t * src_buffer, int buff_size) {

    int wkc = ecat_cycle();

    if ( wkc < expectedWKC ) {
        return -wkc;
    }

    if ( buff_size >= ec_slave[0].Obytes ) {
        memcpy((void*)ec_slave[0].outputs, src_buffer, ec_slave[0].Obytes);
    }

    return ec_slave[0].Obytes;

}

void print_ecat_IOmap(void) {


    DPRINTF(" O:");                  
    for (int j = 0 ; j < ec_slave[0].Obytes; j++ ) {
        DPRINTF(" %2.2x", *(ec_slave[0].outputs + j));
    }

    DPRINTF(" I:");                  
    for (int j = 0 ; j < ec_slave[0].Ibytes; j++ ) {
        DPRINTF(" %2.2x", *(ec_slave[0].inputs + j));
    }   
    DPRINTF(" T:%lld\n",ec_DCtime);

}

