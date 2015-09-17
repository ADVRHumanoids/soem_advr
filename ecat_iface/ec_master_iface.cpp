/* 
 
- init
- discovery of slaves
- pre-op
- map slaves
- start DC
- activate process data transfer
- wait for DC lock
- activate sync0
- safe-op
- wait for all slaves to reach safe-op
- op
- wait for all slaves to reach op
- do your important stuff.....

*/

#include <pthread.h>
#include <string.h>
#include <libgen.h>

#include <string>
#include <fstream>

#include <iit/ecat/ec_master_iface.h>

#define T_OUT_MUL   1
/**
 * 
 */
static uint8_t IOmap[4096];

static int expectedWKC, ecat_thread_run; 

static pthread_t        ecat_thread_id;
static pthread_mutex_t  ecat_mutex = PTHREAD_MUTEX_INITIALIZER;

static pthread_mutex_t  ecat_mux_sync = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t   ecat_cond;

static iit::ecat::ec_timing_t ec_timing;

static const iit::ecat::SlavesMap* userSlaves = NULL;


static int ecat_cycle(void) {

    int wkc;
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUT_US);

    return wkc;
}



/* PI calculation to get linux rx thread synced to DC time */
static void ec_sync(const int64_t reftime, const uint64_t cycletime , int64_t* offsettime)
{
    /* master sync offset with ec_DCtime */
    static const uint32_t sync_point_ns = 300000; //500000; //400000;  
    static int64_t integral = 0;
    int64_t delta;

    delta = (reftime - sync_point_ns) % cycletime;
    if ( delta > (cycletime / 2) ) {
        delta = delta - cycletime;
    }
    if ( delta > 0 ) {
        integral++;
    }
    if ( delta < 0 ) {
        integral--;
    }
    *offsettime = -(delta / 100) - (integral / 20);

}

/* RT EtherCAT thread */
void * ecat_thread( void* cycle_ns )
{
    struct timespec   ts, tleft;
    struct timeval    tp;
    int rc;
    int ht;
    int wkc;
    uint64_t    cycle_time_ns;
    uint64_t    t_prec, t_now;
    int64_t     toff;
    uint64_t    t_delta;

#ifdef __XENO__
    pthread_set_name_np(pthread_self(), "ecat");
    pthread_set_mode_np(0, PTHREAD_WARNSW);
#else
    pthread_setname_np(pthread_self(), "ecat");
#endif


    //rc = pthread_mutex_lock(&mutex);
    rc = clock_gettime(CLOCK_MONOTONIC, &ts);
    ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
    ts.tv_nsec = ht * 1000000;
    cycle_time_ns = *((uint64_t*)(cycle_ns)); /* cycletime in ns */
    //DPRINTF("%llu \n", cycle_time_ns);
    toff = 0;
    pthread_mutex_lock(&ecat_mutex);
    ec_send_processdata();
    pthread_mutex_unlock(&ecat_mutex);
    
    while ( ecat_thread_run ) {
        
        /* calculate next cycle start */
        iit::ecat::add_timespec(&ts, cycle_time_ns + toff);
        /* wait to cycle start */
        rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);

        pthread_mutex_lock(&ecat_mutex);
        //wkc = ec_receive_processdata(EC_TIMEOUTRET);
        wkc = ecat_cycle();
        pthread_mutex_unlock(&ecat_mutex);

        t_now = iit::ecat::get_time_ns();
        t_delta =  t_now - t_prec;
        t_prec = t_now;

        if ( ec_slave[0].hasdc && cycle_time_ns != 0 ) {
            /* calulate toff to get linux time and DC synced */
            ec_sync(ec_DCtime, cycle_time_ns, &toff);
        } else {
            toff = 250000;
        }
        
        ec_timing.recv_dc_time = ec_DCtime;
        ec_timing.offset = toff;
        ec_timing.loop_time = t_delta;
        ec_timing.ecat_rx_wkc = wkc;
        
        if ( wkc > 0 ) {

            pthread_mutex_lock(&ecat_mux_sync);
            pthread_cond_signal(&ecat_cond);
            pthread_mutex_unlock(&ecat_mux_sync);

        } else {
            //DPRINTF("wkc %d\n", wkc);
        }

    }  
      
    DPRINTF("[ECat_master] ecat thread exit clean !!!\n");

    return 0;
}


static void start_ecat_thread(const uint64_t* cycle_time_ns) {

    pthread_attr_t      attr;
    int                 policy;
    cpu_set_t           cpu_set;
    struct sched_param  schedparam;


#ifdef __XENO__
    policy = SCHED_FIFO;
#else
    policy = SCHED_OTHER;
#endif

    CPU_ZERO(&cpu_set);
    CPU_SET(1,&cpu_set);

    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, policy);
    schedparam.sched_priority = sched_get_priority_max(policy);
    pthread_attr_setschedparam(&attr, &schedparam);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    pthread_attr_setaffinity_np(&attr, sizeof(cpu_set), &cpu_set);

    DPRINTF("[ECat_master] Start ecat_thread %llu ns\n", *cycle_time_ns);
    ecat_thread_run = 1;
    pthread_create(&ecat_thread_id, &attr, ecat_thread, (void*)cycle_time_ns);

}

int iit::ecat::req_state_check(uint16 slave, uint16_t req_state) {

    uint16_t act_state;
    uint16_t ec_error_mask = 0x10;
    uint16_t ec_state_mask = 0x0F;

    if ( slave == 0 ) {
        DPRINTF("[ECat_master] Request 0x%02X state for all slaves\n", req_state);
    } else {
        DPRINTF("[ECat_master] Request 0x%02X state for %d slave\n", req_state, slave);
    }

    ec_slave[slave].state = req_state;
    ec_writestate(slave);
    // just check req_state ... no error indication bit is check
    act_state = ec_statecheck(slave, req_state,  EC_TIMEOUTSTATE * T_OUT_MUL); 

    if ( req_state != act_state ) {
        // not all slave reached requested state ... find who and check error indication bit
        ec_readstate();

        if ( slave == 0 ) {

            for ( int i = 1; i<=ec_slavecount ; i++ ) {
                if ( ec_slave[i].state != req_state ) {
                    DPRINTF("Slave %d State=0x%02X StatusCode=0x%04X : %s",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    //if ( ec_slave[i].state & ec_error_mask ) {
                    // attemping to ack
                    ec_slave[i].state = (req_state & ec_state_mask) + EC_STATE_ACK;
                    ec_writestate(i);
                    act_state = ec_statecheck(i, req_state,  EC_TIMEOUTSTATE * T_OUT_MUL); 
                    if ( req_state != act_state ) {
                        // still req_state not reached ...
                        DPRINTF("... Slave %d State=0x%02X StatusCode=0x%04X : %s\n",
                                i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    } else {
                        DPRINTF("... reach State=0x%02X\n", ec_slave[i].state);
                    }
                    //}
                }
            }
        } else {
            DPRINTF("Slave %d State=0x%02X StatusCode=0x%04X : %s\n",
                    slave, ec_slave[slave].state, ec_slave[slave].ALstatuscode, ec_ALstatuscode2string(ec_slave[slave].ALstatuscode));
            //if ( ec_slave[slave].state & ec_error_mask ) {
            // attemping to ack
            ec_slave[slave].state = (req_state & ec_state_mask) + EC_STATE_ACK;
            ec_writestate(slave);
            act_state = ec_statecheck(slave, req_state,  EC_TIMEOUTSTATE * T_OUT_MUL); 
            if ( req_state != act_state ) {
                // still req_state not reached ...
                DPRINTF("... Slave %d State=0x%02X StatusCode=0x%04X : %s\n",
                        slave, ec_slave[slave].state, ec_slave[slave].ALstatuscode, ec_ALstatuscode2string(ec_slave[slave].ALstatuscode));
            }
            //}
        }

    }

    return act_state;
}
/**
 * 
 * 
 * @author amargan (3/31/2014)
 * 
 * @param ifname 
 * 
 * @return int ec_slavecount
 */
int iit::ecat::initialize(const char* ifname)
{

    DPRINTF("[ECat_master] Using %s\n", ifname);
    if ( ! ec_init((char*)ifname) ) {
        DPRINTF("[ECat_master] ECat_rt_soem_Master: ec_init(%s) failed!\n", ifname);
        return 0;
    }

    // retunr workcounter of slave discover datagram = number of slaves found
    ec_config(FALSE, &IOmap);

    DPRINTF("[ECat_master] %d EtherCAT slaves identified.\n", ec_slavecount);
    if ( ec_slavecount < 1 ) {
        DPRINTF("[ECat_master] Failed to identify any slaves! Failing to init.\n");
        return 0;
    }

    req_state_check(0, EC_STATE_PRE_OP);

    return ec_slavecount;
}

/**
 * 
 * 
 * @author amargan (3/31/2014)
 * 
 * @param ecat_cycle_ns 
 * @param ecat_cycle_shift_ns 
 * 
 * @return int expectedWKC
 */
int iit::ecat::operational(const uint64_t* ecat_cycle_ns, 
                           const uint64_t* ecat_cycle_shift_ns)
{
    struct timespec sleep_time;

    if ( ! ec_configdc() ) {
        DPRINTF("[ECat_master] Failed to config DC\n");
    }

    // Configure DC if ...
    if ( *ecat_cycle_ns > 0 ) {
        DPRINTF("[ECat_master] Configure DC\n");
        // Configure the distributed clocks for each slave.
        for ( int i = 1; i <= ec_slavecount; i++ ) {
            ec_dcsync0(i, true, *ecat_cycle_ns, *ecat_cycle_shift_ns);
        }
    }

    req_state_check(0, EC_STATE_SAFE_OP);

    sleep_time = { 0, 50000000};
    clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, NULL);

    if ( req_state_check(0, EC_STATE_OPERATIONAL) != EC_STATE_OPERATIONAL ) {
        // exit .. otherwise with stuck in next loop
        // !! if the bootloader is running the only allowed state are INIT and BOOT
        return 0;
    }

    if ( userSlaves != 0 ) {
        // reset error counter
        for ( auto it = userSlaves->begin(); it != userSlaves->end(); it++ ) {
            it->second->resetError();
        }
    }

    
    // We are now in OP ...
    sleep_time = { 0, 50000};
    if ( *ecat_cycle_ns > 0 ) {
        // Update ec_DCtime so we can calculate stop time below.
        ecat_cycle();
        // Send a barrage of packets to set up the DC clock.
        int64_t stoptime = ec_DCtime + *ecat_cycle_shift_ns;
        // SOEM automatically updates ec_DCtime.
        //DPRINTF("[ECat_master] warm up\n");
        while ( ec_DCtime < stoptime ) {
            ecat_cycle();
            clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, NULL);
            //DPRINTF("[ECat_master] ec_DCtime %ld %ld\n", ec_DCtime, stoptime);

        }

    }
    // We now have data.

    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    DPRINTF("[ECat_master] Calculated workcounter %d\n", expectedWKC);
    DPRINTF("[ECat_master] ec_DCtime %ld\n", ec_DCtime);
    DPRINTF("[ECat_master] o: %d   i: %d\n", ec_slave[0].Obytes, ec_slave[0].Ibytes);


    pthread_mutex_init(&ecat_mutex, NULL);
    pthread_mutex_init(&ecat_mux_sync, NULL);
    pthread_cond_init(&ecat_cond, NULL);
    // start thread, setting distribuited clock DC0 to ecat_cycle_ns
    start_ecat_thread(ecat_cycle_ns);


    return expectedWKC;

}

int iit::ecat::pre_operational(void) {

    DPRINTF("[ECat_master] Stop ecat_thread\n");
    ecat_thread_run = 0;
    //pthread_cancel(ecat_thread_id);
    pthread_join(ecat_thread_id, NULL);

    int state = req_state_check(0, EC_STATE_PRE_OP);

    for ( int i = 1; i <= ec_slavecount; i++ ) {
        ec_dcsync0(i, FALSE, 0, 0); // SYNC0 off
    }

    return state;
}


void iit::ecat::power_off() {

    DPRINTF("[ECat_master] POWER OFF slaves.\n");
    uint16_t power_on_gpio = 0;
    ec_BWR(0x0000, 0x0f10, sizeof(power_on_gpio), &power_on_gpio, EC_TIMEOUTRET3);

}

void iit::ecat::finalize(void) {

    pre_operational();

    req_state_check(0, EC_STATE_INIT);

    ec_close();
    DPRINTF("[ECat_master] close\n");

}

int iit::ecat::setExpectedSlaves(const SlavesMap& expectedSlaves)
{
    int ret = 0;
    if ( ec_slavecount != expectedSlaves.size() ) {
        DPRINTF("[ECat_master] WARNING: expected %d slaves, %d found.\n",
                expectedSlaves.size(), ec_slavecount);
        ret = 1;
    }
    userSlaves = & expectedSlaves;

    return ret;
}


int iit::ecat::recv_from_slaves(ec_timing_t* timing) {

    if ( userSlaves == NULL ) {
        DPRINTF("[ECat_master] FATAL: the expected-slaves map was not initialized.\n");
        return -1;
    }

    int ret;
    // Xenomai pthread_cond_timedwait()
    // The timeout abstime is expressed as an absolute value of the clock attribute passed to pthread_cond_init().
    // By default, CLOCK_REALTIME is used.
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    //ts.tv_sec = ts.tv_sec + 1;
    add_timespec(&ts,250000000);
    
    pthread_mutex_lock(&ecat_mux_sync);
    ret = pthread_cond_timedwait(&ecat_cond, &ecat_mux_sync, &ts);
    pthread_mutex_unlock(&ecat_mux_sync);

    *timing = ec_timing;

    // ret < 0 on error
    if ( ret < 0) {
       return ret; 
    }
    
    // wkc > 0 ... check if wkc == expectedWKC
    ret = ec_timing.ecat_rx_wkc;
    
    if ( ec_timing.ecat_rx_wkc != expectedWKC ) {
        DPRINTF("[ECat_master] WARN: wkc %d != %d expectedWKC\n", ec_timing.ecat_rx_wkc , expectedWKC);
        for ( auto it = userSlaves->begin(); it != userSlaves->end(); it++ ) {
            it->second->readErrReg();
        }
        // ret > 0 but wkc != expectedWKC
        return ret;
    }
    
    for ( auto it = userSlaves->begin(); it != userSlaves->end(); it++ ) {
        it->second->readPDO();
    }
        
    // ret > 0 and wkc == expectedWKC
    return ret;
}

int iit::ecat::send_to_slaves(void) {

    int wkc;

    pthread_mutex_lock(&ecat_mutex);
    
    for ( auto it = userSlaves->begin(); it != userSlaves->end(); it++ ) {
        it->second->writePDO();
    }

    //pthread_mutex_lock(&ecat_mutex);
    wkc = ec_send_processdata();
    //wkc = ecat_cycle();
    pthread_mutex_unlock(&ecat_mutex);

    return wkc;

}

int iit::ecat::send_file(uint16_t slave, std::string filename, uint32_t passwd_firm) {

    int result;
    char * base;

    std::ifstream file_stream(filename);
    std::string file_buff((std::istreambuf_iterator<char>(file_stream)), std::istreambuf_iterator<char>());

    DPRINTF("File read OK, %d bytes.\n",file_buff.length());
    DPRINTF("Update ....\n");
    base = basename((char*)filename.c_str());
    result = ec_FOEwrite(slave, base, passwd_firm, file_buff.length() ,(void*)file_buff.c_str(), EC_TIMEOUTSTATE*10);
    if ( result <= 0 ) {
        char * err =  ec_elist2string();
        DPRINTF("Ec_error : %s\n", err);
        DPRINTF("Fail with code %d\n", result);
    }

    return result;
}


#if 0

static void calc_crc(uint8 *crc, uint8 b)
{
   int j;
   *crc ^= b;
   for(j = 0; j <= 7 ; j++ )
   {
     if(*crc & 0x80)
        *crc = (*crc << 1) ^ 0x07;
     else
        *crc = (*crc << 1);
   }  
}

static uint16 SIIcrc(uint8 *buf)
{
   int i; 
   uint8 crc;
    
   crc = 0xff; 
   for( i = 0 ; i <= 13 ; i++ )
   {
      calc_crc(&crc , *(buf++));  
   } 
   return (uint16)crc;
}

static int eeprom_writealias(int slave, int alias, uint16 crc)
{
   int wkc;
   uint16 aiadr;
   uint8 eepctl;
   int ret;
   
   if((ec_slavecount >= slave) && (slave > 0) && (alias <= 0xffff))
   {
      aiadr = 1 - slave;
      eepctl = 2;
      wkc = ec_APWR(aiadr, ECT_REG_EEPCFG, sizeof(eepctl), &eepctl , EC_TIMEOUTRET); /* force Eeprom from PDI */
      eepctl = 0;
      wkc = ec_APWR(aiadr, ECT_REG_EEPCFG, sizeof(eepctl), &eepctl , EC_TIMEOUTRET); /* set Eeprom to master */

      ret = ec_writeeepromAP(aiadr, 0x04 , alias, EC_TIMEOUTEEP);
      if (ret)
        ret = ec_writeeepromAP(aiadr, 0x07 , crc, EC_TIMEOUTEEP);
        
      return ret;
   }
   
   return 0;
}

int iit::ecat::write_alias(uint16_t slave, int alias) {

    uint8_t buff[1024];
    uint16_t * wbuf = (uint16 *)buff;
    *(wbuf + 0x04) = alias;  
    return eeprom_writealias(slave, alias, SIIcrc(buff));

}

#endif


