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
static iit::ecat::ecm_barrier_t ecm_barrier;
static iit::ecat::ec_timing_t ec_timing;
static iit::ecat::ec_thread_arg_t gl_ec_thread_arg;
static iit::ecat::SlavesMap userSlaves;


int ecm_barrier_init(iit::ecat::ecm_barrier_t &b)
{
    pthread_mutexattr_t mattr;
    pthread_condattr_t cattr;
    int ret;

    b.signaled = 0;
    pthread_mutexattr_init(&mattr);
    pthread_mutexattr_settype(&mattr, PTHREAD_MUTEX_NORMAL);
    pthread_mutexattr_setpshared(&mattr, PTHREAD_PROCESS_PRIVATE);
    pthread_mutexattr_setprotocol(&mattr, PTHREAD_PRIO_NONE);
    ret = pthread_mutex_init(&b.lock, &mattr);
    pthread_mutexattr_destroy(&mattr);
    if (ret)
        return ret;

    pthread_condattr_init(&cattr);
    pthread_condattr_setpshared(&cattr, PTHREAD_PROCESS_PRIVATE);
    ret = pthread_cond_init(&b.barrier, &cattr);
    pthread_condattr_destroy(&cattr);
    if (ret)
        pthread_mutex_destroy(&b.lock);

    return ret;
}

void ecm_barrier_destroy(iit::ecat::ecm_barrier_t &b)
{
    pthread_cond_destroy(&b.barrier);
    pthread_mutex_destroy(&b.lock);
}

int ecm_barrier_wait(iit::ecat::ecm_barrier_t &b)
{
    int ret = 0;
    
    pthread_mutex_lock(&b.lock);
    while (!b.signaled) {
        ret = pthread_cond_wait(&b.barrier, &b.lock);
        if (ret)
            break;
    }
    b.signaled = 0;
    pthread_mutex_unlock(&b.lock);
    return ret;
}

int ecm_barrier_timedwait(iit::ecat::ecm_barrier_t &b, struct timespec &ts)
{
    int ret = 0;
    
    pthread_mutex_lock(&b.lock);
    while (!b.signaled) {
        ret = pthread_cond_timedwait(&b.barrier, &b.lock, &ts);
        if (ret)
            break;
    }
    b.signaled = 0;
    pthread_mutex_unlock(&b.lock);
    return ret;
}

void ecm_barrier_release(iit::ecat::ecm_barrier_t &b)
{
    pthread_mutex_lock(&b.lock);
    b.signaled = 1;
    pthread_cond_broadcast(&b.barrier);
    //pthread_cond_signal(&b.barrier);
    pthread_mutex_unlock(&b.lock);
}



/* PI calculation to get linux rx thread synced to DC time */
static int64_t ec_sync(const int64_t reftime,
                       const uint64_t cycletime ,
                       const uint64_t sync_point_ns,
                       int64_t *offsettime)
{
    /* master sync offset with ec_DCtime */
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
    
    return delta;
}

int iit::ecat::ecat_cycle(void) {

    int wkc;
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUT_US);

    return wkc;
}

/* RT EtherCAT thread */
void * ecat_thread( void * _arg )
{
    struct timespec   ts, tleft;
    int rc;
    int ht;
    int wkc;
    uint64_t    t_prec, t_now;
    uint64_t    ec_cycle_ns, ec_now;
    int64_t     toff;
    uint64_t    delta;

    iit::ecat::ec_thread_arg_t * ptr_ec_th_arg = (iit::ecat::ec_thread_arg_t*)_arg;
    
#ifdef __COBALT__
    pthread_setname_np(pthread_self(), "ecat");
    pthread_setmode_np(0, PTHREAD_WARNSW, 0);
#else
    pthread_setname_np(pthread_self(), "ecat");
#endif

    rc = clock_gettime(CLOCK_REALTIME, &ts);
    ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
    ts.tv_nsec = ht * 1000000;
    //DPRINTF("%llu \n", cycle_time_ns);
    toff = 0;
//     pthread_mutex_lock(&ecat_mutex);
//     ec_send_processdata();
//     pthread_mutex_unlock(&ecat_mutex);
    
    while ( ecat_thread_run ) {
        
        /* calculate next cycle start */
        iit::ecat::add_timespec(&ts, ptr_ec_th_arg->ecat_cycle_ns + toff);
        /* wait to cycle start */
        rc = clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);

        pthread_mutex_lock(&ecat_mutex);
        ec_now = iit::ecat::get_time_ns();
        wkc = iit::ecat::ecat_cycle();
        ec_cycle_ns = iit::ecat::get_time_ns() - ec_now;
        //wkc = 1;
        pthread_mutex_unlock(&ecat_mutex);


        if ( ec_slave[0].hasdc && ptr_ec_th_arg->ecat_cycle_ns != 0 ) {
            /* calulate toff to get linux time and DC synced */
            delta = ec_sync(ec_DCtime,
                            ptr_ec_th_arg->ecat_cycle_ns,
                            ptr_ec_th_arg->sync_point_ns,
                            &toff);
        } else {
            toff = 250000;
        }

        t_now = iit::ecat::get_time_ns();

        ec_timing.recv_dc_time = ec_DCtime;
        ec_timing.offset = toff;
        ec_timing.loop_time = t_now - t_prec;
        ec_timing.ecat_rx_wkc = wkc;
        ec_timing.delta = delta;
        ec_timing.ts = ts;
        
        t_prec = t_now;
        
        if ( wkc > 0 ) {
            ecm_barrier_release(ecm_barrier);
        } else {
            DPRINTF("wkc %d\n", wkc);
        }
        
    }  
      
    DPRINTF("[ECat_master] ecat thread exit clean !!!\n");

    return 0;
}


static void start_ecat_thread(const iit::ecat::ec_thread_arg_t &ec_thread_arg) {

    int                 ret;
    pthread_attr_t      attr;
    int                 policy;
    cpu_set_t           cpu_set;
    struct sched_param  schedparam;


#ifdef __COBALT__
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

    DPRINTF("[ECat_master] Start ecat_thread %lld ns\n", ec_thread_arg.ecat_cycle_ns);
    ecat_thread_run = 1;
    ret = pthread_create(&ecat_thread_id, &attr, ecat_thread, (void*)&ec_thread_arg);

    pthread_attr_destroy ( &attr );
    
    if ( ret ) {
        DPRINTF ( "%s %d %s", __FILE__, __LINE__, "ecat" );
        perror ( "pthread_create fail" );

        exit ( 1 );
    }
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
int iit::ecat::initialize(const char* ifname, bool reset_micro)
{

    ecm_barrier_init(ecm_barrier);
    ec_reset_micro_slaves(reset_micro);
    
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
 * @return int expectedWKC
 */
int iit::ecat::operational(const ec_thread_arg_t &ec_thread_arg) 
{
    struct timespec sleep_time;

    if ( ! ec_configdc() ) {
        DPRINTF("[ECat_master] Failed to config DC\n");
        return 0;
    }
    
    // Configure DC if ...
    if ( ec_thread_arg.ecat_cycle_ns > 0 ) {
        DPRINTF("[ECat_master] Configure DC\n");
        // Configure the distributed clocks for each slave.
        for ( int i = 1; i <= ec_slavecount; i++ ) {
            ec_dcsync0(i, true, ec_thread_arg.ecat_cycle_ns, ec_thread_arg.ecat_cycle_shift_ns);
        }
    }

    if ( req_state_check(0, EC_STATE_SAFE_OP) != EC_STATE_SAFE_OP ) {
        return 0;
    }
    

    //
    // some slaves needs receive pdos before going to OP
    //
    sleep_time = { 0, 50000}; // 50 us
    if ( ec_thread_arg.ecat_cycle_ns > 0 ) {
        // Update ec_DCtime so we can calculate stop time below.
        ecat_cycle();
        // Send a barrage of packets to set up the DC clock.
        int64_t stoptime = ec_DCtime + ec_thread_arg.ecat_cycle_shift_ns;
        //int64_t stoptime = 3e9L;
        // SOEM automatically updates ec_DCtime.
        DPRINTF("[ECat_master] warm up .. run ecat_cycle for %.2f secs\n", (float)(stoptime/1e9L) );
        while ( ec_DCtime < stoptime ) {
            ecat_cycle();
            clock_nanosleep(CLOCK_REALTIME, 0, &sleep_time, NULL);
            //DPRINTF("[ECat_master] ec_DCtime %ld %ld\n", ec_DCtime, stoptime);

        }
    }

    sleep_time = { 0, 50000000}; // 50 ms
    clock_nanosleep(CLOCK_REALTIME, 0, &sleep_time, NULL);

    if ( req_state_check(0, EC_STATE_OPERATIONAL) != EC_STATE_OPERATIONAL ) {
        // exit .. otherwise with stuck in next loop
        // !! if the bootloader is running the only allowed state are INIT and BOOT
        return 0;
    }

    for ( const auto & item : userSlaves ) {
        // reset error counter
        item.second->resetError();
        // initialize tx_pdo
        item.second->writePDO();
    }
    
    // We are now in OP ...
    // We now have data.

    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    DPRINTF("[ECat_master] Calculated workcounter %d\n", expectedWKC);
    DPRINTF("[ECat_master] ec_DCtime %ld\n", ec_DCtime);
    DPRINTF("[ECat_master] o: %d   i: %d\n", ec_slave[0].Obytes, ec_slave[0].Ibytes);

    // start thread, setting distribuited clock DC0 to ecat_cycle_ns
    start_ecat_thread(ec_thread_arg);

    return expectedWKC;

}

int iit::ecat::operational(const uint32_t ecat_cycle_ns, 
                           const uint32_t ecat_cycle_shift_ns)
{
    gl_ec_thread_arg.ecat_cycle_ns = ecat_cycle_ns;
    gl_ec_thread_arg.ecat_cycle_shift_ns = ecat_cycle_shift_ns;
    gl_ec_thread_arg.sync_point_ns = 600000;
    
    operational(gl_ec_thread_arg);
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

void iit::ecat::finalize(bool do_power_off) {

    pre_operational();

    req_state_check(0, EC_STATE_INIT);

    if ( do_power_off ) { power_off(); }
    
    ec_close();
    DPRINTF("[ECat_master] close\n");
    ecm_barrier_destroy(ecm_barrier);
    pthread_mutex_destroy(&ecat_mutex);
}

int iit::ecat::setExpectedSlaves(const SlavesMap& expectedSlaves)
{
    int ret = 0;
    if ( ec_slavecount != expectedSlaves.size() ) {
        DPRINTF("[ECat_master] WARNING: expected %d slaves, %d found.\n",
                expectedSlaves.size(), ec_slavecount);
        ret = 1;
    }
    userSlaves = expectedSlaves;

    return ret;
}

int iit::ecat::recv_from_slaves(ec_timing_t &timing) {

    int ret;
    // Xenomai pthread_cond_timedwait()
    // The timeout abstime is expressed as an absolute value of the clock attribute passed to pthread_cond_init().
    // By default, CLOCK_REALTIME is used.
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    add_timespec(&ts,250000000);
    ret = ecm_barrier_timedwait(ecm_barrier, ts);

    // ret != 0 on error ... ETIMEDOUT == 110
    if ( ret != 0 ) {
       return -ret; 
    } 

    timing = ec_timing;

    // wkc > 0 ... check if wkc == expectedWKC
    ret = ec_timing.ecat_rx_wkc;
    
    if ( userSlaves.size() == 0 ) {
        //DPRINTF("[ECat_master] WARN : the expected-slaves map was not initialized.\n");
        return ret;
    }
    
    if ( ec_timing.ecat_rx_wkc != expectedWKC ) {
        DPRINTF("[ECat_master] WARN: wkc %d != %d expectedWKC\n", ec_timing.ecat_rx_wkc , expectedWKC);
        for ( const auto & item : userSlaves ) {
            item.second->readErrReg();
        }
    } else { 
        for ( const auto & item : userSlaves ) {
            item.second->readPDO();
        }
    }
        
    // ret > 0 and wkc == expectedWKC
    return ret;
}

int iit::ecat::send_to_slaves(bool write_slaves_pdo) {

    int ret = 0;

    pthread_mutex_lock(&ecat_mutex);
    
    if ( write_slaves_pdo ) {
        for ( const auto & item : userSlaves ) {
            item.second->writePDO();
        }
    }
    
    // >0 if processdata is transmitted
    ret = ec_send_processdata();
    //ret = 1;
    pthread_mutex_unlock(&ecat_mutex);

    return ret;

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
        return result;
    }

    return result;
}

int iit::ecat::recv_file(uint16_t slave, std::string filename, uint32_t passwd_file, uint32_t byte_count, std::string save_as) {

    int result;
    char * base;
    int file_size = byte_count;

    std::string file_buff(file_size, 0);

    base = basename((char*)filename.c_str());
    result = ec_FOEread(slave, base, passwd_file, &file_size ,(void*)file_buff.c_str(), EC_TIMEOUTSTATE*10);
    if ( result <= 0 ) {
        char * err =  ec_elist2string();
        DPRINTF("Ec_error : %s\n", err);
        DPRINTF("Fail with code %d\n", result);
        return result;
    }

    DPRINTF("Recv file OK, , %d bytes.\n",file_buff.length());
    std::ofstream file_stream(save_as, std::ofstream::out);
    file_stream << file_buff;
    file_stream.close();
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


