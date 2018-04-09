#include <signal.h>
#include <sys/mman.h>
#include <execinfo.h>
#include <stdio.h>

#ifdef __COBALT__
//    #include <rtdk.h>
#endif

#include <iit/ecat/ec_master_iface.h>
//#include <iit/ecat/ec_slave_type.h>
//#include <iit/ecat/slave_wrapper.h>



static int run_loop = 1;

static void warn_upon_switch(int sig __attribute__((unused)))
{
    // handle rt to nrt contex switch
    void *bt[32];
    int nentries;

    /* Dump a backtrace of the frame which caused the switch to
       secondary mode: */
    nentries = backtrace(bt,sizeof(bt)/sizeof(bt[0]));
    // dump backtrace 
    backtrace_symbols_fd(bt,nentries,fileno(stdout));
}

static void shutdown(int sig __attribute__((unused)))
{
    run_loop = 0;
    DPRINTF("got signal .... Shutdown\n");
}

static void set_signal_handler(void)
{
    signal(SIGINT, shutdown);
    signal(SIGINT, shutdown);
    signal(SIGKILL, shutdown);
#ifdef __COBALT__
    // call pthread_set_mode_np(0, PTHREAD_WARNSW) to cause a SIGXCPU
    // signal to be sent when the calling thread involontary switches to secondary mode
    signal(SIGXCPU, warn_upon_switch);
#endif
}

static int esc_gpio_ll_wr ( uint16_t configadr, uint16_t gpio_val ) {

    int wc = ec_FPWR ( configadr, 0x0F10, sizeof ( gpio_val ), &gpio_val, EC_TIMEOUTRET3 );
    if ( wc <= 0 ) {
        DPRINTF ( "ERROR FPWR(%x, 0x0F10, %d)\n", configadr, gpio_val );
    }
    return wc;
}


///////////////////////////////////////////////////////////////////////////////

using namespace iit::ecat;

ec_timing_t     timing;

int main(int argc, char **argv)
{
    int ret;

    set_signal_handler();

#ifdef __COBALT__
    
    int policy = SCHED_FIFO;
    struct sched_param  schedparam;
    schedparam.sched_priority = sched_get_priority_max(policy);
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &schedparam);

    /* Prevent any memory-swapping for this program */
    ret = mlockall(MCL_CURRENT | MCL_FUTURE);
    if ( ret < 0 ) {
        printf("mlockall failed (ret=%d) %s\n", ret, strerror(ret));
        return 0;
    }
    /*
     * This is a real-time compatible printf() package from
     * Xenomai's RT Development Kit (RTDK), that does NOT cause
     * any transition to secondary (i.e. non real-time) mode when
     * writing output.
     */
//    rt_print_auto_init(1);
#endif

    int         expected_wkc;
    uint32_t    sync_cycle_time_ns = 0;         //  no dc 
    uint32_t    sync_cycle_offset_ns = 0;       //  0ms


    if ( ! ( argc == 5 || argc == 7 ) ) {
    printf("Usage: %s ifname slave_pos filename password\nifname = {eth0,rteth0}\n", argv[0]);
        return 0;
    }

    if ( initialize(argv[1], true) <= 0) {
        finalize();
        return 0;
    }

    int slave_pos = atoi(argv[2]);
    std::string bin_file((const char*)argv[3]);
    int password  = strtol(argv[4], 0, 16);

    esc_gpio_ll_wr ( 0x1000+slave_pos, 0x0 );
    sleep(1);
    esc_gpio_ll_wr ( 0x1000+slave_pos, 0x5 );
    
    // ask all slaves to go in INIT before update one
    req_state_check(0, EC_STATE_INIT);
    // first boot state request is handled by application that jump to bootloader
    // we do NOT have a state change in the slave
    req_state_check(slave_pos, EC_STATE_BOOT);
    // second boot state request is handled by bootloader
    // now the slave should go in BOOT state
    if ( req_state_check(slave_pos, EC_STATE_BOOT) !=  EC_STATE_BOOT) {
        DPRINTF("Slave %d not changed to BOOT state.\n", slave_pos);
        return 0;
    }
    
    //uint16_t flash_cmd = 0x00E1;
    //ec_SDOwrite ( slave_pos, 0x8000, 0x1, false, sizeof ( flash_cmd ), &flash_cmd, EC_TIMEOUTRXM * 30 ); // 21 secs
    //sleep(1);

    DPRINTF("%d %s 0x%04X\n", slave_pos, bin_file.c_str(), password);
    send_file(slave_pos, bin_file.c_str(), password);
    if ( argc == 7 ) {
        bin_file = (const char*)argv[5];
        password  = strtol(argv[6], 0, 16);
        DPRINTF("%d %s 0x%04X\n", slave_pos, bin_file.c_str(), password);
        send_file(slave_pos, bin_file.c_str(), password);
    }
    
    req_state_check(slave_pos, EC_STATE_INIT);

    finalize();


    initialize(argv[1], true);
    expected_wkc = operational(sync_cycle_time_ns, sync_cycle_offset_ns);
    finalize();

    return 0;
}
