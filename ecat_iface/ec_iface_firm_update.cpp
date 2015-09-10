#include <signal.h>
#include <sys/mman.h>
#include <execinfo.h>
#include <stdio.h>

#ifdef __XENO__
    #include <rtdk.h>
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
#ifdef __XENO__
    // call pthread_set_mode_np(0, PTHREAD_WARNSW) to cause a SIGXCPU
    // signal to be sent when the calling thread involontary switches to secondary mode
    signal(SIGXCPU, warn_upon_switch);
#endif
}

///////////////////////////////////////////////////////////////////////////////

using namespace iit::ecat;

ec_timing_t     timing;

int main(int argc, char **argv)
{
    int ret;

    set_signal_handler();

#ifdef __XENO__
    
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
    rt_print_auto_init(1);
#endif

    char *      firmware_file = 0;
    int         expected_wkc;
    //uint64_t    sync_cycle_time_ns = 1e6;     //  1ms
    uint64_t    sync_cycle_time_ns = 0;         //  no dc 
    uint64_t    sync_cycle_offset_ns = 0;       //  0ms


    if ( argc > 1 ) {
    } 
    if ( argc > 2 ) {
        firmware_file = argv[2];
    }
    if ( argc != 5) {
    printf("Usage: %s ifname slave_pos filename password\nifname = {eth0,rteth0}\n", argv[0]);
        return 0;
    }

    if ( initialize(argv[1]) <= 0) {
        finalize();
        return 0;
    }


    int slave_pos = atoi(argv[2]);
    int password  = strtol(argv[4], 0, 16);

    DPRINTF("%d %s 0x%04X\n", slave_pos, argv[3], password);
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

    send_file(slave_pos, argv[3], password);

    req_state_check(slave_pos, EC_STATE_INIT);

    finalize();


    initialize(argv[1]);
    expected_wkc = operational(&sync_cycle_time_ns, &sync_cycle_offset_ns);
    finalize();

    return 0;
}
