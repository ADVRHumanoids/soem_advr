#include <signal.h>
#include <sys/mman.h>
#include <execinfo.h>
#include <stdio.h>

#ifdef __COBALT__
    #include <sys/mman.h>
    #include <xenomai/init.h>
#endif

#include <iit/ecat/ec_master_iface.h>


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
    //signal(SIGXCPU, warn_upon_switch);
    signal(SIGDEBUG, warn_upon_switch);
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

int main(int argc, char * const argv[])
{
    int                 policy, ret;
    struct sched_param  schedparam;
    
    set_signal_handler();
    
    policy = SCHED_OTHER;

#ifdef __COBALT__
    xenomai_init(&argc, &argv);

    policy = SCHED_FIFO;

    mlockall(MCL_CURRENT | MCL_FUTURE);
#endif
    
    schedparam.sched_priority = sched_get_priority_min(policy);
    pthread_setschedparam(pthread_self(), policy, &schedparam);

    int         wkc, expected_wkc;
    uint32_t    sync_cycle_time_ns = 1e9;         //  0 no dc 
    uint32_t    sync_cycle_offset_ns = 1e9;       //  0ms
    ec_timing_t timing;

    if ( argc > 2  ) {
    printf("Usage: %s ifname \nifname = {eth0,rteth0}\n", argv[0]);
        return 0;
    }

    if ( initialize(argv[1], true) <= 0) {
        finalize();
        return 0;
    }
    
    expected_wkc = operational ( sync_cycle_time_ns, sync_cycle_offset_ns );
    
    while ( run_loop ) {
         
        wkc = recv_from_slaves ( timing );
        if ( wkc =! expected_wkc ) {
            printf("OOps wkc =! expected_wkc %d =! %d\n", wkc , expected_wkc);
        } 
         
    }
    
    
    finalize();


    return 0;
}
