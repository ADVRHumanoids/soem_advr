#include <signal.h>
#include <sys/mman.h>
#include <execinfo.h>
#include <stdio.h>

#ifdef __XENO__
    #include <rtdk.h>
#endif

#include "ec_master_iface.h"
#include "ec_slave_type.h"



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

using namespace ec_master_iface;

input_slave_t   slave_input[4];
output_slave_t  slave_output[4];
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

    int         expected_wkc;
    uint64_t    sync_cycle_time_ns = 1e6;       //   1ms
    uint64_t    sync_cycle_offset_ns = 500e6;   // 500ms

    if ( argc > 1 ) {
        expected_wkc = initialize(argv[1], &sync_cycle_time_ns, &sync_cycle_offset_ns);
    } else {
        printf("Usage: %s ifname\nifname = {eth0,rteth0} for example\n", argv[0]);
        return 0;
    }

    if ( expected_wkc < 0) {
        finalize();
        return 0;
    }

    struct timespec sleep_time = { 0, 400000 };
    uint64_t        t_prec = 0, t_now, dt;
    uint64_t        rtt = 0;
    int wkc;
    int retry;

    stat_t s_loop, s_rtt, s_sleep;

    // warm up ... just for stat_t accumulators
    ret = recv_from_slaves(slave_output, &timing);
    if ( ret < 0 ) { DPRINTF("fail recv_from_slaves"); }
    t_prec = get_time_ns();
    sleep_time.tv_nsec = sync_cycle_time_ns - (timing.recv_dc_time % sync_cycle_time_ns) - 150000;
    clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, NULL);
    slave_input[0].test._ts = get_time_ns();
    wkc = send_to_slaves(slave_input);


    while ( run_loop ) {

        // wait for cond_signal 
        // ecat_thread sync with DC
        ret = recv_from_slaves(slave_output, &timing);
        if ( ret < 0 ) { DPRINTF("fail recv_from_slaves"); }

        t_now = get_time_ns();
        dt = t_now - t_prec;
        t_prec = t_now;  
        s_loop(dt);
        //DPRINTF("== loop %d\n", dt); 
        
        //DPRINTF("@@ %d %u\n", slave_output[0].test._sint , slave_output[0].test._usint);
        rtt = get_time_ns() - slave_output[0].test._ulint;
        s_rtt(rtt);
        //DPRINTF("@@ rtt %llu\n", rtt); 
        //DPRINTF(">> %lld %lld %llu\n", timing.recv_dc_time % sync_cycle_time_ns , timing.offset, timing.loop_time); 



        sleep_time.tv_nsec = sync_cycle_time_ns - (timing.recv_dc_time % sync_cycle_time_ns) - 150000;
        //DPRINTF("++ sleep %ld\n", sleep_time.tv_nsec);
        s_sleep(sleep_time.tv_nsec);
        clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, NULL);

        slave_input[0].test._ts = get_time_ns();
        wkc = send_to_slaves(slave_input);

        retry = 2;
        while ( wkc < expected_wkc && retry--) {
            DPRINTF("## wkc %d\n", wkc);
            wkc = send_to_slaves(slave_input);
        }

        if ( ! retry) {
            finalize();
            break;
        }


    }

    finalize();

    DPRINTF("loop\n");
    print_stat(s_loop);
    DPRINTF("rtt\n");
    print_stat(s_rtt);
    DPRINTF("sleep\n");
    print_stat(s_sleep);

    return 0;
}
