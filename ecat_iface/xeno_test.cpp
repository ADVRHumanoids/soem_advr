#include <signal.h>
#include <sys/mman.h>
#include <execinfo.h>
#include <stdio.h>

#ifdef __XENO__
    #include <rtdk.h>
#endif

#include "ec_master_iface.h"
#include "ec_slave_type.h"


#define CONTROLLER_LOOP_PERIOD_NS     1000000
#define CONTROLLER_LOOP_OFFSET_NS   500000000


static int loop = 1;

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
    loop = 0;
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

    int expected_wkc;

    if ( argc > 1 ) {
        expected_wkc = initialize(argv[1], CONTROLLER_LOOP_PERIOD_NS, CONTROLLER_LOOP_OFFSET_NS);
    } else {
        printf("Usage: %s ifname\nifname = {eth0,rteth0} for example\n", argv[0]);
        return 0;
    }

    if ( expected_wkc < 0) {
        finalize();
        return 0;
    }

    struct timespec sleep_time = { 0, 0 };
    uint64_t        t_prec, t_now, dt;
    int rtt = 0;
    int wkc;
    int retry;

    while ( loop ) {

        t_now = get_time_ns();
        dt = t_now - t_prec;
        t_prec = t_now;  
        DPRINTF("== loop %d\n", dt); 

        // wait for cond_signal
        ret = recv_from_slaves(slave_output, &timing);
        if ( ret < 0 ) {
            DPRINTF("fail recv_from_slaves");
        }

        //DPRINTF("@@ %d %u\n", slave_output[0].test._sint , slave_output[0].test._usint);
        rtt = (int)(get_time_ns() - slave_output[0].test._ulint);
        DPRINTF("@@ rtt %d\n", rtt); 

        DPRINTF(">> %lld %lld %llu\n", timing.recv_dc_time % CONTROLLER_LOOP_PERIOD_NS , timing.offset, timing.loop_time); 

        sleep_time.tv_nsec = CONTROLLER_LOOP_PERIOD_NS - (timing.recv_dc_time % CONTROLLER_LOOP_PERIOD_NS) - 150000;
        DPRINTF("++ sleep %ld\n", sleep_time.tv_nsec);

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

    return 0;
}
