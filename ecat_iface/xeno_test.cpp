#include <signal.h>
#include <sys/mman.h>
#include <execinfo.h>
#include <stdio.h>

#ifdef __XENO__
    #include <rtdk.h>
#endif

#include "ec_master_iface.h"


#define CONTROLLER_LOOP_PERIOD_NS 1000000
#define CONTROLLER_LOOP_OFFSET_NS  400000

int loop = 1;

/**********************************************************
 *  slave ecat memory map
 *  
 */
typedef struct {
    uint16_t    _type;
    int32_t     _value;
    uint64_t    _ts;
} __attribute__((__packed__)) rx_pdo_t;

typedef union {
    rx_pdo_t    rx_pdo; 
    uint8_t     buffer[sizeof(rx_pdo_t)];
} rx_sm_t; 

typedef struct tx_pdo {
    uint8_t     _bit_0:1;
    uint8_t     _bit_1:1;
    uint8_t     _bit_2:1;
    uint8_t     _bit_3:1;
    uint8_t     _bit_4:1;
    uint8_t     _bit_5:1;
    uint8_t     _bit_6:1;
    uint8_t     _bit_7:1;
    uint8_t     _bits;
    int8_t      _sint;
    uint8_t     _usint;
    int16_t     _int;
    uint16_t    _uint;
    int32_t     _dint;
    uint32_t    _udint;
    int64_t     _lint;
    uint64_t    _ulint;
    float       _real;
} __attribute__((__packed__)) tx_pdo_t;

typedef union {
    tx_pdo_t    tx_pdo;
    uint8_t     buffer[sizeof(tx_pdo_t)];
} tx_sm_t; 
/*
 * 
 *************************************************************/

rx_sm_t slave_rx;
tx_sm_t slave_tx;



static void warn_upon_switch(int sig __attribute__((unused)))
{
    // handle rt to nrt contex switch
    void *bt[32];
    int nentries;

    /* Dump a backtrace of the frame which caused the switch to
       secondary mode: */
    nentries = backtrace(bt,sizeof(bt)/sizeof(bt[0]));
    // dump backtrace 
    //backtrace_symbols_fd(bt,nentries,fileno(stdout));
}


static void shutdown(int sig __attribute__((unused)))
{
    loop = 0;
    printf("got signal .... Shutdown\n");
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




int main(int argc, char **argv)
{

    set_signal_handler();

#ifdef __XENO__
    /* Prevent any memory-swapping for this program */
    int ret = mlockall(MCL_CURRENT | MCL_FUTURE);
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

    int expected_wc = initialize("eth1", CONTROLLER_LOOP_PERIOD_NS, CONTROLLER_LOOP_OFFSET_NS);

    if ( ! expected_wc ) {
        finalize();
        return 0;
    }

    while ( loop ) {

        slave_rx.rx_pdo._ts = get_time_ns();

        int ret = send_to_slaves(slave_rx.buffer, sizeof(slave_rx));
        if ( ret < 0 ) {
            printf("wkc %d\n", -ret);
        }

        print_ecat_IOmap();

        timespec delay = { 0, 1000000};
        clock_nanosleep(CLOCK_MONOTONIC, 0, &delay, NULL);

    }

    finalize();

    return 0;
}
