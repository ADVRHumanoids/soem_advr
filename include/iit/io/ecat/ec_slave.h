/*
   EC_slave.h

   Copyright (C) 2013 Italian Institute of Technology

   Developer:
       Alessio Margan (2013-, alessio.margan@iit.it)

*/

/**
 * @defgroup Esc
 *
 * @brief ethercat slave controller
 *
 * @author Alessio Margan (2013-, alessio.margan@iit.it)
*/

#ifndef __EC_SLAVE_H__
#define __EC_SLAVE_H__

// SOEM
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

#include <map>
#include <boost/shared_ptr.hpp>

#include "utils.h"

#define IIT_VENDOR_ID       0x00000298

#define IIT_Advr_test_v0_3  0x00000003
#define IIT_Advr_HyQ_IO     0x00000004
#define IIT_Advr_HyQ_Valve  0x00000005

class Esc;
typedef boost::shared_ptr<Esc>  EscPtr;
typedef std::map<int, EscPtr>   esc_map_t;

extern esc_map_t esc;

void slave_factory(ec_slavet slaves[], int slavecount);



/**
 *
 */
class Esc {

public:
    Esc(ec_slavet slave);
    virtual ~Esc();

    void print_IOmap();

    uint16_t alias;
    uint16_t position;
    uint32_t vendor_id;
    uint32_t product_code;


    uint8_t   * inputs, * outputs;
    uint32_t  nbytes_input, nbytes_output;

    // slave input is master output
    template <typename T> void set_slave_inputs(T &slave_inputs) {

        memcpy((void*)outputs, &slave_inputs, nbytes_output);
    }

    // slave output is master input
    template <typename T> void get_slave_outputs(T &slave_outputs) {

        memcpy((void*)&slave_outputs, inputs, nbytes_input);
    }
};

/**
 *
 */
class Esc_test : public Esc {

public:
    Esc_test(ec_slavet slave );
    virtual ~Esc_test() {}

};

/**
 *
 */
class Esc_HyQ_IO : public Esc {

public:
    Esc_HyQ_IO(ec_slavet slave);
    virtual ~Esc_HyQ_IO() {}

};

/**
 *
 */
class Esc_HyQ_Valve : public Esc {

public:
    Esc_HyQ_Valve(ec_slavet slave);
    virtual ~Esc_HyQ_Valve() {}

};


#endif
