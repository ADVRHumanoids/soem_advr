/*
 * slave_wrapper.h
 *
 *  Created on: Jun 25, 2014
 *      Author: Marco Frigerio, based on Alessio Margan's work
 */

#ifndef IIT_IO_ECAT_SLAVE_WRAPPER_H_
#define IIT_IO_ECAT_SLAVE_WRAPPER_H_

#include <soem-1.3.0/ethercattype.h>
#include <soem-1.3.0/nicdrv.h>
#include <soem-1.3.0/ethercatbase.h>
#include <soem-1.3.0/ethercatmain.h>
#include <soem-1.3.0/ethercatdc.h>
#include <soem-1.3.0/ethercatcoe.h>
#include <soem-1.3.0/ethercatfoe.h>
#include <soem-1.3.0/ethercatconfig.h>
#include <soem-1.3.0/ethercatprint.h>

#include "ec_slave_type.h"

namespace iit {
namespace ecat {

class EscWrapper {
public:
    EscWrapper(const ec_slavet& slave_descriptor);
    virtual ~EscWrapper() {};

    //void print_IOmap();

    virtual void readPDO() = 0;
    const uint8_t* getRawData() const;
protected:
    uint16_t alias;
    uint16_t position;
    uint32_t vendor_id;
    uint32_t product_code;


    uint8_t   * inputs, * outputs;
    uint32_t  nbytes_in, nbytes_out;


};


class EscHyQ : public EscWrapper
{
public:
    EscHyQ(const ec_slavet& slave_descriptor) :
        EscWrapper(slave_descriptor)
    {}

    ~EscHyQ() {}

    void readPDO();

    const hyq_io_pdo::rx_pdo_t& getPDO() const;

private:
    hyq_io_pdo::rx_pdo_t dataChunk;
};


inline const hyq_io_pdo::rx_pdo_t& EscHyQ::getPDO() const
{
    return dataChunk;
}



}
}

#endif /* IIT_IO_ECAT_SLAVE_WRAPPER_H_ */
