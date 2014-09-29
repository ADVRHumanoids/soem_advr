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



namespace iit {
namespace ecat {

class EscWrapper {
public:
    EscWrapper(const ec_slavet& slave_descriptor);
    virtual ~EscWrapper() {};

    //void print_IOmap();

    virtual void readPDO() = 0;
    virtual void writePDO() = 0;
    const uint8_t* getRawData() const;
protected:
    uint16_t alias;
    uint16_t position;
    uint32_t vendor_id;
    uint32_t product_code;


    uint8_t   * inputs, * outputs;
    uint32_t  nbytes_in, nbytes_out;


};




template<class ESCTypes>
class BasicEscWrapper : public EscWrapper
{
public:
    typedef typename ESCTypes::pdo_rx pdo_rx_t;
    typedef typename ESCTypes::pdo_tx pdo_tx_t;
public:
    BasicEscWrapper(const ec_slavet& slave_descriptor) :
        EscWrapper(slave_descriptor)
    {}

    void readPDO();
    void writePDO();

    const pdo_rx_t& getRxPDO() const;
    const pdo_tx_t& getTxPDO() const;

    void setTxPDO(const pdo_tx_t&);

protected:
    pdo_rx_t rx_pdo;
    pdo_tx_t tx_pdo;
};


#define TEMPL template<class ESCTypes>
#define CLASS BasicEscWrapper<ESCTypes>
#define SIGNATURE(type) TEMPL inline type CLASS


SIGNATURE(void)::readPDO()
{
    memcpy((void*)&rx_pdo, inputs, nbytes_in);
}

SIGNATURE(void)::writePDO()
{
    memcpy((void*)outputs, &tx_pdo, nbytes_out);
}

SIGNATURE(const typename CLASS::pdo_rx_t&)::getRxPDO() const
{
    return rx_pdo;
}

SIGNATURE(const typename CLASS::pdo_tx_t&)::getTxPDO() const
{
    return tx_pdo;
}


SIGNATURE(void)::setTxPDO(const pdo_tx_t& tx)
{
    tx_pdo = tx;
}


#undef SIGNATURE
#undef CLASS
#undef TEMPL


}
}

#endif /* IIT_IO_ECAT_SLAVE_WRAPPER_H_ */
