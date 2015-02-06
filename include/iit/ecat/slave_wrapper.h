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

#include <string>
#include <map>
#include <exception>

#include <iit/ecat/utils.h>

#define DTYPE_INTEGER8          0x0002
#define DTYPE_INTEGER16         0x0003
#define DTYPE_INTEGER32         0x0004
#define DTYPE_UNSIGNED8         0x0005
#define DTYPE_UNSIGNED16        0x0006
#define DTYPE_UNSIGNED32        0x0007
#define DTYPE_REAL32            0x0008
#define DTYPE_VISIBLE_STRING    0x0009
#define DTYPE_INTEGER64         0x0015
#define DTYPE_UNSIGNED64        0x001B
#define DTYPE_REAL64            0x0011

#define ATYPE_RO 17
#define ATYPE_RW 18


namespace iit {
namespace ecat {

/* Possible error codes returned */
enum ec_wrp_err: int {
    /* No error */
    EC_WRP_OK         = 0,
    /* Function failed. */
    EC_WRP_NOK,
    EC_WRP_SDO_RO,
    EC_WRP_SDO_MISMATCH,
    EC_WRP_SDO_NOTEXIST,
    EC_WRP_SDO_SET_FAIL,
    EC_WRP_SDO_GET_FAIL,
    EC_WRP_SDO_GET_CB_FAIL,
};

typedef struct {
    int index;
    int subindex;
    int datatype;
    int bitlength;
    int access;
    const char * name;
    void * data;
} objd_t;

template <typename T>
inline int check_datatype(int datatype, T t) {

    switch ( datatype ) {
        case DTYPE_INTEGER8:
            if ( typeid(T) != typeid(int8_t) ) {
                return 0;
            }
            break;
        case DTYPE_INTEGER16:
            if ( typeid(T) != typeid(int16_t) ) {
                return 0;
            }
            break;
        case DTYPE_INTEGER32:
            if ( typeid(T) != typeid(int32_t) ) {
                return 0;
            }
            break;
        case DTYPE_INTEGER64:
            if ( typeid(T) != typeid(int64_t) ) {
                return 0;
            }
            break;
        case DTYPE_UNSIGNED8:
            if ( typeid(T) != typeid(uint8_t) ) {
                return 0;
            }
            break;
        case DTYPE_UNSIGNED16:
            if ( typeid(T) != typeid(uint16_t) ) {
                return 0;
            }
            break;
        case DTYPE_UNSIGNED32:
            if ( typeid(T) != typeid(uint32_t) ) {
                return 0;
            }
            break;
        case DTYPE_UNSIGNED64:
            if ( typeid(T) != typeid(uint64_t) ) {
                return 0;
            }
            break;
        case DTYPE_REAL32:
            if ( typeid(T) != typeid(float) ) {
                return 0;
            }
            break;
        case DTYPE_REAL64:
            if ( typeid(T) != typeid(double) ) {
                return 0;
            }
            break;
        case DTYPE_VISIBLE_STRING:
            if ( typeid(T) != typeid(char*) ) {
                return 0;
            }
            break;
        default:
            return EC_WRP_NOK;
    }

    return EC_WRP_OK;

}


class EscError : public std::runtime_error {
public:
    EscError (int err_no, const std::string& what_arg) :
        std::runtime_error(what_arg) { error_num = err_no; }
    EscError (int err_no, const char* what_arg):
        std::runtime_error(what_arg) { error_num = err_no; }
private:
    int error_num;
};


class EscWrapper {
public:
    EscWrapper(const ec_slavet& slave_descriptor);
    virtual ~EscWrapper() {};

    //void print_IOmap();

    virtual void readPDO() = 0;
    virtual void writePDO() = 0;

    virtual const objd_t * get_SDO_objd() = 0;

    const uint8_t* getRawData() const;

    const uint16_t get_configadr() { return configadr;}

protected:
    uint16_t alias;
    uint16_t position;
    uint16_t configadr;
    uint32_t vendor_id;
    uint32_t product_code;


    uint8_t   * inputs, * outputs;
    uint32_t  nbytes_in, nbytes_out;


};




template<class EscPDOTypes, class EscSDOTypes>
class BasicEscWrapper : public EscWrapper {

public:
    typedef typename EscPDOTypes::pdo_rx    pdo_rx_t;
    typedef typename EscPDOTypes::pdo_tx    pdo_tx_t;
    typedef EscSDOTypes                     sdo_t;

public:
    BasicEscWrapper(const ec_slavet& slave_descriptor) :
    EscWrapper(slave_descriptor) {}

    void init_sdo_lookup(void);

    sdo_t& getSDO_ptr();
    objd_t * getObjd_ptr(const char * name);

    template<typename T>
    int set_SDO_byname(const char * name, T t);
    int set_SDO_byname(const char * name);

    template<typename T>
    int get_SDO_byname(const char * name, T &t);
    int get_SDO_byname(const char * name);

    const pdo_rx_t& getRxPDO() const;
    const pdo_tx_t& getTxPDO() const;

    void setTxPDO(const pdo_tx_t&);

protected:

    // callbacks
    virtual void on_readPDO(void)       {}
    virtual void on_writePDO(void)      {}
    virtual int on_setSDO(const objd_t * sdo)    { return EC_WRP_OK;}
    virtual int on_getSDO(const objd_t * sdo)    { return EC_WRP_OK;}

protected:

    sdo_t    sdo;

    pdo_rx_t        rx_pdo;
    pdo_tx_t        tx_pdo;

    std::map<std::string, const objd_t*> sdo_look_up;

private:

    void readPDO(void);
    void writePDO(void);

    int set_SDO(int slave_pos, const objd_t *sdo);
    int get_SDO(int slave_pos, const objd_t *sdo);
};


#define TEMPL template<class EscPDOTypes, class EscSDOTypes>
#define CLASS BasicEscWrapper<EscPDOTypes,EscSDOTypes>
#define SIGNATURE(type) TEMPL inline type CLASS


SIGNATURE(void)::readPDO()
{
    memcpy((void*)&rx_pdo, inputs, nbytes_in);
    on_readPDO();
}

SIGNATURE(void)::writePDO()
{
    on_writePDO();
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

SIGNATURE(void)::init_sdo_lookup(void) {

    const objd_t * sdo = get_SDO_objd();
    while ( sdo && sdo->index ) {
        sdo_look_up[sdo->name] = sdo;
        get_SDO(position, sdo);
        sdo ++;
    }
}

SIGNATURE(typename CLASS::sdo_t&)::getSDO_ptr() {
    return sdo;
}

SIGNATURE(objd_t *)::getObjd_ptr(const char * name) {
    objd_t * sdo;
    try {
        sdo = sdo_look_up.at(name);
    } catch ( const std::out_of_range& oor ) {
        return 0;
    }
    return sdo;
}

SIGNATURE(int)::set_SDO_byname(const char * name) {

    // look up name in SDOs
    const objd_t * sdo;
    try {
        sdo = sdo_look_up.at(name);
    } catch ( const std::out_of_range& oor ) {
        throw EscError(EC_WRP_SDO_NOTEXIST, sdo->name);
    }
    if ( sdo->access == ATYPE_RO ) {
        DPRINTF("ERROR sdo obj is READ_ONLY %s\n", sdo->name);
        throw EscError(EC_WRP_SDO_RO, sdo->name);
    }
    if ( set_SDO(position, sdo) != EC_WRP_OK ) {
        throw EscError(EC_WRP_SDO_SET_FAIL, sdo->name);
    }

    return EC_WRP_OK;
}

TEMPL
template<typename T>
inline int CLASS::set_SDO_byname(const char * name, T t) {

    // look up name in SDOs
    const objd_t * sdo;
    try {
        sdo = sdo_look_up.at(name);
    } catch ( const std::out_of_range& oor ) {
        throw EscError(EC_WRP_SDO_NOTEXIST, sdo->name);
    }
    if ( sdo->access == ATYPE_RO ) {
        DPRINTF("ERROR sdo obj is READ_ONLY %s\n", sdo->name);
        throw EscError(EC_WRP_SDO_RO, sdo->name);
    }
    if ( check_datatype(sdo->datatype, t) != EC_WRP_OK ) {
        DPRINTF("ERROR set_SDO_byname %s datatype mismatch %s\n", sdo->name, typeid(T).name() );
        throw EscError(EC_WRP_SDO_MISMATCH, sdo->name);
    }
    *((T*)sdo->data) = t;
    if ( set_SDO(position, sdo) != EC_WRP_OK ) {
        throw EscError(EC_WRP_SDO_SET_FAIL, sdo->name);
    }

    return EC_WRP_OK;
}

SIGNATURE(int)::get_SDO_byname(const char * name) {

    // look up name in SDOs
    const objd_t * sdo;
    try {
        sdo = sdo_look_up.at(name);
    } catch ( const std::out_of_range& oor ) {
        throw EscError(EC_WRP_SDO_NOTEXIST, sdo->name);
    }
    if ( get_SDO(position, sdo) != EC_WRP_OK) {
        throw EscError(EC_WRP_SDO_GET_FAIL, sdo->name);
    }

    return EC_WRP_OK;
}

TEMPL
template<typename T>
inline int CLASS::get_SDO_byname(const char * name, T &t) {

    int ret;
    // look up name in SDOs
    const objd_t * sdo;
    try {
        sdo = sdo_look_up.at(name);
    } catch ( const std::out_of_range& oor ) {
        throw EscError(EC_WRP_SDO_NOTEXIST, sdo->name);
    }
    if ( check_datatype(sdo->datatype, t) != EC_WRP_OK ) {
        DPRINTF("ERROR get_SDO_byname %s datatype mismatch %s\n", sdo->name, typeid(T).name() );
        throw EscError(EC_WRP_SDO_MISMATCH, sdo->name);
    }
    if ( get_SDO(position, sdo) != EC_WRP_OK) {
        throw EscError(EC_WRP_SDO_GET_FAIL, sdo->name);
    }
    t = *((T*)sdo->data);

    return EC_WRP_OK;  
}

SIGNATURE(int)::set_SDO(int slave_pos, const objd_t *sdo) {

    char * err;
    ec_errort   ec_error;
    int final_size, wkc;

    if ( !sdo ) {
        return 0;
    }

    if ( on_setSDO(sdo) != EC_WRP_OK ) {
        return EC_WRP_SDO_GET_CB_FAIL;
    }

    //DPRINTF("set_SDO %s [%d.0x%04X.0x%02X]\n", sdo->name, slave_pos, sdo->index, sdo->subindex);
    final_size = sdo->bitlength/8;
    //return workcounter from last slave response
    wkc = ec_SDOwrite(slave_pos, sdo->index, sdo->subindex, false, final_size, sdo->data, EC_TIMEOUTRXM);
    if ( wkc <= 0 || final_size != sdo->bitlength/8 ) {
        // ERROR ...
        DPRINTF("Slave %d >> ", slave_pos);
        if ( wkc <= 0 ) {
            err =  ec_elist2string();
            DPRINTF("Ec_error : %s\n", err);
        } else {
            DPRINTF("SDO write fail : %d != %d\n", final_size, sdo->bitlength/8);
        }
        return EC_WRP_SDO_SET_FAIL;
    }
    // OK ...
    return EC_WRP_OK; 
}

SIGNATURE(int)::get_SDO(int slave_pos, const objd_t *sdo) {

    char * err;
    ec_errort   ec_error;
    int final_size, wkc;

    if ( !sdo ) {
        return 0;
    }

    //DPRINTF("get_SDO %s [%d.0x%04X.0x%02X]\n", sdo->name, slave_pos, sdo->index, sdo->subindex);
    final_size = sdo->bitlength/8;
    //return workcounter from last slave response
    wkc = ec_SDOread(slave_pos, sdo->index, sdo->subindex, false, &final_size, sdo->data, EC_TIMEOUTRXM);
    if ( wkc <= 0 || final_size != sdo->bitlength/8 ) {
        DPRINTF("Slave %d >> ", slave_pos);
        if ( wkc <= 0 ) {
            err =  ec_elist2string();
            DPRINTF("Ec_error : %s\n", err);
        } else {
            DPRINTF("SDO read fail : %d != %d\n", final_size, sdo->bitlength/8);
        }
        return EC_WRP_SDO_GET_FAIL;
    }
    // OK ....

    on_getSDO(sdo);

    return EC_WRP_OK;; 
}


#undef SIGNATURE
#undef CLASS
#undef TEMPL


}
}

#endif /* IIT_IO_ECAT_SLAVE_WRAPPER_H_ */
