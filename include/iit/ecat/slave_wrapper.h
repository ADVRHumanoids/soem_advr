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
    /* erros */
    EC_WRP_NOK,
    EC_WRP_SDO_RO,
    EC_WRP_SDO_MISMATCH,
    EC_WRP_SDO_NOTEXIST,
    EC_WRP_SDO_WRITE_FAIL,
    EC_WRP_SDO_READ_FAIL,
    EC_WRP_SDO_WRITE_CB_FAIL,
    EC_WRP_INVALID_DTYPE,
    EC_WRP_PDO_TX_SIZE,
    EC_WRP_PDO_RX_SIZE,
};

#define STRFY(x)    #x

static const char * ec_wrp_err_msg[] = {
    STRFY(EC_WRP_OK),
    /* erros */
    STRFY(EC_WRP_NOK),
    STRFY(EC_WRP_SDO_RO),
    STRFY(EC_WRP_SDO_MISMATCH),
    STRFY(EC_WRP_SDO_NOTEXIST),
    STRFY(EC_WRP_SDO_WRITE_FAIL),
    STRFY(EC_WRP_SDO_READ_FAIL),
    STRFY(EC_WRP_SDO_WRITE_CB_FAIL),
    STRFY(EC_WRP_INVALID_DTYPE),
    STRFY(EC_WRP_PDO_TX_SIZE),
    STRFY(EC_WRP_PDO_RX_SIZE),
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


class EscWrpError : public std::runtime_error {
public:
    EscWrpError (int err_no, const std::string & what_arg) : error_num(err_no),
        std::runtime_error(make_what(what_arg,err_no))
    {}

    EscWrpError (int err_no, const char* what_arg) : error_num(err_no),
        std::runtime_error(make_what(std::string(what_arg),err_no))
    {}

private:
    std::string make_what(std::string what_arg, int err_no) {
        std::ostringstream msg;
        msg << "EC_Wrp_Error " << err_no << ":" << ec_wrp_err_msg[err_no] << " " << what_arg;
        return msg.str();
    }
    int error_num;
};


template <typename T>
inline int check_datatype(int datatype, T t) {

    switch ( datatype ) {
        case DTYPE_INTEGER8:
            if ( typeid(T) != typeid(int8_t) ) {
                return EC_WRP_NOK;
            }
            break;
        case DTYPE_INTEGER16:
            if ( typeid(T) != typeid(int16_t) ) {
                return EC_WRP_NOK;
            }
            break;
        case DTYPE_INTEGER32:
            if ( typeid(T) != typeid(int32_t) ) {
                return EC_WRP_NOK;
            }
            break;
        case DTYPE_INTEGER64:
            if ( typeid(T) != typeid(int64_t) ) {
                return EC_WRP_NOK;
            }
            break;
        case DTYPE_UNSIGNED8:
            if ( typeid(T) != typeid(uint8_t) ) {
                return EC_WRP_NOK;
            }
            break;
        case DTYPE_UNSIGNED16:
            if ( typeid(T) != typeid(uint16_t) ) {
                return EC_WRP_NOK;
            }
            break;
        case DTYPE_UNSIGNED32:
            if ( typeid(T) != typeid(uint32_t) ) {
                return EC_WRP_NOK;
            }
            break;
        case DTYPE_UNSIGNED64:
            if ( typeid(T) != typeid(uint64_t) ) {
                return EC_WRP_NOK;
            }
            break;
        case DTYPE_REAL32:
            if ( typeid(T) != typeid(float) ) {
                return EC_WRP_NOK;
            }
            break;
        case DTYPE_REAL64:
            if ( typeid(T) != typeid(double) ) {
                return EC_WRP_NOK;
            }
            break;
        case DTYPE_VISIBLE_STRING:
            if ( typeid(T) != typeid(char*) ) {
                return EC_WRP_NOK;
            }
            break;
        default:
            throw EscWrpError(EC_WRP_INVALID_DTYPE, " ");
    }

    return EC_WRP_OK;

}



class EscWrapper {
public:
    EscWrapper(const ec_slavet& slave_descriptor);
    virtual ~EscWrapper() {};

    //void print_IOmap();

    virtual void readPDO() = 0;
    virtual void writePDO() = 0;

    virtual const objd_t * get_SDOs(void) = 0;

    virtual uint16_t get_ESC_type(void) = 0;

    const uint8_t* getRawData(void) const;

    const uint16_t get_configadr() { return configadr;}

    void readErrReg(void);
    void resetError(void);

protected:
    uint16_t alias;
    uint16_t position;
    uint16_t configadr;
    uint32_t vendor_id;
    uint32_t product_code;

    uint16_t topology, active_ports;
    
    // error counter on port[n]
    uint16  invalid_frame[4], rx_error[4], fw_error[4], lost_link[4];

    uint8_t   * inputs, * outputs;
    uint32_t  nbytes_in, nbytes_out;

    const ec_slavet * ec_slave_desc;
};




template<class EscPDOTypes, class EscSDOTypes>
class BasicEscWrapper : public EscWrapper {

public:
    typedef typename EscPDOTypes::pdo_rx    pdo_rx_t;
    typedef typename EscPDOTypes::pdo_tx    pdo_tx_t;
    typedef EscSDOTypes                     sdo_t;

public:
    BasicEscWrapper(const ec_slavet& slave_descriptor) :
    EscWrapper(slave_descriptor) {

        char err_msg[256];

        int actual_state = ec_statecheck(position,EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

//         if ( actual_state != EC_STATE_INIT && actual_state != EC_STATE_BOOT ) {
//                  
//             if ( nbytes_in  != sizeof(pdo_rx_t) ) {
//                 snprintf(err_msg, sizeof(err_msg), "nbytes_in %d != %d pdo_rx_t", nbytes_in, sizeof(pdo_rx_t));
//                 throw EscWrpError(EC_WRP_PDO_RX_SIZE, err_msg);
//             }
//             if ( nbytes_out != sizeof(pdo_tx_t) ) {
//                 snprintf(err_msg, sizeof(err_msg), "nbytes_out %d != %d pdo_tx_t", nbytes_out, sizeof(pdo_tx_t));
//                 throw EscWrpError(EC_WRP_PDO_TX_SIZE, err_msg);
//             }
//         }

    }

    void init_sdo_lookup(void);

    const objd_t * getSDObjd(const char * name);

    template<typename T>
    int writeSDO_byname(const char * name, T t);

    template<typename T>
    int readSDO_byname(const char * name, T &t);
    int readSDO_byname(const char * name);

    template<typename T>
    int getSDO_byname(const char * name, T &t);

    virtual const pdo_rx_t& getRxPDO() const;
    virtual const pdo_tx_t& getTxPDO() const;
    virtual void setTxPDO(const pdo_tx_t&);

protected:

    // callbacks
    virtual void on_readPDO(void)       {}
    virtual void on_writePDO(void)      {}
    virtual int on_readSDO(const objd_t * sdo)    { return EC_WRP_OK;}
    virtual int on_writeSDO(const objd_t * sdo)    { return EC_WRP_OK;}

protected:

    sdo_t    sdo;

    pdo_rx_t        rx_pdo;
    pdo_tx_t        tx_pdo;

    std::map<std::string, const objd_t*> sdo_look_up;

private:

    void readPDO(void);
    void writePDO(void);

    int readSDO(const objd_t *sdo);
    int writeSDO(const objd_t *sdo);
};


#define TEMPL template<class EscPDOTypes, class EscSDOTypes>
#define CLASS BasicEscWrapper<EscPDOTypes,EscSDOTypes>
#define SIGNATURE(type) TEMPL inline type CLASS


SIGNATURE(void)::readPDO() {
    memcpy((void*)&rx_pdo, inputs, nbytes_in);
    on_readPDO();
}

SIGNATURE(void)::writePDO() {
    on_writePDO();
    memcpy((void*)outputs, &tx_pdo, nbytes_out);
}

SIGNATURE(const typename CLASS::pdo_rx_t&)::getRxPDO() const {
    return rx_pdo;
}

SIGNATURE(const typename CLASS::pdo_tx_t&)::getTxPDO() const {
    return tx_pdo;
}


SIGNATURE(void)::setTxPDO(const pdo_tx_t& tx) {
    tx_pdo = tx;
}

SIGNATURE(void)::init_sdo_lookup(void) {

    const objd_t * sdo = get_SDOs();
    while ( sdo && sdo->index ) {
        sdo_look_up[sdo->name] = sdo;
        if ( readSDO(sdo) != EC_WRP_OK) {
            // raise EscWrpError or IGNORE and continue ?!?!?
            throw EscWrpError(EC_WRP_SDO_READ_FAIL, sdo->name);
        }
        sdo ++;
    }
}

SIGNATURE(const objd_t *)::getSDObjd(const char * name) {
    const objd_t * sdo = 0;
    try {
        sdo = sdo_look_up.at(name);
    } catch ( const std::out_of_range& oor ) {
        throw EscWrpError(EC_WRP_SDO_NOTEXIST, name);
    }
    return sdo;
}

TEMPL
template<typename T>
inline int CLASS::writeSDO_byname(const char * name, const T t) {

    // look up name in SDOs
    const objd_t * sdo = getSDObjd(name);

    if ( sdo->access == ATYPE_RO ) {
        DPRINTF("ERROR sdo obj is READ_ONLY %s\n", sdo->name);
        throw EscWrpError(EC_WRP_SDO_RO, sdo->name);
    }

    // check if sdobj datatype match template variable
    if ( check_datatype(sdo->datatype, t) != EC_WRP_OK ) {
        DPRINTF("ERROR set_SDO_byname %s datatype mismatch %s\n", sdo->name, typeid(T).name() );
        throw EscWrpError(EC_WRP_SDO_MISMATCH, sdo->name);
    }

    *((T*)sdo->data) = t;

    if ( writeSDO(sdo) != EC_WRP_OK ) {
        throw EscWrpError(EC_WRP_SDO_WRITE_FAIL, sdo->name);
    }

    return EC_WRP_OK;
}

TEMPL
template<typename T>
inline int CLASS::getSDO_byname(const char * name, T &t) {

    // look up name in SDOs
    const objd_t * sdo = getSDObjd(name);

    // check if sdobj datatype match template variable
    if ( check_datatype(sdo->datatype, t) != EC_WRP_OK ) {
        DPRINTF("ERROR get_SDO_byname %s datatype mismatch %s\n", sdo->name, typeid(T).name() );
        throw EscWrpError(EC_WRP_SDO_MISMATCH, sdo->name);
    }

    t = *((T*)sdo->data);

    return EC_WRP_OK;  
}

TEMPL
template<typename T>
inline int CLASS::readSDO_byname(const char * name, T &t) {

    // look up name in SDOs
    const objd_t * sdo = getSDObjd(name);

    // check if sdobj datatype match template variable
    if ( check_datatype(sdo->datatype, t) != EC_WRP_OK ) {
        DPRINTF("ERROR get_SDO_byname %s datatype mismatch %s\n", sdo->name, typeid(T).name() );
        throw EscWrpError(EC_WRP_SDO_MISMATCH, sdo->name);
    }
    // read using mailbox
    if ( readSDO(sdo) != EC_WRP_OK) {
        throw EscWrpError(EC_WRP_SDO_READ_FAIL, sdo->name);
    }
    t = *((T*)sdo->data);

    return EC_WRP_OK;  
}

SIGNATURE(int)::readSDO_byname(const char * name) {

    // look up name in SDOs
    const objd_t * sdo = getSDObjd(name);

    // read using mailbox
    if ( readSDO(sdo) != EC_WRP_OK) {
        throw EscWrpError(EC_WRP_SDO_READ_FAIL, sdo->name);
    }

    return EC_WRP_OK;  
}

SIGNATURE(int)::writeSDO(const objd_t *sdo) {

    char * err;
    ec_errort   ec_error;
    int final_size, wkc;

    if ( !sdo ) {
        return 0;
    }

    if ( on_writeSDO(sdo) != EC_WRP_OK ) {
        return EC_WRP_SDO_WRITE_CB_FAIL;
    }

    //DPRINTF("set_SDO %s [%d.0x%04X.0x%02X]\n", sdo->name, slave_pos, sdo->index, sdo->subindex);
    final_size = sdo->bitlength/8;
    //return workcounter from last slave response
    // use a long timeout on rxm because flash_cmd on C2000 to save params to flash takes time .... 
    wkc = ec_SDOwrite(position, sdo->index, sdo->subindex, false, final_size, sdo->data, EC_TIMEOUTRXM * 10);
    if ( wkc <= 0 || final_size != sdo->bitlength/8 ) {
        // ERROR ...
        DPRINTF("*** Slave %d %s >> ", position, sdo->name);
        if ( wkc <= 0 ) {
            err =  ec_elist2string();
            DPRINTF("SDO write fail : %s\n", err);
        } else {
            DPRINTF("SDO write fail : mismatch size %d != %d\n", final_size, sdo->bitlength/8);
        }
        return EC_WRP_SDO_WRITE_FAIL;
    }
    // OK ...
    return EC_WRP_OK; 
}

SIGNATURE(int)::readSDO(const objd_t *sdo) {

    char * err;
    //ec_errort   ec_error;
    int final_size, wkc;

    if ( !sdo ) {
        return EC_WRP_NOK;
    }

    //DPRINTF("get_SDO %s [%d.0x%04X.0x%02X]\n", sdo->name, slave_pos, sdo->index, sdo->subindex);
    final_size = sdo->bitlength/8;
    //return workcounter from last slave response
    wkc = ec_SDOread(position, sdo->index, sdo->subindex, false, &final_size, sdo->data, EC_TIMEOUTRXM);
    if ( wkc <= 0 || final_size != sdo->bitlength/8 ) {
        DPRINTF("*** Slave %d %s >> ", position, sdo->name);
        if ( wkc <= 0 ) {
            err =  ec_elist2string();
            DPRINTF("SDO read fail : %s\n", err);
        } else {
            DPRINTF("SDO read fail : mismatch size %d != %d\n", final_size, sdo->bitlength/8);
        }
        return EC_WRP_SDO_READ_FAIL;
    }
    // OK ....

    on_readSDO(sdo);

    return EC_WRP_OK;; 
}


#undef SIGNATURE
#undef CLASS
#undef TEMPL


}
}

#endif /* IIT_IO_ECAT_SLAVE_WRAPPER_H_ */
