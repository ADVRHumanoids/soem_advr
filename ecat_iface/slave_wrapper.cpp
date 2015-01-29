/*
 * slave_wrapper.cpp
 *
 *  Created on: Jun 25, 2014
 *      Author: mfrigerio
 */

#include <string.h>

#include <iit/ecat/slave_wrapper.h>

using namespace iit::ecat;

//typename TestESCtemp<class EscPDOTypes, class EscSDOTypes>::sdo_t    TestESCtemp<ESCParamTypes>::flash_param;
//template<class EscPDOTypes, class EscSDOTypes>

//#define TEMPL template<class EscPDOTypes, class EscSDOTypes>
//#define CLASS BasicEscWrapper<EscPDOTypes,EscSDOTypes>
//TEMPL
//typename CLASS::sdo_t CLASS::sdo;
//#undef TEMPL
//#undef CLASS

EscWrapper::EscWrapper(const ec_slavet& slave_descriptor) : alias(0)
{
    ec_slavet _slave_arg = slave_descriptor;

    configadr =     slave_descriptor.configadr;
    position =      slave_descriptor.configadr & 0x0f;
    vendor_id =     slave_descriptor.eep_man;
    product_code =  slave_descriptor.eep_id;

    inputs     = slave_descriptor.inputs;
    nbytes_in  = slave_descriptor.Ibytes;
    outputs    = slave_descriptor.outputs;
    nbytes_out = slave_descriptor.Obytes;

    DPRINTF(">> factory %x id %d : conf_addr %x pos %d rev %d alias %d\n",
            vendor_id,
            product_code,
            configadr,
            position,
            _slave_arg.eep_rev,
            _slave_arg.aliasadr);
    DPRINTF("   Ibytes %d Obytes %d\n", nbytes_in, nbytes_out);

}


