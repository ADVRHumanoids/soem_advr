/*
 * slave_wrapper.cpp
 *
 *  Created on: Jun 25, 2014
 *      Author: mfrigerio
 */


#include <iit/ecat/slave_wrapper.h>

using namespace iit;

ecat::EscWrapper::EscWrapper(const ec_slavet& slave_descriptor) : alias(0)
{
    //DPRINTF(">> factory %x id %d conf_addr %x rev %d alias %d\n",_slave_arg.eep_man, _slave_arg.eep_id, _slave_arg.configadr & 0x0f, _slave_arg.eep_rev, _slave_arg.aliasadr);
    //DPRINTF("   Ibytes %d Obytes %d\n", _slave_arg.Ibytes, _slave_arg.Obytes);

    position =      slave_descriptor.configadr & 0x0f;
    vendor_id =     slave_descriptor.eep_man;
    product_code =  slave_descriptor.eep_id;

    inputs     = slave_descriptor.inputs;
    nbytes_in  = slave_descriptor.Ibytes;
    outputs    = slave_descriptor.outputs;
    nbytes_out = slave_descriptor.Obytes;

}
