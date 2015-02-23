/*
 * slave_wrapper.cpp
 *
 *  Created on: Jun 25, 2014
 *      Author: mfrigerio
 */

#include <string.h>

#include <iit/ecat/slave_wrapper.h>

using namespace iit::ecat;


EscWrapper::EscWrapper(const ec_slavet& slave_descriptor) : alias(0)
{
    ec_slavet _slave_arg = slave_descriptor;

    configadr =     slave_descriptor.configadr;
    position =      slave_descriptor.configadr & 0xff;
    alias =         slave_descriptor.aliasadr;
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
            alias);
    DPRINTF("\tsupport DC %d\n", slave_descriptor.hasdc);
    DPRINTF("\tIbytes %d Obytes %d\n", nbytes_in, nbytes_out);

}


