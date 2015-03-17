/*
 * slave_wrapper.cpp
 *
 *  Created on: Jun 25, 2014
 *      Author: mfrigerio
 */

#include <string.h>

#include <iit/ecat/slave_wrapper.h>

using namespace iit::ecat;


EscWrapper::EscWrapper(const ec_slavet & slave_descriptor) : alias(0)
{
    ec_slave_desc = &slave_descriptor;
    
    configadr =     slave_descriptor.configadr;
    position =      slave_descriptor.configadr & 0xff;
    alias =         slave_descriptor.aliasadr;
    vendor_id =     slave_descriptor.eep_man;
    product_code =  slave_descriptor.eep_id;

    inputs     = slave_descriptor.inputs;
    nbytes_in  = slave_descriptor.Ibytes;
    outputs    = slave_descriptor.outputs;
    nbytes_out = slave_descriptor.Obytes;

    topology = slave_descriptor.topology;
    active_ports = slave_descriptor.activeports;
    
    DPRINTF(">> factory %x id %d : conf_addr %x pos %d rev %d alias %d\n",
            vendor_id,
            product_code,
            configadr,
            position,
            ec_slave_desc->eep_rev,
            alias);
    DPRINTF("\tsupport DC %d\n", slave_descriptor.hasdc);
    DPRINTF("\ttopology %d port act %x\n", topology, active_ports);
    DPRINTF("\tIbytes %d Obytes %d\n", nbytes_in, nbytes_out);

}

void EscWrapper::resetError(void) {

    uint16_t    error_cnt;
    uint8_t     fw_error_cnt, lost_link_cnt;
    
    // NOTE: Error are cleared if one of the Error regs is written. Write value is ignored (write 0)
    ec_BWR(0x0000, ECT_REG_RXERR, sizeof(error_cnt), &error_cnt, EC_TIMEOUTRET3);
    ec_BWR(0x0000, ECT_REG_FWRXERR, sizeof(fw_error_cnt), &fw_error_cnt, EC_TIMEOUTRET3);
    ec_BWR(0x0000, ECT_REG_LOSTLINK, sizeof(lost_link_cnt), &lost_link_cnt, EC_TIMEOUTRET3);
     
}

void EscWrapper::readErrReg(void)
{
    uint16_t    error_cnt, dl_status, h, b;
    uint8_t     fw_error_cnt, lost_link_cnt;
    int         wkc;
    
    wkc = ec_FPRD(configadr, ECT_REG_DLSTAT, sizeof(dl_status), &dl_status,  EC_TIMEOUTRET3);
    
    if ( wkc <= 0 ) {
        DPRINTF("ESC %d : fail read  ECT_REG_DLSTAT\n", position);
        return;
    }
    
    h = b = 0;
    
    if ((dl_status & 0x0300) == 0x0200) /* port0 open and communication established */
    {
        h++; b |= 0x01;
    }
    if ((dl_status & 0x0c00) == 0x0800) /* port1 open and communication established */
    {
        h++; b |= 0x02;
    }
    if ((dl_status & 0x3000) == 0x2000) /* port2 open and communication established */
    {
        h++; b |= 0x04;
    }
    if ((dl_status & 0xc000) == 0x8000) /* port3 open and communication established */
    {
        h++; b |= 0x08;
    }

            
    if ( h != topology ) {
        DPRINTF("ESC %d : topology changed %d\t --> %d\n", position, topology, h );
    }
    if ( b != active_ports) {
        DPRINTF("ESC %d : active ports changed %x\t --> %x\n", position, active_ports, b );
    }
        
    // 
    for (int i=0; i<4; i++) {
        if ( ec_slave_desc->activeports & ( 1<<i) ) {
            wkc = ec_FPRD(configadr, ECT_REG_RXERR, sizeof(error_cnt), &error_cnt,  EC_TIMEOUTRET3);
            invalid_frame[i] = error_cnt & 0x00FF;
            rx_error[i] = (error_cnt >> 8) & 0x00FF;
            wkc = ec_FPRD(configadr, ECT_REG_FWRXERR, sizeof(fw_error_cnt), &fw_error_cnt, EC_TIMEOUTRET3);
            fw_error[i] = fw_error_cnt;
            wkc = ec_FPRD(configadr, ECT_REG_LOSTLINK, sizeof(lost_link_cnt), &lost_link_cnt, EC_TIMEOUTRET3);
            lost_link[i] = lost_link_cnt;
            
            DPRINTF("ESC %d Port[%d] : invalid frame %d\trx error %d\tforwarded error %d\tlost link %d\n",
                    position, i, invalid_frame[i], rx_error[i], fw_error[i], lost_link[i] );
        }
    }
    DPRINTF("\n");
    
}