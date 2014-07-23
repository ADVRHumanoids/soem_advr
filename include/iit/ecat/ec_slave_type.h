#ifndef __EC_SLAVE_TYPE_H__
#define __EC_SLAVE_TYPE_H__

namespace test_pdo {

// TX  slave_input -- master output
    typedef struct {
        uint16_t    _type;
        int32_t     _value;
        uint64_t    _ts;
    } __attribute__((__packed__)) tx_pdo_t;


// RX  slave_output -- master input
    typedef struct {
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
    } __attribute__((__packed__)) rx_pdo_t;


}


namespace hyq_io_pdo {

// TX  slave_input
    typedef struct {
        uint16_t    type;
        int32_t     value;
        int16_t     aout[2];
        uint8_t     _bit_0:1;
        uint8_t     _bit_1:1;
        uint8_t     _bit_2:1;
        uint8_t     _bit_3:1;
        uint8_t     _bit_4:1;
        uint8_t     _bit_5:1;
        uint8_t     _bit_6:1;
        uint8_t     _bit_7:1;
        uint8_t     _bits;
    }  __attribute__((__packed__)) tx_pdo_t;

// RX slave output
    typedef struct rx_pdo {
        uint8_t     _bit_0:1;
        uint8_t     _bit_1:1;
        uint8_t     _bit_2:1;
        uint8_t     _bit_3:1;
        uint8_t     _bit_4:1;
        uint8_t     _bit_5:1;
        uint8_t     _bit_6:1;
        uint8_t     _bit_7:1;
        uint8_t     _bits;
        int32_t     _abs_enc[3];
        int32_t     _rel_enc[3];
        int16_t     _ain[12];
    } __attribute__((__packed__)) rx_pdo_t;

}

namespace hyq_valve_pdo {

// TX slave input
    typedef struct {
        uint16_t    type;
        int32_t     value;
        int16_t     aout[12];
    }  __attribute__((__packed__)) tx_pdo_t;

// RX  slave_output
    typedef struct rx_pdo {
        uint8_t     _bit_0:1;
        uint8_t     _bit_1:1;
        uint8_t     _bit_2:1;
        uint8_t     _bit_3:1;
        uint8_t     _bit_4:1;
        uint8_t     _bit_5:1;
        uint8_t     _bit_6:1;
        uint8_t     _bit_7:1;
        uint8_t     _bits;
    } __attribute__((__packed__)) rx_pdo_t;


}

namespace bigman_pdo {

// TX  slave_input -- master output
    typedef struct {
        float       _type;
        float       _value;
        uint64_t    _ts;
    } __attribute__((__packed__)) tx_pdo_t;


// RX  slave_output -- master input
    typedef struct {
        float       _pos_ref;
        float       _pos;
        float       _tor_ref;
        float       _tor;
        uint64_t    _ts;
    } __attribute__((__packed__)) rx_pdo_t; 


}

typedef union {
    test_pdo::tx_pdo_t      test;
    hyq_io_pdo::tx_pdo_t    hyq_io;
    hyq_valve_pdo::tx_pdo_t hyq_valve;
    bigman_pdo::tx_pdo_t    bigman;
} input_slave_t;

typedef union {
    test_pdo::rx_pdo_t       test;
    hyq_io_pdo::rx_pdo_t     hyq_io;
    hyq_valve_pdo::rx_pdo_t  hyq_valve;
    bigman_pdo::rx_pdo_t     bigman;
} output_slave_t;

#endif
