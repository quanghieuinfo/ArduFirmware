#pragma once

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"
#include <AP_SerialManager/AP_SerialManager.h>
class AP_Compass_Backend_Serial : public AP_Compass_Backend
{
public:
    // constructor
    AP_Compass_Backend_Serial();

    // static detection function
    static bool detect(uint8_t serial_instance);

    //
    int8_t get_portnum() const;
    
protected:

    // baudrate used during object construction:
    virtual uint32_t initial_baudrate(uint8_t serial_instance) const;

    // the value 0 is special to the UARTDriver - it's "use default"
    virtual uint16_t rx_bufsize() const { return 0; }
    virtual uint16_t tx_bufsize() const { return 0; }

    AP_HAL::UARTDriver *uart = nullptr;

    // read
    virtual void read(void) override;

    int8_t port_num;
};
