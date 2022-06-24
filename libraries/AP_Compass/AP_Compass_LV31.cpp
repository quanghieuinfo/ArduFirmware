/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_Compass_LV31.h"
#include <ctype.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_Compass_LV31::AP_Compass_LV31(uint8_t serial_instance):AP_Compass_Backend_Serial()
{
    uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Compass, serial_instance);
    if (uart == nullptr) {
        return;
    }
    port_num = AP::serialmanager().find_portnum(AP_SerialManager::SerialProtocol_Compass, serial_instance);

    auto devid = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL,get_portnum(),0,0);
    if(!register_compass(devid,_compass_instance)){
        hal.console->printf("Can't register compass\n");
    }else
    {
        set_dev_id(_compass_instance,devid);
        set_rotation(_compass_instance,ROTATION_NONE);
        set_external(_compass_instance,true);
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Compass_LV31::update_serial,void));
    }
    
}

void AP_Compass_LV31::crc16(uint8_t & ck_a,uint8_t & ck_b)
{
    ck_a = 0; ck_b = 0;
    for (uint8_t i = 2; i < 12; i++)
    {
        ck_a += buffer[i];
        ck_b += ck_a;
    }
    ck_a &= 0xFF;
    ck_b &= 0xFF;
}
//read
void AP_Compass_LV31::read(void)
{
    drain_accumulated_samples(_compass_instance);
}

void AP_Compass_LV31::update_serial()
{
    WITH_SEMAPHORE(sem);
    if (!port_opened) {
        port_opened = true;
        uart->begin(115200, rx_bufsize(), tx_bufsize());
    }

    uint32_t nbytes = uart->available();
    if (nbytes < ARRAY_SIZE(buffer)) {
        return;
    }
    // read any available lines from the compass
    while (nbytes-->0) {
        int16_t b = uart->read();
        if (b == -1) {
            break;
        }
        if (buffer_used == 0 && b != 0xB5) {
            //discard
            continue;
        }
        if (buffer_used == 1 && b != 0x62) {
            //discard
            if (b == 0xB5) {
                buffer[0] = b;
            } else {
                buffer_used = 0;
            }
            continue;
        }
        buffer[buffer_used++] = b;
        if (buffer_used == ARRAY_SIZE(buffer)) {
            buffer_used = 0;
            uint8_t ck_a = 0, ck_b = 0;
            crc16(ck_a,ck_b);
            if (buffer[12] != ck_a || buffer[13] != ck_b) {
                //bad CRC, discard
                continue;
            }
            int16_t _mag_x,_mag_y,_mag_z;
            _mag_x = buffer[6]<<8 | buffer[7];
            _mag_y = buffer[8]<<8 | buffer[9];
            _mag_z = buffer[10]<<8 | buffer[11];
            Vector3f raw_field = Vector3f(_mag_x, _mag_y, _mag_z);
            accumulate_sample(raw_field,_compass_instance);
        }
    }
}
