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
#include "AP_Compass_Backend_Serial.h"


#include <ctype.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the Compass. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the Compass
*/
AP_Compass_Backend_Serial::AP_Compass_Backend_Serial():AP_Compass_Backend()
{

}

uint32_t AP_Compass_Backend_Serial::initial_baudrate(const uint8_t serial_instance) const
{
    return AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_Compass, serial_instance);
}

/*
   detect if a Serial Compass is connected. We'll detect by simply
   checking for SerialManager configuration
*/
bool AP_Compass_Backend_Serial::detect(uint8_t serial_instance)
{
    return AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Compass, serial_instance) != nullptr;
}

int8_t AP_Compass_Backend_Serial::get_portnum() const
{
    if (uart != nullptr)
    {
        return port_num;
    }
    return -1;
}
/*
   read the value of the sensor
*/
void AP_Compass_Backend_Serial::read(void)
{

}
