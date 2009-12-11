/***************************************************************************
 *   Copyright (C) 2009 by Ralf Kaestner                                   *
 *   ralf.kaestner@gmail.com                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include "sensors.h"

int robox_sensors_init(robox_sensors_p sensors, const char* check_dev,
  const char* ok_dev) {
  int result;

  if (!(result = robox_device_open(&sensors->check_dev, check_dev, 
      robox_device_input, ROBOX_SENSORS_READ_TIMEOUT)))
    return robox_device_open(&sensors->ok_dev, ok_dev, 
      robox_device_output, ROBOX_SENSORS_READ_TIMEOUT);                                        
  else
    return result;
}

int robox_sensors_destroy(robox_sensors_p sensors) {
  int result;

  if (!(result = robox_device_close(&sensors->check_dev)))
    return robox_device_close(&sensors->ok_dev);
  else
    return result;
}
