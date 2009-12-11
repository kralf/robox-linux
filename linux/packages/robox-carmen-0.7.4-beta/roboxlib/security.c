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

#include <stdio.h>

#include "security.h"

const char* robox_security_errors[] = {
  "success",
  "error starting security module",
  "emergency stop error",
  "supervisor error",
  "bumper state error",
};

int robox_security_init(robox_security_p security, const char* estop_dev,
  const char* sstop_dev, const char* watchdog_dev, const char* 
  flashlight_dev) {
  security->watchdog = 0;

  int result;
  if (!(result = robox_device_open(&security->estop_dev, estop_dev, 
      robox_device_input, ROBOX_SECURITY_READ_TIMEOUT)) &&
    !(result = robox_device_open(&security->sstop_dev, sstop_dev, 
      robox_device_input, ROBOX_SECURITY_READ_TIMEOUT)) &&
    !(result = robox_device_open(&security->watchdog_dev, watchdog_dev, 
      robox_device_output, ROBOX_SECURITY_READ_TIMEOUT)))
    return robox_device_open(&security->flashlight_dev, flashlight_dev, 
      robox_device_output, ROBOX_SECURITY_READ_TIMEOUT);
  else
    return result;
}

int robox_security_destroy(robox_security_p security) {
  int result;

  if (!(result = robox_device_close(&security->estop_dev)) &&
    !(result = robox_device_close(&security->sstop_dev)) &&
    !(result = robox_device_close(&security->watchdog_dev)))
    return robox_device_close(&security->flashlight_dev);
  else
    return result;
}

int robox_security_start(robox_security_p security) {
  if (!robox_device_write(&security->watchdog_dev, 0)) {
    security->watchdog = 0;
    return ROBOX_SECURITY_ERROR_NONE;
  }
  else
    return ROBOX_SECURITY_ERROR_START;
}

void robox_security_stop(robox_security_p security) {
  robox_device_write(&security->watchdog_dev, 0);
  robox_device_write(&security->flashlight_dev, 0);

  security->watchdog = 0;
}

int robox_security_check(robox_security_p security, robox_bumper_p bumper) {
  int result = ROBOX_SECURITY_ERROR_NONE;
  int estop_val, sstop_val;

  security->watchdog = !security->watchdog;
  robox_device_write(&security->watchdog_dev, security->watchdog);

  if (robox_device_read(&security->estop_dev, &estop_val) || !estop_val)
    result = ROBOX_SECURITY_ERROR_ESTOP;
  else if (robox_device_read(&security->sstop_dev, &sstop_val) || !sstop_val)
    result = ROBOX_SECURITY_ERROR_SUPERVISOR;
  else if (robox_bumper_get_state(bumper) == robox_bumper_pressed)
    result = ROBOX_SECURITY_ERROR_BUMPER_STATE;

  robox_device_write(&security->flashlight_dev, (result) ? 1 : 0);

  return result;
}
