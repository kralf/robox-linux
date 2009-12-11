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

#include "motors.h"

#include "global.h"

const char* robox_motors_errors[] = {
  "success",
  "error starting motors",
  "error stopping motors",
};

robox_motors_current_t robox_motors_zero_current = {
  ROBOX_MOTORS_ZERO_CURRENT,
  ROBOX_MOTORS_ZERO_CURRENT,
};

int robox_motors_init(robox_motors_p motors, const char* enable_dev, 
  const char* right_dev, const char* left_dev, const char* brake_dev) {
  int result;

  if (!(result = robox_device_open(&motors->enable_dev,
      enable_dev, robox_device_output, ROBOX_MOTORS_READ_TIMEOUT)) &&
    !(result = robox_device_open(&motors->right_dev, right_dev, 
      robox_device_output, ROBOX_MOTORS_READ_TIMEOUT)) &&
    !(result = robox_device_open(&motors->left_dev, left_dev, 
      robox_device_output, ROBOX_MOTORS_READ_TIMEOUT)))
    return robox_device_open(&motors->brake_dev, brake_dev, 
      robox_device_output, ROBOX_MOTORS_READ_TIMEOUT);                                        
  else
    return result;
}

int robox_motors_destroy(robox_motors_p motors) {
  int result;

  if (!(result = robox_device_close(&motors->enable_dev)) &&
    !(result = robox_device_close(&motors->right_dev)) &&
    !(result = robox_device_close(&motors->left_dev)))
    return robox_device_close(&motors->brake_dev);
  else
    return result;
}

int robox_motors_start(robox_motors_p motors) {
  if (!robox_motors_set_current(motors, &robox_motors_zero_current) &&
    !robox_motors_release(motors) &&
    !robox_device_write(&motors->enable_dev, 1))
    return ROBOX_MOTORS_ERROR_NONE;
  else
    return ROBOX_MOTORS_ERROR_START;
}

int robox_motors_stop(robox_motors_p motors) {
  if (!robox_device_write(&motors->enable_dev, 0) &&
    !robox_motors_brake(motors) &&
    !robox_motors_set_current(motors, &robox_motors_zero_current))
    return ROBOX_MOTORS_ERROR_NONE;
  else
    return ROBOX_MOTORS_ERROR_STOP;
}

int robox_motors_brake(robox_motors_p motors) {
  return robox_device_write(&motors->brake_dev, 0);
}

int robox_motors_release(robox_motors_p motors) {
  return robox_device_write(&motors->brake_dev, 1);
}

int robox_motors_get_current(robox_motors_p motors, robox_motors_current_p
  current) {
  int right_current, left_current, right_result, left_result;

  if (!(right_result = robox_device_read(&motors->right_dev, &right_current)))
    current->right = right_current;
  if (!(left_result = robox_device_read(&motors->left_dev, &left_current)))
    current->left = left_current;
    
  if (right_result)
    return right_result;
  else
    return left_result;
}

int robox_motors_set_current(robox_motors_p motors, robox_motors_current_p
  current) {
  int right_result, left_result;

  current->right = clip(current->right, ROBOX_MOTORS_MIN_CURRENT, 
    ROBOX_MOTORS_MAX_CURRENT);
  current->left = clip(current->left, ROBOX_MOTORS_MIN_CURRENT, 
    ROBOX_MOTORS_MAX_CURRENT);

  right_result = robox_device_write(&motors->right_dev, current->right);
  left_result = robox_device_write(&motors->left_dev, current->left);

  if (right_result)
    return right_result;
  else
    return left_result;
}
