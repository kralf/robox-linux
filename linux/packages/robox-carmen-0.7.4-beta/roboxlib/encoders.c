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

#include "encoders.h"

#include "global.h"

int robox_encoders_init(robox_encoders_p encoders, const char* right_dev,
  const char* left_dev, ssize_t num_pulses) {
  int result;

  encoders->num_pulses = num_pulses;

  if (!(result = robox_device_open(&encoders->right_dev, right_dev, 
      robox_device_input, ROBOX_ENCODERS_READ_TIMEOUT)))
    return robox_device_open(&encoders->left_dev, left_dev, 
      robox_device_input, ROBOX_ENCODERS_READ_TIMEOUT);                                        
  else
    return result;
}

int robox_encoders_destroy(robox_encoders_p encoders) {
  int result;

  if (!(result = robox_device_close(&encoders->right_dev)))
    return robox_device_close(&encoders->left_dev);
  else
    return result;
}

int robox_encoders_get_position(robox_encoders_p encoders, robox_encoders_pos_p 
  position) {
  int result;

  if (!(result = robox_device_read(&encoders->right_dev, &position->right)))
    return robox_device_read(&encoders->left_dev, &position->left);
  else
    return result;
}

double robox_encoders_to_angle(robox_encoders_p encoders, int old_value, 
  int new_value) {
  int delta_value = 0;

  if (new_value > old_value) {
    int delta_max = old_value+ROBOX_ENCODERS_MAX_VALUE-new_value;

    if (delta_max > new_value-old_value)
      delta_value = new_value-old_value;
    else
      delta_value = -delta_max;
  }
  else {
    int delta_max = ROBOX_ENCODERS_MAX_VALUE-old_value+new_value;

    if (delta_max > old_value-new_value)
      delta_value = new_value-old_value;
    else
      delta_value = delta_max;
  }

  return delta_value/(4.0*encoders->num_pulses)*2.0*M_PI;
}

int robox_encoders_get_velocity(robox_encoders_p encoders, robox_encoders_pos_p 
  position, double dtime, robox_encoders_vel_p velocity) {
  robox_encoders_pos_t pos;
  int result;

  if (!(result = robox_encoders_get_position(encoders, &pos))) {
    velocity->right = robox_encoders_to_angle(encoders, pos.right, 
      position->right)/dtime;
    velocity->left = robox_encoders_to_angle(encoders, pos.left, 
      position->left)/dtime;
    
    *position = pos;
  }

  return result;
}
