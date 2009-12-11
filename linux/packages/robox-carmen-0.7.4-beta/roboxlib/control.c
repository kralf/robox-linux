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
#include <string.h>

#include <timer.h>

#include "control.h"

#include "global.h"

const char* robox_control_errors[] = {
  "success",
  "error starting controller",
  "error performing control cycle iteration",
};

void robox_control_init(robox_control_p control, robox_encoders_p encoders,
  robox_motors_p motors, robox_drive_p drive, double p_gain, double i_gain, 
  double d_gain) {
  control->encoders = encoders;
  control->motors = motors;
  control->drive = drive;

  control->p_gain = p_gain;
  control->i_gain = i_gain;
  control->d_gain = d_gain;
}

void robox_control_destroy(robox_control_p control) {
  control->encoders = 0;
  control->motors = 0;
  control->drive = 0;
}

int robox_control_start(robox_control_p control) {
  if (!robox_encoders_get_position(control->encoders, &control->enc_pos) &&
    !robox_motors_get_current(control->motors, &control->motor_curr)) {
    memset(&control->enc_vel_error, 0, sizeof(robox_encoders_vel_t));
    control->timestamp = 0.0;

    return ROBOX_CONTROL_ERROR_NONE;
  }
  else
    return ROBOX_CONTROL_ERROR_START;
}

int robox_control_iterate(robox_control_p control, robox_drive_vel_p set_vel) {
  robox_encoders_vel_t enc_vel, enc_set_vel, enc_acc;
  double dt = (control->timestamp > 0.0) ? timer_stop(control->timestamp) : 0.0;

  if (!robox_encoders_get_velocity(control->encoders,  &control->enc_pos, 
      dt, &enc_vel)) {
    if (dt > 0.0) {
      enc_acc.right = (enc_vel.right-control->enc_vel.right)/dt;
      enc_acc.left = (enc_vel.left-control->enc_vel.left)/dt;
    }
    else {
      memset(&enc_vel, 0, sizeof(robox_encoders_vel_t));
      memset(&enc_acc, 0, sizeof(robox_encoders_vel_t));
    }
    
    robox_drive_velocity_to_encoders(control->drive, set_vel, &enc_set_vel);
    control->enc_vel_error.right += enc_set_vel.right-enc_vel.right;
    control->enc_vel_error.left += enc_set_vel.left-enc_vel.left;

    control->motor_curr.right +=
      control->p_gain*(enc_set_vel.right-enc_vel.right)+
      control->i_gain*control->enc_vel_error.right+
      control->d_gain*enc_acc.right;
    control->motor_curr.left +=
      control->p_gain*(enc_set_vel.left-enc_vel.left)+
      control->i_gain*control->enc_vel_error.left+
      control->d_gain*enc_acc.left;

    robox_motors_set_current(control->motors, &control->motor_curr);

    control->enc_vel = enc_vel;
    timer_start(&control->timestamp);

    return ROBOX_CONTROL_ERROR_NONE;
  }
  else
    return ROBOX_CONTROL_ERROR_ITERATE;
}
