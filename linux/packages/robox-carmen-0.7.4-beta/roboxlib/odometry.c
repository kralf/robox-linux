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

#include <timer.h>

#include "odometry.h"

#include "global.h"

const char* robox_odometry_errors[] = {
  "success",
  "error starting odometry",
  "error integrating odometry",
};

void robox_odometry_init(robox_odometry_p odometry, robox_encoders_p 
  encoders, robox_drive_p drive) {
  odometry->encoders = encoders;
  odometry->drive = drive;
}

void robox_odometry_destroy(robox_odometry_p odometry) {
  odometry->encoders = 0;
  odometry->drive = 0;
}

int robox_odometry_start(robox_odometry_p odometry) {
  if (!robox_encoders_get_position(odometry->encoders, &odometry->enc_pos)) {
    timer_start(&odometry->timestamp);
    return ROBOX_ODOMETRY_ERROR_NONE;
  }
  else
    return ROBOX_ODOMETRY_ERROR_START;
}

double robox_odometry_mod_2pi(double theta) {
    int n = floor(theta/(2.0*M_PI));
    theta -= n*(2.0*M_PI);
    
    if (theta > M_PI)
      theta -= (2.0*M_PI);

    return theta;
}

int robox_odometry_integrate(robox_odometry_p odometry, robox_drive_pose_p 
  pose, robox_drive_vel_p velocity) {
  robox_encoders_vel_t enc_vel;
  double dt = timer_stop(odometry->timestamp);

  if (!robox_encoders_get_velocity(odometry->encoders,  &odometry->enc_pos, 
      dt, &enc_vel)) {
    robox_drive_velocity_from_encoders(odometry->drive, &enc_vel, velocity);

    pose->theta = robox_odometry_mod_2pi(pose->theta+velocity->rotational*dt);
    pose->x += velocity->translational*dt*cos(pose->theta);
    pose->y += velocity->translational*dt*sin(pose->theta);

    timer_start(&odometry->timestamp);

    return ROBOX_ODOMETRY_ERROR_NONE;
  }
  else
    return ROBOX_ODOMETRY_ERROR_INTEGRATE;
}
