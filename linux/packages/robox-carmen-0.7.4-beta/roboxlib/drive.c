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

#include "drive.h"

#include "global.h"

void robox_drive_init(robox_drive_p drive, double gear_trans, double 
  wheel_base, double wheel_right_radius, double wheel_left_radius) {
  drive->gear_trans = gear_trans;

  drive->wheel_base = wheel_base;
  drive->wheel_right_radius = wheel_right_radius;
  drive->wheel_left_radius = wheel_left_radius;
}

void robox_drive_velocity_from_encoders(robox_drive_p drive, 
  robox_encoders_vel_p enc_vel, robox_drive_vel_p drive_vel) {
  double omega_right = -drive->wheel_right_radius*enc_vel->right/
    drive->gear_trans;
  double omega_left = drive->wheel_left_radius*enc_vel->left/
    drive->gear_trans;

  drive_vel->translational = 0.5*(omega_right+omega_left);
  drive_vel->rotational = (omega_right-omega_left)/drive->wheel_base;
}

void robox_drive_velocity_to_encoders(robox_drive_p drive, robox_drive_vel_p 
  drive_vel, robox_encoders_vel_p enc_vel) {
  double omega_right = drive_vel->translational+0.5*drive->wheel_base*
    drive_vel->rotational;
  double omega_left = drive_vel->translational-0.5*drive->wheel_base*
    drive_vel->rotational;

  enc_vel->right = -drive->gear_trans*omega_right/drive->wheel_right_radius;
  enc_vel->left = drive->gear_trans*omega_left/drive->wheel_left_radius;
}
