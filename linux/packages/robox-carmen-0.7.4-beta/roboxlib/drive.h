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

#ifndef ROBOX_DRIVE_H
#define ROBOX_DRIVE_H

#include "encoders.h"

/** \brief Structure defining the RoboX drive
  */
typedef struct robox_drive_t {
  double gear_trans;            //!< The robot's gear transmission.

  double wheel_base;            //!< The robot's wheel base in [m].
  double wheel_right_radius;    //!< The robot's right wheel radius in [m].
  double wheel_left_radius;     //!< The robot's left wheel radius in [m].
} robox_drive_t, *robox_drive_p;

/** \brief Structure defining the RoboX drive pose
  */
typedef struct robox_drive_pose_t {
  double x;                     //!< The x-coordinate of the pose in [m].
  double y;                     //!< The y-coordinate of the pose in [m].
  double theta;                 //!< The orientation of the pose in [rad].
} robox_drive_pose_t, *robox_drive_pose_p;

/** \brief Structure defining the RoboX drive velocity
  */
typedef struct robox_drive_vel_t {
  double translational;         //!< The translational velocity in [m/s].
  double rotational;            //!< The rotational velocity in [rad/s].
} robox_drive_vel_t, *robox_drive_vel_p;

/** \brief Initialize RoboX drive
  * \param[in] drive The RoboX drive to be initialized.
  * \param[in] gear_trans The gear transmission of the robot.
  * \param[in] wheel_base The wheel base of the robot in [m].
  * \param[in] wheel_right_radius The right wheel radius of the robot in [m].
  * \param[in] wheel_left_radius The left wheel radius of the robot in [m].
  */
void robox_drive_init(
  robox_drive_p drive,
  double gear_trans,
  double wheel_base,
  double wheel_right_radius,
  double wheel_left_radius);

/** \brief Compute drive velocity from encoder velocity
  * \param[in] drive The drive to compute the velocity for.
  * \param[in] enc_vel The encoder velocity that will be used for computing 
  *   the drive velocity.
  * \param[out] drive_vel The resulting drive velocity.
  */
void robox_drive_velocity_from_encoders(
  robox_drive_p drive,
  robox_encoders_vel_p enc_vel,
  robox_drive_vel_p drive_vel);

/** \brief Compute encoder velocity from drive velocity
  * \param[in] drive The drive to compute the encoder velocity for.
  * \param[in] drive_vel The drive velocity that will be used for computing 
  *   the corresponding encoder velocity.
  * \param[out] enc_vel The resulting encoder velocity.
  */
void robox_drive_velocity_to_encoders(
  robox_drive_p drive,
  robox_drive_vel_p drive_vel,
  robox_encoders_vel_p enc_vel);

#endif
