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

#ifndef ROBOX_ODOMETRY_H
#define ROBOX_ODOMETRY_H

#include "encoders.h"
#include "drive.h"

/** \brief Predefined RoboX odometry error codes
  */
#define ROBOX_ODOMETRY_ERROR_NONE                 0
#define ROBOX_ODOMETRY_ERROR_START                1
#define ROBOX_ODOMETRY_ERROR_INTEGRATE            2

/** \brief Structure defining the RoboX odometry
  */
typedef struct robox_odometry_t {
  robox_encoders_p encoders;      //!< The robot's encoders.
  robox_drive_p drive;            //!< The robot's drive.

  robox_encoders_pos_t enc_pos;   //!< The recent encoder position.
  double timestamp;               //!< The recent timestamp.
} robox_odometry_t, *robox_odometry_p;

/** \brief Predefined RoboX odometry error descriptions
  */
extern const char* robox_odometry_errors[];

/** \brief Initialize RoboX odometry
  * \param[in] odometry The RoboX odometry to be initialized.
  * \param[in] encoders The initialized robot encoders.
  * \param[in] drive The initialized robot drive.
  */
void robox_odometry_init(
  robox_odometry_p odometry,
  robox_encoders_p encoders,
  robox_drive_p drive);

/** \brief Destroy RoboX odometry
  * \param[in] odometry The initialized RoboX odometry to be destroyed.
  */
void robox_odometry_destroy(
  robox_odometry_p odometry);

/** \brief Start the odometry
  * \param[in] odometry The odometry to be started.
  * \return The resulting error code.
  */
int robox_odometry_start(
  robox_odometry_p odometry);

/** \brief Integrate the odometry
  * \param[in] odometry The odometry to be used for integration.
  * \param[in] pose The pose to be updated with the integrated odometry value.
  * \param[in] velocity The velocity to be updated by the odometry.
  * \return The resulting error code.
  */
int robox_odometry_integrate(
  robox_odometry_p odometry,
  robox_drive_pose_p pose,
  robox_drive_vel_p velocity);

#endif
