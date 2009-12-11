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

#ifndef ROBOX_CONTROL_H
#define ROBOX_CONTROL_H

#include "encoders.h"
#include "motors.h"
#include "drive.h"

/** \brief Predefined RoboX control error codes
  */
#define ROBOX_CONTROL_ERROR_NONE                 0
#define ROBOX_CONTROL_ERROR_START                1
#define ROBOX_CONTROL_ERROR_ITERATE              2

/** \brief Structure defining the RoboX velocity control
  */
typedef struct robox_control_t {
  robox_encoders_p encoders;           //!< The robot's encoders.
  robox_motors_p motors;               //!< The robot's motors.
  robox_drive_p drive;                 //!< The robot's drive.

  double p_gain;                       //!< The controller's proportional gain.
  double i_gain;                       //!< The controller's integral gain.
  double d_gain;                       //!< The controller's differential gain.

  robox_encoders_pos_t enc_pos;        //!< The recent encoder position.
  robox_encoders_vel_t enc_vel;        //!< The recent encoder velocity.
  robox_encoders_vel_t enc_vel_error;  //!< The integrated velocity error.
  robox_motors_current_t motor_curr;   //!< The recent motor current.
  double timestamp;                    //!< The recent timestamp.
} robox_control_t, *robox_control_p;

/** \brief Predefined RoboX control error descriptions
  */
extern const char* robox_control_errors[];

/** \brief Initialize RoboX controller
  * \param[in] control The RoboX controller to be initialized.
  * \param[in] encoders The initialized robot encoders.
  * \param[in] motors The initialized robot motors.
  * \param[in] drive The initialized robot drive.
  * \param[in] p_gain The controller's proportional gain.
  * \param[in] i_gain The controller's integral gain.
  * \param[in] d_gain The controller's differential gain.
  */
void robox_control_init(
  robox_control_p control,
  robox_encoders_p encoders,
  robox_motors_p motors,
  robox_drive_p drive,
  double p_gain,
  double i_gain,
  double d_gain);

/** \brief Destroy RoboX controller
  * \param[in] control The initialized RoboX controller to be destroyed.
  */
void robox_control_destroy(
  robox_control_p control);

/** \brief Start the controller
  * \param[in] control The controller to be started.
  * \return The resulting error code.
  */
int robox_control_start(
  robox_control_p control);

/** \brief Perform a control cycle iteration
  * \param[in] control The controller to perform the iteration for.
  * \param[in] set_vel The setpoint velocity to be used during the control 
  *   cycle iteration.
  * \return The resulting error code.
  */
int robox_control_iterate(
  robox_control_p control,
  robox_drive_vel_p set_vel);

#endif
