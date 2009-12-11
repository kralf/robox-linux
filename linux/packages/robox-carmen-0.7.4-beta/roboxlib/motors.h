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

#ifndef ROBOX_MOTORS_H
#define ROBOX_MOTORS_H

#include "device.h"

/** \brief Predefined RoboX motor constants
  */
#define ROBOX_MOTORS_READ_TIMEOUT               0.01

#define ROBOX_MOTORS_MIN_CURRENT                0.0
#define ROBOX_MOTORS_ZERO_CURRENT               2048.0
#define ROBOX_MOTORS_MAX_CURRENT                4095.0

/** \brief Predefined RoboX motor error codes
  */
#define ROBOX_MOTORS_ERROR_NONE                 0
#define ROBOX_MOTORS_ERROR_START                1
#define ROBOX_MOTORS_ERROR_STOP                 2

/** \brief Structure defining the RoboX motors
  */
typedef struct robox_motors_t {
  robox_device_t enable_dev;            //!< The motor enable device.
  robox_device_t right_dev;             //!< The right motor device.
  robox_device_t left_dev;              //!< The left motor device.
  robox_device_t brake_dev;             //!< The motor braking device.
} robox_motors_t, *robox_motors_p;

/** \brief Structure defining the RoboX motor current
  */
typedef struct robox_motors_current_t {
  double right;             //!< The right motor's current value in [cu]
  double left;              //!< The left motor's current value in [cu]
} robox_motors_current_t, *robox_motors_current_p;

/** \brief Predefined RoboX motor error descriptions
  */
extern const char* robox_motors_errors[];

/** \brief Predefined RoboX motor zero current
  */
extern robox_motors_current_t robox_motors_zero_current;

/** \brief Initialize RoboX motors
  * \param[in] motors The RoboX motors to be initialized.
  * \param[in] enable_dev The name of the motor enable device.
  * \param[in] right_dev The name of the right motor device.
  * \param[in] left_dev The name of the left motor device.
  * \param[in] brake_dev The name of the motor braking device.
  * \return The resulting device error code.
  */
int robox_motors_init(
  robox_motors_p motors,
  const char* enable_dev,
  const char* right_dev,
  const char* left_dev,
  const char* brake_dev);

/** \brief Destroy RoboX motors
  * \param[in] motors The initialized RoboX motors to be destroyed.
  * \return The resulting device error code.
  */
int robox_motors_destroy(
  robox_motors_p motors);

/** \brief Start the motors
  * \param[in] motors The motors to be started.
  * \return The resulting error code.
  */
int robox_motors_start(
  robox_motors_p motors);

/** \brief Stop the motors
  * \param[in] motors The motors to be stopped.
  * \return The resulting error code.
  */
int robox_motors_stop(
  robox_motors_p motors);

/** \brief Fasten the brake
  * \param[in] motors The motors to fasten the brake for.
  * \return The resulting device error code.
  */
int robox_motors_brake(
  robox_motors_p motors);

/** \brief Release the brake
  * \param[in] motors The motors to release the brake for.
  * \return The resulting device error code.
  */
int robox_motors_release(
  robox_motors_p motors);

/** \brief Retrieve motor current
  * \param[in] motors The motors to retrieve the current for.
  * \param[out] current The retrieved motor current.
  * \return The resulting device error code.
  */
int robox_motors_get_current(
  robox_motors_p motors,
  robox_motors_current_p current);

/** \brief Set motor current
  * \param[in] motors The motors to set the current for.
  * \param[in] current The current to be set for the motors.
  * \return The resulting device error code.
  */
int robox_motors_set_current(
  robox_motors_p motors,
  robox_motors_current_p current);

#endif
