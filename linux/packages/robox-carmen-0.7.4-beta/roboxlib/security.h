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

#ifndef ROBOX_SECURITY_H
#define ROBOX_SECURITY_H

#include "bumper.h"

/** \brief Predefined RoboX security constants
  */
#define ROBOX_SECURITY_READ_TIMEOUT               0.01

/** \brief Predefined RoboX security error codes
  */
#define ROBOX_SECURITY_ERROR_NONE                 0
#define ROBOX_SECURITY_ERROR_START                1
#define ROBOX_SECURITY_ERROR_ESTOP                2
#define ROBOX_SECURITY_ERROR_SUPERVISOR           3
#define ROBOX_SECURITY_ERROR_BUMPER_STATE         4

/** \brief Structure defining the RoboX security module
  */
typedef struct robox_security_t {
  robox_device_t estop_dev;         //!< The emergency stop device.
  robox_device_t sstop_dev;         //!< The supervisor stop device.
  robox_device_t watchdog_dev;      //!< The watchdog device.
  robox_device_t flashlight_dev;    //!< The flashlight device.

  int watchdog;                     //!< The current watchdog value.
} robox_security_t, *robox_security_p;

/** \brief Predefined RoboX security error descriptions
  */
extern const char* robox_security_errors[];

/** \brief Initialize RoboX security module
  * \param[in] security The RoboX security module to be initialized.
  * \param[in] estop_dev The name of the RoboX emergency stop device.
  * \param[in] sstop_dev The name of the RoboX supervisor stop device.
  * \param[in] watchdog_dev The name of the RoboX watchdog device.
  * \param[in] flashlight_dev The name of the RoboX flashlight device.
  * \return The resulting device error code.
  */
int robox_security_init(
  robox_security_p security,
  const char* estop_dev,
  const char* sstop_dev,
  const char* watchdog_dev,
  const char* flashlight_dev);

/** \brief Destroy RoboX security module
  * \param[in] security The initialized RoboX security module to be destroyed.
  * \return The resulting device error code.
  */
int robox_security_destroy(
  robox_security_p security);

/** \brief Start security module
  * \param[in] security The security module to be started.
  * \return The resulting error code.
  */
int robox_security_start(
  robox_security_p security);

/** \brief Stop security module
  * \param[in] security The security module to be stopped.
  */
void robox_security_stop(
  robox_security_p security);

/** \brief Perform security check
  * \note This check has to be performed regularly in order to satisfy
  *   the robot's hardware watchdog conditions.
  * \param[in] security The security module to be checked.
  * \param[in] bumper The bumper module to be checked for insecure 
  *   bumper states.
  * \return The resulting error code.
  */
int robox_security_check(
  robox_security_p security,
  robox_bumper_p bumper);

#endif
