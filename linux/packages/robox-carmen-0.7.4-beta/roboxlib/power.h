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

#ifndef ROBOX_POWER_H
#define ROBOX_POWER_H

#include "device.h"

/** \brief Predefined RoboX power constants
  */
#define ROBOX_POWER_READ_TIMEOUT               0.01

/** \brief Predefined RoboX power error codes
  */
#define ROBOX_POWER_ERROR_NONE                 0
#define ROBOX_POWER_ERROR_UP                   1

/** \brief Structure defining the RoboX power module
  */
typedef struct robox_power_t {
  robox_device_t engage_dev;    //!< The power engage device.
  robox_device_t battery_dev;   //!< The battery device.
} robox_power_t, *robox_power_p;

/** \brief Predefined RoboX power error descriptions
  */
extern const char* robox_power_errors[];

/** \brief Initialize RoboX power module
  * \param[in] power The RoboX power module to be initialized.
  * \param[in] engage_dev The name of the RoboX power engage device.
  * \param[in] battery_dev The name of the RoboX battery device.
  * \return The resulting device error code.
  */
int robox_power_init(
  robox_power_p power,
  const char* engage_dev,
  const char* battery_dev);

/** \brief Destroy RoboX power module
  * \param[in] power The initialized RoboX power module to be destroyed.
  * \return The resulting device error code.
  */
int robox_power_destroy(
  robox_power_p power);

/** \brief Switch on power
  * \param[in] power The power module to be switched on.
  * \return The resulting error code.
  */
int robox_power_up(
  robox_power_p power);

/** \brief Switch off power
  * \param[in] power The power module to be switched off.
  */
void robox_power_down(
  robox_power_p power);

#endif
