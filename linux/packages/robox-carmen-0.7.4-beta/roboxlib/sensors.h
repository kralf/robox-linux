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

#ifndef ROBOX_SENSORS_H
#define ROBOX_SENSORS_H

#include "device.h"

/** \brief Predefined RoboX sensor constants
  */
#define ROBOX_SENSORS_READ_TIMEOUT               0.01

/** \brief Structure defining the RoboX encoders
  */
typedef struct robox_sensors_t {
  robox_device_t check_dev;     //!< The sensor check device.
  robox_device_t ok_dev;        //!< The sensor okay device.
} robox_sensors_t, *robox_sensors_p;

/** \brief Initialize RoboX sensors
  * \param[in] sensors The RoboX sensors to be initialized.
  * \param[in] check_dev The name of the sensor check device.
  * \param[in] ok_dev The name of the sensor okay device.
  * \return The resulting device error code.
  */
int robox_sensors_init(
  robox_sensors_p sensors,
  const char* check_dev,
  const char* ok_dev);

/** \brief Destroy RoboX sensors
  * \param[in] sensors The initialized RoboX sensors to be destroyed.
  * \return The resulting device error code.
  */
int robox_sensors_destroy(
  robox_sensors_p sensors);

#endif
