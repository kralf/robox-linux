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

#ifndef ROBOX_DEVICE_H
#define ROBOX_DEVICE_H

#include <unistd.h>

/** \brief Predefined RoboX device error codes
  */
#define ROBOX_DEVICE_ERROR_NONE                  0
#define ROBOX_DEVICE_ERROR_OPEN                  1
#define ROBOX_DEVICE_ERROR_CLOSE                 2
#define ROBOX_DEVICE_ERROR_TIMEOUT               5
#define ROBOX_DEVICE_ERROR_READ                  3
#define ROBOX_DEVICE_ERROR_WRITE                 4

/** \brief Device type enumeratable type
  */
typedef enum {
  robox_device_input = 0,      //!< Input device.
  robox_device_output = 1,     //!< Output device.
} robox_device_type_t;

/** \brief Predefined RoboX device error descriptions
  */
extern const char* robox_device_errors[];

/** \brief Structure defining a RoboX device
  */
typedef struct robox_device_t {
  char name[256];             //!< The name of the RoboX device.
  robox_device_type_t type;   //!< The type of the RoboX device.
  int fd;                     //!< The file descriptor of the RoboX device.

  double timeout;             //!< Device select timeout in [s].

  ssize_t num_read;           //!< Number of values read from device.
  ssize_t num_written;        //!< Number of values written to device.
} robox_device_t, *robox_device_p;

/** \brief Open RoboX device
  * \param[in] dev The RoboX device to be opened.
  * \param[in] name The name of the device.
  * \param[in] type The type of the device.
  * \param[in] timeout The select timeout of the device in [s].
  * \return The resulting error code.
  */
int robox_device_open(
  robox_device_p dev,
  const char* name,
  robox_device_type_t type,
  double timeout);

/** \brief Close RoboX device
  * \param[in] dev The opened RoboX device to be closed.
  * \return The resulting error code.
  */
int robox_device_close(
  robox_device_p dev);

/** \brief Read from RoboX device
  * \param[in] dev The RoboX device to be read.
  * \param[out] value The value read from the device.
  * \return The resulting error code.
  */
int robox_device_read(
  robox_device_p dev,
  int* value);

/** \brief Write to RoboX device
  * \param[in] dev The RoboX device to be written.
  * \param[in] value The value that will be written to the device.
  * \return The resulting error code.
  */
int robox_device_write(
  robox_device_p dev,
  int value);

#endif
