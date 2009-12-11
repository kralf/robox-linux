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

#ifndef ROBOX_BUMPER_H
#define ROBOX_BUMPER_H

#include "device.h"

/** \brief Predefined RoboX bumper constants
  */
#define ROBOX_BUMPER_READ_TIMEOUT               0.01

/** \brief Bumper state enumeratable type
  */
typedef enum {
  robox_bumper_released = 0,    //!< Bumper released.
  robox_bumper_pressed = 1,     //!< Bumper pressed.
} robox_bumper_state_t;

/** \brief Structure defining the RoboX bumper
  */
typedef struct robox_bumper_t {
  robox_device_p segment_devs;     //!< The bumper segment devices.
  ssize_t num_segments;            //!< The number of bumper segments.
} robox_bumper_t, *robox_bumper_p;

/** \brief Initialize RoboX bumper
  * \param[in] bumper The RoboX bumper to be initialized.
  * \param[in] dev_dir The name of the RoboX bumper device directory.
  * \return The resulting device error code.
  */
int robox_bumper_init(
  robox_bumper_p bumper,
  const char* dev_dir);

/** \brief Destroy RoboX bumper
  * \param[in] bumper The initialized RoboX bumper to be destroyed.
  * \return The resulting device error code.
  */
int robox_bumper_destroy(
  robox_bumper_p bumper);

/** \brief Retrieve bumper state
  * \param[in] bumper The bumper to retrieve the state for.
  * \return The retrieved bumper state.
  */
robox_bumper_state_t robox_bumper_get_state(
  robox_bumper_p bumper);

/** \brief Retrieve bumper segment states
  * \param[in] bumper The bumper to retrieve the segment states for.
  * \return The resulting device error code.
  */
int robox_bumper_get_segment_states(
  robox_bumper_p bumper,
  robox_bumper_state_t segment_states[]);

#endif
