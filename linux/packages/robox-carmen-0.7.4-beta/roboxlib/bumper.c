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

#include <stdlib.h>
#include <stdio.h>
#include <dirent.h>

#include "bumper.h"

int robox_bumper_init(robox_bumper_p bumper, const char* dev_dir) {
  char dev_name[256];
  int result = ROBOX_DEVICE_ERROR_NONE;

  bumper->segment_devs = 0;
  bumper->num_segments = 0;
  DIR* dir = opendir(dev_dir);

  if (dir) {
    struct dirent* dev;

    while ((dev = readdir(dir))) {
      if ((dev->d_type == DT_CHR) || (dev->d_type == DT_LNK)) {
        bumper->segment_devs = realloc(bumper->segment_devs, 
          (bumper->num_segments+1)*sizeof(robox_device_t));
        sprintf(dev_name, "%s/%s", dev_dir, dev->d_name);
  
        if (!(result = robox_device_open(
          &bumper->segment_devs[bumper->num_segments], dev_name,  
          robox_device_input, ROBOX_BUMPER_READ_TIMEOUT)))
          ++bumper->num_segments;
        else
          break;
      }
    }

    closedir(dir);
  }
  
  return result;
}

int robox_bumper_destroy(robox_bumper_p bumper) {
  int i, result = ROBOX_DEVICE_ERROR_NONE;

  for (i = bumper->num_segments-1; i >= 0; --i) {
    if (!(result = robox_device_close(&bumper->segment_devs[i])))
      --bumper->num_segments;
    else
      break;
  }

  if (!bumper->num_segments)
    free(bumper->segment_devs);

  return result;
}

robox_bumper_state_t robox_bumper_get_state(robox_bumper_p bumper) {
  int i;
  robox_bumper_state_t segment_states[bumper->num_segments];

  if (!robox_bumper_get_segment_states(bumper, segment_states)) {
    for (i = 0; i < bumper->num_segments; ++i)
      if (segment_states[i] == robox_bumper_pressed)
      return robox_bumper_pressed;
  }
  else
    return robox_bumper_pressed;

  return robox_bumper_released;
}

int robox_bumper_get_segment_states(robox_bumper_p bumper, robox_bumper_state_t 
  segment_states[]) {
  int i, result = ROBOX_DEVICE_ERROR_NONE;

  for (i = 0; i < bumper->num_segments; ++i) {
    int val;
    if (!(result = robox_device_read(&bumper->segment_devs[i], &val))) {
      segment_states[i] = (val) ? robox_bumper_pressed : robox_bumper_released;
    }
    else
      break;
  }

  return result;
}
