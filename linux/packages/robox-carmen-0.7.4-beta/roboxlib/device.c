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

#include <stdio.h>
#include <string.h>
#include <fcntl.h>

#include "device.h"

const char* robox_device_errors[] = {
  "success",
  "error opening RoboX device",
  "error closing RoboX device",
  "RoboX device select timeout",
  "error reading from RoboX device",
  "error writing to RoboX device",
};

int robox_device_open(robox_device_p dev, const char* name, robox_device_type_t 
  type, double timeout) {
  int mode = (type == robox_device_output) ? O_RDWR :  O_RDONLY;
  dev->fd = open(name, mode | O_NDELAY);

  if (dev->fd > 0) {
    strcpy(dev->name, name);
    dev->type = type;

    dev->timeout = timeout;

    dev->num_read = 0;
    dev->num_written = 0;
  }
  else {
    fprintf(stderr, "error opening device %s\n", name);
    return ROBOX_DEVICE_ERROR_OPEN;
  }

  return ROBOX_DEVICE_ERROR_NONE;
}

int robox_device_close(robox_device_p dev) {
  if (!close(dev->fd)) {
    dev->name[0] = 0;
    dev->fd = -1;
  }
  else
    return ROBOX_DEVICE_ERROR_CLOSE;

  return ROBOX_DEVICE_ERROR_NONE;
}

int robox_device_read(robox_device_p dev, int* value) {
  char data[256];
  ssize_t num_read = 0;
  struct timeval time;
  fd_set set;
  int error;

  time.tv_sec = 0;
  time.tv_usec = dev->timeout*1e6;

  FD_ZERO(&set);
  FD_SET(dev->fd, &set);

  error = select(dev->fd+1, &set, NULL, NULL, &time);
  if (error == 0)
    return ROBOX_DEVICE_ERROR_TIMEOUT;

  num_read = read(dev->fd, data, sizeof(data));
  if ((num_read > 0) && (sscanf(data, "%i\n", value) == 1))
    ++dev->num_read;
  else
    return ROBOX_DEVICE_ERROR_READ;
  
  return ROBOX_DEVICE_ERROR_NONE;
}

int robox_device_write(robox_device_p dev, int value) {
  char data[256];  
  ssize_t num = sprintf(data, "%i\n", value);
  ssize_t num_written = 0;

  while (num_written < num) {
    ssize_t n;
    while ((n = write(dev->fd, &data[num_written], num-num_written)) == 0);
    if (n > 0)
      num_written += n;
    else
      return ROBOX_DEVICE_ERROR_WRITE;
  }
  ++dev->num_written;
  
  return ROBOX_DEVICE_ERROR_NONE;
}
