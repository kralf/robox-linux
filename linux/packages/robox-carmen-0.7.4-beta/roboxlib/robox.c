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

#include "robox.h"

#include "global.h"

const char* robox_errors[] = {
  "success",
  "error initializing robot",
  "error starting robot",
  "error resetting robot",
};

void* robox_run(void* arg);
void robox_cleanup(void* arg);

param_t robox_default_params[] = {
  {ROBOX_PARAMETER_MODEL, "0"},

  {ROBOX_PARAMETER_SECURITY_ESTOP_DEV, "/dev/robox/security/stop/emergency"},
  {ROBOX_PARAMETER_SECURITY_SSTOP_DEV, "/dev/robox/security/stop/supervisor"},
  {ROBOX_PARAMETER_SECURITY_WATCHDOG_DEV, "/dev/robox/security/watchdog"},
  {ROBOX_PARAMETER_SECURITY_FLASHLIGHT_DEV, "/dev/robox/security/flashlight"},
  {ROBOX_PARAMETER_POWER_ENGAGE_DEV, "/dev/robox/power/engage"},
  {ROBOX_PARAMETER_POWER_BATTERY_DEV, "/dev/robox/power/battery"},
  {ROBOX_PARAMETER_SENSORS_CHECK_DEV, "/dev/robox/sensors/check"},
  {ROBOX_PARAMETER_SENSORS_OK_DEV, "/dev/robox/sensors/ok"},
  {ROBOX_PARAMETER_ENCODER_RIGHT_DEV, "/dev/robox/sensors/encoders/right"},
  {ROBOX_PARAMETER_ENCODER_LEFT_DEV, "/dev/robox/sensors/encoders/left"},
  {ROBOX_PARAMETER_BUMPER_DEV_DIR, "/dev/robox/sensors/bumper"},
  {ROBOX_PARAMETER_MOTOR_ENABLE_DEV, "/dev/robox/drive/motor/enable"},
  {ROBOX_PARAMETER_MOTOR_RIGHT_DEV, "/dev/robox/drive/motor/right"},
  {ROBOX_PARAMETER_MOTOR_LEFT_DEV, "/dev/robox/drive/motor/left"},
  {ROBOX_PARAMETER_MOTOR_BRAKE_DEV, "/dev/robox/drive/brake/disengage"},

  {ROBOX_PARAMETER_ENCODER_PULSES, "500"},
  {ROBOX_PARAMETER_GEAR_TRANSMISSION, "50.0"},
  {ROBOX_PARAMETER_WHEEL_BASE, "0.545"},
  {ROBOX_PARAMETER_WHEEL_RIGHT_RADIUS, "0.09"},
  {ROBOX_PARAMETER_WHEEL_LEFT_RADIUS, "0.09"},

  {ROBOX_PARAMETER_CONTROL_P_GAIN, "1e-1"},
  {ROBOX_PARAMETER_CONTROL_I_GAIN, "1e-4"},
  {ROBOX_PARAMETER_CONTROL_D_GAIN, "-4e-2"},
};
 
config_t robox_default_config = {
  robox_default_params,
  sizeof(robox_default_params)/sizeof(param_t),
};

int robox_init(robox_robot_p robot, config_p config) {
  thread_mutex_init(&robot->mutex);
  config_init_default(&robot->config, &robox_default_config);
  if (config)
    config_set(&robot->config, config);

  robot->model = config_get_int(&robot->config, ROBOX_PARAMETER_MODEL);

  robot->security_error = ROBOX_SECURITY_ERROR_NONE;
  memset(&robot->pose, 0, sizeof(robox_drive_pose_t));
  memset(&robot->velocity, 0, sizeof(robox_drive_vel_t));

  memset(&robot->set_vel, 0, sizeof(robox_drive_vel_t));

  if (!robox_security_init(&robot->security, 
      config_get_string(&robot->config, ROBOX_PARAMETER_SECURITY_ESTOP_DEV),
      config_get_string(&robot->config, ROBOX_PARAMETER_SECURITY_SSTOP_DEV),
      config_get_string(&robot->config, ROBOX_PARAMETER_SECURITY_WATCHDOG_DEV),
      config_get_string(&robot->config, 
        ROBOX_PARAMETER_SECURITY_FLASHLIGHT_DEV)) && 
    !robox_power_init(&robot->power, 
      config_get_string(&robot->config, ROBOX_PARAMETER_POWER_ENGAGE_DEV),
      config_get_string(&robot->config, ROBOX_PARAMETER_POWER_BATTERY_DEV)) &&
    !robox_sensors_init(&robot->sensors, 
      config_get_string(&robot->config, ROBOX_PARAMETER_SENSORS_CHECK_DEV),
      config_get_string(&robot->config, ROBOX_PARAMETER_SENSORS_OK_DEV)) &&
    !robox_encoders_init(&robot->encoders, 
      config_get_string(&robot->config, ROBOX_PARAMETER_ENCODER_LEFT_DEV),
      config_get_string(&robot->config, ROBOX_PARAMETER_ENCODER_RIGHT_DEV),
      config_get_int(&robot->config, ROBOX_PARAMETER_ENCODER_PULSES)) &&
    !robox_bumper_init(&robot->bumper, 
      config_get_string(&robot->config, ROBOX_PARAMETER_BUMPER_DEV_DIR)) &&
    !robox_motors_init(&robot->motors, 
      config_get_string(&robot->config, ROBOX_PARAMETER_MOTOR_ENABLE_DEV),
      config_get_string(&robot->config, ROBOX_PARAMETER_MOTOR_RIGHT_DEV),
      config_get_string(&robot->config, ROBOX_PARAMETER_MOTOR_LEFT_DEV),
      config_get_string(&robot->config, ROBOX_PARAMETER_MOTOR_BRAKE_DEV))) {
    robox_drive_init(&robot->drive, 
      config_get_float(&robot->config, ROBOX_PARAMETER_GEAR_TRANSMISSION),
      config_get_float(&robot->config, ROBOX_PARAMETER_WHEEL_BASE),
      config_get_float(&robot->config, ROBOX_PARAMETER_WHEEL_RIGHT_RADIUS),
      config_get_float(&robot->config, ROBOX_PARAMETER_WHEEL_LEFT_RADIUS));
    robox_odometry_init(&robot->odometry, &robot->encoders, &robot->drive);
    robox_control_init(&robot->control, 
      &robot->encoders, &robot->motors, &robot->drive, 
      config_get_float(&robot->config, ROBOX_PARAMETER_CONTROL_P_GAIN),
      config_get_float(&robot->config, ROBOX_PARAMETER_CONTROL_I_GAIN),
      config_get_float(&robot->config, ROBOX_PARAMETER_CONTROL_D_GAIN));

    return ROBOX_ERROR_NONE;
  }
  else
    return ROBOX_ERROR_INIT;
}

void robox_destroy(robox_robot_p robot) {
  robox_control_destroy(&robot->control);
  robox_odometry_destroy(&robot->odometry);

  robox_motors_destroy(&robot->motors);
  robox_bumper_destroy(&robot->bumper);
  robox_encoders_destroy(&robot->encoders);
  robox_sensors_destroy(&robot->sensors);
  robox_power_destroy(&robot->power);
  robox_security_destroy(&robot->security);

  config_destroy(&robot->config);
  thread_mutex_destroy(&robot->mutex);
}

int robox_start(robox_robot_p robot, double frequency) {
  if (!robox_power_up(&robot->power) &&
    !robox_security_start(&robot->security) &&
    !robox_motors_start(&robot->motors) &&
    !robox_odometry_start(&robot->odometry) &&
    !robox_control_start(&robot->control) &&
    !thread_start(&robot->thread, robox_run, robox_cleanup, robot, frequency))
    return ROBOX_ERROR_NONE;
  else
    return ROBOX_ERROR_START;
}

int robox_stop(robox_robot_p robot) {
  thread_exit(&robot->thread, 1);
  return ROBOX_ERROR_NONE;
}

int robox_reset(robox_robot_p robot) {
  int result = ROBOX_ERROR_RESET;

  thread_mutex_lock(&robot->mutex);

  if (!robox_control_start(&robot->control) &&
    !robox_odometry_start(&robot->odometry)) {
    memset(&robot->pose, 0, sizeof(robox_drive_pose_t));
    memset(&robot->velocity, 0, sizeof(robox_drive_vel_t));

    memset(&robot->set_vel, 0, sizeof(robox_drive_vel_t));

    result = ROBOX_ERROR_NONE;
  }

  thread_mutex_unlock(&robot->mutex);

  return result;
}

void robox_get_state(robox_robot_p robot, robox_drive_pose_p pose, 
  robox_drive_vel_p velocity) {
  thread_mutex_lock(&robot->mutex);

  *pose = robot->pose;
  *velocity = robot->velocity;

  thread_mutex_unlock(&robot->mutex);
}

void robox_set_velocity(robox_robot_p robot, robox_drive_vel_p velocity) {
  thread_mutex_lock(&robot->mutex);

  robot->set_vel = *velocity;

  thread_mutex_unlock(&robot->mutex);
}

void* robox_run(void* arg) {
  int security_error;
  robox_robot_p robot = arg;

  thread_mutex_lock(&robot->mutex);

  if (!(security_error = robox_security_check(&robot->security, 
    &robot->bumper))) {
    if (robot->security_error) {
      robox_motors_start(&robot->motors);
      robox_control_start(&robot->control);
    }
    else
      robox_control_iterate(&robot->control, &robot->set_vel);
  }
  else
    robox_motors_stop(&robot->motors);

  robot->security_error = security_error;
  robox_odometry_integrate(&robot->odometry, &robot->pose, 
    &robot->velocity);

  thread_mutex_unlock(&robot->mutex);

  return 0;
}

void robox_cleanup(void* arg) {
  robox_robot_p robot = arg;

  robox_motors_stop(&robot->motors);
  robox_security_stop(&robot->security);
  robox_power_down(&robot->power);
}
