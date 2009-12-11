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

#ifndef ROBOX_H
#define ROBOX_H

#include <config.h>
#include <thread.h>

#include "security.h"
#include "power.h"
#include "sensors.h"
#include "encoders.h"
#include "bumper.h"
#include "motors.h"

#include "drive.h"
#include "odometry.h"
#include "control.h"

/** \brief Predefined RoboX constants
  */
#define ROBOX_CONFIG_ARG_PREFIX                   "--robox-"

#define ROBOX_PARAMETER_MODEL                     "model"

#define ROBOX_PARAMETER_SECURITY_ESTOP_DEV        "security-estop-dev"
#define ROBOX_PARAMETER_SECURITY_SSTOP_DEV        "security-sstop-dev"
#define ROBOX_PARAMETER_SECURITY_WATCHDOG_DEV     "security-watchdog-dev"
#define ROBOX_PARAMETER_SECURITY_FLASHLIGHT_DEV   "security-flashlight-dev"
#define ROBOX_PARAMETER_POWER_ENGAGE_DEV          "power-engage-dev"
#define ROBOX_PARAMETER_POWER_BATTERY_DEV         "power-battery-dev"
#define ROBOX_PARAMETER_SENSORS_CHECK_DEV         "sensors-check-dev"
#define ROBOX_PARAMETER_SENSORS_OK_DEV            "sensors-ok-dev"
#define ROBOX_PARAMETER_ENCODER_RIGHT_DEV         "enc-right-dev"
#define ROBOX_PARAMETER_ENCODER_LEFT_DEV          "enc-left-dev"
#define ROBOX_PARAMETER_BUMPER_DEV_DIR            "bumper-dev-dir"
#define ROBOX_PARAMETER_MOTOR_ENABLE_DEV          "motor-enable-dev"
#define ROBOX_PARAMETER_MOTOR_RIGHT_DEV           "motor-right-dev"
#define ROBOX_PARAMETER_MOTOR_LEFT_DEV            "motor-left-dev"
#define ROBOX_PARAMETER_MOTOR_BRAKE_DEV           "motor-brake-dev"

#define ROBOX_PARAMETER_ENCODER_PULSES            "enc-pulses"
#define ROBOX_PARAMETER_GEAR_TRANSMISSION         "gear-trans"
#define ROBOX_PARAMETER_WHEEL_BASE                "wheel-base"
#define ROBOX_PARAMETER_WHEEL_RIGHT_RADIUS        "wheel-right-radius"
#define ROBOX_PARAMETER_WHEEL_LEFT_RADIUS         "wheel-left-radius"

#define ROBOX_PARAMETER_CONTROL_P_GAIN            "control-p-gain"
#define ROBOX_PARAMETER_CONTROL_I_GAIN            "control-i-gain"
#define ROBOX_PARAMETER_CONTROL_D_GAIN            "control-d-gain"

/** \brief Predefined RoboX error codes
  */
#define ROBOX_ERROR_NONE                      0
#define ROBOX_ERROR_INIT                      1
#define ROBOX_ERROR_START                     2
#define ROBOX_ERROR_RESET                     3

/** \brief Predefined RoboX error descriptions
  */
extern const char* robox_errors[];

/** \brief Predefined RoboX default configuration
  */
extern config_t robox_default_config;

/** \brief RoboX model enumeratable type
  */
typedef enum {
  robox_model_robox = 0,            //!< Standard RoboX model.
  robox_model_biba = 1,             //!< Biba model.
} robox_model_t;

/** \brief Structure defining the RoboX robot
  */
typedef struct robox_robot_t {
  robox_model_t model;              //!< The robot's model.

  robox_security_t security;        //!< The robot's security module.
  robox_power_t power;              //!< The robot's power module.
  robox_sensors_t sensors;          //!< The robot's sensor module.
  robox_encoders_t encoders;        //!< The robot's encoder module.
  robox_bumper_t bumper;            //!< The robot's bumper.
  robox_motors_t motors;            //!< The robot's motors.

  robox_drive_t drive;              //!< The robot's drive.
  robox_odometry_t odometry;        //!< The robot's odometry.
  robox_control_t control;          //!< The robot's controller.

  config_t config;                  //!< The robot's configuration parameters.

  thread_t thread;                  //!< The robot's main thread.
  thread_mutex_t mutex;             //!< The access mutex of the robot.

  int security_error;               //!< The robot's recent security error.
  robox_drive_pose_t pose;          //!< The robot's recent pose.
  robox_drive_vel_t velocity;       //!< The robot's recent velocity.

  robox_drive_vel_t set_vel;        //!< The robot's setpoint velocity.
} robox_robot_t, *robox_robot_p;

/** \brief Initialize robot
  * \param[in] robot The robot to be initialized.
  * \param[in] config The optional robot configuration parameters. Can be null.
  * \return The resulting error code.
  */
int robox_init(
  robox_robot_p robot,
  config_p config);

/** \brief Destroy robot
  * \param[in] robot The robot to be destroyed.
  */
void robox_destroy(
  robox_robot_p robot);

/** \brief Start robot
  * \param[in] robot The initialized robot to be started.
  * \param[in] frequency The main thread cycle frequency in [Hz].
  * \return The resulting error code.
  */
int robox_start(
  robox_robot_p robot,
  double frequency);

/** \brief Stop robot
  * \param[in] robot The started robot to be stopped.
  * \return The resulting error code.
  */
int robox_stop(
  robox_robot_p robot);

/** \brief Reset robot
  * \note This method safely resets the robot's velocity and odometry.
  * \param[in] robot The started robot to be reset.
  * \return The resulting error code.
  */
int robox_reset(
  robox_robot_p robot);

/** \brief Receive the robot's recent state
  * \note This method safely accesses the robot's pose and velocity.
  * \param[in] robot The started robot to receive the state for.
  * \param[out] pose The recent pose of the robot.
  * \param[out] velocity The recent velocity of the robot.
  */
void robox_get_state(
  robox_robot_p robot,
  robox_drive_pose_p pose,
  robox_drive_vel_p velocity);

/** \brief Set the robot's velocity
  * \note This method safely accesses the robot's setpoint velocity.
  * \param[in] robot The started robot to set the velocity for.
  * \param[out] velocity The velocity to be set for the robot.
  */
void robox_set_velocity(
  robox_robot_p robot,
  robox_drive_vel_p velocity);

#endif
