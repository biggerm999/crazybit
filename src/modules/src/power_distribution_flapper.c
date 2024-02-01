/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB & Flapper Drones (https:\\flapper-drones.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * power_distribution_flapper.c - Power distribution code for the Flapper Nimble+
 */
#define DEBUG_MODULE "PWR_DIST"

#include "power_distribution.h"

#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "autoconf.h"
#include "config.h"

#include "debug.h"
#include "math.h"
#include "cfassert.h"

#ifndef CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#  define DEFAULT_IDLE_THRUST 0
#else
#  define DEFAULT_IDLE_THRUST CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;

struct flapperConfig_s {
  uint8_t pitchServoNeutral;
  uint8_t yawServoNeutral;
  int8_t rollBias;
  uint16_t maxThrust;
};

struct flapperConfig_s flapperConfig = {
  .pitchServoNeutral = 50,
  .yawServoNeutral = 50,
  .rollBias = 0,
  .maxThrust = 60000,
};

static float thrust;
static uint16_t act_max = 65535;

const int16_t inc_L10 = 90;
const int16_t inc_R10 = 90;
const int16_t amp_L10 = -35;
const int16_t amp_R10 = 35;
const int16_t inc_L20 = (90 + 28);
const int16_t inc_R20 = (90 - 28);
const int16_t amp_L20 = 35;
const int16_t amp_R20 = -35;

const float D = 0.6;

const int16_t nf = 2;
// r1 = 168, r2 = 20, r3 = 168, r4 = 15
// amp1 = 30, f1 = (0.35)
const double a1[2] = {0.878152, 0.131394};
const double b1[2] = {-0.447441, 0.095463};

// flex_max = 55, f2 = (0.3, 0.65, 0.65)
const double a2[2] = {0.219197, 0.035761};
const double b2[2] = {0.301966, -0.110659};


void powerDistributionInit(void);
bool powerDistributionTest(void);

// static float pitch_ampl = 0.4f; // 1 = full servo stroke

#if CONFIG_POWER_DISTRIBUTION_FLAPPER_REVB
  uint32_t idPitch = 1;
  uint32_t idYaw = 2;
#else
  uint32_t idPitch = 0;
  uint32_t idYaw = 2;
#endif

static uint8_t limitServoNeutral(uint8_t value)
{
  if(value > 75)
  {
    value = 75;
  }
  else if(value < 25)
  {
    value = 25;
  }

  return (uint8_t)value;
}

static int8_t limitRollBias(uint8_t value)
{
  if(value > 25)
  {
    value = 25;
  }
  else if(value < -25)
  {
    value = -25;
  }

  return (uint8_t)value;
}

int powerDistributionMotorType(uint32_t id)
{
  int type = 1;
  if (id == idPitch || id == idYaw)
  {
    type = 0;
  }

  return type;
}

uint16_t powerDistributionStopRatio(uint32_t id)
{
  uint16_t stopRatio = 0;
  if (id == idPitch)
  {
    stopRatio = flapperConfig.pitchServoNeutral*act_max/100.0f;
  }
  else if (id == idYaw)
  {
    stopRatio = flapperConfig.yawServoNeutral*act_max/100.0f;
  }

  return stopRatio;
}

void powerDistributionInit(void)
{
  #if CONFIG_POWER_DISTRIBUTION_FLAPPER_REVB
    DEBUG_PRINT("Using Flapper power distribution | PCB revB (2021)\n");
  #else
    DEBUG_PRINT("Using Flapper power distribution | PCB revD (2022) or newer\n");
  #endif

}

bool powerDistributionTest(void)
{
  bool pass = true;
  return pass;
}

uint16_t limitThrust(int32_t value, int32_t min, int32_t max, bool* isCapped)
{
  if (value < min) {
    return min;
  }

  if (value > max) {
    *isCapped = true;
    return max;
  }

  return value;
}

void powerDistribution(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
  // Only legacy mode is currently supported
  ASSERT(control->controlMode == controlModeLegacy);


  thrust = fmin(control->thrust, flapperConfig.maxThrust);

  flapperConfig.pitchServoNeutral=limitServoNeutral(flapperConfig.pitchServoNeutral);
  flapperConfig.yawServoNeutral=limitServoNeutral(flapperConfig.yawServoNeutral);
  flapperConfig.rollBias=limitRollBias(flapperConfig.rollBias);

  #if CONFIG_POWER_DISTRIBUTION_FLAPPER_REVB
    motorThrustUncapped->motors.m2 = flapperConfig.pitchServoNeutral*act_max / 100.0f + pitch_ampl * control->pitch; // pitch servo
    motorThrustUncapped->motors.m3 = flapperConfig.yawServoNeutral*act_max / 100.0f - control->yaw; // yaw servo
    motorThrustUncapped->motors.m1 =  0.5f * control->roll + thrust * (1.0f + flapperConfig.rollBias / 100.0f); // left motor
    motorThrustUncapped->motors.m4 = -0.5f * control->roll + thrust * (1.0f - flapperConfig.rollBias / 100.0f); // right motor
  #else
    // motorThrustUncapped->motors.m1 = flapperConfig.pitchServoNeutral * act_max / 100.0f + pitch_ampl * control->pitch; // pitch servo
    // motorThrustUncapped->motors.m3 = flapperConfig.yawServoNeutral*act_max / 100.0f - control->yaw; // yaw servo
    // motorThrustUncapped->motors.m2 =  0.5f * control->roll + thrust * (1.0f + flapperConfig.rollBias / 100.0f); // left motor
    // motorThrustUncapped->motors.m4 = -0.5f * control->roll + thrust * (1.0f - flapperConfig.rollBias / 100.0f); // right motor
    
    float x = 2 * 3.14f * control->step / 12;

    // Set the main wing angles
    // R1 : m1, L1 : m4
    int32_t amp_L1 = amp_L10 + 0.0004 * fmin(0, (control->roll));
    int32_t amp_R1 = amp_R10 + 0.0004 * fmax(0, (control->roll));
    int32_t inc_L1 = inc_L10 + D * amp_L10 - 0.0004f * (control->pitch);
    int32_t inc_R1 = inc_R10 + D * amp_R10 + 0.0004f * (control->pitch);
    
    int32_t servo_cmd_L1 = inc_L1;
    int32_t servo_cmd_R1 = inc_R1;
    for (int i = 1; i <= nf; i++)
    {
      servo_cmd_L1 = servo_cmd_L1 + amp_L1 * (a1[i - 1] * cos(i * x) + b1[i - 1] * sin(i * x));
      servo_cmd_R1 = servo_cmd_R1 + amp_R1 * (a1[i - 1] * cos(i * x) + b1[i - 1] * sin(i * x));
    }
    
    // servo_cmd_xx [deg] -> motorThrustUncapped.motors.mx [uint16_t]
    motorThrustUncapped->motors.m4 = 0x7FFF + (servo_cmd_L1-90) * 362.0f;
    motorThrustUncapped->motors.m1 = 0x7FFF + (servo_cmd_R1-90) * 362.0f;

    int32_t amp_L2 = amp_L20 - 0.0002 * fmax(0, (control->roll));
    int32_t amp_R2 = amp_R20 - 0.0002 * fmin(0, (control->roll));
    int32_t inc_L2 = inc_L20 + 0.0004f * (control->pitch);
    int32_t inc_R2 = inc_R20 - 0.0004f * (control->pitch);

    int32_t servo_cmd_L2 = inc_L2;
    int32_t servo_cmd_R2 = inc_R2;
    for (int i = 1; i <= nf; i++)
    {
      servo_cmd_L2 = servo_cmd_L2 + (amp_L2 * a2[i - 1] - amp_L1 * a1[i - 1]) * cos(i * x) + (amp_L2 * b2[i - 1] - amp_L1 * b1[i - 1]) * sin(i * x);
      servo_cmd_R2 = servo_cmd_R2 + (amp_R2 * a2[i - 1] - amp_R1 * a1[i - 1]) * cos(i * x) + (amp_R2 * b2[i - 1] - amp_R1 * b1[i - 1]) * sin(i * x);
    }

    // Set the folding wing angles
    // R2 : m2, L2 : m3
    motorThrustUncapped->motors.m2 = 0x7FFF + (servo_cmd_L2-90) * 362.0f;
    motorThrustUncapped->motors.m3 = 0x7FFF + (servo_cmd_R2-90) * 362.0f;
  #endif
  

  // motorThrustUncapped->motors.m1 = control->thrust;
  // motorThrustUncapped->motors.m2 = 300 * control->roll;
  // motorThrustUncapped->motors.m3 = 300 * control->pitch;
  // motorThrustUncapped->motors.m4 = 300 * control->yaw;
}

bool powerDistributionCap(const motors_thrust_uncapped_t* motorThrustBatCompUncapped, motors_thrust_pwm_t* motorPwm)
{
  bool isCapped = false;

  #if CONFIG_POWER_DISTRIBUTION_FLAPPER_REVB
    motorPwm->motors.m2 = limitThrust(motorThrustBatCompUncapped->motors.m2, 0, UINT16_MAX, &isCapped); // pitch servo
    motorPwm->motors.m3 = limitThrust(motorThrustBatCompUncapped->motors.m3, 0, UINT16_MAX, &isCapped); // yaw servo
    motorPwm->motors.m1 = limitThrust(motorThrustBatCompUncapped->motors.m1, idleThrust, UINT16_MAX, &isCapped); // left motor
    motorPwm->motors.m4 = limitThrust(motorThrustBatCompUncapped->motors.m4, idleThrust, UINT16_MAX, &isCapped); // right motor
  #else
    motorPwm->motors.m1 = limitThrust(motorThrustBatCompUncapped->motors.m1, 0, UINT16_MAX, &isCapped); // pitch servo
    motorPwm->motors.m3 = limitThrust(motorThrustBatCompUncapped->motors.m3, 0, UINT16_MAX, &isCapped); // yaw servo
    motorPwm->motors.m2 = limitThrust(motorThrustBatCompUncapped->motors.m2, idleThrust, UINT16_MAX, &isCapped); // left motor
    motorPwm->motors.m4 = limitThrust(motorThrustBatCompUncapped->motors.m4, idleThrust, UINT16_MAX, &isCapped); // right motor
  #endif

  return isCapped;
}

uint32_t powerDistributionGetIdleThrust() {
  return idleThrust;
}

/**
 * Power distribution parameters
 */
PARAM_GROUP_START(powerDist)
/**
 * @brief Motor thrust to set at idle (default: 0)
 *
 * This is often needed for brushless motors as
 * it takes time to start up the motor. Then a
 * common value is between 3000 - 6000.
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_PERSISTENT, idleThrust, &idleThrust)
PARAM_GROUP_STOP(powerDist)

/**
 *
 * Flapper Drone configration parameters
 */
PARAM_GROUP_START(flapper)
/**
 * @brief Roll bias <-25%; 25%> (default 0%)
 *
 * This parameter can be used if uneven performance of the left and right flapping mechanaisms and/or wings
 * is observed, which in flight results in a drift in roll/sideways flight. Positive values make the drone roll
 * more to the right, negative values to the left.
 */
PARAM_ADD(PARAM_INT8 | PARAM_PERSISTENT, motBiasRoll, &flapperConfig.rollBias)
/**
 * @brief Pitch servo neutral <25%; 75%> (default 50%)
 *
 * The parameter sets the neutral position of the pitch servo, such that the left and right wing-pairs are
 * aligned when observed from the side. If in flight you observe too much drift forward (nose down) increase the value
 * and vice versa if the drift is backward (nose up).
 */
PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, servPitchNeutr, &flapperConfig.pitchServoNeutral)
/**
 * @brief Yaw servo neutral <25%; 75%> (default 50%)
 *
 * The parameter sets the neutral position of the yaw servo, such that the yaw control arm is pointed spanwise. If in flight
 * you observe drift in the clock-wise direction, increase this parameter and vice-versa if the drift is counter-clock-wise.
 */
PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, servYawNeutr, &flapperConfig.yawServoNeutral)
/**
 * @brief Yaw servo neutral <25%; 75%> (default 50%)
 *
 * The parameter sets the neutral position of the yaw servo, such that the yaw control arm is pointed spanwise. If in flight
 * you observe drift in the clock-wise direction, increase this parameter and vice-versa if the drift is counter-clock-wise.
 */
PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, flapperMaxThrust, &flapperConfig.maxThrust)

PARAM_GROUP_STOP(flapper)
