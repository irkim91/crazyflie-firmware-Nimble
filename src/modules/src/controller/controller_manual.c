#include "controller_manual.h"
#include "log.h"
#include "param.h"
#include <stdbool.h>
#include <stdint.h>

static struct {
  float rollScale;
  float pitchScale;
  float yawScale;
  float thrustScale;
} manualConfig = {
    .rollScale = 327.67f,
    .pitchScale = 327.67f,
    .yawScale = 327.67f,
    .thrustScale = 1.0f,
};

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;

static int32_t limitCommand(int32_t value, int32_t min, int32_t max,
                            bool *isCapped) {
  if (value < min) {
    return min;
  }

  if (value > max) {
    *isCapped = true;
    return max;
  }

  return value;
}

void controllerManualInit(void) {}

bool controllerManualTest(void) { return true; }

void controllerManual(control_t *control, const setpoint_t *setpoint,
                      const sensorData_t *sensors, const state_t *state,
                      const stabilizerStep_t stabilizerStep) {
  control->controlMode = controlModeLegacy;

  cmd_thrust = setpoint->thrust;
  cmd_roll = setpoint->attitude.roll;
  cmd_pitch = setpoint->attitude.pitch;
  cmd_yaw = setpoint->attitudeRate.yaw;

  // Direct mapping of setpoint to control with scaling and limiting
  bool isCapped = false; // Shared flag, though distinct warning could be useful

  int32_t raw_thrust = (int32_t)(setpoint->thrust * manualConfig.thrustScale);
  control->thrust = (float)limitCommand(raw_thrust, 0, UINT16_MAX, &isCapped);

  int32_t raw_roll =
      (int32_t)(setpoint->attitude.roll * manualConfig.rollScale);
  control->roll =
      (int16_t)limitCommand(raw_roll, INT16_MIN, INT16_MAX, &isCapped);

  int32_t raw_pitch =
      (int32_t)(setpoint->attitude.pitch * manualConfig.pitchScale);
  control->pitch =
      (int16_t)limitCommand(raw_pitch, INT16_MIN, INT16_MAX, &isCapped);

  int32_t raw_yaw =
      (int32_t)(setpoint->attitudeRate.yaw * manualConfig.yawScale);
  control->yaw =
      (int16_t)limitCommand(raw_yaw, INT16_MIN, INT16_MAX, &isCapped);
}

/**
 * Tuning settings for the manual controller
 */
PARAM_GROUP_START(ctrlMan)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, rollScale, &manualConfig.rollScale)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitchScale, &manualConfig.pitchScale)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yawScale, &manualConfig.yawScale)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, thrustScale,
          &manualConfig.thrustScale)
PARAM_GROUP_STOP(ctrlMan)

/**
 * Logging variables for the manual controller inputs
 */
LOG_GROUP_START(ctrlMan)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_GROUP_STOP(ctrlMan)
