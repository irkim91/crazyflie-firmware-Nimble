#include "controller_manual.h"
#include "log.h"
#include "param.h"

static struct {
  float rollScale;
  float pitchScale;
  float yawScale;
  float thrustScale;
} manualConfig = {
    .rollScale = 32767.0f,
    .pitchScale = 32767.0f,
    .yawScale = 32767.0f,
    .thrustScale = 1.0f,
};

void controllerManualInit(void) {}

bool controllerManualTest(void) { return true; }

void controllerManual(control_t *control, const setpoint_t *setpoint,
                      const sensorData_t *sensors, const state_t *state,
                      const stabilizerStep_t stabilizerStep) {
  control->controlMode = controlModeLegacy;

  // Direct mapping of setpoint to control with scaling
  control->thrust = setpoint->thrust * manualConfig.thrustScale;
  control->roll = (int16_t)(setpoint->attitude.roll * manualConfig.rollScale);
  control->pitch =
      (int16_t)(setpoint->attitude.pitch * manualConfig.pitchScale);
  control->yaw = (int16_t)(setpoint->attitudeRate.yaw * manualConfig.yawScale);
}

/**
 * Tuning settings for the manual controller
 */
PARAM_GROUP_START(ctrlMan)
/**
 * @brief Scaling factor for roll command (default: 1.0)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, rollScale, &manualConfig.rollScale)
/**
 * @brief Scaling factor for pitch command (default: 1.0)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitchScale, &manualConfig.pitchScale)
/**
 * @brief Scaling factor for yaw command (default: 1.0)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yawScale, &manualConfig.yawScale)
/**
 * @brief Scaling factor for thrust command (default: 65535.0)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, thrustScale,
          &manualConfig.thrustScale)
PARAM_GROUP_STOP(ctrlMan)
