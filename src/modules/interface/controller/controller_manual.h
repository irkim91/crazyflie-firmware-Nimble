#ifndef __CONTROLLER_MANUAL_H__
#define __CONTROLLER_MANUAL_H__

#include "stabilizer_types.h"

void controllerManualInit(void);
bool controllerManualTest(void);
void controllerManual(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const stabilizerStep_t stabilizerStep);

#endif //__CONTROLLER_MANUAL_H__
