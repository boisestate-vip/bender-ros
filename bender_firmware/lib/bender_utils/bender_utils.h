#ifndef BENDER_FIRMWARE_BENDER_UTILS_H
#define BENDER_FIRMWARE_BENDER_UTILS_H

// #define M_PI_2 1.5707963267948966f
#define RPM_TO_RAD_S 0.10471975511965977f
#define RAD_S_TO_RPM 9.549296585513721F

#include <Arduino.h>

float clamp(const float val, const float min_val, const float max_val);
float absf(const float val);

#endif  // BENDER_FIRMWARE_BENDER_UTILS_H
