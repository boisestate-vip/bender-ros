#include "bender_utils.h"

float clamp(const float val, const float min_val, const float max_val)
{
    return min(max(val, min_val), max_val);
}

float absf(const float val)
{
    return val > 0 ? val : -val;
}