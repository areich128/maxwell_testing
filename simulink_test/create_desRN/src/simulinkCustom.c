#include "simulinkCustom.h"
#include <stdint.h>

uint64_t g_err = 0;             //Error variable
uint64_t g_err_temp = 0;        // Temporary error variable, cleared next loop
uint32_t g_loop_time = 0;       // Time in current loop
uint32_t g_J2000_time = J2000_MISSION_EPOCH_DAY;    // Current time in seconds
float g_J2000_frac_time = 0.0;  // Current time in seconds (fractional)