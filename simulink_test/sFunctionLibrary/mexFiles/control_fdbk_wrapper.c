
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#define SIMPLIFIED_RTWTYPES_COMPATIBILITY
#include "rtwtypes.h"
#undef SIMPLIFIED_RTWTYPES_COMPATIBILITY
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>
#include "global.h"
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define u_1_width 4
#define u_2_width 4
#define u_3_width 3
#define u_4_width 3
#define u_5_width 3
#define u_6_width 1
#define u_7_width 4
#define y_width 6
#define y_1_width 4

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Output function
 *
 */
extern void control_fdbk_Outputs_wrapper(const uint8_T *op_mode,
			const real32_T *q_BN,
			const real32_T *q_des_RN,
			const real32_T *gyro_rates,
			const real32_T *mag_bf,
			const real32_T *ctl_gain,
			const boolean_T *gps_result_override,
			const real32_T *q_BR_noGPS,
			real32_T *out_u,
			real32_T *q_BR);

void control_fdbk_Outputs_wrapper(const uint8_T *op_mode,
			const real32_T *q_BN,
			const real32_T *q_des_RN,
			const real32_T *gyro_rates,
			const real32_T *mag_bf,
			const real32_T *ctl_gain,
			const boolean_T *gps_result_override,
			const real32_T *q_BR_noGPS,
			real32_T *out_u,
			real32_T *q_BR)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
// Unitialised memory - please fix me
float * pos_eci;
float * pos_vel;
float * I_RW_values;
float * GsValues;
float * JsValues;
float * wheel_speeds;
gps_result = *gps_result_override;

real32_T q_BR_temp[4];
if (!gps_result)
{
    q_BR_temp[0] = q_BR_noGPS[0];
    q_BR_temp[1] = q_BR_noGPS[1];
    q_BR_temp[2] = q_BR_noGPS[2];
    q_BR_temp[3] = q_BR_noGPS[3];
}
control_fdbk(*op_mode, pos_eci, pos_vel, q_BN, q_des_RN,
             q_BR_temp, gyro_rates, mag_bf, ctl_gain, I_RW_values,
             GsValues, JsValues, wheel_speeds, out_u);

// Extract qBR
q_BR[0] = q_BR_temp[0];
q_BR[1] = q_BR_temp[1];
q_BR[2] = q_BR_temp[2];
q_BR[3] = q_BR_temp[3];
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


