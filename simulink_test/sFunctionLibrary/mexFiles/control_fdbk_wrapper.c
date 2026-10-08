
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
#include "mtx.h"
#include "att_det.h"
#include "ctl_alg.h"
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define u_1_width 4
#define u_2_width 4
#define u_3_width 3
#define u_4_width 3
#define u_5_width 4
#define u_6_width 1
#define u_7_width 4
#define u_8_width 3
#define u_9_width 1
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
			const real32_T *des_rates,
			const real_T *u1,
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
			const real32_T *des_rates,
			const real_T *u1,
			real32_T *out_u,
			real32_T *q_BR)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
float gyro_loc[3], des_loc[3], mag_loc[3], gain_loc[4];
float q_BN_loc[4], q_RN_loc[4], q_BR_loc[4];
float out_loc[7] = {0.0f};
float dcm_BN_arr[9], dcm_RN_arr[9], dcm_NR_arr[9];
struct mtx_matrix q_BN_m, q_RN_m, dcm_BN_m, dcm_RN_m, dcm_NR_m;
int i;

for (i = 0; i < 3; i++) {
    gyro_loc[i] = gyro_rates[i];
    des_loc[i]  = des_rates[i];
    mag_loc[i]  = mag_bf[i];
}
for (i = 0; i < 4; i++) {
    gain_loc[i] = ctl_gain[i];
    q_BN_loc[i] = q_BN[i];
    q_RN_loc[i] = q_des_RN[i];
    q_BR_loc[i] = q_BR_noGPS[i];
}

gps_result = (uint8_t)(*gps_result_override);

mtx_create(4, 1, q_BN_loc, &q_BN_m);
mtx_create(4, 1, q_RN_loc, &q_RN_m);
mtx_create(3, 3, dcm_BN_arr, &dcm_BN_m);
mtx_create(3, 3, dcm_RN_arr, &dcm_RN_m);
mtx_create(3, 3, dcm_NR_arr, &dcm_NR_m);
q_2_dcm(&q_BN_m, &dcm_BN_m);
q_2_dcm(&q_RN_m, &dcm_RN_m);
mtx_trans(&dcm_RN_m, &dcm_NR_m);

control_fdbk(*op_mode, &dcm_BN_m, &dcm_NR_m, q_BR_loc,
             gyro_loc, des_loc, mag_loc, gain_loc, out_loc);

for (i = 0; i < 6; i++) out_u[i] = out_loc[i];
for (i = 0; i < 4; i++) q_BR[i]  = q_BR_loc[i];
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


