
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
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define u_1_width 3
#define u_2_width 3
#define u_3_width 4
#define u_4_width 1
#define u_5_width 1
#define u_6_width 1
#define y_width 3

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
extern void create_des_rates_DCOMM_Outputs_wrapper(const uint8_T *opmode,
			const real32_T *pos_ecef,
			const real32_T *gnd_ecef,
			const real32_T *q_BN,
			const uint8_T *leapsec,
			const uint32_T *J2000_time,
			const real32_T *J2000_frac_time,
			real32_T *des_rates_bf);

void create_des_rates_DCOMM_Outputs_wrapper(const uint8_T *opmode,
			const real32_T *pos_ecef,
			const real32_T *gnd_ecef,
			const real32_T *q_BN,
			const uint8_T *leapsec,
			const uint32_T *J2000_time,
			const real32_T *J2000_frac_time,
			real32_T *des_rates_bf)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
float pos_loc[3], gnd_loc[3], q_loc[4];
float rates_loc[3] = {0.0f, 0.0f, 0.0f};
int i;

g_J2000_time      = *J2000_time;
g_J2000_frac_time = *J2000_frac_time;

for (i = 0; i < 3; i++) { pos_loc[i] = pos_ecef[i]; gnd_loc[i] = gnd_ecef[i]; }
for (i = 0; i < 4; i++) q_loc[i] = q_BN[i];

create_des_rates_DCOMM(*opmode, pos_loc, gnd_loc, rates_loc, q_loc, *leapsec);

for (i = 0; i < 3; i++) des_rates_bf[i] = rates_loc[i];
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


