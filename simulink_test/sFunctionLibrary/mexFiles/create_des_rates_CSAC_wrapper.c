
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
#include "mtx.h"
#include "att_det.h"
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define u_1_width 4
#define u_2_width 3
#define u_3_width 1
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
extern void create_des_rates_CSAC_Outputs_wrapper(const uint8_T *opmode,
			const real32_T *q_BN,
			const real32_T *phi_cone,
			const uint8_T *new_mode,
			real32_T *des_rates_bf);

void create_des_rates_CSAC_Outputs_wrapper(const uint8_T *opmode,
			const real32_T *q_BN,
			const real32_T *phi_cone,
			const uint8_T *new_mode,
			real32_T *des_rates_bf)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
float phi_loc[3], q_loc[4];
int i;
for (i = 0; i < 3; i++) phi_loc[i] = phi_cone[i];
for (i = 0; i < 4; i++) q_loc[i] = q_BN[i];
create_des_rates_CSAC(*opmode, phi_loc, des_rates_bf, q_loc, *new_mode);
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


