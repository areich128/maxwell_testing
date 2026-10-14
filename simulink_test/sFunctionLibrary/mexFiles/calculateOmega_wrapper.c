
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
#define u_width 3
#define u_1_width 3
#define u_2_width 4
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
extern void calculateOmega_Outputs_wrapper(const real32_T *prev_vec,
			const real32_T *current_vec,
			const real32_T *q_BN,
			real32_T *des_rates_bf);

void calculateOmega_Outputs_wrapper(const real32_T *prev_vec,
			const real32_T *current_vec,
			const real32_T *q_BN,
			real32_T *des_rates_bf)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
float prev_loc[3], cur_loc[3], q_loc[4];
  int i;
  for (i = 0; i < 3; i++) { prev_loc[i] = prev_vec[i]; cur_loc[i] = current_vec[i]; }
  for (i = 0; i < 4; i++) q_loc[i] = q_BN[i];
  calculateOmega(prev_loc, cur_loc, des_rates_bf, q_loc);
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


