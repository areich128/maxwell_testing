
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
#include "propagator.h"
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define u_1_width 6
#define y_width 6
#define y_1_width 1

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
extern void rk4_sfun_Outputs_wrapper(const real32_T *dT,
			const real32_T *x0,
			real32_T *xF,
			int32_T *status);

void rk4_sfun_Outputs_wrapper(const real32_T *dT,
			const real32_T *x0,
			real32_T *xF,
			int32_T *status)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
static struct param_table prm;
float xin[6], xout[6];
int i;
/* same values as param_table_default in global.c (not compiled in the Simulink build) */
prm.Re = 6378.1363f;
prm.Mu = 3.986004415e5f;
prm.J2 = 0.001082626683553f;
prm.J3 = -2.532656485e-6f;
prm.J4 = -1.619621591e-6f;
prm.wE = 7.292462e-5f;
prm.Cd = 2.2f;
prm.Area = 0.02f;
prm.Mass = 2.0f;
prm.rho_A_delta = 0.0f;
for (i = 0; i < 6; i++) xin[i] = x0[i];
*status = rk4(*dT, xin, xout, &prm);
for (i = 0; i < 6; i++) xF[i] = xout[i];
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


