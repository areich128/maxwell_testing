
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
#include "rw_set.h"
#include "mtx.h"
#include "global.h"
#include "util.h"
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 4
#define u_1_width 3
#define u_2_width 12
#define u_3_width 1
#define u_4_width 3
#define y_width 3
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
extern void WheelTorque_Command_Outputs_wrapper(const real32_T *wheel_speeds,
			const real32_T *mag_bf,
			const real32_T *GsValues,
			const int32_T *disabled_wheel,
			const real32_T *out_u,
			real32_T *pwm_dipole,
			real32_T *rw_torque);

void WheelTorque_Command_Outputs_wrapper(const real32_T *wheel_speeds,
			const real32_T *mag_bf,
			const real32_T *GsValues,
			const int32_T *disabled_wheel,
			const real32_T *out_u,
			real32_T *pwm_dipole,
			real32_T *rw_torque)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
WheelTorque_Command((float *)wheel_speeds,
                    (float *)mag_bf,
                    (float *)GsValues,
                    (float *)pwm_dipole,
                    (float *)rw_torque,
                    *disabled_wheel,
                    (float *)out_u);
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


