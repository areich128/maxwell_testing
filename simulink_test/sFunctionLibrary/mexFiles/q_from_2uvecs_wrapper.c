
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
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 3
#define u_1_width 3
#define y_width 4

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
extern void q_from_2uvecs_Outputs_wrapper(const real32_T *v1,
			const real32_T *v2,
			real32_T *q_v1v2);

void q_from_2uvecs_Outputs_wrapper(const real32_T *v1,
			const real32_T *v2,
			real32_T *q_v1v2)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
q_from_2uvecs(v1,v2,q_v1v2);
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


