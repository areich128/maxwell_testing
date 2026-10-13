
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
extern void sc_to_gs_unitvec_Outputs_wrapper(const real32_T *sc_pos_vec_ecef,
			const real32_T *gnd_stn_pos_vec_ecef,
			real32_T *sc_2_gs_normlzd);

void sc_to_gs_unitvec_Outputs_wrapper(const real32_T *sc_pos_vec_ecef,
			const real32_T *gnd_stn_pos_vec_ecef,
			real32_T *sc_2_gs_normlzd)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
    
sc_to_gs_unitvec((float *)sc_pos_vec_ecef, (float *)gnd_stn_pos_vec_ecef, sc_2_gs_normlzd);

/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


