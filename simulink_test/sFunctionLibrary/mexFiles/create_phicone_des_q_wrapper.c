
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
#define u_1_width 3
#define u_2_width 3
#define u_3_width 1
#define u_4_width 3
#define y_width 3
#define y_1_width 4
#define y_2_width 9

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
extern void create_phicone_des_q_Outputs_wrapper(const uint8_T *op_mode,
			const real32_T *pos_vec,
			const real32_T *sun_vec,
			const real32_T *phi_cone_angle,
			const real32_T *b_eci,
			real32_T *phi_vec_normlzd,
			real32_T *des_RN_q,
			real32_T *dcm_NR);

void create_phicone_des_q_Outputs_wrapper(const uint8_T *op_mode,
			const real32_T *pos_vec,
			const real32_T *sun_vec,
			const real32_T *phi_cone_angle,
			const real32_T *b_eci,
			real32_T *phi_vec_normlzd,
			real32_T *des_RN_q,
			real32_T *dcm_NR)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
float pos_loc[3], sun_loc[3], b_loc[3];
float dcm_arr[9] = {0.0f};
struct mtx_matrix dcm_m;
int i, j;
for (i = 0; i < 3; i++) { pos_loc[i] = pos_vec[i]; sun_loc[i] = sun_vec[i]; b_loc[i] = b_eci[i]; }
mtx_create(3, 3, dcm_arr, &dcm_m);
create_phicone_des_q(*op_mode, pos_loc, sun_loc, des_RN_q, *phi_cone_angle, phi_vec_normlzd, b_loc, &dcm_m);
for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++)
        dcm_NR[3*i + j] = mtx_get(i+1, j+1, &dcm_m);
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


