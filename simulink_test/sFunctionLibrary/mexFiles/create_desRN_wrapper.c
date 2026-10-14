
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
#include "mex.h"
   #include <stdio.h>
   #include "mtx.h"
   #include "att_det.h"
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define u_1_width 3
#define u_2_width 1
#define u_3_width 3
#define y_width 4
#define y_1_width 9

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
extern void create_desRN_Outputs_wrapper(const uint8_T *opmode,
			const real32_T *des_vec,
			const uint8_T *antenna,
			const real32_T *b_eci,
			real32_T *des_RN,
			real32_T *dcm_NR_out);

void create_desRN_Outputs_wrapper(const uint8_T *opmode,
			const real32_T *des_vec,
			const uint8_T *antenna,
			const real32_T *b_eci,
			real32_T *des_RN,
			real32_T *dcm_NR_out)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
float dcm_NR_arr[9] = {0.0f};
   struct mtx_matrix dcm_NR_m;
   int i, j;

   mtx_create(3, 3, dcm_NR_arr, &dcm_NR_m);

   create_desRN(*opmode, (float *)des_vec, des_RN, (float *)b_eci, *antenna, &dcm_NR_m);

   for (i = 0; i < 3; i++) {
       for (j = 0; j < 3; j++) {
           dcm_NR_out[3*i + j] = mtx_get(i+1, j+1, &dcm_NR_m);
       }
   }
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


