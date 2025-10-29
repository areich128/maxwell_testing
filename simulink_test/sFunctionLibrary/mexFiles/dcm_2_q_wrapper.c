
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
#include "mex.h"
#include "uart_dbg.h"
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 9
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
extern void dcm_2_q_Outputs_wrapper(const real32_T *dcm,
			real32_T *q);

void dcm_2_q_Outputs_wrapper(const real32_T *dcm,
			real32_T *q)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
// Inputs
struct mtx_matrix dcmMtx;
mtx_create(3,3,dcm,&dcmMtx);

// Outputs
struct mtx_matrix qMtx;
mtx_create_ones(4,1,&qMtx);

dcm_2_q(&dcmMtx,&qMtx);

// Set Outputs
q[0] = mtx_get(1,1,&qMtx);
q[1] = mtx_get(2,1,&qMtx);
q[2] = mtx_get(3,1,&qMtx);
q[3] = mtx_get(4,1,&qMtx);
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


