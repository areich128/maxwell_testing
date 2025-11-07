
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
#define u_width 4
#define y_width 9

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
extern void q_2_dcm_Outputs_wrapper(const real32_T *q,
			real32_T *dcm);

void q_2_dcm_Outputs_wrapper(const real32_T *q,
			real32_T *dcm)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
// Inputs
struct mtx_matrix dcmMtx;
mtx_create_ones(3,3,&dcmMtx);

// Outputs
struct mtx_matrix qMtx;
mtx_create(4,1,q,&qMtx);

q_2_dcm(&qMtx,&dcmMtx);

// Set Outputs
dcm[0] = mtx_get(1,1,&dcmMtx);
dcm[1] = mtx_get(1,2,&dcmMtx);
dcm[2] = mtx_get(1,3,&dcmMtx);
dcm[3] = mtx_get(2,1,&dcmMtx);
dcm[4] = mtx_get(2,2,&dcmMtx);
dcm[5] = mtx_get(2,3,&dcmMtx);
dcm[6] = mtx_get(3,1,&dcmMtx);
dcm[7] = mtx_get(3,2,&dcmMtx);
dcm[8] = mtx_get(3,3,&dcmMtx);
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


