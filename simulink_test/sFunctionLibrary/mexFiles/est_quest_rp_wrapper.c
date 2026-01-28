
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
#include "mex.h"
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 6
#define u_1_width 6
#define y_width 4
#define y_1_width 9
#define y_2_width 1

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
extern void est_quest_rp_Outputs_wrapper(const real32_T *b_k,
			const real32_T *eci_k,
			real32_T *state_quaternion,
			real32_T *dcm_out,
			int32_T *status);

void est_quest_rp_Outputs_wrapper(const real32_T *b_k,
			const real32_T *eci_k,
			real32_T *state_quaternion,
			real32_T *dcm_out,
			int32_T *status)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
// Inputs
    struct mtx_matrix b_k_mtx;
    mtx_create(3,2,b_k,&b_k_mtx);

    struct mtx_matrix eci_k_mtx;
    mtx_create(3,2,eci_k,&eci_k_mtx);  

    // mexPrintf("Step\n");
    // mexPrintf("Input eci_k\n");
    // mexPrintf("%f,%f,%f; %f,%f,%f\n",
    //           eci_k[0],eci_k[1],eci_k[2],eci_k[3],eci_k[4],eci_k[5]);
    // 
    // mexPrintf("eci_k_mtx\n");
    // mexPrintf("Col 1: %f,%f,%f; Col 2: %f,%f,%f\n",
    //           mtx_get(1,1,&eci_k_mtx),
    //           mtx_get(2,1,&eci_k_mtx),
    //           mtx_get(3,1,&eci_k_mtx),
    //           mtx_get(1,2,&eci_k_mtx),
    //           mtx_get(2,2,&eci_k_mtx),
    //           mtx_get(3,2,&eci_k_mtx));
    // mexPrintf("\n");
    
    // Outputs

    //mexPrintf("You made it here 1\n");
    struct mtx_matrix dcm_out_mtx;
    mtx_create_ones(3,3,&dcm_out_mtx);

    struct est_state state_mtx;
    mtx_create_ones(4,1,&(state_mtx.att_quaternion));
    mtx_create_ones(3,1,&(state_mtx.sat_body_rates));

    // Run Function
    //mexPrintf("You made it here 2\n");
    int32_t statusFlag = 0;
    statusFlag = est_quest_rp(&b_k_mtx, &eci_k_mtx, 
                          &state_mtx, &dcm_out_mtx);
    *status = statusFlag;

    // Assign Outputs
    state_quaternion[0] = mtx_get(1,1,&(state_mtx.att_quaternion)); // IDK if this is the correct syntax
    state_quaternion[1] = mtx_get(2,1,&(state_mtx.att_quaternion));
    state_quaternion[2] = mtx_get(3,1,&(state_mtx.att_quaternion));
    state_quaternion[3] = mtx_get(4,1,&(state_mtx.att_quaternion));
    
    // mexPrintf("Step\n");
    // mexPrintf("Quat Out\n");
    // mexPrintf("%f,%f,%f,%f\n",
    //           state_quaternion[0],
    //           state_quaternion[1],
    //           state_quaternion[2],
    //           state_quaternion[3]);
    // mexPrintf("\n");
    
    dcm_out[0] = mtx_get(1,1,&(dcm_out_mtx));
    dcm_out[1] = mtx_get(1,2,&(dcm_out_mtx));
    dcm_out[2] = mtx_get(1,3,&(dcm_out_mtx));
    dcm_out[3] = mtx_get(2,1,&(dcm_out_mtx));
    dcm_out[4] = mtx_get(2,2,&(dcm_out_mtx));
    dcm_out[5] = mtx_get(2,3,&(dcm_out_mtx));
    dcm_out[6] = mtx_get(3,1,&(dcm_out_mtx));
    dcm_out[7] = mtx_get(3,2,&(dcm_out_mtx));
    dcm_out[8] = mtx_get(3,3,&(dcm_out_mtx));

    // mexPrintf("DCM Out\n");
    // mexPrintf("%f,%f,%f\n",
    //           dcm_out[0],
    //           dcm_out[1],
    //           dcm_out[2]);
    // mexPrintf("\n");
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


