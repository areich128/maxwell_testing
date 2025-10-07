
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
#include "conversions.h"
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 4
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
extern void TestQuat2Euler_Outputs_wrapper(const real32_T *q,
			real32_T *e);

void TestQuat2Euler_Outputs_wrapper(const real32_T *q,
			real32_T *e)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
//float qIn;
    //qIn[0] = *q[0];
    //qIn[1] = *q[1];
    //qIn[2] = *q[2];
    //qIn[3] = *q[3];


    // TODO work out the data type conversions here
    //float eOut;
    //real32_T* eOut[3];
    //real32_T* qIn[4];
    //*qIn[0] = *q[0];

    // Test external code
    quat2euler321(q,e); // Somehow the pointers to arrays are not being filled???

    // WHy does this produce NAN?
     ///e[0] = atan2f(2*(q[1]*q[2]+q[0]*q[3]),q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3]);
   // e[1] = asinf(-2*(q[1]*q[3]-q[0]*q[2]));
   // e[2]= atan2f(2*(q[2]*q[3]+q[0]*q[1]),q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]);


    // THe code below correctly assigns data in
    //e[1] = q[1];

    //*e = *eOut;
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


