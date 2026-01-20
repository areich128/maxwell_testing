#define S_FUNCTION_NAME att_sun_vec_sfun
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "mtx.h"
#include "att_det.h"
#include "mex.h"

#define SUN 13

#define SUNO 3

#define INPUT_WIDTH (SUN)
#define OUTPUT_WIDTH (SUNO)

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, INPUT_WIDTH);
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S,1)) return;
    ssSetOutputPortWidth(S, 0, OUTPUT_WIDTH);

    ssSetNumSampleTimes(S, 1);

    /* specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetOptions(S,
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE |
                 SS_OPTION_USE_TLC_WITH_ACCELERATOR);
}

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S); 
}

static void mdlOutputs(SimStruct *S, int_T tid) {
    int_T i;
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
    real_T *y = ssGetOutputPortRealSignal(S,0);
    //int_T width = ssGetOutputPortWidth(S,0);

	/*Input from Simulink*/
    double uIdx[INPUT_WIDTH];
	for(i=0;i<INPUT_WIDTH;i++) {
		uIdx[i] = *uPtrs[i];
	}

	/*Input*/
	float sun_sens_read[SUN];

    struct mtx_matrix sun_sens_norm;
    struct mtx_matrix sun_sens_volt;
    struct mtx_matrix sun_vec;
                                    
   //Sun Sensor Normal Vectors, XYZ in Satellite Body Frame
    static float norm_sun_sens_cart[] =
    {
        0.866,0.000,-0.500,    //+X1
        0.866,-0.433,0.250,    //+X2
        0.866,0.433,0.250,     //+X3
        0.500,-0.866,0.000,    //-Y2
        0.000,-0.866,-0.500,    //-Y1
        0.500,0.866,0.000,     //+Y2
        0.000,0.866,-0.500,    //+Y1
        0.000,0.866,0.500,     //+Y4
        0.000,-0.866,0.500,    //-Y4
        0.000,0.000,1.000,     //+Z1
        -0.866,-0.433,-0.250,  //-X1
        -0.866,0.000,0.500,    //-X3
        -0.866,0.433,-0.250 }; //-X2
                                 
                  
    /*From Simulink Input*/
	sun_sens_read[0] = (float) uIdx[0];
	sun_sens_read[1] = (float) uIdx[1];
	sun_sens_read[2] = (float) uIdx[2];
	sun_sens_read[3] = (float) uIdx[3];
	sun_sens_read[4] = (float) uIdx[4];
	sun_sens_read[5] = (float) uIdx[5];
	sun_sens_read[6] = (float) uIdx[6];
	sun_sens_read[7] = (float) uIdx[7];
	sun_sens_read[8] = (float) uIdx[8];
	sun_sens_read[9] = (float) uIdx[9];
	sun_sens_read[10] = (float) uIdx[10];
	sun_sens_read[11] = (float) uIdx[11];
	sun_sens_read[12] = (float) uIdx[12];

    mtx_create(13,3,norm_sun_sens_cart,&sun_sens_norm);
    mtx_create(13,1,sun_sens_read,&sun_sens_volt);
    mtx_create_ones(3,1,&sun_vec);

    /*Check Sun Vector*/
    est_sun_vec_ls(&sun_sens_volt,&sun_sens_norm,&sun_vec,SS_V_CUTOFF);

    /*Save output*/
    y[0] = mtx_get(1,1,&sun_vec);
    y[1] = mtx_get(2,1,&sun_vec);
    y[2] = mtx_get(3,1,&sun_vec);
}

static void mdlTerminate(SimStruct *S){}

#ifdef MATLAB_MEX_FILE 
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
