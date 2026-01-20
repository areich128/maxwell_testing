#include "mtx.h"
#include "att_det.h"
#include "mex.h"

#define SUN 15
#define SUNO 3

/* Sensor Biases */
#define AD7991_1_BIAS  -1078.0f
#define AD7991_2_BIAS  -1078.0f
#define AD7991_3_BIAS  -1076.0f
#define AD7991_4_BIAS  -1076.0f
#define AD7991_5_BIAS  -1039.0f
#define AD7991_6_BIAS  -1047.0f
#define AD7991_7_BIAS  -1038.0f
#define AD7991_8_BIAS  -1045.0f
#define AD7991_9_BIAS  -1078.0f
#define AD7991_10_BIAS -1080.0f
#define AD7991_11_BIAS -1078.0f
#define AD7991_12_BIAS -1079.0f
#define AD7991_13_BIAS -1016.0f
#define AD7991_14_BIAS -1016.0f
#define AD7991_15_BIAS -1016.0f

#define AD7991_12_SENS (0.0061f/1.334f)*0.323 //+Z (Reference Sensor)
#define AD7991_1_SENS  0.971f*AD7991_12_SENS 
#define AD7991_2_SENS  0.963f*AD7991_12_SENS 
#define AD7991_3_SENS  0.963f*AD7991_12_SENS 
#define AD7991_4_SENS  0.962f*AD7991_12_SENS 
#define AD7991_5_SENS  0.945f*AD7991_12_SENS 
#define AD7991_6_SENS  1.340f*AD7991_12_SENS 
#define AD7991_7_SENS  0.000f*AD7991_12_SENS 
#define AD7991_8_SENS  1.320f*AD7991_12_SENS 
#define AD7991_9_SENS  0.964f*AD7991_12_SENS 
#define AD7991_10_SENS 0.969f*AD7991_12_SENS 
#define AD7991_11_SENS 0.963f*AD7991_12_SENS 
#define AD7991_13_SENS 0.995f*AD7991_12_SENS 
#define AD7991_14_SENS 0.943f*AD7991_12_SENS
#define AD7991_15_SENS 0.940f*AD7991_12_SENS

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[]){

	/*Input*/
    struct mtx_matrix sun_sens_norm;
    static struct mtx_matrix sun_sens_volt;
    struct mtx_matrix sun_vec;
    
    //Initialize
    int i = 0;
    unsigned int css_ad7991_1[4];
    unsigned int css_ad7991_2[4];
    unsigned int css_ad7991_3[4];
    unsigned int css_ad7991_4[3]; // ADC16 Empty
    float css_vec[15] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
                         0.0,0.0,0.0,0.0,0.0,0.0,0.0};
                              
   //Sun Sensor Normal Vectors, XYZ in Satellite Body Frame
    static float norm_sun_sens_cart[] =
    {
        0.866,0.000,-0.500,    //+X1
        0.866,-0.433,0.250,    //+X2
        0.866,0.433,0.250,     //+X3
        0.500,0.866,0.000,      //+Y3
        0.500,-0.866,0.000,    //-Y2
        0.000,-0.866,-0.500,   //-Y1
        0.500,0.866,0.000,     //+Y2
        0.000,0.866,-0.500,    //+Y1
        0.000,0.866,0.500,     //+Y4
        0.500,-0.866,0.000,    //-Y3
        0.000,-0.866,0.500,    //-Y4
        0.000,0.000,1.000,     //+Z1
        -0.866,-0.433,-0.250,  //-X1
        -0.866,0.000,0.500,    //-X3
        -0.866,0.433,-0.250 }; //-X2
                                 
                  
    /*From Simulink Input*/
	css_ad7991_1[0] = mxGetScalar(prhs[0]);
	css_ad7991_1[1] = mxGetScalar(prhs[1]);
	css_ad7991_1[2] = mxGetScalar(prhs[2]);
	css_ad7991_1[3] = mxGetScalar(prhs[3]);
	css_ad7991_2[0] = mxGetScalar(prhs[4]);
	css_ad7991_2[1] = mxGetScalar(prhs[5]);
	css_ad7991_2[2] = mxGetScalar(prhs[6]);
	css_ad7991_2[3] = mxGetScalar(prhs[7]);
	css_ad7991_3[0] = mxGetScalar(prhs[8]);
	css_ad7991_3[1] = mxGetScalar(prhs[9]);
	css_ad7991_3[2] = mxGetScalar(prhs[10]);
	css_ad7991_3[3] = mxGetScalar(prhs[11]);
	css_ad7991_4[0] = mxGetScalar(prhs[12]);
	css_ad7991_4[1] = mxGetScalar(prhs[13]);
	css_ad7991_4[2] = mxGetScalar(prhs[14]);
    
    //Calibrate Sensors
    float bias_ad7991[15] = {AD7991_1_BIAS, AD7991_2_BIAS,AD7991_3_BIAS,AD7991_4_BIAS,
                    AD7991_5_BIAS, AD7991_6_BIAS,AD7991_7_BIAS,AD7991_8_BIAS,
                    AD7991_9_BIAS, AD7991_10_BIAS,AD7991_11_BIAS,
                    AD7991_12_BIAS,AD7991_13_BIAS,AD7991_14_BIAS,AD7991_15_BIAS};

    float sens_ad7991[15] = {AD7991_1_SENS, AD7991_2_SENS,AD7991_3_SENS,AD7991_4_SENS,
                    AD7991_5_SENS, AD7991_6_SENS,AD7991_7_SENS,AD7991_8_SENS,
                    AD7991_9_SENS, AD7991_10_SENS,AD7991_11_SENS,
                    AD7991_12_SENS,AD7991_13_SENS,AD7991_14_SENS,AD7991_15_SENS};

   	for(i=0;i<4;i++) {
			css_vec[i]    = ((float)css_ad7991_1[i] + bias_ad7991[i])
									* sens_ad7991[i];
			css_vec[4+i]  = ((float)css_ad7991_2[i] + bias_ad7991[4+i])
									*sens_ad7991[4+i];
			css_vec[8+i]  = ((float)css_ad7991_3[i] + bias_ad7991[8+i])
									*sens_ad7991[8+i];
			if(i<3) css_vec[12+i] = ((float)css_ad7991_4[i] + bias_ad7991[12+i])
									*sens_ad7991[12+i];
	}
          
    mtx_create(15,3,norm_sun_sens_cart,&sun_sens_norm);
    mtx_create(15,1,css_vec,&sun_sens_volt);
    mtx_create_ones(3,1,&sun_vec);
    
    /*Check Sun Vector*/
    est_sun_vec_ls(&sun_sens_volt,&sun_sens_norm,&sun_vec,SS_V_CUTOFF);
 
    /*Save output*/
    double *outMatrix;      /* output matrix */
    double *outMatrix2;      /* output matrix */
    plhs[0] = mxCreateDoubleMatrix(3,1,1);
    plhs[1] = mxCreateDoubleMatrix(15,1,1);
    outMatrix = mxGetPr(plhs[0]);
    outMatrix[0] = mtx_get(1,1,&sun_vec);
    outMatrix[1] = mtx_get(2,1,&sun_vec);
    outMatrix[2] = mtx_get(3,1,&sun_vec);
    
    outMatrix2 = mxGetPr(plhs[1]);
    outMatrix2[0] = mtx_get(1,1,&sun_sens_volt);
    outMatrix2[1] = mtx_get(2,1,&sun_sens_volt);
    outMatrix2[2] = mtx_get(3,1,&sun_sens_volt);
    outMatrix2[3] = mtx_get(4,1,&sun_sens_volt);
    outMatrix2[4] = mtx_get(5,1,&sun_sens_volt);
    outMatrix2[5] = mtx_get(6,1,&sun_sens_volt);
    outMatrix2[6] = mtx_get(7,1,&sun_sens_volt);
    outMatrix2[7] = mtx_get(8,1,&sun_sens_volt);
    outMatrix2[8] = mtx_get(9,1,&sun_sens_volt);
    outMatrix2[9] = mtx_get(10,1,&sun_sens_volt);
    outMatrix2[10] = mtx_get(11,1,&sun_sens_volt);
    outMatrix2[11] = mtx_get(12,1,&sun_sens_volt);
    outMatrix2[12] = mtx_get(13,1,&sun_sens_volt);
    outMatrix2[13] = mtx_get(14,1,&sun_sens_volt);
    outMatrix2[14] = mtx_get(15,1,&sun_sens_volt);
    
}

