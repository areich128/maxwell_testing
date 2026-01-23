#include "mex.h"
#include "att_det.h"
#include "mtx.h"

/* ---------------- Constants ---------------- */
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

#define N_SS 15

static float norms[15*3] = {
     0.866,  0.000, -0.500,
     0.866, -0.433,  0.250,
     0.866,  0.433,  0.250,
     0.500,  0.866,  0.000,
     0.500, -0.866,  0.000,
     0.000, -0.866, -0.500,
     0.500,  0.866,  0.000,
     0.000,  0.866, -0.500,
     0.000,  0.866,  0.500,
     0.500, -0.866,  0.000,
     0.000, -0.866,  0.500,
     0.000,  0.000,  1.000,
    -0.866, -0.433, -0.250,
    -0.866,  0.000,  0.500,
    -0.866,  0.433, -0.250
};

/* ---------------- MEX Entry ---------------- */

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    /* ---------- Input checking ---------- */
    if (nrhs != 1)
        mexErrMsgTxt("Expected 1 input: ADC readings (15x1)");

    if (!mxIsDouble(prhs[0]) || mxGetNumberOfElements(prhs[0]) != 15)
        mexErrMsgTxt("Input must be 15-element double array");

    if (nlhs != 1)
        mexErrMsgTxt("One output required");

    /* ---------- Copy ADC values ---------- */
    double *adc_in = mxGetPr(prhs[0]);
    float adc_f[15];

    for (int i = 0; i < 15; i++)
        adc_f[i] = (float)adc_in[i];

    /* ---------- Convert ADC -> voltages ---------- */
    float voltages[15];
    float sensor_bias_1[4] = {AD7991_1_BIAS,AD7991_2_BIAS,AD7991_3_BIAS,AD7991_4_BIAS};
    float sensor_bias_2[4] = {AD7991_5_BIAS,AD7991_6_BIAS,AD7991_7_BIAS,AD7991_8_BIAS};
    float sensor_bias_3[4] = {AD7991_9_BIAS,AD7991_10_BIAS,AD7991_11_BIAS,AD7991_12_BIAS};
    float sensor_bias_4[3] = {AD7991_13_BIAS,AD7991_14_BIAS,AD7991_15_BIAS};
    float sensor_sens_1[4] = {AD7991_1_SENS,AD7991_2_SENS,AD7991_3_SENS,AD7991_4_SENS};
    float sensor_sens_2[4] = {AD7991_5_SENS,AD7991_6_SENS,AD7991_7_SENS,AD7991_8_SENS};
    float sensor_sens_3[4] = {AD7991_9_SENS,AD7991_10_SENS,AD7991_11_SENS,AD7991_12_SENS};
    float sensor_sens_4[3] = {AD7991_13_SENS,AD7991_14_SENS,AD7991_15_SENS};

    for(int i = 0; i < 4;i++){
        voltages[i] =(adc_f[i]+sensor_bias_1[i])*sensor_sens_1[i];
        voltages[i+4] =(adc_f[i+4]+sensor_bias_2[i])*sensor_sens_2[i];
        voltages[i+8] =(adc_f[i+8]+sensor_bias_3[i])*sensor_sens_3[i];
        if(i<4){
            voltages[i+12] =(adc_f[i+12]+sensor_bias_4[i])*sensor_sens_4[i];
        }
    }

    /* ---------- Create matrices ---------- */
    struct mtx_matrix sun_sens;
    struct mtx_matrix sun_sens_norm;
    struct mtx_matrix sun_vec;

    mtx_create(15, 1, voltages, &sun_sens);
    mtx_create(15, 3, norms, &sun_sens_norm);
    mtx_create_ones(3, 1, &sun_vec);

    /* ---------- Estimate sun vector ---------- */
    est_sun_vec_ls(&sun_sens,
                   &sun_sens_norm,
                   &sun_vec,
                   SS_V_CUTOFF);

    /* ---------- Output ---------- */
    plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
    double *out = mxGetPr(plhs[0]);

    for (int i = 0; i < 3; i++)
        out[i] = sun_vec.data[i];
}
