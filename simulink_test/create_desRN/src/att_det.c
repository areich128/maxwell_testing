#define _USE_MATH_DEFINES

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mtx.h"
#include "att_det.h"
#include "conversions.h"
#include <stdint.h>
#include "ref_rotation.h"
#include "simulinkCustom.h"
#include "erh.h"

/* Pranith -> Extended Kalman Filter to estimate angular vel and quaternion(eci->body)*/
/* Quest acts as our measurement for q_BN */

void state_ekf(struct mtx_matrix* sun_sens_volt, struct mtx_matrix* sun_sens_norm,  float ss_v_cutoff,
    struct mtx_matrix* mag_sens, struct mtx_matrix* mag_eci, struct mtx_matrix* sun_eci,struct mtx_matrix* sun_vec,
    struct est_state* state, struct est_state *state_post, struct est_state *mes, struct ekf_mat *info,float *ctrl_t,float kd_term)
    {

//State transition function
uint8_t i,b;
struct mtx_matrix prev_dcm;
struct mtx_matrix ctrl_torq,J,Jinv,JW,med0,med1,med2,w_dot,w_deriv;
struct mtx_matrix dwd_med, eye, KD, med3,dwd_dot;
float prev_ang_v[3];
float prev_qBN[4];
float ang_v[3];
float q_BN[4];
float jtw[3];
float ans[3];
float dwd_dw[9];
float dot[9];
float j_mat[9]={0.0797, -0.00005, 0.00125, -0.00005, 0.1291, -0.00103, 0.00125, -0.00103, 0.1310}; //Moments of inertia matrix

mtx_create(3,1,ctrl_t,&ctrl_torq);
mtx_create(3,3,j_mat,&J);
mtx_create_ones(3,1,&JW);
mtx_create_ones(3,1,&med0);
mtx_create_ones(3,1,&med1);
mtx_create_ones(3,3,&med2);
mtx_create_ones(3,3,&Jinv);
mtx_create_ones(3,3,&w_deriv);

for(i=0;i<3;i++) prev_ang_v[i]=mtx_get(i+1,1,&state_post->sat_body_rates);
for(i=0;i<4;i++) prev_qBN[i]=mtx_get(i+1,1,&state_post->att_quaternion);

//Quaternion prediction
q_2_dcm(&state_post->att_quaternion,&prev_dcm);
if(!att_det_reck(&prev_dcm, &state->sat_body_rates, &state->att_quaternion)) PRINT("ATTITUDE MODEL ERR\r\n"); 


//Angular Velocity prediction
mtx_mult(&J,&state->sat_body_rates,&JW);
for(i=0;i<3;i++) jtw[i]=mtx_get(i+1,1,&JW);
crossp(prev_ang_v,jtw,ans);
for(i=0;i<3;i++) mtx_set(i+1,i,&med0,ans[i]);
mtx_scale(&med0,-1.0f,&med1);
mtx_sum(&ctrl_torq,&med1,&med2);
mtx_inv(&J,&Jinv);
mtx_mult(&Jinv,&med2,&w_dot);
mtx_scale(&w_dot,0.10f,&w_deriv);
mtx_sum(&state_post->sat_body_rates,&w_deriv,&state->sat_body_rates);

//F_Jacobian
for(b=0;b<3;b++) ang_v[b]=mtx_get(b+1,1,&state->sat_body_rates);
for(b=0;b<3;b++) q_BN[b]=mtx_get(b+1,1,&state->att_quaternion);

//Previous Updated State
float wx = prev_ang_v[0], wy = prev_ang_v[1], wz = prev_ang_v[2];
float b0 = prev_qBN[0], b1 = prev_qBN[1], b2=prev_qBN[2], b3=prev_qBN[3];

//Current process model state
float w1 = ang_v[0], w2 = ang_v[1], w3 = ang_v[2];
float q0 = q_BN[0], q1 = q_BN[1], q2=q_BN[2], q3=q_BN[3];

float g11 = j_mat[0];
float g12 = j_mat[1];
float g13 = j_mat[2];
float g21 = j_mat[3];
float g22 = j_mat[4];
float g23 = j_mat[5];
float g31 = j_mat[6];
float g32 = j_mat[7];
float g33 = j_mat[8];

dwd_dw[0] = g31*wy - g21*wz;
dwd_dw[1] = g31*wx + 2*g32*wy + g33*wz - g22*wz;
dwd_dw[2] = g33*wy - g21*wx - g22*wy - 2*g23*wz;
dwd_dw[3] = g11*wz - 2*g31*wx - g32*wy - g33*wz;
dwd_dw[4] = g12*wz - g32*wx;
dwd_dw[5] = g11*wx + g12*wy + 2*g13*wz - g33*wx;
dwd_dw[6] = 2*g21*wx + g22*wy + g23*wz - g11*wy;
dwd_dw[7] = g22*wx - g11*wx - 2*g12*wy - g13*wz;
dwd_dw[8] = g23*wx - g13*wy;

mtx_create(3,3,dwd_dw,&dwd_med);
mtx_create_eye(3,3,&eye);
mtx_create_ones(3,3,&KD);
mtx_create_ones(3,3,&med3);
mtx_create_ones(3,3,&dwd_dot);
mtx_scale(&eye,(float)-1.0f*kd_term,&KD);
mtx_scale(&dwd_med,-1.0f,&dwd_med);
mtx_sum(&KD,&dwd_med,&med3);
mtx_mult(&Jinv,&med3,&dwd_dot);

for(b=0;b<3;b++) dot[b]=mtx_get(b+1,1,&dwd_dot);

float f_jacobian[49] = {
   
    dot[0], dot[1], dot[2],              0, 0, 0, 0,
  
    dot[3], dot[4], dot[5],              0, 0, 0, 0,
  
    dot[6], dot[7], dot[8],              0, 0, 0, 0,

  
    -b2/2,   b1/2,   b0/2,       0,   -wx/2,   -wy/2,   -wz/2,
    
     b3/2,   b0/2,  -b1/2,      wx/2,    0,     wz/2,   -wy/2,

    -b0/2,   b3/2,  -b2/2,      wy/2,  -wz/2,    0,     wx/2,
   
    -b1/2,  -b2/2,   b3/2,      wz/2,   wy/2,  -wx/2,    0
};


//Model Covariance
struct mtx_matrix F,FT,P0,FP0,FP0FT;
mtx_create(7,7,f_jacobian,&F);
mtx_create_ones(7,7,&FT);
mtx_create_ones(7,7,&P0);
mtx_create_ones(7,7,&FP0);
mtx_create_ones(7,7,&FP0FT);
mtx_copy(&info->P,&P0);       
mtx_trans(&F,&FT);
mtx_mult(&F,&P0,&FP0);
mtx_mult(&FP0,&FT,&FP0FT);
mtx_sum(&FP0FT,&info->Q,&info->P);


//Innovation
struct mtx_matrix dcm_est,state_pred,min_sp,measurement,innov;
float sp[7]={w1,w2,w3,q0,q1,q2,q3};
float state_mes[7];
mtx_create_eye(3,3,&dcm_est);

if(est_simp(sun_sens_volt,sun_sens_norm,mag_sens,sun_eci,mag_eci, //Quest Output
	sun_vec,mes,&dcm_est,ss_v_cutoff)==0){
        g_err|=(0x01ull<<ATT_ERR_2); }

for(i=0; i<4; i++) state_mes[i]=mtx_get(i+1,1,&mes->att_quaternion);
for(i=0; i<3; i++) state_mes[i+4]=mtx_get(i+1,1,&mes->sat_body_rates);

mtx_create(7,1,sp,&state_pred);
mtx_create(7,1,state_mes,&measurement);
mtx_create_ones(7,1,&innov);
mtx_create_ones(7,1,&min_sp);
mtx_scale(&state_pred,-1.0f,&min_sp);
mtx_sum(&measurement,&min_sp,&innov);


//Innovation Covariance
struct mtx_matrix H,HT,HP,HPHT,Sinv,PHT,S,K;

mtx_create_eye(7,7,&H);
mtx_create_ones(7,7,&HT);
mtx_create_ones(7,7,&HP);
mtx_create_ones(7,7,&HPHT);
mtx_create_ones(7,7,&S);
mtx_trans(&H,&HT);
mtx_mult(&H,&info->P,&HP);
mtx_mult(&HP,&HT,&HPHT);
mtx_sum(&HPHT,&info->R,&S);

//Kalman Gain
mtx_create_ones(7,7,&Sinv);
mtx_create_ones(7,7,&K);
mtx_inv(&S,&Sinv);
mtx_mult(&info->P,&HT,&PHT);
mtx_mult(&PHT,&Sinv,&K);


//State Update
struct mtx_matrix addit,state_mtx,state_post_mtx;
mtx_create_ones(7,1,&addit);
mtx_create(7,1,sp,&state_mtx);
mtx_create(7,1,sp,&state_post_mtx);
mtx_mult(&K,&innov,&addit);
mtx_sum(&state_mtx,&addit,&state_post_mtx);

for(b=0; b<4; b++) {
    mtx_set(b+1,1,&state_post->att_quaternion,mtx_get(b+4,1,&state_post_mtx));
    if(b>=1)mtx_set(b,1,&state_post->sat_body_rates,mtx_get(b,1,&state_post_mtx));
}


//Model Covariance Update
struct mtx_matrix KH,minKH,inter,P_prev,ident;
mtx_create_ones(7,7,&P_prev);
mtx_create_ones(7,7,&KH);
mtx_create_ones(7,7,&minKH);
mtx_create_ones(7,7,&inter);
mtx_create_eye(7,7,&ident);
mtx_copy(&info->P,&P_prev);
mtx_mult(&K,&H,&KH);
mtx_scale(&KH,-1.0f,&minKH);
mtx_sum(&ident,&minKH,&inter);
mtx_mult(&inter,&P_prev,&info->P);

}

void q_2_dcm(struct mtx_matrix* q, struct mtx_matrix* dcm){

	mtx_set(1,1,dcm, 	
        mtx_get(1,1,q)*mtx_get(1,1,q)
            +mtx_get(2,1,q)*mtx_get(2,1,q)
            -mtx_get(3,1,q)*mtx_get(3,1,q)
            -mtx_get(4,1,q)*mtx_get(4,1,q));

    mtx_set(1,2,dcm,                                
        2*(mtx_get(2,1,q)*mtx_get(3,1,q)+mtx_get(1,1,q)*mtx_get(4,1,q)));

    mtx_set(1,3,dcm,                                
        2*(mtx_get(2,1,q)*mtx_get(4,1,q)-mtx_get(1,1,q)*mtx_get(3,1,q)));

    mtx_set(2,1,dcm,                 		    
        2*(mtx_get(2,1,q)*mtx_get(3,1,q)-mtx_get(1,1,q)*mtx_get(4,1,q)) );

    mtx_set(2,2,dcm,                                
        mtx_get(1,1,q)*mtx_get(1,1,q)
            -mtx_get(2,1,q)*mtx_get(2,1,q)
            +mtx_get(3,1,q)*mtx_get(3,1,q)
            -mtx_get(4,1,q)*mtx_get(4,1,q));

    mtx_set(2,3,dcm,                                
        2*(mtx_get(3,1,q)*mtx_get(4,1,q)+mtx_get(1,1,q)*mtx_get(2,1,q)));

    mtx_set(3,1,dcm,                 		    
        2*(mtx_get(2,1,q)*mtx_get(4,1,q)+mtx_get(1,1,q)*mtx_get(3,1,q)));

    mtx_set(3,2,dcm,                                
        2*(mtx_get(3,1,q)*mtx_get(4,1,q)-mtx_get(1,1,q)*mtx_get(2,1,q)));

    mtx_set(3,3,dcm,                                
        mtx_get(1,1,q)*mtx_get(1,1,q)
            -mtx_get(2,1,q)*mtx_get(2,1,q)
            -mtx_get(3,1,q)*mtx_get(3,1,q)
            +mtx_get(4,1,q)*mtx_get(4,1,q));
}

void dcm_2_q(struct mtx_matrix* dcm, struct mtx_matrix* q){

    float dcm_trace = 0.0;

    //Handle 90 Deg rotations
    dcm_trace = mtx_trace(dcm);

    if(dcm_trace>0){

        mtx_set(1,1,q, 0.5f*sqrtf(1.0f+dcm_trace));
        mtx_set(2,1,q,(mtx_get(2,3,dcm)-mtx_get(3,2,dcm))/(4.0f*mtx_get(1,1,q)));
        mtx_set(3,1,q,(mtx_get(3,1,dcm)-mtx_get(1,3,dcm))/(4.0f*mtx_get(1,1,q)));
        mtx_set(4,1,q,(mtx_get(1,2,dcm)-mtx_get(2,1,dcm))/(4.0f*mtx_get(1,1,q)));

    } else if((mtx_get(1,1,dcm) > mtx_get(2,2,dcm)) && 
              (mtx_get(1,1,dcm) > mtx_get(3,3,dcm))){

        mtx_set(2,1,q, 0.5f*sqrtf(1.0f + mtx_get(1,1,dcm) - mtx_get(2,2,dcm)
                                      - mtx_get(3,3,dcm)));
        mtx_set(1,1,q,(mtx_get(2,3,dcm)-mtx_get(3,2,dcm))/(4.0f*mtx_get(2,1,q)));
        mtx_set(3,1,q,(mtx_get(1,2,dcm)+mtx_get(2,1,dcm))/(4.0f*mtx_get(2,1,q)));
        mtx_set(4,1,q,(mtx_get(1,3,dcm)+mtx_get(3,1,dcm))/(4.0f*mtx_get(2,1,q)));

    } else if(mtx_get(2,2,dcm) > mtx_get(3,3,dcm)){

        mtx_set(3,1,q, -0.5f*sqrtf(1.0f + mtx_get(2,2,dcm) - mtx_get(1,1,dcm)
                                      - mtx_get(3,3,dcm)));
        mtx_set(1,1,q,(mtx_get(3,1,dcm)-mtx_get(1,3,dcm))/(4.0f*mtx_get(3,1,q)));
        mtx_set(2,1,q,(mtx_get(1,2,dcm)+mtx_get(2,1,dcm))/(4.0f*mtx_get(3,1,q)));
        mtx_set(4,1,q,(mtx_get(2,3,dcm)+mtx_get(3,2,dcm))/(4.0f*mtx_get(3,1,q)));

    } else {

        mtx_set(4,1,q, 0.5f*sqrtf(1.0f + mtx_get(3,3,dcm) - mtx_get(1,1,dcm)
                                      - mtx_get(2,2,dcm)));
        mtx_set(1,1,q,(mtx_get(1,2,dcm)-mtx_get(2,1,dcm))/(4.0f*mtx_get(4,1,q)));
        mtx_set(2,1,q,(mtx_get(1,3,dcm)+mtx_get(3,1,dcm))/(4.0f*mtx_get(4,1,q)));
        mtx_set(3,1,q,(mtx_get(2,3,dcm)+mtx_get(3,2,dcm))/(4.0f*mtx_get(4,1,q)));

    }

    if(mtx_get(1,1,q)<0) mtx_scale(q,-1.0f,q);
}


/* -- calc_q_err -----------------------------------------------
 * Returns a quaternion of the difference in the current attitude
 * from the desired
 * [BN]*[NR] -> [BR]
 * --------------------------------------------------------------------------*/
void calc_q_err(float *q_bn, float *q_rn, float *q_br) {
	float q_mag;

    q_br[0] = q_bn[0]*q_rn[0] + q_bn[1]*q_rn[1] + q_bn[2]*q_rn[2] + q_bn[3]*q_rn[3];
    q_br[1] = q_bn[1]*q_rn[0] - q_bn[0]*q_rn[1] - q_bn[3]*q_rn[2] + q_bn[2]*q_rn[3];
    q_br[2] = q_bn[2]*q_rn[0] + q_bn[3]*q_rn[1] - q_bn[0]*q_rn[2] - q_bn[1]*q_rn[3];
    q_br[3] = q_bn[3]*q_rn[0] - q_bn[2]*q_rn[1] + q_bn[1]*q_rn[2] - q_bn[0]*q_rn[3];

	q_mag = sqrtf(q_br[0]*q_br[0]+q_br[1]*q_br[1]+q_br[2]*q_br[2]+q_br[3]*q_br[3]);

	q_br[0] /= q_mag;
    q_br[1] /= q_mag;
    q_br[2] /= q_mag;
    q_br[3] /= q_mag;

	if(q_br[0]<0.0f){
		q_br[0] *= -1.0f;
		q_br[1] *= -1.0f;
		q_br[2] *= -1.0f;
		q_br[3] *= -1.0f;
	}
}

/*
 * Function: cal_sun_sens
 * ------------------------
 *  calibrate sun sensors, remove albedo effects
 *
 *  result: mtx_matrix of eci->body frame rotation
 */
void cal_sun_sens( struct mtx_matrix* sun_sens_volt, 
                  struct mtx_matrix* sun_sens_norm, 
                  struct mtx_matrix* sun_sens_cal,
                  float ss_v_cutoff){

    int32_t i;
    struct mtx_matrix cur_sens;
    struct mtx_matrix cur_sum;
    struct mtx_matrix tmp_133a;
    struct mtx_matrix tmp_131a;
    
    // int sun_sens_pair[20] = {15,9,18,8,11,10,9,12,7,6,5,8,18,17,20,19,14,13,16,15}; //Sim Mapping
    int sun_sens_pair[20] = {11, 10,  9, 12,
                              1, 20,  9, 14,
                              3,  2,  1,  4,
                             19, 18, 17, 20,
                             15, 14, 13, 16}; // flight mapping. CSS2 doesn't officially have opposites, but these are the closest to opposite
    //int sun_sens_pair[13] = {11,12,13,6,8,4,5,9,8,9,1,2,3}; //Flight Mapping
    
    mtx_create_ones(1,3,&cur_sens);
    mtx_create_ones(CSS_COUNT,3,&tmp_133a);
    mtx_create_ones(CSS_COUNT,1,&tmp_131a);
    mtx_create_ones(CSS_COUNT,3,&cur_sum);
    
    /*Compare sun sensor against sun sensor on opposite side of craft*/
    for (i=1;i<=CSS_COUNT;i++){
        /*If voltage is less than voltage at opposite location, set to 0*/
        if(mtx_get(i,1,sun_sens_volt)>mtx_get(sun_sens_pair[i-1],1,sun_sens_volt) && 
           mtx_get(i,1,sun_sens_volt)>ss_v_cutoff){
            mtx_set(i,1,sun_sens_cal, mtx_get(i,1,sun_sens_volt));
        } else {
            mtx_set(i,1,sun_sens_cal, 0);
        }
    }
}

/*
 * Function: est_sun_vec_ls
 * ------------------------
 *  Estimates a Sun pointing vector 
 *  based on sun sensor readings using
 *  a Least Squares algorithm
 *
 *  result: Sun vector in the body frame
 */

int32_t est_sun_vec_ls(struct mtx_matrix* sun_sens_volt, 
                    struct mtx_matrix* sun_sens_norm, 
                    struct mtx_matrix* sun_vec,
                    float ss_v_cutoff){
	int32_t i;
    int32_t count = 0;
    struct mtx_matrix norm_trans;
    struct mtx_matrix sun_right;
    struct mtx_matrix sun_left;
    struct mtx_matrix sun_left_inverse;
    struct mtx_matrix sun_sens_cal;
    struct mtx_matrix tmp_31;
    struct mtx_matrix sun_sens_norm_cal;

	/* Initialize Matrices */
    mtx_create_ones(3,1,&sun_right);
    mtx_create_ones(3,1,&tmp_31);
    mtx_create_ones(3,3,&sun_left);
    mtx_create_ones(3,3,&sun_left_inverse);
    mtx_create_ones(3,CSS_COUNT,&norm_trans);
    mtx_create_ones(CSS_COUNT,1,&sun_sens_cal);
    mtx_create_val(CSS_COUNT,3,&sun_sens_norm_cal,0);

    cal_sun_sens(sun_sens_volt,sun_sens_norm,&sun_sens_cal, ss_v_cutoff);

    // static uint32_t print_cnt = 0;
    // print_cnt++;
    /* Check Sun Sensors against threshold voltage*/
	for (i=1;i<=CSS_COUNT;i++){
		if(mtx_get(i,1,&sun_sens_cal)!=0) {
             count++;
             mtx_set(i,1,&sun_sens_norm_cal,    mtx_get(i,1,sun_sens_norm));
             mtx_set(i,2,&sun_sens_norm_cal,    mtx_get(i,2,sun_sens_norm));
             mtx_set(i,3,&sun_sens_norm_cal,    mtx_get(i,3,sun_sens_norm));
        }
	}

	if (count < 3) { 
		for (i=0;i<3;i++){
			mtx_set(1,1,sun_vec,0.0);
			mtx_set(2,1,sun_vec,0.0);
			mtx_set(3,1,sun_vec,0.0);
		}
        g_err|=(0x01ull << ATT_ERR_3);
		return 0;
	}

    // if (print_cnt%7==0){
    //     PRINT("CSS1: %f %f %f %f\r\n",mtx_get(1,1,&sun_sens_cal),
    //                                   mtx_get(2,1,&sun_sens_cal), 
    //                                   mtx_get(3,1,&sun_sens_cal),
    //                                   mtx_get(4,1,&sun_sens_cal));
    //     PRINT("CSS2: %f %f %f %f\r\n",mtx_get(5,1,&sun_sens_cal),
    //                                mtx_get(6,1,&sun_sens_cal),
    //                                mtx_get(7,1,&sun_sens_cal),
    //                                mtx_get(8,1,&sun_sens_cal));
    //     PRINT("CSS3: %f %f %f %f\r\n",mtx_get(9,1,&sun_sens_cal),
    //                                mtx_get(10,1,&sun_sens_cal),
    //                                mtx_get(11,1,&sun_sens_cal),
    //                                mtx_get(12,1,&sun_sens_cal));
    //     PRINT("CSS4: %f %f %f %f\r\n",mtx_get(13,1,&sun_sens_cal),
    //                                mtx_get(14,1,&sun_sens_cal),
    //                                mtx_get(15,1,&sun_sens_cal),
    //                                mtx_get(16,1,&sun_sens_cal));
    //     PRINT("CSS5: %f %f %f %f\r\n",mtx_get(17,1,&sun_sens_cal),
    //                                mtx_get(18,1,&sun_sens_cal),
    //                                mtx_get(19,1,&sun_sens_cal),
    //                                mtx_get(20,1,&sun_sens_cal));
    //     print_cnt = 0;
    // }

	/*Calculate Least Squares solution - inv(norm'*norm)*norm'*sun_sens*/
    //Zero out norms
    mtx_trans(&sun_sens_norm_cal,&norm_trans);
    mtx_mult(&norm_trans, &sun_sens_cal, &sun_right);
    mtx_mult(&norm_trans, &sun_sens_norm_cal, &sun_left);
    mtx_inv(&sun_left, &sun_left_inverse);
    mtx_mult(&sun_left_inverse, &sun_right, &tmp_31);
    mtx_scale(&tmp_31,1.0f/SS_MAX,sun_vec);

	return 1;
}

/*
 * Function: est_quest
 * ------------------------
 *  Estimates a DCM matching the rotation between the observed 
 *  body frame vectors and the reference eci vectors using the 
 *  QUEST method. 
 *  Rodrigues parameters are used for Eigenvector estimate
 *  returns -1 when rotation = 180 deg.
 *
 *  result: mtx_matrix of eci->body frame rotation
 */

int32_t est_quest_rp(struct mtx_matrix* b_k, struct mtx_matrix* eci_k, 
                    struct est_state* state, struct mtx_matrix* dcm_out){

    struct mtx_matrix q_out;
    struct mtx_matrix b;
    struct mtx_matrix s;
    struct mtx_matrix z;
    struct mtx_matrix tmp_33a;
    struct mtx_matrix tmp_33b;
    struct mtx_matrix eye_33;
    struct mtx_matrix m_inv;
    struct mtx_matrix norm_obs;
    struct mtx_matrix norm_eci;
    struct mtx_matrix tmp_13a;
    struct mtx_matrix p;
    struct mtx_matrix q_scale_sub;
    float sigma;
    float a_i;

    uint8_t i;
    uint8_t j;
    float norm_obs_in;
    float norm_eci_in;
    float q_scale;
    /*Eigenvalue of K ~= 1*/
    float eig_val = 1.0;

    mtx_create_ones(3,1,&norm_obs);
    mtx_create_ones(3,1,&norm_eci);
    mtx_create_ones(4,1,&q_out);
    mtx_create_val(3,3,&b,0);
    mtx_create_ones(3,3,&s);
    mtx_create_ones(3,1,&z);
    mtx_create_eye(3,3,&eye_33);
    mtx_create_ones(3,3,&m_inv);
    mtx_create_ones(3,1,&p);
    mtx_create_ones(1,1,&q_scale_sub);
    mtx_create_ones(1,3,&tmp_13a);
    mtx_create_val(3,3,&tmp_33a,0);
    mtx_create_val(3,3,&tmp_33b,0);

    #define PLIM -1
    static int print_cnt = 0;
    if(print_cnt<=PLIM){
        PRINT("est 1 state.att_quaternion: ");
        mtx_print(&(state->att_quaternion));
    }

    /*B = SUM(a_i*|b_k|*|eci_k|')*/
    a_i = 0.5f;
    if(print_cnt<=PLIM){
        PRINT("b_k: ");
        mtx_print(b_k);
        PRINT("eci_k: ");
        mtx_print(eci_k);
    }

    for (i=1;i<=b_k->cols;i++){
        norm_obs_in = sqrtf(mtx_get(1,i,b_k)*mtx_get(1,i,b_k)
                            + mtx_get(2,i,b_k)*mtx_get(2,i,b_k)
                            + mtx_get(3,i,b_k)*mtx_get(3,i,b_k)); 
        norm_eci_in = sqrtf(mtx_get(1,i,eci_k)*mtx_get(1,i,eci_k)
                            + mtx_get(2,i,eci_k)*mtx_get(2,i,eci_k)
                            + mtx_get(3,i,eci_k)*mtx_get(3,i,eci_k)); 
        mtx_set(1,1,&norm_obs,   mtx_get(1,i,b_k)/norm_obs_in); 
        mtx_set(2,1,&norm_obs,   mtx_get(2,i,b_k)/norm_obs_in); 
        mtx_set(3,1,&norm_obs,   mtx_get(3,i,b_k)/norm_obs_in); 
        mtx_set(1,1,&norm_eci,   mtx_get(1,i,eci_k)/norm_eci_in); 
        mtx_set(2,1,&norm_eci,   mtx_get(2,i,eci_k)/norm_eci_in); 
        mtx_set(3,1,&norm_eci,   mtx_get(3,i,eci_k)/norm_eci_in); 
        mtx_trans(&norm_eci,&tmp_13a);
        mtx_mult(&norm_obs,&tmp_13a,&tmp_33a);
        mtx_scale(&tmp_33a,a_i,&tmp_33b);
        mtx_sum(&b,&tmp_33b,&tmp_33a);
        mtx_copy(&tmp_33a,&b);
    }

    if(print_cnt<=PLIM){
        PRINT("b: ");
        mtx_print(&b);
    }

    /*sigma, S, Z*/
    sigma = mtx_trace(&b);
    mtx_trans(&b,&tmp_33a);
    mtx_sum(&b,&tmp_33a,&s);
    mtx_set(1,1,&z, mtx_get(2,3,&b)-mtx_get(3,2,&b));
    mtx_set(2,1,&z, mtx_get(3,1,&b)-mtx_get(1,3,&b));
    mtx_set(3,1,&z, mtx_get(1,2,&b)-mtx_get(2,1,&b));

    if(print_cnt<=PLIM){
        PRINT("z: ");
        mtx_print(&z);
    }


    /*Pranith&Alex->New Algorithm(Newton-Raphson) to calculate max eigenvalue*/
    struct mtx_matrix s_squared;
    struct mtx_matrix z_transpose;
    struct mtx_matrix in_between;
    struct mtx_matrix after;
    struct mtx_matrix k;
    struct mtx_matrix salter;
    struct mtx_matrix sinbetween;
    struct mtx_matrix Z_res;
    mtx_create_ones(3,3,&s_squared);
    mtx_create_ones(1,3,&in_between);
    mtx_create_ones(1,1,&after);
    mtx_create_ones(1,1,&Z_res);
    mtx_create_ones(1,3,&z_transpose);
    mtx_create_ones(3,3,&salter);
    mtx_create_ones(3,3,&sinbetween);
    mtx_create_ones(4,4,&k);
    mtx_mult(&s,&s,&s_squared);
    float p2 = mtx_trace(&s_squared);
    mtx_trans(&z,&z_transpose);
    mtx_mult(&z_transpose,&z,&Z_res);
    float Z = mtx_get(1, 1, &Z_res);
    float znorms = 0.0f;
    for(i=0; i<3; i++){
        znorms+=(mtx_get(i+1,1,&z)*mtx_get(i+1,1,&z));
    }
    znorms=sqrtf(znorms);

    float A = (sigma*sigma) -(0.5* p2) + Z;
    mtx_mult(&z_transpose,&s,&in_between);
    mtx_mult(&in_between,&z,&after);
    float B = (sigma*Z) - mtx_get(1,1,&after);
    float D = 0; 
    mtx_scale(&eye_33,-1.0*sigma, &sinbetween);
    mtx_sum(&s,&sinbetween,&salter);
    mtx_set(1,1,&k,sigma);
    for(i=1; i<=3; i++){
        mtx_set(1,i+1,&k,mtx_get(1,i,&z_transpose));
        mtx_set(i+1,1,&k,mtx_get(i,1,&z));
    }
    for(i=1; i<=3; i++){
        for(j=1; j<=3; j++){
            mtx_set(i+1,j+1,&k,mtx_get(i,j,&salter));
        }
    }
    D=mtx_det(&k);

    float lambcurr;
    float lambprev;
    float diff = 1;
    float resl = 0.0f;
    float best_lambda = 0.0f;
    float min_fabs = 1e9f;
    float fallback_lambda = 0.5f;
    float fabs_f = 0.0f;
    float it = 0.0f;
    uint8_t v;

   //Find to the tenths place max EV
    for (v = 0; v < 11; v++) {
    it = 0.5f + 0.1f * v;
    resl = a_initial(A, B, D, it);
    fabs_f = fabsf(resl);
    
    if ((fabs_f < 1e-3f) && (it > best_lambda)) {
        best_lambda = it;
        }
    if (fabs_f < min_fabs) {
        min_fabs = fabs_f;
        fallback_lambda = it;
        }
    }

    lambcurr = (best_lambda > 0.0f) ? best_lambda : fallback_lambda;

    do{
        lambprev=lambcurr;
        lambcurr= lambprev - a_initial(A,B,D,lambcurr)/p_derivative(A,B,lambcurr);
        diff = fabsf(lambcurr-lambprev);
    }while(diff>1e-6);

    if(lambcurr<0.5) lambcurr=1.0f;
    eig_val=lambcurr;


    /*M_inv = (eig_val + sigma)*I - S*/
    mtx_scale(&eye_33, eig_val + sigma, &tmp_33a);
    mtx_scale(&s, -1, &tmp_33b);
    mtx_sum(&tmp_33a, &tmp_33b, &m_inv);

    if(print_cnt<=PLIM){
        PRINT("m_inv: ");
        mtx_print(&m_inv);
    }

    /*Check if matrix is non-invertable (rot=180)*/
    if (mtx_det(&m_inv)<MIN_MINV_DET) PRINT("ERR->Noninvertible\r\n");

    /* p = inv(M_inv)*Z*
     * q = 1/sqrt(1+p'*p)*[p;1] */
    mtx_inv(&m_inv,&tmp_33a);
    mtx_mult(&tmp_33a,&z,&p);
    mtx_trans(&p,&tmp_13a);
    mtx_mult(&tmp_13a,&p,&q_scale_sub);
    q_scale = 1.0f/sqrtf(1.0f+mtx_get(1,1,&q_scale_sub));
    mtx_set(1,1,&state->att_quaternion, q_scale); // SCALAR
    mtx_set(2,1,&state->att_quaternion, q_scale*mtx_get(1,1,&p));
    mtx_set(3,1,&state->att_quaternion, q_scale*mtx_get(2,1,&p));
    mtx_set(4,1,&state->att_quaternion, q_scale*mtx_get(3,1,&p));
    

    q_2_dcm(&state->att_quaternion, dcm_out);

    if(print_cnt<=PLIM){
        PRINT("est 2 state.att_quaternion: ");
        mtx_print(&state->att_quaternion);
        print_cnt++;
    }

    return 1;  
}

/* Pranith & Alex -> characteristic polynomial and derivative functions of det(K-lambda*I) */
float a_initial(float A, float B, float D, float lamb){
    return (lamb*lamb*lamb*lamb) - (1*lamb*lamb*lamb) + (A*lamb*lamb) - (B*lamb) + D;
}

float p_derivative(float A, float B, float lamb){
    return (4*lamb*lamb*lamb) - (3*lamb*lamb) + (2*A*lamb) - B;
}

/*
 * Function: est_simp
 * ------------------------
 *  Simplified Estimator
 *  Integrates rate gyros if necessary and calls QUEST estimator
 *
 *  result: mtx_matrix of eci->body frame rotation
 */

int32_t est_simp(struct mtx_matrix* sun_sens_volt, struct mtx_matrix* sun_sens_norm,
                    struct mtx_matrix* mag_sens, 
                    struct mtx_matrix* sun_eci, struct mtx_matrix* mag_eci, 
                    struct mtx_matrix* sun_vec,
                    struct est_state* state, struct mtx_matrix* dcm_out,
                    float ss_v_cutoff){

    static int32_t init_dcm = 0;
    static struct mtx_matrix prior_dcm;

    int32_t rv;
    struct mtx_matrix b_k;
    struct mtx_matrix eci_k;
    struct mtx_matrix post_dcm;
    mtx_create_ones(3,2,&b_k);
    mtx_create_ones(3,2,&eci_k);
    mtx_create_ones(3,3,&post_dcm);

    //Initialize prior DCM
    if(init_dcm==0){
        mtx_create_eye(3,3,&prior_dcm);
        init_dcm = 1;
    }

    /*Add magnetic field vector to observation matrix*/
    mtx_set(1,1,&b_k,   mtx_get(1,1,mag_sens));    
    mtx_set(2,1,&b_k,   mtx_get(2,1,mag_sens));    
    mtx_set(3,1,&b_k,   mtx_get(3,1,mag_sens));    
    mtx_set(1,1,&eci_k, mtx_get(1,1,mag_eci));
    mtx_set(2,1,&eci_k, mtx_get(2,1,mag_eci));
    mtx_set(3,1,&eci_k, mtx_get(3,1,mag_eci));

    //If three sensors are active, add sun sensor to observation matrix, otherwise use dynamics model
    if(est_sun_vec_ls(sun_sens_volt, sun_sens_norm, sun_vec, ss_v_cutoff)==1){
        mtx_set(1,2,&b_k,   mtx_get(1,1,sun_vec));    
        mtx_set(2,2,&b_k,   mtx_get(2,1,sun_vec));    
        mtx_set(3,2,&b_k,   mtx_get(3,1,sun_vec));    
        mtx_set(1,2,&eci_k, mtx_get(1,1,sun_eci));
        mtx_set(2,2,&eci_k, mtx_get(2,1,sun_eci));
        mtx_set(3,2,&eci_k, mtx_get(3,1,sun_eci));
        rv = est_quest_rp(&b_k,&eci_k,state,dcm_out); // ATT Error 2
    } else {
        rv = att_det_reck(&prior_dcm, &state->sat_body_rates, &state->att_quaternion);
        q_2_dcm(&state->att_quaternion, dcm_out);
        
    }
    mtx_copy(dcm_out,&prior_dcm);

    return rv;
}

/* Pranith->Quaternion Dynamics Model */
uint8_t att_det_reck(struct mtx_matrix *prior_dcm, struct mtx_matrix *mtx_gyro_rates, struct mtx_matrix *quat_out){
    struct mtx_matrix quat_der;
    struct mtx_matrix quat_initial;
    struct mtx_matrix quat_rate;
    struct mtx_matrix inter;
    struct mtx_matrix mediate;
    uint8_t b;

    float gyro_rates[3] = {mtx_get(1, 1, mtx_gyro_rates), mtx_get(2, 1, mtx_gyro_rates), mtx_get(3, 1, mtx_gyro_rates)};
    uint8_t ret = 0;

    mtx_create_val(4,4,&quat_rate,0.0f);
    mtx_create_ones(4,1,&quat_der);
    mtx_create_ones(4,1,&quat_initial);
    mtx_create_ones(4,1,&inter);
    mtx_create_ones(4,1,&mediate);


    dcm_2_q(prior_dcm,&quat_initial);

    for(b=0; b<4; b++){
        mtx_set(b+1, b+1, &quat_rate, 0);
    }
    for (b=2; b<5; b++){
        mtx_set(b,1,&quat_rate, gyro_rates[b-2]);
        mtx_set(1,b,&quat_rate, gyro_rates[b-2]*-1.0f);
    }

    mtx_set(3,2,&quat_rate, gyro_rates[2]*-1.0f);
    mtx_set(4,2,&quat_rate, gyro_rates[1]);
    mtx_set(2,3,&quat_rate, gyro_rates[2]);
    mtx_set(2,4,&quat_rate, gyro_rates[1]*-1.0f);
    mtx_set(4,3,&quat_rate, gyro_rates[0]*-1.0f);
    mtx_set(3,4,&quat_rate, gyro_rates[0]);


    mtx_mult(&quat_rate, &quat_initial, &inter);
    mtx_scale(&inter,0.5f,&quat_der);
    mtx_scale(&quat_der, 0.1f, &mediate);
    mtx_sum(&quat_initial, &mediate,quat_out);
    
    float mag = mtx_norm(quat_out);
    if (mag < 1e-6f) {
    PRINT("ERR->DIVIDE BY 0\r\n");
    return ret;
    }
    mtx_scale(quat_out, 1.0f/mag, quat_out);

    ret = 1;
    return ret;
}

/*
 * Function: create_sun_q 
 * ------------------------
 * Create sun pointing desired quaternion
 *
 */

void create_sun_des_q(uint8_t op_mode, float *sun_vec, float *des_RN_q, float *q_BR, struct mtx_matrix *q_BN_mtx,uint8_t flag){


    if((flag && (1<<0))==0) return;

    float sun_vec_normlzd[3] = {0.0,0.0,0.0};
    float sun_vec_norm = 0.0f;
    uint8_t i=0;


    sun_vec_norm = norm(sun_vec);  //normalize
    if(sun_vec_norm!=0.0f){
        for(i=0; i<3; i++)
            sun_vec_normlzd[i] = sun_vec[i]/sun_vec_norm;
    } else { 
        sun_vec_normlzd[0] = -1.0f;
        sun_vec_normlzd[1] = 0.0f;
        sun_vec_normlzd[2] = 0.0f;
    }

    float des_bf_vec[3] = {1.0, 0.0, 0.0}; //+X solar panels
    if(((flag) & (1<<4))==0){   // no gps result
        q_from_2uvecs(des_bf_vec, sun_vec_normlzd, q_BR); //Directly compute error quaternion
        return;
    } 
    create_desRN(op_mode,sun_vec,des_RN_q);                    //Gps Result
}


/* Pranith->Improved Algorithm to find phicone */
void create_phicone_des_q(uint8_t op_mode, float *pos_vec, float *sun_vec, float *des_RN_q, float phi_cone_angle,float *phi_vec_normlzd)
{   
   /*
    * Input ECI position and sun vectors and point GPS antenna (-X) along an intermediate
    vector between the two which lies on the same plane as those two vectors.  This results
    in a reference attitude that is "phi_cone_angle" degrees off of the true zenith pointing
    vector.
    */

    // Initializing matrices/variables

    float sun_vec_orth[3] = {0.0,0.0,0.0};
    float sun_vec_orth_normlzd[3]={0.0};
    float sun_vec_proj[3]={0.0};
    float pos_vec_normlzd[3]={0.0};
    float phi_vec[3] = {0.0,0.0,0.0};
    float phi_vec_norm = 0.0f;
    float pos_vec_norm = 0.0f;
    float sun_vec_orth_norm = 0.0f;
 

    float sin_phi = 0.0f;
    float cos_phi = 0.0f;
    float phi_rad = 0.0f;

    float unitx[3] = {1.0,0.0f,0.0f};
    float unity[3] = {0.0f,1.0f,0.0f};
    // float rep[3] = {0.0f};
    float dotter=0.0f;
    float sun_vec_norm = 0.0f;
    uint32_t i=0;


    phi_rad = phi_cone_angle * ((float)M_PI / 180.0);
    cos_phi=cos(phi_rad);
    sin_phi=sin(phi_rad);

    pos_vec_norm = norm(pos_vec);
    sun_vec_norm = norm(sun_vec);
    if(pos_vec_norm==0.0f) {
        phi_vec_normlzd[0]=-1.0f;
        phi_vec_normlzd[1]=0.0f;
        phi_vec_normlzd[2]=0.0f;
        // PRINT("NO POSITION VECTOR\r\n"); //rbf
    }
    // else{
    for(i=0; i<3; i++){
        pos_vec_normlzd[i]=pos_vec[i]/pos_vec_norm;  //normalizing the pos vector
        sun_vec[i]/=sun_vec_norm;
    }
    dotter=dot(sun_vec,pos_vec_normlzd);
    for(i=0; i<3; i++){ //calculating projection of sun vector onto pos vector first
        sun_vec_proj[i]=dotter*pos_vec_normlzd[i];
        sun_vec_orth[i]=sun_vec[i]-sun_vec_proj[i]; //calculate sun vector orthogonal to pos vector
    }
    
    sun_vec_orth_norm=norm(sun_vec_orth);
    if(sun_vec_orth_norm<1e-6){   //edge case handle
        if(fabs(dot(pos_vec_normlzd, unitx)) < 0.99f) crossp(pos_vec_normlzd, unitx, sun_vec_orth); //replacement "fake" sun vector
        else if(fabs(dot(pos_vec_normlzd, unity)) < 0.99f){
            crossp(pos_vec_normlzd, unity, sun_vec_orth);
            sun_vec_orth_norm=norm(sun_vec_orth);
        }
    }
    for(i=0; i<3; i++){
        sun_vec_orth_normlzd[i]=sun_vec_orth[i]/sun_vec_orth_norm; //normalized orthogonal vector
    }
    for(i=0; i<3; i++){ //calculate phi vector
        phi_vec[i]=(cos_phi*pos_vec_normlzd[i]) + (sin_phi*sun_vec_orth_normlzd[i]);
    }
    phi_vec_norm = norm(phi_vec);
    for(i=0; i<3; i++){
        phi_vec_normlzd[i]=phi_vec[i]/phi_vec_norm;
        // printf("PHICONE VECTOR-> %f\r\n",phi_vec_normlzd[i]); ///rbf
        }

    create_desRN(op_mode, phi_vec_normlzd, des_RN_q);
 
}

/*************************************************************************************
 * Ground Station pointing functions - written by Anthony Zara
 ************************************************************************************/

/*
 * Function: sc_to_gs_vec
 * ------------------------
 * Position vector FROM spacecraft TO ground station (used for GS pointing)
 *
 */
void sc_to_gs_vec(float *sc_pos_vec_ecef, float *gnd_stn_pos_vec_ecef, float *sc_2_gs)
{   
    int i;
    for(i=0;i<3;i++){sc_2_gs[i] = gnd_stn_pos_vec_ecef[i] - sc_pos_vec_ecef[i];}
}

/*
 * Function: gs_to_sc_vec
 * ------------------------
 * Position vector FROM ground station TO spacecraft (used to compute elev angle)
 *
 */
void gs_to_sc_vec(float *sc_pos_vec_ecef, float *gnd_stn_pos_vec_ecef, float *gs_2_sc)
{   
    int i;
    for(i=0;i<3;i++){gs_2_sc[i] = sc_pos_vec_ecef[i] - gnd_stn_pos_vec_ecef[i];}
}

/*
 * Function: angle_2_gnd_stn
 * ------------------------
 * Zenith angle of spacecraft with respect to ground station, distance to ground station
 *
 */
void gs_angle_distance(float *sc_pos_vec_ecef, float *gnd_stn_pos_vec_ecef, float *angle, float *distance)
{
    /*
    * Input ECEF position vector of spacecraft and ground station, returns Zenith angle in degrees, distance to GS
    */   
    float gs_2_sc[3] = {0.0, 0.0, 0.0};
    float angle_rad;

    gs_to_sc_vec(sc_pos_vec_ecef, gnd_stn_pos_vec_ecef, gs_2_sc);
    *distance = norm(gs_2_sc);
    angle_rad = acos(dot(gnd_stn_pos_vec_ecef, gs_2_sc) / (norm(gnd_stn_pos_vec_ecef)*norm(gs_2_sc)));
    *angle = angle_rad * 180 / ((float)M_PI);
}

/*
 * Function: sc_to_gs_unitvec
 * ------------------------
 * NORMALIZED Position vector FROM spacecraft TO ground station (used for GS pointing)
 *
 */
void sc_to_gs_unitvec(float *sc_pos_vec_ecef, float *gnd_stn_pos_vec_ecef, float *sc_2_gs_normlzd)
{   
    float sc_2_gs[3];
    float vec_norm;
    uint8_t i;

    sc_to_gs_vec(sc_pos_vec_ecef, gnd_stn_pos_vec_ecef, sc_2_gs);

    vec_norm = norm(sc_2_gs);
    for(i=0; i<3; i++){sc_2_gs_normlzd[i] = sc_2_gs[i]/vec_norm;}
    
}


/*
 * Function: create_gnd_stn_des_q
 * ------------------------
 * Create ground station pointing desired quaternion for downlink operations

 */

void create_gnd_stn_des_q(uint8_t op_mode, struct mtx_matrix* q_BN_vecmtx,
    float *sc_pos_vec_ecef, float *gnd_stn_pos_vec_ecef, float *des_RN_q, uint8_t antenna,uint8_t leapsec)
{   
   /*
    * Input ECEF position vector of spacecraft and ground station and point either the X-band (-X) or T-DAGHR (+X)
    * along the resultant difference vector.
    */
   

    float sc_2_gs_vec_normlzd[3] = {0.0,0.0,0.0};
    float pos_vec_norm = 0.0f;
    float sc_2_gs_vec_normlzed_ECI[3]={0.0}; 
    
    /* Normalize Pos Vec (in ECEF) */
    pos_vec_norm = norm(sc_pos_vec_ecef);
    if(pos_vec_norm!=0.0f){
        // If we have a position vec, use spacecraft to ground station pointing vector
        sc_to_gs_unitvec(sc_pos_vec_ecef, gnd_stn_pos_vec_ecef, sc_2_gs_vec_normlzd);
    } else { // If no info on vec, point it to +Z (in ECI)
        sc_2_gs_vec_normlzd[0] = -1.0f;
        sc_2_gs_vec_normlzd[1] = 0.0f;
        sc_2_gs_vec_normlzd[2] = 0.0f;
    }

    if(!S_ECEF2ECI((float)g_J2000_time,g_J2000_frac_time,sc_2_gs_vec_normlzd,sc_2_gs_vec_normlzed_ECI,leapsec)){
        PRINT("ERROR CONVERTING FROM ECEF TO ECI IN DCOMM\r\n"); //rbf
    } 
    else{
        // PRINT("SUCCESSFUL CONVERSION\r\n"); //rbf
    }

    create_desRN(op_mode,sc_2_gs_vec_normlzed_ECI,des_RN_q); 

}

/*Pranith -> Algorithm to compute desired quaternion*/
void create_desRN(uint8_t opmode, float *des_vec, float *des_RN){ //des_vec is in ECI frame, DCM expressed in ECI
uint8_t i,b;
struct mtx_matrix dcm_NR, dcm_RN, quat_des;
float unitx[3] = {1.0f,0.0f,0.0f};
float unity[3] = {0.0f,1.0f,0.0f};
float xaxis[3] = {0.0f};
float yaxis[3] = {0.0f};
float zaxis[3] = {0.0f};
float xnorm=1, ynorm=1,znorm=1,q_norm=1;
uint8_t min = 0;

xnorm=norm(des_vec);

if(opmode==CTRL_SUN){
    for(i=0; i<3; i++) xaxis[i]=des_vec[i]/xnorm;
}
else if(opmode==CSAC){
    printf("Csac\n");
    for(i=0; i<3; i++) xaxis[i]=(des_vec[i]*-1.0f)/xnorm;
}

if (fabs(dot(xaxis, unitx)) < 0.99f) crossp(xaxis, unitx, yaxis);
else if(fabs(dot(xaxis, unity)) < 0.99f) crossp(xaxis, unity, yaxis);
else PRINT("SHOULD NOT PRINT!, DES QUAT ERR\r\n");

ynorm=norm(yaxis);
for(i=0; i<3; i++) yaxis[i]/=ynorm;
crossp(xaxis,yaxis,zaxis);          //z is automatically normalized, but to be safe

znorm = norm(zaxis);
for(i=0; i<3; i++) zaxis[i] /= znorm; 

mtx_create_ones(3,3,&dcm_NR);
mtx_create_ones(3,3,&dcm_RN);
mtx_create_ones(4,1,&quat_des);
for(i=0;i<3;i++){                           //each column represents an axis
    for(b=0;b<3;b++){
       if(i==0) mtx_set(b+1,i+1,&dcm_NR,xaxis[b]);            
       else if(i==1) mtx_set(b+1,i+1,&dcm_NR,yaxis[b]);
       else mtx_set(b+1,i+1,&dcm_NR,zaxis[b]);
    }
}
if(mtx_det(&dcm_NR)<0) PRINT("ERR->DCM det is %f\r\n",mtx_det(&dcm_NR));

mtx_trans(&dcm_NR,&dcm_RN);
dcm_2_q(&dcm_RN, &quat_des);
for(i = 0; i<4; i++){
    des_RN[i]=mtx_get(i+1, 1,&quat_des);           //final desired attitude quaternion
}

if(des_RN[0]<0) min=1;
q_norm=normq(des_RN);             //normalize
if(q_norm==0) PRINT("ERR->qnorm is 0\r\n");
for(i=0; i<4; i++){
    des_RN[i]/= (min ? -1*q_norm : q_norm);
}

}

/* Pranith */
/* New algorithm to calculate desired rates in DCOMM mode */
void create_des_rates_DCOMM(uint8_t opmode, float *pos_ecef, float *gnd_ecef, 
float *des_rates_bf, float *q_BN, uint8_t leapsec){

static uint8_t start = 0;
float sc_2_gs_normlzd_ECEF[3]={0.0f};
float sc_2_gs_normlzd_ECI[3]={0.0f};
float prev_sc_2_gs_normlzd_ECEF[3]={0.0f};
static float prev_sc_2_gs_normlzd[3]={0.0f};
uint8_t i;

if(!start){ 
for(i = 0; i<3; i++){
    des_rates_bf[i]=0.0f;
    }
sc_to_gs_unitvec(pos_ecef, gnd_ecef, prev_sc_2_gs_normlzd_ECEF); //set current to previous for the next iteration
if(!S_ECEF2ECI((float)g_J2000_time,g_J2000_frac_time,prev_sc_2_gs_normlzd_ECEF,prev_sc_2_gs_normlzd,leapsec)){
    PRINT("ERR 1\r\n");
}
start=1;
return;
}

sc_to_gs_unitvec(pos_ecef, gnd_ecef, sc_2_gs_normlzd_ECEF);  //Not the first time in the loop
if(!S_ECEF2ECI((float)g_J2000_time,g_J2000_frac_time,sc_2_gs_normlzd_ECEF,sc_2_gs_normlzd_ECI,leapsec)){
    PRINT("ERR 2\r\n");
} 


calculateOmega(prev_sc_2_gs_normlzd,sc_2_gs_normlzd_ECI,des_rates_bf,q_BN); //ECI vectors

}


/* Pranith */
/* New algorithm to calculate desired rates in CSAC Mode */
void create_des_rates_CSAC(uint8_t opmode, float *phi_cone,
float *des_rates_bf, float *q_BN){

static uint8_t start = 0;
static float prev_phi_cone[3]={0.0f};
uint8_t i;
if(!start){ 
for(i = 0; i<3; i++){
    des_rates_bf[i]=0.0f;
    prev_phi_cone[i]=phi_cone[i];
    start=1;
}
    return;
}

calculateOmega(prev_phi_cone, phi_cone, des_rates_bf, q_BN);
}


/* Pranith */
void calculateOmega(float *prev_vec, float *current_vec, float *des_rates_bf, float *q_BN){
float two_vec_cross[3]={0.0f};
float cnorm;
double two_vec_dot;
double dot_clamped;
double vec_angle;
float des_rates[3]={0.0f};
uint8_t i,b;
struct mtx_matrix des_rates_mtx, des_rates_bf_mtx, q_BN_mtx;
struct mtx_matrix dcm_BN;

two_vec_dot=dot(prev_vec,current_vec); //add dodot

double norm_prev = 0.0;
double norm_curr = 0.0;
for (i = 0; i < 3; i++) {
    norm_prev += (double)prev_vec[i] * (double)prev_vec[i];
    norm_curr += (double)current_vec[i] * (double)current_vec[i];
}
norm_prev = sqrt(norm_prev);
norm_curr = sqrt(norm_curr);

dot_clamped = fmin(fmax(two_vec_dot / (norm_prev * norm_curr), -1.0), 1.0);
vec_angle = acos(dot_clamped);

crossp(prev_vec, current_vec,two_vec_cross);
cnorm=norm(two_vec_cross);
for(i=0; i<3;i++) two_vec_cross[i]/=cnorm;

for(b = 0; b<3; b++){
des_rates[b]= two_vec_cross[b] * ( (vec_angle)/(0.50)); //10hz //over 5 iterations
}

mtx_create(3,1,des_rates,&des_rates_mtx); //convert des rates into matrix form
mtx_create(4,1,q_BN,&q_BN_mtx);            //convert q_Bn into matrix form
mtx_create_ones(3,1,&des_rates_bf_mtx); //where we will store final des rates mtx
mtx_create_ones(3,3, &dcm_BN);          //initalize dcm matrix for ECI->body frame rotation
q_2_dcm(&q_BN_mtx,&dcm_BN);             //convert q_Bn matrix form into dcm 
mtx_mult(&dcm_BN,&des_rates_mtx,&des_rates_bf_mtx);

for(i=0; i<3; i++){
    des_rates_bf[i]=mtx_get(i+1,1,&des_rates_bf_mtx); //final body rates

}

//set current vector to previous for next loop
for(i = 0; i<3; i++){
    prev_vec[i]=current_vec[i];
}

}

/*
 * Function: q_from_2uvecs
 * -------------------------
 *  Finds quaternion rotation that brings v1 congruent to v2
 */
 void q_from_2uvecs(float *v1, float *v2, float *q_v1v2){
	float q_v1v2_norm = 0.0f;
	float temp_vec[3] = {0.0, 0.0, 0.0};

	crossp(v1, v2, temp_vec);

    // |temp_vec| = sin(theta)
    float halfangle = asinf(sqrtf(dot(temp_vec,temp_vec)));

    if(dot(v1,v2)<0)
        halfangle = (float)M_PI - halfangle;

    halfangle *= 0.5;

    if (halfangle>1e-10){
        q_v1v2[0] = cosf(halfangle);
        q_v1v2[1] = temp_vec[0]/sqrtf(dot(temp_vec,temp_vec))*sinf(halfangle);
        q_v1v2[2] = temp_vec[1]/sqrtf(dot(temp_vec,temp_vec))*sinf(halfangle);
        q_v1v2[3] = temp_vec[2]/sqrtf(dot(temp_vec,temp_vec))*sinf(halfangle);

        q_v1v2_norm = sqrtf( q_v1v2[0]*q_v1v2[0] +
                q_v1v2[1]*q_v1v2[1] + q_v1v2[2]*q_v1v2[2] +
                q_v1v2[3]*q_v1v2[3]);

        q_v1v2[0] /= q_v1v2_norm;
        q_v1v2[1] /= q_v1v2_norm;
        q_v1v2[2] /= q_v1v2_norm;
        q_v1v2[3] /= q_v1v2_norm;
	} else {
        q_v1v2[0] = 1.0;
        q_v1v2[1] = 0.0;
        q_v1v2[2] = 0.0;
        q_v1v2[3] = 0.0;
    }
 }
