#define _USE_MATH_DEFINES

#include <stdio.h>
#include <math.h>
#include "mtx.h"
#include "att_det.h"
#include "conversions.h"
#include "types.h"
#include "mex.h"
//#include "uart_dbg.h"

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

/*
 * Function: body_rate_dcm_rot
 * ------------------------
 *  Rotates a DCM array by a
 *  body rate vector array
 *
 *  result: DCM
 */

void body_rate_dcm_rot(struct mtx_matrix* body_rates, struct mtx_matrix* prior_dcm, 
                        struct mtx_matrix* rot_dcm){
	float timestep = 1.0f/UPDATE_RATE;

    struct mtx_matrix w_ss;
    struct mtx_matrix diff_dcm;
    struct mtx_matrix diff_dcm_scaled;
    struct mtx_matrix body_rates_rad;

    mtx_create_ones(3,3,&w_ss);
    mtx_create_ones(3,3,&diff_dcm);
    mtx_create_ones(3,3,&diff_dcm_scaled);
    mtx_create_ones(3,1,&body_rates_rad);

    //Body Rate reversal???
    mtx_scale(body_rates,-1.0f*(float)M_PI/180.0f,&body_rates_rad);
    mtx_ss(&body_rates_rad, &w_ss);
    mtx_mult(&w_ss,prior_dcm,&diff_dcm);
    mtx_scale(&diff_dcm,timestep,&diff_dcm_scaled);
    mtx_sum(&diff_dcm_scaled,prior_dcm,rot_dcm);

	return;
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
    
//    int sun_sens_pair[13] = {4,5,6,1,2,3,12,11,10,9,8,7,12}; //Sim Mapping
//    int sun_sens_pair[13] = {11,12,13,6,8,4,5,9,8,9,1,2,3}; //Flight Mapping
    int sun_sens_pair[15] = {13,14,15,6,7,9,5,6,11,4,9,6,1,2,3}; //Flight Mapping
    
    mtx_create_ones(1,3,&cur_sens);
    mtx_create_ones(SS_COUNT,3,&tmp_133a);
    mtx_create_ones(SS_COUNT,1,&tmp_131a);
    mtx_create_ones(SS_COUNT,3,&cur_sum);

    
    /*Compare sun sensor against sun sensor on opposite side of craft*/
    for (i=1;i<=SS_COUNT;i++){
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
    int32_t ss_count = 0;
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
    mtx_create_ones(3,SS_COUNT,&norm_trans);
    mtx_create_ones(SS_COUNT,1,&sun_sens_cal);
    mtx_create_val(SS_COUNT,3,&sun_sens_norm_cal,0);

    static uint32_t print_cnt = 0;
    static uint32_t loop_cnt = 0;
    print_cnt++;
    loop_cnt++;

    cal_sun_sens(sun_sens_volt,sun_sens_norm,&sun_sens_cal, ss_v_cutoff);

    /* Check Sun Sensors against threshold voltage*/
	for (i=1;i<=SS_COUNT;i++){
		if(mtx_get(i,1,&sun_sens_cal)!=0) {
		     ss_count++;
		     mtx_set(i,1,&sun_sens_norm_cal,    mtx_get(i,1,sun_sens_norm));
		     mtx_set(i,2,&sun_sens_norm_cal,    mtx_get(i,2,sun_sens_norm));
		     mtx_set(i,3,&sun_sens_norm_cal,    mtx_get(i,3,sun_sens_norm));
                if (print_cnt%6==0){
			
                    PRINT("NORM%d: %f %f %f\r\n",i,mtx_get(i,1,&sun_sens_norm_cal),
                                                   mtx_get(i,2,&sun_sens_norm_cal),
                                                   mtx_get(i,3,&sun_sens_norm_cal));   
		}
            }
	}

	if (ss_count < 3) { 
		for (i=0;i<3;i++){
			mtx_set(1,1,sun_vec,0.0);
			mtx_set(2,1,sun_vec,0.0);
			mtx_set(3,1,sun_vec,0.0);
		}
		return 0;
	}
    if (print_cnt%6==0)
        PRINT("CSS1: %f %f %f %f\r\n",mtx_get(1,1,&sun_sens_cal),
                                      mtx_get(2,1,&sun_sens_cal), 
                                      mtx_get(3,1,&sun_sens_cal),
                                      mtx_get(4,1,&sun_sens_cal));
    if (print_cnt%6==0)
        PRINT("CSS2: %f %f %f\r\n",mtx_get(5,1,&sun_sens_cal),
                                   mtx_get(6,1,&sun_sens_cal),
                                   mtx_get(7,1,&sun_sens_cal));
    if (print_cnt%6==0)
        PRINT("CSS3: %f %f %f\r\n",mtx_get(8,1,&sun_sens_cal),
                                   mtx_get(9,1,&sun_sens_cal),
                                   mtx_get(10,1,&sun_sens_cal));
    if (print_cnt%6==0){
        PRINT("CSS4: %f %f %f\r\n",mtx_get(11,1,&sun_sens_cal),
                                   mtx_get(12,1,&sun_sens_cal),
                                   mtx_get(13,1,&sun_sens_cal));
        print_cnt = 0;
    }
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

    int32_t k;
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

    /*B = SUM(a_i*|b_k|*|eci_k|')*/
    a_i = (1.0f/b_k->cols);
    for (k=1;k<=b_k->cols;k++){
        norm_obs_in = sqrtf(mtx_get(1,k,b_k)*mtx_get(1,k,b_k)
                            + mtx_get(2,k,b_k)*mtx_get(2,k,b_k)
                            + mtx_get(3,k,b_k)*mtx_get(3,k,b_k)); 
        norm_eci_in = sqrtf(mtx_get(1,k,eci_k)*mtx_get(1,k,eci_k)
                            + mtx_get(2,k,eci_k)*mtx_get(2,k,eci_k)
                            + mtx_get(3,k,eci_k)*mtx_get(3,k,eci_k)); 
        mtx_set(1,1,&norm_obs,   mtx_get(1,k,b_k)/norm_obs_in); 
        mtx_set(2,1,&norm_obs,   mtx_get(2,k,b_k)/norm_obs_in); 
        mtx_set(3,1,&norm_obs,   mtx_get(3,k,b_k)/norm_obs_in); 
        mtx_set(1,1,&norm_eci,   mtx_get(1,k,eci_k)/norm_eci_in); 
        mtx_set(2,1,&norm_eci,   mtx_get(2,k,eci_k)/norm_eci_in); 
        mtx_set(3,1,&norm_eci,   mtx_get(3,k,eci_k)/norm_eci_in); 
        mtx_trans(&norm_eci,&tmp_13a);
        mtx_mult(&norm_obs,&tmp_13a,&tmp_33a);
        mtx_scale(&tmp_33a,a_i,&tmp_33b);
        mtx_sum(&b,&tmp_33b,&tmp_33a);
        mtx_copy(&tmp_33a,&b);
    }

    /*sigma, S, Z*/
    sigma = mtx_trace(&b);
    mtx_trans(&b,&tmp_33a);
    mtx_sum(&b,&tmp_33a,&s);
    mtx_set(1,1,&z, mtx_get(2,3,&b)-mtx_get(3,2,&b));
    mtx_set(2,1,&z, mtx_get(3,1,&b)-mtx_get(1,3,&b));
    mtx_set(3,1,&z, mtx_get(1,2,&b)-mtx_get(2,1,&b));

    /*M_inv = (eig_val + sigma)*I - S*/
    mtx_scale(&eye_33, eig_val + sigma, &tmp_33a);
    mtx_scale(&s, -1, &tmp_33b);
    mtx_sum(&tmp_33a, &tmp_33b, &m_inv);

    /*Check if matrix is non-invertable (rot=180)*/
    if (mtx_det(&m_inv)<MIN_MINV_DET) return 0;

    /* p = inv(M_inv)*Z*
     * q = 1/sqrt(1+p'*p)*[p;1] */
    mtx_inv(&m_inv,&tmp_33a);
    mtx_mult(&tmp_33a,&z,&p);
    mtx_trans(&p,&tmp_13a);
    mtx_mult(&tmp_13a,&p,&q_scale_sub);
    q_scale = 1.0f/sqrtf(1.0f+mtx_get(1,1,&q_scale_sub));
//     mtx_set(1,1,&q_out, q_scale*mtx_get(1,1,&p));
//     mtx_set(2,1,&q_out, q_scale*mtx_get(2,1,&p));
//     mtx_set(3,1,&q_out, q_scale*mtx_get(3,1,&p));
//     mtx_set(4,1,&q_out, q_scale); // SCALAR
    // CHANGE TO SCALAR FIRST
    mtx_set(1,1,&q_out, q_scale); // SCALAR
    mtx_set(2,1,&q_out, q_scale*mtx_get(1,1,&p));
    mtx_set(3,1,&q_out, q_scale*mtx_get(2,1,&p));
    mtx_set(4,1,&q_out, q_scale*mtx_get(3,1,&p));
    

    q_2_dcm(&q_out, dcm_out);
    dcm_2_q(dcm_out,&q_out);
    mtx_copy(&q_out,&state->att_quaternion);

    return 1;  
}

/*
 * Function: rot_obs_vec
 * ------------------------
 *  Rotate observation vectors by theta (degrees)
 *
 *  result: mtx_matrix of rotated 3 vectors
 */
void rot_obs_vec(struct mtx_matrix* b_k, float theta, int32_t axis,
                    struct mtx_matrix* b_out){
    struct mtx_matrix r;
    float theta_rad = (float)M_PI*theta/180.0f;
    mtx_create_eye(3,3,&r); 
    
    /*Create DCM for rotation about x,y,z axis according to input*/
    switch(axis){
        case 1 : 
            mtx_set(2,2,&r,   cosf(theta_rad));
            mtx_set(2,3,&r,  -sinf(theta_rad));
            mtx_set(3,2,&r,   sinf(theta_rad));
            mtx_set(3,3,&r,   cosf(theta_rad));
            break;
        case 2 : 
            mtx_set(1,1,&r,   cosf(theta_rad));
            mtx_set(1,3,&r,   sinf(theta_rad));
            mtx_set(3,1,&r,  -sinf(theta_rad));
            mtx_set(3,3,&r,   cosf(theta_rad));
            break;
        case 3 : 
            mtx_set(1,1,&r,   cosf(theta_rad));
            mtx_set(1,2,&r,  -sinf(theta_rad));
            mtx_set(2,1,&r,   sinf(theta_rad));
            mtx_set(2,2,&r,   cosf(theta_rad));
            break;
    }
    /*Multiply input vectors by R*/
    mtx_mult(&r,b_k,b_out); 
    return;
}

/*
 * Function: est_quest
 * ------------------------
 *  A wrapper around the est_quest_rp function to handle the case 
 *  when the rotation from ECI->Body Frame is at 180 deg.
 *
 *  result: mtx_matrix of eci->body frame rotation
 */
int32_t est_quest(struct mtx_matrix* b_k, struct mtx_matrix* eci_k, 
                    struct est_state* state, struct mtx_matrix* dcm_out){
    int32_t k;
    struct mtx_matrix obs_vec;
    mtx_create_ones(3,2,&obs_vec);

    /*Try rotation around each axis*/
    mtx_copy(b_k,&obs_vec);
    for (k=0;k<=3;k++){ 
        if(est_quest_rp(&obs_vec,eci_k,state,dcm_out)==0){
            return 0; // ATT Error 2
        } else {
            rot_obs_vec(b_k,ROT_ANGLE,k+1,&obs_vec);
        }
    }
    return 1;
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
                    struct est_state* state, struct mtx_matrix* dcm_out,
                    float ss_v_cutoff){

    static int32_t init_dcm = 0;
    static struct mtx_matrix prior_dcm;

    int32_t rv;
    struct mtx_matrix sun_vec;
    struct mtx_matrix b_k;
    struct mtx_matrix eci_k;
    struct mtx_matrix post_dcm;
    mtx_create_ones(3,1,&sun_vec);
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

    /*If three sensors are active, add sun sensor to observation matrix
     *otherwise add integrated gyro X vector*/
    if(est_sun_vec_ls(sun_sens_volt, sun_sens_norm, &sun_vec, ss_v_cutoff)==1){
        mtx_set(1,2,&b_k,   mtx_get(1,1,&sun_vec));    
        mtx_set(2,2,&b_k,   mtx_get(2,1,&sun_vec));    
        mtx_set(3,2,&b_k,   mtx_get(3,1,&sun_vec));    
        mtx_set(1,2,&eci_k, mtx_get(1,1,sun_eci));
        mtx_set(2,2,&eci_k, mtx_get(2,1,sun_eci));
        mtx_set(3,2,&eci_k, mtx_get(3,1,sun_eci));
    } else {
        //q_2_dcm(&state->att_quaternion,&prior_dcm);
        body_rate_dcm_rot(&state->sat_body_rates,&prior_dcm,&post_dcm);
        mtx_set(1,2,&eci_k,   1.0f);    
        mtx_set(2,2,&eci_k,   0.0f);    
        mtx_set(3,2,&eci_k,   0.0f);    
        mtx_set(1,2,&b_k, mtx_get(1,1,&post_dcm));
        mtx_set(2,2,&b_k, mtx_get(2,1,&post_dcm));
        mtx_set(3,2,&b_k, mtx_get(3,1,&post_dcm));
    }
    rv = est_quest(&b_k,&eci_k,state,dcm_out);
    mtx_copy(dcm_out,&prior_dcm);

    return rv;
}

/*
 * Function: est_ekf
 * ------------------------
 *  EKF for sun vector from sun sensor measurements and rate gyro
 */
static float array_X0ref[3] = {
		-1.0f,
		0.0f,
		0.0f
};
int32_t est_ekf(struct mtx_matrix* ss_volt, struct mtx_matrix* ss_norm,
                    struct mtx_matrix* mag_sens,
                    struct mtx_matrix* sun_eci, struct mtx_matrix* mag_eci,
                    struct est_state* state, struct mtx_matrix* dcm_out,
                    float ss_v_cutoff){

    int32_t rv = 1;
    uint32_t i = 0;
    static uint32_t att_iter = 0;
    static struct mtx_matrix BG, gyro_bias, phi, philast, phidot, sun_vec_body, xbar, Pbar,
    				Q, gamma, SNC_term, ss_comp, Htilde, y, e, K, R, xhat, Xhat, P, temp, temp2, temp3;

    if(att_iter==0){
    	mtx_create_eye(3,3,&BG);
    	mtx_create_val(3,1, &gyro_bias, 0.0f);
    	mtx_scale(&gyro_bias,-1.0f,&gyro_bias);
    	mtx_create_eye(3,3,&phi);
		mtx_create_eye(3,3,&philast);
		mtx_create_val(3,3,&phidot, 0.0f);
    	mtx_create(3,1, array_X0ref, &sun_vec_body);
    	mtx_create_val(3,1, &xbar, 0.0f);
    	mtx_create_eye(3,3,&Pbar);
    	mtx_scale(&Pbar,1e3,&Pbar);
    	mtx_create_eye(3,3,&Q);
    	mtx_scale(&Q,SIGMA_SNC*SIGMA_SNC,&Q);
    	mtx_create_eye(3,3,&gamma);
    	mtx_scale(&gamma,(TSTEP*TSTEP)/(2.0f),&gamma);
    	mtx_mult(&gamma,&Q,&Q);
//    	mtx_trans(&gamma,&gamma);
    	mtx_create_val(3,3, &SNC_term, 0.0f);
    	mtx_mult(&Q,&gamma,&SNC_term);
    	mtx_create_val(13,1, &ss_comp, 0.0f);
    	mtx_create(13,3, ss_norm->data,&Htilde); // Doesn't change
    	mtx_scale(&Htilde, CSS_SCALE_FACTOR, &Htilde);
    	mtx_create_val(13,1, &y, 0.0f);
    	mtx_create_val(13,1, &e, 0.0f);
    	mtx_create_val(3,13, &K, 0.0f);
    	mtx_create_eye(13,13, &R);
    	mtx_create_val(3,1, &xhat, 0.0f);
    	mtx_create_val(3,1, &Xhat, 0.0f);
    	mtx_create(3,3, Pbar.data,&P);


    } else {
    	// Compute phi and phidot
    	mtx_sum(&state->sat_body_rates,&gyro_bias,&state->sat_body_rates);
    	mtx_scale(&state->sat_body_rates,-1.0f,&state->sat_body_rates);
    	mtx_create_val(3,1, &temp, 0.0f);
    	mtx_mult(&BG,&state->sat_body_rates,&temp); //temp = - gyro rates in body frame
    	mtx_ss(&temp, &phidot);
		mtx_create_val(3,3, &temp, 0.0f);
    	mtx_scale(&phidot, TSTEP ,&temp); // temp = phidot*TSTEP = -w_cross*dt
		mtx_sum(&philast,&temp,&phi);
     	mtx_print(&phi);

        while(1);

    	// Compute Xref
    	mtx_create_val(3,1, &temp2, 0.0f);
    	mtx_mult(&temp,&sun_vec_body,&temp2); // temp2 = phodot*TSTEP*sun_vec_body = -wdtXsun = sunXwdt = Xdot*dt
    	mtx_sum(&sun_vec_body, &temp2, &sun_vec_body);
    	mtx_scale(&sun_vec_body, 1.0f/(mtx_norm(&sun_vec_body)),&sun_vec_body);
//    	mtx_print(&sun_vec_body);

    	// Time update
    	mtx_mult(&phi,&xhat,&xbar);
//    	mtx_print(&xbar);

    	mtx_mult(&phi,&P,&temp); // temp = phiP
    	mtx_create_val(3,3, &temp2, 0.0f);
    	mtx_trans(&phi,&temp2);// temp2 = phi^T
    	mtx_mult(&temp,&temp2,&Pbar);
    	if(SNC_FLAG){
    		mtx_sum(&Pbar,&SNC_term,&Pbar);
    	}
//    	mtx_print(&Pbar);
    }

	// Compute CSS
	mtx_mult(ss_norm,&sun_vec_body,&ss_comp);
	mtx_scale(&ss_comp,CSS_SCALE_FACTOR,&ss_comp);
	// Compute Htilde
	mtx_create_val(13,1, &temp2, 0.0f);
	mtx_scale(&ss_comp,1.0f/CSS_SCALE_FACTOR,&temp2);// temp2 = ss_comp/SF = ss_norm^Tsun
	mtx_create(13,3, ss_norm->data,&Htilde);
	mtx_scale(&Htilde, CSS_SCALE_FACTOR, &Htilde);
	for(i=1; i<=13; i++){
		if(mtx_get(i,1,&temp2)<(cosf(CSS_HALF_ANGLE))){
			mtx_set(i,1,&ss_comp,0.0f);
			mtx_set(i,1,&Htilde,0.0f);
			mtx_set(i,2,&Htilde,0.0f);
			mtx_set(i,3,&Htilde,0.0f);
		}
	}
//	mtx_print(&ss_comp);
//	mtx_print(&Htilde);
	// Compute prefit
	mtx_scale(&ss_comp, -1.0f, &ss_comp);
	mtx_sum(ss_volt, &ss_comp, &y);
//	mtx_print(&y);
	//Compute K
	mtx_create_val(3,13, &temp, 0.0f);
	//mtx_create(13,3, Htilde.data, &temp2);
	mtx_trans(&Htilde,&temp);//temp = Htilde^T
	mtx_mult(&Htilde,&Pbar,&temp2);// temp2 = HP
	mtx_create_val(13,13, &temp3, 0.0f);
	mtx_mult(&temp2,&temp,&temp3);// temp3 = HPH^T
	mtx_sum(&temp3, &R, &temp3);// temp3 = HPH^T + R
	mtx_inv(&temp3,&temp3);// temp3 = (HPH^T + R)^-1
	mtx_mult(&Pbar,&temp,&K);
	mtx_mult(&K,&temp3,&K);
//	mtx_print(&K);
	// Measurement Update
	// xhat
	mtx_create_val(13,1, &temp, 0.0f);
	mtx_mult(&Htilde,&xbar,&temp);// temp = Hx
	mtx_scale(&temp, -1.0f, &temp);//temp = -Hxbar
	mtx_sum(&y,&temp,&temp);// temp = y - Hxbar
	mtx_create_val(3,1, &temp3, 0.0f);
	mtx_mult(&K,&temp,&temp3);// temp3 = K(y - Hx)
	mtx_sum(&xbar,&temp3,&xhat);
//	mtx_print(&xhat);
	//Xhat
	mtx_sum(&sun_vec_body,&xhat,&Xhat);
	mtx_scale(&Xhat,1/mtx_norm(&Xhat),&Xhat);
//	mtx_print(&Xhat);
	//P
	mtx_create_val(3,13, &temp, 0.0f);
	mtx_mult(&K,&R,&temp);// temp = KR
	mtx_create_val(13,3, &temp2, 0.0f);
	mtx_trans(&K, &temp2);// temp2 = K^T
	mtx_create_val(3,3, &temp3, 0.0f);
	mtx_mult(&temp, &temp2, &temp3); // temp3 = KRK^T

	mtx_create_eye(3,3, &temp);//temp = I
	mtx_create_val(3,3, &temp2, 0.0f);
	mtx_mult(&K, &Htilde, &temp2);// temp2 = KH
	mtx_scale(&temp2, -1.0f, &temp2);// temp2 = -KH
	mtx_sum(&temp, &temp2, &temp);// temp = (I - KH)
	mtx_trans(&temp, &temp2);// temp2 = (I - KH)^T
	mtx_mult(&temp, &Pbar, &temp); // temp = (I - KH)P
	mtx_mult(&temp, &temp2, &temp);// temp = (I - KH)P(I - KH)^T
	mtx_sum(&temp, &temp3, &P);
//	mtx_print(&P);
	//e
	mtx_create_val(13,1, &temp, 0.0f);
	mtx_mult(&Htilde, &xhat, &temp); //temp = Hxhat

	mtx_scale(&temp, -1.0f, &temp);//temp = -Hxhat
	mtx_sum(&y, &temp, &e);
//	mtx_print(&e);
	mtx_create_eye(3,3,&philast);
	if(att_iter>=EKF_ON_AFTER){
		mtx_copy(&Xhat, &sun_vec_body);
		mtx_create_val(3,1, &xhat, 0.0f);

		mtx_copy(&Xhat,sun_eci);
	} else {
		mtx_copy(&Xhat,sun_eci);
	}

    att_iter++;
    return rv;

}

/*
 * Function: create_att_state
 * ------------------------
 * Initialize estimator state struct 
 *
 */

void create_att_state(struct est_state* state){
    mtx_create_val(3,3,&state->att_err,0);
    mtx_create_val(3,1,&state->sat_body_rates,0);
    mtx_create_val(3,1,&state->gyro_bias,0);
    mtx_create_val(4,1,&state->att_quaternion,0);
    mtx_set(1,1,&state->att_quaternion,1);
}

/*
 * Function: create_sun_q 
 * ------------------------
 * Create sun pointing desired quaternion
 * Inputs: sun_vec is sun vector in body frame
 * Output: des_BR_q is quaternion that represents body relative to reference
 *
 */

void create_sun_des_q(uint8_t op_mode, float *sun_vec, float *des_RB_q){

    struct mtx_matrix des_RN_dcm, des_RN_q_vecmtx;

    float sun_vec_normlzd[3] = {0.0,0.0,0.0};
    float sun_vec_norm = 0.0f;
    uint32_t i=0;

    mtx_create_ones(3,3,&des_RN_dcm);
    mtx_create_ones(4,1,&des_RN_q_vecmtx);

    /* Normalize Sun Vec (in whatever frame) */
    sun_vec_norm = norm(sun_vec);
    if(sun_vec_norm!=0.0f){
        for(i=0; i<3; i++)
            sun_vec_normlzd[i] = sun_vec[i]/sun_vec_norm;
    } else { // If no info on vec, point it to -X (in whatever frame)
        sun_vec_normlzd[0] = -1.0f;
        sun_vec_normlzd[1] = 0.0f;
        sun_vec_normlzd[2] = 0.0f;
    }

    /* Mount desired attitude:
     * Computes angle between sun vec and desired vector
     */
    float des_bf_vec[3] = {-1.0, 0.0, 0.0};
    q_from_2uvecs(des_bf_vec, sun_vec_normlzd, des_RB_q);
}

/*
 * Function: create_ram_frame
 * ------------------------
 * Create ram pointing desired quaternion
 *
 */

void create_ram_des_Rframe(uint8_t op_mode, struct mtx_matrix* q_BN_vecmtx,
						float *vel_vec, float *sun_vec, float *pos_vec, float *des_RN_q, float *des_rates){

    struct mtx_matrix dcm_RN;
    struct mtx_matrix q_RN_vecmtx;

    float vel_vec_normlzd[3] = {0.0, 0.0, 0.0};
	float vel_vec_norm = 0.0f;
    float pos_vec_normlzd[3] = {0.0, 0.0, 0.0};
	float pos_vec_norm = 0.0f;
	float primary[3] = {0.0, 0.0, 0.0};
	float secondary[3] = {0.0, 0.0, 0.0};
	float secondary_norm = 0.0f;
	float tertiary[3] = {0.0, 0.0, 0.0};
	float tertiary_norm = 0.0f;
	float des_RN_q_norm = 0.0f;
	float qr_dt[4] = {0.0f,0.0f,0.0f,0.0f};
	static float wr_init;
	static float qr_last[4];
	float angle;
	float sin_halfang;
	uint32_t i=0;

    // Create body frame vectors
	float primary_BF[3] = {0.0,0.0,-1.0};
	float secondary_BF[3] = {-1.0,0.0,0.0};
	float tertiary_BF[3];
	crossp(primary_BF,secondary_BF,tertiary_BF);

    mtx_create_ones(4,1, &q_RN_vecmtx);
	mtx_create_eye(3,3, &dcm_RN);

    /* Normalize Sun Vec (in ECI) */
    /*sun_vec_norm = sqrtf(sun_vec[0]*sun_vec[0]+sun_vec[1]*sun_vec[1]+sun_vec[2]*sun_vec[2]);
    if(sun_vec_norm!=0.0f){
        for(i=0; i<3; i++)
            sun_vec_normlzd[i] = sun_vec[i]/sun_vec_norm;
    } else { // If no info on vec, point it to -X (in ECI)
        sun_vec_normlzd[0] = -1.0f;
        sun_vec_normlzd[1] = 0.0f;
        sun_vec_normlzd[2] = 0.0f;
    }*/

    /* Normalize Vel Vec (in ECI) */
    vel_vec_norm = sqrtf(vel_vec[0]*vel_vec[0]+vel_vec[1]*vel_vec[1]+vel_vec[2]*vel_vec[2]);
    if(vel_vec_norm!=0.0f){
        for(i=0; i<3; i++)
            vel_vec_normlzd[i] = vel_vec[i]/vel_vec_norm;
    } else { // If no info on vec, point it to -Z (in ECI)
        vel_vec_normlzd[0] = 0.0f;
        vel_vec_normlzd[1] = 0.0f;
        vel_vec_normlzd[2] = -1.0f;
    }

	// Normalize Pos Vec (in ECI)
	pos_vec_norm = sqrtf(pos_vec[0]*pos_vec[0]+pos_vec[1]*pos_vec[1]+pos_vec[2]*pos_vec[2]);
    if(pos_vec_norm!=0.0f){
        for(i=0; i<3; i++)
            pos_vec_normlzd[i] = pos_vec[i]/pos_vec_norm;
    } else { // If no info on vec, point it to +Z (in ECI)
        pos_vec_normlzd[0] = -1.0f;
        pos_vec_normlzd[1] = 0.0f;
        pos_vec_normlzd[2] = 0.0f;
    }

	for(i=0; i<3; i++){
		primary[i] = vel_vec_normlzd[i];
		secondary[i] = pos_vec_normlzd[i];
	}
	crossp(primary,secondary,tertiary);
	tertiary_norm = sqrtf(tertiary[0]*tertiary[0]+tertiary[1]*tertiary[1]+tertiary[2]*tertiary[2]);
	if(tertiary_norm !=0.0f){
		for(i=0; i<3; i++)
			tertiary[i] = tertiary[i]/tertiary_norm;
	} else {
		tertiary[0] = 0.0f;
		tertiary[1] = 1.0f;
		tertiary[2] = 0.0f;
	}
	crossp(tertiary,primary,secondary);
	secondary_norm = sqrtf(secondary[0]*secondary[0]+secondary[1]*secondary[1]+secondary[2]*secondary[2]);
	if(secondary_norm !=0.0f){
		for(i=0; i<3; i++)
			secondary[i] = secondary[i]/secondary_norm;
	} else {
		secondary[0] = -1.0f;
		secondary[1] = 0.0f;
		secondary[2] = 0.0f;
	}

    /* Compute reference frame from primary vector and secondary vector*/
	for(i=0; i<3; i++){
		if(primary_BF[i] == 1.0 || primary_BF[i] == -1.0){
			mtx_set(i+1,1,&dcm_RN,primary_BF[i]*primary[0]);
			mtx_set(i+1,2,&dcm_RN,primary_BF[i]*primary[1]);
			mtx_set(i+1,3,&dcm_RN,primary_BF[i]*primary[2]);
		}else if(secondary_BF[i] == 1.0 || secondary_BF[i] == -1.0){
			mtx_set(i+1,1,&dcm_RN,secondary_BF[i]*secondary[0]);
			mtx_set(i+1,2,&dcm_RN,secondary_BF[i]*secondary[1]);
			mtx_set(i+1,3,&dcm_RN,secondary_BF[i]*secondary[2]);
		}else{
			mtx_set(i+1,1,&dcm_RN,tertiary_BF[i]*tertiary[0]);
			mtx_set(i+1,2,&dcm_RN,tertiary_BF[i]*tertiary[1]);
			mtx_set(i+1,3,&dcm_RN,tertiary_BF[i]*tertiary[2]);
		}
	}
	dcm_2_q(&dcm_RN, &q_RN_vecmtx);

	for(i=0;i<=3;i++){
		des_RN_q[i] = mtx_get(i+1,1,&q_RN_vecmtx);//des_BR_q[i];//
	}
	des_RN_q_norm = sqrtf( des_RN_q[0]*des_RN_q[0] +
                des_RN_q[1]*des_RN_q[1] + des_RN_q[2]*des_RN_q[2] +
                des_RN_q[3]*des_RN_q[3]);

    des_RN_q[0] /= des_RN_q_norm;
    des_RN_q[1] /= des_RN_q_norm;
    des_RN_q[2] /= des_RN_q_norm;
    des_RN_q[3] /= des_RN_q_norm;
	if(des_RN_q[0]<0.0f){
		des_RN_q[0] *= -1.0f;
		des_RN_q[1] *= -1.0f;
		des_RN_q[2] *= -1.0f;
		des_RN_q[3] *= -1.0f;
	}
	for(i = 0; i<3; i++){
		des_rates[i] = 0.0f;
	}

	if(wr_init==0){
		for(i=0;i<=3;i++){
			qr_last[i] = des_RN_q[i];
		}
		wr_init = 1.0f;
	} else{
	// Calcuate difference between current and last q ref
	calc_q_err(des_RN_q,qr_last,qr_dt);
	// Find angle and axis of rotation
	if(qr_dt[0]<0.7071f){
		angle = 2.0f*acosf(qr_dt[0]);
		sin_halfang = sin(angle/2.0f);
	}else{
		sin_halfang = sqrt(qr_dt[1]*qr_dt[1]+qr_dt[2]*qr_dt[2]+qr_dt[3]*qr_dt[3]);
		angle = 2.0f*asinf(sin_halfang);
	}
	// Compute desired rates
	for(i=0;i<3;i++){
		des_rates[i] = qr_dt[i+1]/sin_halfang*angle*UPDATE_RATE;//same as /dt
	}
	// save current quaternion
	for(i=0;i<=3;i++){
		qr_last[i] = des_RN_q[i];
	}
	}
}

void smooth_mag(float *mag_in, float *mag_out){
    static float mag_x[AVG_WIN];
    static float mag_y[AVG_WIN];
    static float mag_z[AVG_WIN];
    static int32_t init_cnt=0;
    int32_t i=0;

    for(i=0;i<AVG_WIN-1;i++){
        mag_x[i] = mag_x[i+1];
        mag_y[i] = mag_y[i+1];
        mag_z[i] = mag_z[i+1];
    }

    mag_x[AVG_WIN-1] = mag_in[0];
    mag_y[AVG_WIN-1] = mag_in[1];
    mag_z[AVG_WIN-1] = mag_in[2];

    //Skip filter if still initializing
    if(init_cnt<AVG_WIN){
        mag_out[0] = mag_in[0];
        mag_out[1] = mag_in[1];
        mag_out[2] = mag_in[2];
        init_cnt++;
    } else {
        mag_out[0] = 0.0f;
        mag_out[1] = 0.0f;
        mag_out[2] = 0.0f;

        for(i=0;i<AVG_WIN;i++){
            mag_out[0] += mag_x[i];
            mag_out[1] += mag_y[i];
            mag_out[2] += mag_z[i];
        }
        mag_out[0] /= (float) AVG_WIN;
        mag_out[1] /= (float) AVG_WIN;
        mag_out[2] /= (float) AVG_WIN;
    }
}
