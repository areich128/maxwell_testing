/* ***********************************************************************
 * file:    att_det.h
 * author:	? (No header edit by Andre L Antunes de Sa)
 * created on:	?
 *
 * att dependencies?
 *
 * ***********************************************************************
 */

#ifndef __ATT_DET_H_
#define __ATT_DET_H_

#include "simulinkCustom.h"
#include <stdint.h>
#include "mtx.h"

#define SS_MAX 3.0f
#define SS_V_CUTOFF	.259f * SS_MAX //cos(75)*SS_MAX
#define UPDATE_RATE	10 //Hz
#define THETA_RP .25f
#define MAX_ROTS 3
#define ROT_ANGLE .5f
#define MIN_MINV_DET .00001f
#define AVG_WIN 8 /*Must be Even*/


struct est_state{
	struct mtx_matrix att_quaternion; //4x1
	struct mtx_matrix sat_body_rates; //3x1
}; 

struct ekf_mat{
	struct mtx_matrix P;
	struct mtx_matrix Q;
	struct mtx_matrix R;
};

void q_2_dcm(struct mtx_matrix* q, struct mtx_matrix* dcm);
void dcm_2_q(struct mtx_matrix* dcm, struct mtx_matrix* q);
void calc_q_err(float *q_bn, float *q_rn, float *q_br);
int32_t est_sun_vec_ls(struct mtx_matrix* sun_sens_volt, struct mtx_matrix* sun_sens_norm, struct mtx_matrix* sun_vec, float ss_v_cutoff);
int32_t est_quest_rp(struct mtx_matrix* b_k, struct mtx_matrix* eci_k, struct est_state* state, struct mtx_matrix* dcm_out);
int32_t est_simp(struct mtx_matrix* sun_sens_volt, struct mtx_matrix* sun_sens_norm, struct mtx_matrix* mag_sens, struct mtx_matrix* sun_eci, struct mtx_matrix* mag_eci, struct mtx_matrix* sun_vec, struct est_state* state, struct mtx_matrix* dcm_out, float ss_v_cutoff);
void create_sun_des_q(uint8_t op_mode, float *sun_vec, float *des_RN_q, float *q_BR, struct mtx_matrix *q_BN_mtx, uint8_t flag);
void create_phicone_des_q(uint8_t op_mode, float *pos_vec, float *sun_vec, float *des_RN_q, float phi_cone_angle, float *phi_vec_normlzd);
void sc_to_gs_vec(float *sc_pos_vec_ecef, float *gnd_stn_pos_vec_ecef, float *sc_2_gs);
void gs_to_sc_vec(float *sc_pos_vec_ecef, float *gnd_stn_pos_vec_ecef, float *gs_2_sc);
void gs_angle_distance(float *sc_pos_vec_ecef, float *gnd_stn_pos_vec_ecef, float *angle, float *distance);
void sc_to_gs_unitvec(float *sc_pos_vec_ecef, float *gnd_stn_pos_vec_ecef, float *sc_2_gs_normlzd);
void create_gnd_stn_des_q(uint8_t op_mode, struct mtx_matrix* q_BN_vecmtx, float *sc_pos_vec_ecef, float *gnd_stn_pos_vec_ecef, float *des_RN_q, uint8_t antenna, uint8_t leapsec);
void create_desRN(uint8_t opmode, float *des_vec, float *des_RN);
void create_desRN_DCOMM(uint8_t opmode, float *sc_gs, float *des_RN);
void create_des_rates_DCOMM(uint8_t opmode, float *pos_ecef, float *gnd_ecef, 
float *des_rates_bf, float *q_BN, uint8_t leapsec);
void create_des_rates_CSAC(uint8_t opmode, float *phi_cone,
float *des_rates_bf, float *q_BN);
void calculateOmega(float *prev_vec, float *current_vec, float *des_rates_bf, float *q_BN);
void q_from_2uvecs(float *v1, float *v2, float *q_v1v2);
float p_derivative(float A, float B, float lamb);
float a_initial(float A, float B, float D, float lamb);
uint8_t att_det_reck(struct mtx_matrix *prior_dcm, struct mtx_matrix *mtx_gyro_rates, struct mtx_matrix *quat_out);
void state_ekf(struct mtx_matrix* sun_sens_volt, struct mtx_matrix* sun_sens_norm,  float ss_v_cutoff,
    struct mtx_matrix* mag_sens, struct mtx_matrix* mag_eci, struct mtx_matrix* sun_eci,struct mtx_matrix* sun_vec,
    struct est_state* state, struct est_state *state_post, struct est_state *mes, struct ekf_mat *info,float *ctrl_t,float kd_term);

#endif
