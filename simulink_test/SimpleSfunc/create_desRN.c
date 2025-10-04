#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "create_desRN.h"
#include "mtx.h"
#include "mex.h"
#include "conversions.h"
#include "global.h"

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