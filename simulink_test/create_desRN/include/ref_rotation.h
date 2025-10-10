/* File:   ecef2eci.h
 * Author: Nicola Baresi and Jesse Keefer
 * Created On: 10/11/2015
*/
#ifndef __REF_ROTATION_H
#define __REF_ROTATION_H

// Include Statements
#include <stdint.h>

 /* Externally available functions*/
int32_t ECEF2ECI(double t, double t_frac, float* x_ECEF, float* vx_ECEF, 
	float* x_ECI, float* vx_ECI, uint8_t leapsec_since_epoch);
int32_t S_ECEF2ECI(float t, float t_frac, float* x_ECEF,
    float* x_ECI, uint8_t leapsec_since_epoch); //Pranith//single parameter transformation
int32_t ECI2ECEF(double t, double t_frac, float* x_ECI, float* vx_ECI, 
	float* x_ECEF, float* vx_ECEF, uint8_t leapsec_since_epoch);

#endif
