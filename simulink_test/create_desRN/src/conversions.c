/* file: conversions.c
 * author: Jacob Cook
 * created on: 6/13/2014
 */

/* NOTE (Anthony Zara 10/19/22): 
 * Most of these functions are commented out because we do not need them,
 * and they take up crucial binary space in the event we need to upload the
 * FSW to the spacecraft on-orbit.
 */

#define _USE_MATH_DEFINES

#include <math.h>
#include "conversions.h"

#include "simulinkCustom.h"
// TODO 999 What do we need here?
//#define CIRC_ORBIT (1e-3)

/* -- quat2euler123 -------------------------------------------------------
 *	Roll, pitch, yaw
 * ---------------------------------------------------------------------*/
void quat2euler321(float *q, float *e){

    e[0] = atan2f(2*(q[1]*q[2]+q[0]*q[3]),q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3]);
    e[1] = asinf(-2*(q[1]*q[3]-q[0]*q[2]));
    e[2]= atan2f(2*(q[2]*q[3]+q[0]*q[1]),q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]);
}

/* -- norm ---------------------------------------------------------------
 *
 * ---------------------------------------------------------------------*/
float norm(float *v)
{
	return sqrtf( v[0]*v[0] + v[1]*v[1] + v[2]*v[2] );
}

/* -- norm quaternion ----------------------------------------------------
 *
 * ---------------------------------------------------------------------*/
float normq(float *v)
{
	return sqrtf( v[0]*v[0] + v[1]*v[1] + v[2]*v[2] + v[3]*v[3]);
}

/* -- crossp -------------------------------------------------------------
 *
 * --------------------------------------------------------------------*/

void crossp(float *u, float *v, float *x)
{
	x[0] = u[1]*v[2] - u[2]*v[1];
	x[1] = u[2]*v[0] - u[0]*v[2];
	x[2] = u[0]*v[1] - u[1]*v[0];
}

/* -- dot ----------------------------------------------------------------
*
* ---------------------------------------------------------------------*/
float dot(float *x, float *y)
{
	return ( x[0]*y[0] + x[1]*y[1] + x[2]*y[2] );
}

/* -- julian_date --------------------------------------------------------
 * this function returns the Julian date based on the the current time.
 * This function is dependant on J0 definition in the magSat_Config.h
 * file.
 * ---------------------------------------------------------------------*/
float julian_date(float t)
{
	return (float)J0 + t/86400.0f;
}

/* -- universal_time -----------------------------------------------------
 * This function calculates the Universal time from the Julian Date.
 * ---------------------------------------------------------------------*/
float universal_time(float JD)
{
	return (JD - 2451545.0f)/36525.0f;
}

/* -- gmst --------------------------------------------------------------
 *
 * function prototype:
 * 	th_gmst = gmst(t)
 *		t = time in seconds past Epoch (J0)
 * --------------------------------------------------------------------*/
#define GMST_C0 (67310.54841)
#define GMST_C1 (876600.0*3600.0 + 8640184.812866)
#define GMST_C2 (0.093104)
#define GMST_C3 (6.2e-6)
#define GMST_SEC2RAD (7.2722e-5) /* M_PI/240.0/180.0 */

double gmst(double t)
{
	double JD, T_UT1, th_gmst, gmst_sec;

	/* compute current Julian Date */
	JD = julian_date((float)t);
	T_UT1 = universal_time((float)JD);

	/*PRINT("JD = %f\n\r", JD);*/
	/*PRINT("T_UT1 = %f\n\r",T_UT1);*/

	/* compute greenwich mean standard time (Vallado eq. 3-47) */
	gmst_sec = GMST_C0 + GMST_C1*T_UT1 +
						GMST_C2*pow(T_UT1,2) + GMST_C3*pow(T_UT1, 3);

	/*PRINT("gmst_sec = %f\n\r", gmst_sec);*/

	/* reduce down to within one revolution */
	gmst_sec = fmod(gmst_sec, 86400);
	/*PRINT("gmst_sec = %f\n\r", gmst_sec);*/

	/* convert to radians */
	th_gmst = GMST_SEC2RAD*gmst_sec;

	if(th_gmst < 0)
		th_gmst += 2*M_PI;

	/*PRINT("th_gmst = %f\n\r", th_gmst);*/
	return th_gmst;
}
