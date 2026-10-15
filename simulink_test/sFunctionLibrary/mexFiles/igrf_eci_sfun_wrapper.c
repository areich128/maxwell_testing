
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
#include <string.h>
#include "global.h"
#include "igrf_model.h"
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 3
#define u_1_width 3
#define u_2_width 1
#define u_3_width 1
#define u_4_width 1
#define y_width 3
#define y_1_width 1

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
extern void igrf_eci_sfun_Outputs_wrapper(const real32_T *eci_pos,
			const real32_T *eci_vel,
			const uint32_T *J2000_time,
			const real32_T *J2000_frac,
			const uint8_T *leapsec,
			real32_T *mag_eci,
			int32_T *status);

void igrf_eci_sfun_Outputs_wrapper(const real32_T *eci_pos,
			const real32_T *eci_vel,
			const uint32_T *J2000_time,
			const real32_T *J2000_frac,
			const uint8_T *leapsec,
			real32_T *mag_eci,
			int32_T *status)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
static struct param_table prm;      /* only Re and leapsec are read by igrf_eci */
struct pos_state pos;
struct adcs_mode mode;
int i;
memset(&pos, 0, sizeof(pos));
memset(&mode, 0, sizeof(mode));
prm.Re = 6378.1363f;                /* same as global.c default */
prm.leapsec = *leapsec;
mode.param_ptr = &prm;
for (i = 0; i < 3; i++) { pos.eci_vec[i] = eci_pos[i]; pos.vel_vec[i] = eci_vel[i]; }
pos.J2000_time = *J2000_time;
pos.J2000_frac_time = *J2000_frac;
*status = igrf_eci(&pos, &mode);
for (i = 0; i < 3; i++) mag_eci[i] = pos.mag_eci_vec[i];
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


