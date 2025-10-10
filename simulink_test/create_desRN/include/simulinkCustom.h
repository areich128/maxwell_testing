#include <stdint.h>

#define PRINT(fmt,arg...)


extern uint64_t g_err; ///< Global error flag

/* ADCS Controller Modes (Moved from ctl_alg.h) */
/******************************************************************************
@brief Mode definition for the drift controller
******************************************************************************/
#define CTRL_DRIFT		(0x0)

/******************************************************************************
@brief Mode definition for the sun pointing controller
******************************************************************************/
#define CTRL_SUN    	(0x1)

/******************************************************************************
@brief Mode definition for DCOMM mode
******************************************************************************/
#define DCOMM	    	(0x2)

/******************************************************************************
@brief Mode definition for CSAC Operation
******************************************************************************/
#define CSAC  		   	(0x3)

/******************************************************************************
@brief Mode definition for Reaction Wheel spool-up
******************************************************************************/
#define RW_SET    		(0x4)

/******************************************************************************
@brief Mode definition for B-dot and RW Momentum Dumping
******************************************************************************/
#define BDOT  		   	(0x5)


/* CDH Operational Modes - 10.19.22 - Zara */
#define CDH_CX          (0b00 << MODE_CDH_SHIFT) // Commissioning
#define CDH_SAFE        (0b01 << MODE_CDH_SHIFT) // Safe Mode
#define CDH_NOM         (0b10 << MODE_CDH_SHIFT) // Nominal Mode
#define CDH_SCI         (0b11 << MODE_CDH_SHIFT) // Science Mode

/* Ground Station Flags - 10.19.22 - Zara */
#define GS1_EN		(1 << 0) // Ground Station 1 Enable
#define GS2_EN		(1 << 1) // Ground Station 2 Enable
#define GS3_EN		(1 << 2) // Ground Station 3 Enable
#define GS4_EN		(1 << 3) // Ground Station 4 Enable

/* Antenna to use in pointing flags - 10.19.22 - Zara */
#define TX_SIDE		(1 << 4) // ANDs with bit 4 in mode->tx_state
							 // tx_state&TX_SIDE = 0 for X-band antenna on -X s/c side
							 // tx_state&TX_SIDE = 1 for T_DAGHR antenna on +X s/c side

/* EPS Power State - 11.2.22 - Zara */
#define EPS_PWR_GOOD 0x00 	// Case for RECIEVE_EPS_DATA
#define EPS_PWR_LOW 0XFF	// Case for RECIEVE_EPS_DATA
#define MODE_CDH_LOW_PWR (1 << 0) // Where EPS power status is stored in mode.cdh_mode

/* Reaction Wheel Saturation Limits - 11.15.22 - Zara*/
#define RW_DESAT_THRESH 	    58000 // Wheelspeed Threshold [deci-RPM] to send CDH a message requesting RW desat.
#define RW_DESAT_THRESH_URGENT 	62000 // Wheelspeed threshold [deci-RPM] to send CDH an URGENT message for RW desat when multiple wheels faster than this.
#define N_RWS_SAT_FOR_URGENT 	2	  // Number of wheels faster than RW_DESAT_THRESH_URGENT to send an urgent desat request

//6.6.21 - Mukta - added photodiode directions for additional board
#define CSS_TOTAL	5	//Total number of CSS boards
#define CSS_COUNT	20	//16 ///Number of Coarse Sun Sensors
// TODO: change after CSS test analysis complete - Alex
#define COARSE_SUN_SENS_NORM_VECS \
{   -0.866,-0.500,0.000,\
    -0.866,0.000,0.500,\
    -0.866,0.500,0.000,\
    -0.866,0.000,-0.500,\
    \
    0.000,0.500,0.866,\
    -0.500,0.000,0.866,\
    0.000,-0.500,0.866,\
    0.500,0.000,0.866,\
    \
    0.000,0.500,-0.866,\
    0.500,0.000,-0.866,\
    0.000,-0.500,-0.866,\
    -0.500,0.000,-0.866,\
    \
    -0.500,0.866,0.000,\
    0.000,0.866,0.500,\
    0.500,0.866,0.000,\
    0.000,0.866,-0.500,\
    \
    0.000,-0.866,-0.500,\
    0.500,-0.866,0.000,\
    0.000,-0.866,0.500,\
    -0.500,-0.866,0.000,\
}; ///Sun Sensor Normal Vectors, XYZ in Satellite Body Frame 
//Photodiodes 	1-4 in Pyramid 1 with -Y face, 
//				5-8 in Pyramid 2 with -X face, 
//				9-12 in Pyramid 3 with +Y face, 
//				13-16 in Pyramid 4 with +Y face, // CHANGE
//				17-20 in Pyramid 5 with -Y face, // CHANGE
//8-11 in Pyramid 3 with +Y face, and 12-15 in Pyramid 4 with +Y face

#define J2000_MISSION_EPOCH_DAY 808660800  // Epoch 08/17/25 //Pranith

/* System Time */
extern uint32_t g_J2000_time; ///<Time in int sec since J2000
extern float g_J2000_frac_time; ///<Time in frac sec since J2000
extern uint32_t g_loop_time;
