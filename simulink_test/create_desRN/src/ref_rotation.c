/* ******************************************************************
 * file: ref_rotation.c
 * author: Nicola Baresi, Jesse Keefer
 * Created on: 10/11/2015
 * Purpose: Rotate from/to ECI frame to/from ECEF frame
 * ******************************************************************
 */

#include "simulinkCustom.h"
#include "ref_rotation.h"
#include "timer.h"
#include "eop_data.h"
#include <math.h>

/* Constant Variable for ECEF/ECI Conversions */
const double sec2rad      = 4.84813681109536E-06; // Conversion factor from arcsec to radians
const double DPI          = 6.283185307179586476925287; // 2*pi value
const int    LEAP_SECONDS = 22;

/* Earth Orientation Parameter Structure */
struct eop{

    float xp;
    float yp;
    float DUT1;
    float LOD;
    float dX;
    float dY;
    int   DTAI;

};

/* Internally available functions */
static int32_t GetEarthOrientationParameters(double mjd, struct eop* EOP);
static int32_t PolarMotionMatrix(double TT, float xp, float yp, float* W);
static int32_t SiderealRotationMatrix(double UT1, float LOD, float* R);
static int32_t PrecessionNutationMatrix(double TT, float dX, float dY, float* PN);

// Define Externally Available Functions

/* ECEF to ECI */
int32_t ECEF2ECI(double t, double t_frac, float* x_ECEF, float* vx_ECEF, 
    float* x_ECI, float* vx_ECI, uint8_t leapsec_since_epoch){

    int32_t rv = 0;
    
    // Time Variables
    double time = t/86400.0 + t_frac/86400.0;           // days since J2000
    double mjd  = 51544.5   + time;    // Modified Julian Date
    double UT1;                        // Uniform Time (millisec)
    double TT;                         // Terrestrial Time (millisec)
    
    // Rotation Matrices
    float W[9],R[3],PN[9];
    
    // Intermediate Frames + Auxiliary Variables
    float x_TIRS[6];
    float x_CIRS[6];
    float temp[3];
    
    // EOP Parameters
    struct eop EOP;

    /* Get Earth Orientation Parameters from File */
    GetEarthOrientationParameters(mjd, &EOP);
    
    /* Compute Additional Time Variables */
    UT1  = t + t_frac + EOP.DUT1 - LEAP_SECONDS - leapsec_since_epoch;     // UT1 in seconds (s)
    TT   = t + t_frac + EOP.DTAI + 32.184;           // terrestrial time in seconds (s)
    
    /* REDUCTION FORMULAS */
    /* == Polar Motion === */
    PolarMotionMatrix(TT,EOP.xp,EOP.yp,W);
    
    // Rotate Position Vector from ECEF to TIRS
    x_TIRS[0] = W[0]*x_ECEF[0] + W[1]*x_ECEF[1] + W[2]*x_ECEF[2];
    x_TIRS[1] = W[3]*x_ECEF[0] + W[4]*x_ECEF[1] + W[5]*x_ECEF[2];
    x_TIRS[2] = W[6]*x_ECEF[0] + W[7]*x_ECEF[1] + W[8]*x_ECEF[2];
    
    // Rotate Velocity Vector from ECEF to TIRS
    x_TIRS[3] = W[0]*vx_ECEF[0] + W[1]*vx_ECEF[1] + W[2]*vx_ECEF[2];
    x_TIRS[4] = W[3]*vx_ECEF[0] + W[4]*vx_ECEF[1] + W[5]*vx_ECEF[2];
    x_TIRS[5] = W[6]*vx_ECEF[0] + W[7]*vx_ECEF[1] + W[8]*vx_ECEF[2];
    
    /* == Sidereal Rotation === */
    SiderealRotationMatrix(UT1,EOP.LOD,R);
    
    // Auxiliary Variables
    temp[0] = x_TIRS[3] - R[2]*x_TIRS[1];
    temp[1] = x_TIRS[4] + R[2]*x_TIRS[0];
    temp[2] = x_TIRS[5];
    
    // Rotate Position Vector from TIRS to CIRS
    x_CIRS[0] = R[0]*x_TIRS[0] - R[1]*x_TIRS[1];
    x_CIRS[1] = R[1]*x_TIRS[0] + R[0]*x_TIRS[1];
    x_CIRS[2] = x_TIRS[2];
    
    // Rotate Velocity Vector from TIRS to CIRS
    x_CIRS[3] = R[0]*temp[0] - R[1]*temp[1];
    x_CIRS[4] = R[1]*temp[0] + R[0]*temp[1];
    x_CIRS[5] = temp[2];
    
    
    
    /* == Precession-Nutation === */
    PrecessionNutationMatrix(TT,EOP.dX,EOP.dY,PN);
    
    // Rotate Position Vector from CIRS to ECI
    x_ECI[0] = PN[0]*x_CIRS[0] + PN[1]*x_CIRS[1] + PN[2]*x_CIRS[2];
    x_ECI[1] = PN[3]*x_CIRS[0] + PN[4]*x_CIRS[1] + PN[5]*x_CIRS[2];
    x_ECI[2] = PN[6]*x_CIRS[0] + PN[7]*x_CIRS[1] + PN[8]*x_CIRS[2];
    
    
    // Rotate Velocity Vector from CIRS to ECI
    vx_ECI[0] = PN[0]*x_CIRS[3] + PN[1]*x_CIRS[4] + PN[2]*x_CIRS[5];
    vx_ECI[1] = PN[3]*x_CIRS[3] + PN[4]*x_CIRS[4] + PN[5]*x_CIRS[5];
    vx_ECI[2] = PN[6]*x_CIRS[3] + PN[7]*x_CIRS[4] + PN[8]*x_CIRS[5];
    
    // Error Handling if NaN ECI
    // If you're seeing this issue, look at eop_data.h 
    // Earth Orientation Parameters might be outdated. Find MATLAB script in Teams to update
    // Last Updated: Nov 8 2024 (Adhitya Sripennem)
    if(isnan(x_ECI[0])|| isnan(x_ECI[1]) || isnan(x_ECI[2]) || isnan(vx_ECI[0]) || isnan(vx_ECI[1]) || isnan(vx_ECI[2])){
        PRINT("ERROR: ref_rotation.c: NaN values in ECI \r\n");
    }


    rv = 1;
    return rv;
}
/* Pranith->Conversion for just position */
/* ECEF to ECI */
int32_t S_ECEF2ECI(float t, float t_frac, float* x_ECEF,
    float* x_ECI, uint8_t leapsec_since_epoch){

    int32_t rv = 0;
    
    // Time Variables
    double time = t/86400.0 + t_frac/86400.0;           // days since J2000
    double mjd  = 51544.5   + time;    // Modified Julian Date
    double UT1;                        // Uniform Time (millisec)
    double TT;                         // Terrestrial Time (millisec)
    
    // Rotation Matrices
    float W[9],R[3],PN[9];
    
    // Intermediate Frames + Auxiliary Variables
    float x_TIRS[6];
    float x_CIRS[6];
    
    // EOP Parameters
    struct eop EOP;

    /* Get Earth Orientation Parameters from File */
    GetEarthOrientationParameters(mjd, &EOP);
    
    /* Compute Additional Time Variables */
    UT1  = t + t_frac + EOP.DUT1 - LEAP_SECONDS - leapsec_since_epoch;     // UT1 in seconds (s)
    TT   = t + t_frac + EOP.DTAI + 32.184;           // terrestrial time in seconds (s)
    
    /* REDUCTION FORMULAS */
    /* == Polar Motion === */
    PolarMotionMatrix(TT,EOP.xp,EOP.yp,W);
    
    // Rotate Position Vector from ECEF to TIRS
    x_TIRS[0] = W[0]*x_ECEF[0] + W[1]*x_ECEF[1] + W[2]*x_ECEF[2];
    x_TIRS[1] = W[3]*x_ECEF[0] + W[4]*x_ECEF[1] + W[5]*x_ECEF[2];
    x_TIRS[2] = W[6]*x_ECEF[0] + W[7]*x_ECEF[1] + W[8]*x_ECEF[2];
    
    
    /* == Sidereal Rotation === */
    SiderealRotationMatrix(UT1,EOP.LOD,R);
    

    
    // Rotate Position Vector from TIRS to CIRS
    x_CIRS[0] = R[0]*x_TIRS[0] - R[1]*x_TIRS[1];
    x_CIRS[1] = R[1]*x_TIRS[0] + R[0]*x_TIRS[1];
    x_CIRS[2] = x_TIRS[2];
    

    
    
    /* == Precession-Nutation === */
    PrecessionNutationMatrix(TT,EOP.dX,EOP.dY,PN);
    
    // Rotate Position Vector from CIRS to ECI
    x_ECI[0] = PN[0]*x_CIRS[0] + PN[1]*x_CIRS[1] + PN[2]*x_CIRS[2];
    x_ECI[1] = PN[3]*x_CIRS[0] + PN[4]*x_CIRS[1] + PN[5]*x_CIRS[2];
    x_ECI[2] = PN[6]*x_CIRS[0] + PN[7]*x_CIRS[1] + PN[8]*x_CIRS[2];
    
    
    
    // Error Handling if NaN ECI 
    // If you're seeing this issue, look at eop_data.h 
    // Earth Orientation Parameters might be outdated. Find MATLAB script in Teams to update
    // Last Updated: Nov 8 2024 (Adhitya Sripennem)
    if(isnan(x_ECI[0])|| isnan(x_ECI[1]) || isnan(x_ECI[2])){
        PRINT("ERROR: ref_rotation.c: NaN values in ECI \r\n");
    }


    rv = 1;
    return rv;
}


/* ECEF to ECI */
int32_t ECI2ECEF(double t, double t_frac, float* x_ECI, float* vx_ECI,
 float* x_ECEF, float* vx_ECEF, uint8_t leapsec_since_epoch){
    
    int32_t rv = 0;
    
    // Time Variables
    double time = t/86400.0 + t_frac/86400.;           // days since J2000
    double mjd  = 51544.5 + time;      // Modified Julian Date
    double UT1;                        // Uniform Time (millisec)
    double TT;                         // Terrestrial Time (millisec)
    
    // Rotation Matrices
    float W[9],R[3],PN[9];
    
    // Intermediate Frames
    float x_CIRS[6];
    float x_TIRS[6];
    
    // EOP Parameters
    struct eop EOP;
    
    /* Get Earth Orientation Parameters from File */
    GetEarthOrientationParameters(mjd, &EOP);
    
    
    /* Compute Additional Time Variables */
    UT1  = t + EOP.DUT1 - LEAP_SECONDS - leapsec_since_epoch;     // UT1 in seconds (s)
    TT   = t + EOP.DTAI + 32.184;           // terrestrial time in seconds (s)
    
    
    
    /* REDUCTION FORMULAS */
    /* == Precession-Nutation === */
    PrecessionNutationMatrix(TT,EOP.dX,EOP.dY,PN);
    
    // Rotate Position Vector from CIRS to ECI
    x_CIRS[0] = PN[0]*x_ECI[0] + PN[3]*x_ECI[1] + PN[6]*x_ECI[2];
    x_CIRS[1] = PN[1]*x_ECI[0] + PN[4]*x_ECI[1] + PN[7]*x_ECI[2];
    x_CIRS[2] = PN[2]*x_ECI[0] + PN[5]*x_ECI[1] + PN[8]*x_ECI[2];
    
    
    // Rotate Velocity Vector from CIRS to ECI
    x_CIRS[3] = PN[0]*vx_ECI[0] + PN[3]*vx_ECI[1] + PN[6]*vx_ECI[2];
    x_CIRS[4] = PN[1]*vx_ECI[0] + PN[4]*vx_ECI[1] + PN[7]*vx_ECI[2];
    x_CIRS[5] = PN[2]*vx_ECI[0] + PN[5]*vx_ECI[1] + PN[8]*vx_ECI[2];
    
    
    
    /* == Sideral Rotation === */
    SiderealRotationMatrix(UT1,EOP.LOD,R);
    
    // Rotate Position Vector from TIRS to CIRS
    x_TIRS[0] =   R[0]*x_CIRS[0] + R[1]*x_CIRS[1];
    x_TIRS[1] = - R[1]*x_CIRS[0] + R[0]*x_CIRS[1];
    x_TIRS[2] =   x_CIRS[2];
    
    // Rotate Velocity Vector from TIRS to CIRS
    x_TIRS[3] =   R[0]*x_CIRS[3] + R[1]*x_CIRS[4] + R[2]*x_TIRS[1];
    x_TIRS[4] = - R[1]*x_CIRS[3] + R[0]*x_CIRS[4] - R[2]*x_TIRS[0];
    x_TIRS[5] =   x_CIRS[5];
    
    
    
    /* == Polar Motion === */
    PolarMotionMatrix(TT,EOP.xp,EOP.yp,W);
    
    // Rotate Position Vector from ECEF to TIRS
    x_ECEF[0] = W[0]*x_TIRS[0] + W[3]*x_TIRS[1] + W[6]*x_TIRS[2];
    x_ECEF[1] = W[1]*x_TIRS[0] + W[4]*x_TIRS[1] + W[7]*x_TIRS[2];
    x_ECEF[2] = W[2]*x_TIRS[0] + W[5]*x_TIRS[1] + W[8]*x_TIRS[2];
    
    // Rotate Velocity Vector from ECEF to TIRS
    vx_ECEF[0] = W[0]*x_TIRS[3] + W[3]*x_TIRS[4] + W[6]*x_TIRS[5];
    vx_ECEF[1] = W[1]*x_TIRS[3] + W[4]*x_TIRS[4] + W[7]*x_TIRS[5];
    vx_ECEF[2] = W[2]*x_TIRS[3] + W[5]*x_TIRS[4] + W[8]*x_TIRS[5];
    
    rv = 1;
    return rv;
}



// Define Local Functions
static int32_t GetEarthOrientationParameters(double mjd_in, struct eop* EOP){

	int32_t rv = 0;

	// Declare EOP Variables
	int32_t MJD;//yr, mo, day, MJD;   // Dates
	float xp,yp;                // Polar Motion Variables (arcsec)
	float DUT1;                 // Diff between UT1 and UTC (sec)
	float LOD;					// Length of Day

	float dX, dY;				// Free Core Nutation corrections
	int32_t DTAI;				// Diff between TAI and UTC (sec)

	// Declare Variable for Linear Interpolation
	int32_t MJD_up;//yr_up,mo_up,day_up,MJD_up;      // Dates
    float   xp_up,yp_up;                // Polar Motion Variables (arcsec)
    float   DUT1_up;                    // Difference between UT1 and UTC (sec)
    float   LOD_up;                     // Length Of Day

    float   dX_up,dY_up;                // Free Core Nutation corrections
    //int32_t DTAI_up;                  // Difference between TAI and UTC (sec)
    int32_t index;

	// Determine the row of EOP values to obtain. 
	// The EOP data matrix is in the form of:
	// [yr, mo, day, MJD, xp, yp, DUT1, LOD, ddPsi, ddEpd, dX, dY, DTAI]
    index = floor(mjd_in) - eop_data[0][3]; //MJD increases by 1 per row. Use
                                            // this to find the starting index

	MJD   = eop_data[index][3];
	xp    = eop_data[index][4];
	yp    = eop_data[index][5];
	DUT1  = eop_data[index][6];
	LOD   = eop_data[index][7];
	dX    = eop_data[index][10];
	dY    = eop_data[index][11];
	DTAI  = eop_data[index][12];

	// Read in the next line so values can be interpreted
	MJD_up   = eop_data[index + 1][3];
	xp_up    = eop_data[index + 1][4];
	yp_up    = eop_data[index + 1][5];
	DUT1_up  = eop_data[index + 1][6];
	LOD_up   = eop_data[index + 1][7];
	//ddPsi_up = eop_data[index + 1][8];
	//ddEps_up = eop_data[index + 1][9];
	dX_up    = eop_data[index + 1][10];
	dY_up    = eop_data[index + 1][11];
	//DTAI_up  = eop_data[index + 1][12];

	// Perform linear interpolation to match input MJD
	xp   = (xp_up - xp)/(MJD_up - MJD)*(mjd_in - MJD) + xp;
	yp   = (yp_up - yp)/(MJD_up - MJD)*(mjd_in - MJD) + yp;
	DUT1 = (DUT1_up - DUT1)/(MJD_up - MJD)*(mjd_in - MJD) + DUT1;
	LOD  = (LOD_up - LOD)/(MJD_up - MJD)*(mjd_in - MJD) + LOD;
	dX   = (dX_up - dX)/(MJD_up - MJD)*(mjd_in - MJD) + dX;
    dY   = (dY_up - dY)/(MJD_up - MJD)*(mjd_in - MJD) + dY;

    /* Convert from arcsec to radians */
    xp = xp*sec2rad;
    yp = yp*sec2rad;
    dX = dX*sec2rad;
    dY = dY*sec2rad;

	// Store the EOP patameters
	EOP->xp   = xp;
    EOP->yp   = yp;
    EOP->DUT1 = DUT1;
    EOP->LOD  = LOD;
    EOP->dX   = dX;
    EOP->dY   = dY;
    EOP->DTAI = DTAI;
    
    rv = 1;

    return rv;
}


/* Polar Motion Matrix */
static int32_t PolarMotionMatrix(double TT, float xp, float yp, float* W){
	
	int32_t rv = 0;

	// Terrestrial Time and TIO (Instantaneous Prime Meridian)
	float T_TT = TT/86400.0/36525.0;
    float sp   = -0.000047*T_TT*sec2rad;
    
    // Initialize Cosines & Sines
    float cx = cos(xp);
    float sx = sin(xp);
    float cy = cos(yp);
    float sy = sin(yp);
    float cs = cos(sp);
    float ss = sin(sp);
    
    // Store Polar Motion DCM
    W[0] = cx*cs;
    W[1] = (-cy*ss + sy*sx*cs);
    W[2] = (-sy*ss - cy*sx*cs);
    
    W[3] = cx*ss;
    W[4] = (cy*cs + sy*sx*ss);
    W[5] = (sy*cs - cy*sx*ss);
    
    W[6] = sx;
    W[7] = (-sy*cx);
    W[8] = cy*cx;
    
    rv = 1;

    return rv;
}

/* Sideral-Rotation Matrix */
static int32_t SiderealRotationMatrix(double UT1, float LOD, float* R){
    
    int32_t rv = 0;

    double MUT1 = UT1/86400.0;

    // Compute Earth Rotation Angle
    double ERA = DPI*(0.7790572732640 + 1.00273781191135448*MUT1);
    
    if(ERA < 0.)
        ERA = DPI + ERA;
    
    // Initialize Cosines and Sines
    float cE = cos(ERA);
    float sE = sin(ERA);
    
    // Earth's Spin Rate
    float wE = (7.292115146706979E-5)*(1-LOD/86400.);
    
    // Store Values
    R[0] = cE;
    R[1] = sE;
    R[2] = wE;
    
    rv = 1;

    return rv;
}


/* Precession-Nutation Matrix */
static int32_t PrecessionNutationMatrix(double TT, float dX, float dY, float* PN){
    
	int32_t rv = 0;

    // Terrestrial Time and TIO (Istantaneous Prime Meridian)
    float T_TT = TT/86400.0/36525.0;
    
    // Declare Delauney Variables, Cosines, and Sines
    float OM, D, F;
    float c1,c2,c3,c4;
    float s1,s2,s3,s4;
    
    // Delauney Variables (rad)
    OM = 2.1824391966 -   33.7570459536*T_TT;
    D  = 5.1984665887 + 7771.3771455937*T_TT;
    F  = 1.6279050815 + 8433.4661569164*T_TT;
    
    // Cosines & Sines
    c1 = cos(OM);
    s1 = sin(OM);
    c2 = cos(2*(F + OM - D));
    s2 = sin(2*(F + OM - D));
    c3 = cos(2*OM);
    s3 = sin(2*OM);
    c4 = cos(2*(F+OM));
    s4 = sin(2*(F+OM));
    
    
    // Declare X, Y, Z, a
    float X, Y;
    float Z, a;
    
    // Compute X & Y
    X = (-17251+2004191898*T_TT - 429783*T_TT*T_TT - 6844318*s1 - 523908*s2 
        + 82169*s3 - 90552*s4 + T_TT*205833*c1)*sec2rad*1E-6;
    Y = (-25896*T_TT - 22407275*T_TT*T_TT + 9205236*c1 + 573033*c2 - 89618*c3 
        + 97847*c4 + T_TT*153042*s1)*sec2rad*1E-6;
    
    // Include Core Free Nutation Effects
    X = X + dX;
    Y = Y + dY;
    
    // Compute a
    Z = sqrt(1 - X*X - Y*Y);
    a = 1/(1 + Z);
    

    // Store PN Direction Cosine Matrix
    PN[0] = 1 - a*X*X;
    PN[1] = -a*X*Y;
    PN[2] = X;
    
    PN[3] = -a*X*Y;
    PN[4] = 1 - a*Y*Y;
    PN[5] = Y;
    
    PN[6] = -X;
    PN[7] = -Y;
    PN[8] = 1 - a*(X*X + Y*Y);
    
    rv = 1;
    return rv;
}
