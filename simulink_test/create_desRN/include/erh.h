/******************************************************************************
@file erh.h
@brief Header file for the ADCS error handler

This file defines the error masks and error bit shifts to define errors that might
occur on the satellite in various systems.

Author: unknown

Extra Info:
******************************************************************************/

#ifndef ERROR_H_
#define ERROR_H_

#include <stdint.h>

// Errors that that will skip through the main loop until solved. (Except POS, STOR, MON)
#define ERRORS_SKIP_LOOP (    \
	ERROR_AD7991 |            \
	ERROR_ADS7924 |           \
	(0x01ull << OBS_ERR_20) | \
	(0x01ull << OBS_ERR_21) | \
	(0x01ull << OBS_ERR_22) | \
	(0x01ull << OBS_ERR_23) | \
	(0x01ull << OBS_ERR_24) | \
	(0x01ull << OBS_ERR_25) | \
	(0x01ull << OBS_ERR_26) | \
	(0x01ull << OBS_ERR_27) | \
	(0x01ull << OBS_ERR_28) | \
	(0x01ull << OBS_ERR_29) | \
	ERROR_ATT_MASK)
//Pranith-When are these set?
#define ERROR_CDH_ACT_MASK (0x1ull << 0)
#define ERROR_SYS_MASK (0x7Full << 1)
#define ERROR_SD_MASK  (0x07ull << 8)
#define ERROR_MON_MASK (0x03ull << 11)
#define ERROR_PWM_MASK (0x1Full << 13)
#define ERROR_RW_MASK  (0x0Full << 18)
#define ERROR_CTL_MASK (0x0Full << 22)
#define ERROR_ATT_MASK (0x0Full << 26)
#define ERROR_POS_MASK (0x1Full << 30)
#define ERROR_OBS_MASK (0x1FFFFFFFull << 35)

#define ERROR_AD7991 (        \
	(0x01ull << OBS_ERR_4) |  \
	(0x01ull << OBS_ERR_5) |  \
	(0x01ull << OBS_ERR_6) |  \
	(0x01ull << OBS_ERR_7) |  \
	(0x01ull << OBS_ERR_8) |  \
	(0x01ull << OBS_ERR_9) |  \
	(0x01ull << OBS_ERR_10) | \
	(0x01ull << OBS_ERR_11))

#define ERROR_ADS7924 (       \
	(0x01ull << OBS_ERR_12) | \
	(0x01ull << OBS_ERR_13) | \
	(0x01ull << OBS_ERR_14) | \
	(0x01ull << OBS_ERR_15) | \
	(0x01ull << OBS_ERR_16) | \
	(0x01ull << OBS_ERR_17) | \
	(0x01ull << OBS_ERR_18) | \
	(0x01ull << OBS_ERR_19))

#define ERR_LOOP_RESET (     \
	(0x01ull << POS_ERR_1) | \
	(0x01ull << POS_ERR_2) | \
	(0x01ull << POS_ERR_3) | \
	(0x01ull << POS_ERR_4) | \
	ERROR_ATT_MASK |         \
	ERROR_CTL_MASK |         \
	(0x01ull << PWM_ERR_1) | \
	(0x01ull << PWM_ERR_2) | \
	(0x01ull << PWM_ERR_3) | \
	(0x01ull << PWM_ERR_4) | \
	ERROR_MON_MASK)

enum ERROR_LIST
{
	/* Complete List of Error Flags
	 * Used to shift certain number of bits when setting flag in g_err
	 *
	 * uint64_t g_err structure (updated 10/11/2022) - Anthony Zara:
	 * | CDH_ACT 1 bit | SYS 7 bits | SD 3 bits | MON 2 bits | PWM 5 bits | RW 4 Bits | CTL 4 bits | ATT 4 bits | POS 5 bits | OBS 29 bits |
	 */

	// ** NOTE: Errors with name XXX_ERR_# are place holders for actual error names ** //

	CDH_REQ_ACT,
	SYS_ERR_1,
	SYS_ERR_2,
	SYS_ERR_3,
	SYS_ERR_4,
	SYS_ERR_5,
	SYS_ERR_6,
	SYS_ERR_7,
	SD_ERR_1,
	SD_ERR_2,
	SD_ERR_3,
	MON_ERR_1,
	MON_ERR_2,
	PWM_ERR_1,
	PWM_ERR_2,
	PWM_ERR_3,
	PWM_ERR_4,
	APP_ERR_1,
	RW_ERR_1, 
	RW_ERR_2,
	RW_ERR_3,
	RW_ERR_4,
	CTL_ERR_1,
	CTL_ERR_2,
	CTL_ERR_3,
	CTL_ERR_4,
	ATT_ERR_1,
	ATT_ERR_2,
	ATT_ERR_3,
	ATT_ERR_4,
	POS_ERR_1,
	POS_ERR_2,
	POS_ERR_3,
	POS_ERR_4,
	POS_ERR_5, // No POS_ERR_6 as defined in 0116 ADCS_Error_Dictionary
	OBS_ERR_1,
	OBS_ERR_2,
	OBS_ERR_3,
	OBS_ERR_4,
	OBS_ERR_5,
	OBS_ERR_6,
	OBS_ERR_7,
	OBS_ERR_8,
	OBS_ERR_9,
	OBS_ERR_10,
	OBS_ERR_11,
	OBS_ERR_12,
	OBS_ERR_13,
	OBS_ERR_14,
	OBS_ERR_15,
	OBS_ERR_16,
	OBS_ERR_17,
	OBS_ERR_18,
	OBS_ERR_19,
	OBS_ERR_20,
	OBS_ERR_21,
	OBS_ERR_22,
	OBS_ERR_23,
	OBS_ERR_24,
	OBS_ERR_25,
	OBS_ERR_26,
	OBS_ERR_27,
	OBS_ERR_28,
	OBS_ERR_29
};

enum EEPROM_ERROR_LIST
{
	/* Complete List of Error Flags
	 * Used to shift certain number of bits when setting flag in g_err
	 *
	 * uint64_t g_err structure (updated 10/11/2022) - Anthony Zara:
	 * | CDH_ACT 1 bit | SYS 7 bits | SD 3 bits | MON 2 bits | PWM 5 bits | RW 4 Bits | CTL 4 bits | ATT 4 bits | POS 5 bits | OBS 29 bits |
	 */

	// ** NOTE: Errors with name XXX_ERR_# are place holders for actual error names ** //

	EEPROM_CDH_REQ_ACT,
	EEPROM_SYS_ERR_1,
	EEPROM_SYS_ERR_2,
	EEPROM_SYS_ERR_3,
	EEPROM_SYS_ERR_4,
	EEPROM_SYS_ERR_5,
	EEPROM_SYS_ERR_6,
	EEPROM_SYS_ERR_7,
	EEPROM_SD_ERR_1,
	EEPROM_SD_ERR_2,
	EEPROM_SD_ERR_3,
	EEPROM_MON_ERR_1,
	EEPROM_MON_ERR_2,
	EEPROM_PWM_ERR_1,
	EEPROM_PWM_ERR_2,
	EEPROM_PWM_ERR_3,
	EEPROM_PWM_ERR_4,
	EEPROM_APP_ERR_1,
	EEPROM_RW_ERR_1, 
	EEPROM_RW_ERR_2,
	EEPROM_RW_ERR_3,
	EEPROM_RW_ERR_4,
	EEPROM_CTL_ERR_1,
	EEPROM_CTL_ERR_2,
	EEPROM_CTL_ERR_3,
	EEPROM_CTL_ERR_4,
	EEPROM_ATT_ERR_1,
	EEPROM_ATT_ERR_2,
	EEPROM_ATT_ERR_3,
	EEPROM_ATT_ERR_4,
	EEPROM_POS_ERR_1,
	EEPROM_POS_ERR_2,
	EEPROM_POS_ERR_3,
	EEPROM_POS_ERR_4,
	EEPROM_POS_ERR_5,
	EEPROM_OBS_ERR_1,
	EEPROM_OBS_ERR_2,
	EEPROM_OBS_ERR_3,
	EEPROM_OBS_ERR_4,
	EEPROM_OBS_ERR_5,
	EEPROM_OBS_ERR_6,
	EEPROM_OBS_ERR_7,
	EEPROM_OBS_ERR_8,
	EEPROM_OBS_ERR_9,
	EEPROM_OBS_ERR_10,
	EEPROM_OBS_ERR_11,
	EEPROM_OBS_ERR_12,
	EEPROM_OBS_ERR_13,
	EEPROM_OBS_ERR_14,
	EEPROM_OBS_ERR_15,
	EEPROM_OBS_ERR_16,
	EEPROM_OBS_ERR_17,
	EEPROM_OBS_ERR_18,
	EEPROM_OBS_ERR_19,
	EEPROM_OBS_ERR_20,
	EEPROM_OBS_ERR_21,
	EEPROM_OBS_ERR_22,
	EEPROM_OBS_ERR_23,
	EEPROM_OBS_ERR_24,
	EEPROM_OBS_ERR_25,
	EEPROM_OBS_ERR_26,
	EEPROM_OBS_ERR_27,
	EEPROM_OBS_ERR_28,
	EEPROM_OBS_ERR_29
};


#endif /* ERROR_H_ */
