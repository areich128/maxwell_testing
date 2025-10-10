/* ***********************************************************************
 * file: 		timer.h
 * author:		jacob cook
 * created on:	7/8/2014
 * ***********************************************************************
 */

#ifndef __TIMER_H
#define __TIMER_H

#include <stdint.h>


/* All timers are set up to run on the Master clock signal */
#define TMR_SYS_CLK (24000000) /* 24Mhz */
#define TMR_SYS_CLK_PERIOD (42e-9f) /* 40 ns */

#define TIMER2_BASE (0x48040000)
#define TIMER3_BASE (0x48042000)
#define TIMER4_BASE (0x48044000)
#define TIMER5_BASE (0x48046000)
#define TIMER7_BASE (0x4804A000)

/* IRQ Flags */
#define TMR_TCAR 		(0x1 << 2)
#define TMR_OVF			(0x1 << 1)
#define TMR_MAT			(0x1)
#define TMR_ALL_INT	(0x7)

/* TCLR register */
#define TMR_TCLR_ST 					(0x1)
#define TMR_TCLR_PRE 					(0x1 << 5)
#define TMR_TCLR_PTV_SHIFT 		(2)
#define TMR_TCLR_PTV_MASK 		(0x7 << TMR_TCLR_PTV_SHIFT)
#define TMR_TCLR_GPO_OUT 			(0x0 << 14)
#define TMR_TCLR_GPO_IN				(0x1 << 14)
#define TMR_TCLR_PT						(0x1 << 12)
#define TMR_TCLR_TRG_SHIFT		(10)
#define TMR_TCLR_TRG_MASK			(0x3 << TMR_TCLR_TRG_SHIFT)
#define TMR_TCLR_TRG_NONE			(0x0 << TMR_TCLR_TRG_SHIFT)
#define TMR_TCLR_TRG_OVFL			(0x1 << TMR_TCLR_TRG_SHIFT)
#define TMR_TCLR_TRG_OVMA			(0x2 << TMR_TCLR_TRG_SHIFT)
#define TMR_TCLR_SCPWM_POS		(0x0 << 7)
#define TMR_TCLR_SCPWM_NEG		(0x1 << 7)
#define TMR_TCLR_AR						(0x1 << 1)
#define TMR_TCLR_CE						(0x1 << 6)

/* TWPS register */
#define TMR_TWPS_TMAR					(0x1 << 4)
#define TMR_TWPS_TTGR					(0x1 << 3)
#define TMR_TWPS_TLDR					(0x1 << 2)
#define TMR_TWPS_TCRR					(0x1 << 1)
#define TMR_TWPS_TCLR					(0x1)

/* TSICR register */
#define TMR_TSICR_POSTED 			(0x1 << 2)
#define TMR_TSICR_SFT					(0x1 << 1)

struct tmrCtlr
{
	uint32_t tidr; 					/* 0x00 */
	uint32_t res1[3];				/* 0x04 - 0x0C */
	uint32_t tiocp_cfg;			/* 0x10 */
	uint32_t res2[3];				/* 0x14 - 0x1C */
	uint32_t irq_eoi;				/* 0x20 */
	uint32_t irqstatus_raw;	/* 0x24 */
	uint32_t irqstatus;			/* 0x28 */
	uint32_t irqenable_set;	/* 0x2C */
	uint32_t irqenable_clr;	/* 0x30 */
	uint32_t irqwakeen;			/* 0x34 */
	uint32_t tclr;					/* 0x38 */
	uint32_t tcrr;					/* 0x3C */
	uint32_t tldr;					/* 0x40 */
	uint32_t ttgr;					/* 0x44 */
	uint32_t twps;					/* 0x48 */
 	uint32_t tmar;					/* 0x4C */
	uint32_t tcar1;					/* 0x50 */
	uint32_t tsicr;					/* 0x54 */
	uint32_t tcar2;					/* 0x58 */
};

int32_t timer_soft_reset(struct tmrCtlr *ctlr);
int32_t timer_start(struct tmrCtlr *ctlr);
int32_t timer_stop(struct tmrCtlr *ctlr);
void timer_disable_int(struct tmrCtlr *ctlr, uint32_t flags);
void timer_enable_int(struct tmrCtlr *ctlr, uint32_t flags);
uint32_t timer_check_rstatus(struct tmrCtlr *ctlr, uint32_t flag);
uint32_t timer_check_status(struct tmrCtlr *ctlr, uint32_t flag);
void timer_clear_status(struct tmrCtlr *ctlr, uint32_t flag);
int32_t timer_wait_to_write(struct tmrCtlr *ctlr, uint32_t reg);
int32_t timer_set_prescaler(struct tmrCtlr *ctlr, uint32_t prescale);
int32_t timer_set_polarity(struct tmrCtlr *ctlr, uint32_t pol);
int32_t timer_set_period(struct tmrCtlr *ctlr, float period);;
int32_t timer_set_reload(struct tmrCtlr *ctlr, uint32_t load_val);
int32_t timer_trigger_reload(struct tmrCtlr *ctlr);
int32_t timer_set_cnt(struct tmrCtlr *ctlr, uint32_t cnt);

#endif
