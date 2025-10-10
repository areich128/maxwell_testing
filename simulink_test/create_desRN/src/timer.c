/* file: 		timer.c
 * author: 	jacob cook
 * created on: 7/9/2014
 */

#include "simulinkCustom.h"
#include <stdint.h>
#include "io.h"
#include "timer.h"


/* -- timer_soft_reset ---------------------------------------------------
 * 
 * ---------------------------------------------------------------------*/
int32_t timer_soft_reset(struct tmrCtlr *ctlr)
{
	/* set soft reset bit */
	or_regl(0x1, &ctlr->tiocp_cfg);

	/* wait for reset to complete */
    int32_t timeout = TIMEOUT_LEN;
	while(((readl(&ctlr->tiocp_cfg) & 0x1)==0x1) && --timeout) PRINT("%#lx\r\n",timeout);
    if(timeout == 0) return 0;
    return 1;
}

/* -- timer_start --------------------------------------------------------
 * 
 * ---------------------------------------------------------------------*/
int32_t timer_start(struct tmrCtlr *ctlr)
{
	if(timer_wait_to_write(ctlr, TMR_TWPS_TCLR)==0) return 0;
	or_regl(TMR_TCLR_ST, &ctlr->tclr);
    return 1;
}

/* -- timer_stop ---------------------------------------------------------
 *
 * ---------------------------------------------------------------------*/
int32_t timer_stop(struct tmrCtlr *ctlr)
{
	if(timer_wait_to_write(ctlr, TMR_TWPS_TCLR)==0) return 0;
	and_regl(~TMR_TCLR_ST, &ctlr->tclr);
    return 1;
}

/* -- timer_disable_int --------------------------------------------------
 *
 * ---------------------------------------------------------------------*/
void timer_disable_int(struct tmrCtlr *ctlr, uint32_t flags)
{
	or_regl(flags, &ctlr->irqenable_clr);
}

/* -- timer_enable_int ---------------------------------------------------
 *
 * ---------------------------------------------------------------------*/
void timer_enable_int(struct tmrCtlr *ctlr, uint32_t flags)
{
	or_regl(flags, &ctlr->irqenable_set);
}

/* -- timer_check_rstatus ------------------------------------------------
 *
 * ---------------------------------------------------------------------*/
uint32_t timer_check_rstatus(struct tmrCtlr *ctlr, uint32_t flag)
{
	return (readl(&ctlr->irqstatus_raw) & flag) ? 1 : 0;
}

/* -- timer_check_status ------------------------------------------------
 *
 * ---------------------------------------------------------------------*/
uint32_t timer_check_status(struct tmrCtlr *ctlr, uint32_t flag)
{
	return (readl(&ctlr->irqstatus) & flag) ? 1 : 0;
}

/* -- timer_clear_status -------------------------------------------------
 *
 * ---------------------------------------------------------------------*/
void timer_clear_status(struct tmrCtlr *ctlr, uint32_t flag)
{
	or_regl(flag, &ctlr->irqstatus);
}

/* -- timer_wait_to_write ------------------------------------------------
 *
 * ---------------------------------------------------------------------*/
int32_t timer_wait_to_write(struct tmrCtlr *ctlr, uint32_t reg)
{
	if(readl(&ctlr->tsicr) & TMR_TSICR_POSTED)
    {
	    int32_t timeout = TIMEOUT_LEN;
		while((reg & readl(&ctlr->twps)) && --timeout);
        if(timeout == 0) return 0;
    }
    return 1;
}

/* -- timer_set_prescaler ------------------------------------------------
 * divisor  = 2^(presvale + 1) 
 * ---------------------------------------------------------------------*/
int32_t timer_set_prescaler(struct tmrCtlr *ctlr, uint32_t prescale)
{
	if(prescale == 0)
	{
		if(timer_wait_to_write(ctlr, TMR_TWPS_TCLR)==0) return 0;
		/* clear PRE bit is tclr register */
		else and_regl(~TMR_TCLR_PRE, &ctlr->tclr);
		return 1;
	}

	/* set prescale value */
	if(timer_wait_to_write(ctlr, TMR_TWPS_TCLR)==0) return 0;
	else and_regl(~TMR_TCLR_PTV_MASK, &ctlr->tclr);
	if(timer_wait_to_write(ctlr, TMR_TWPS_TCLR)==0) return 0;
	else or_regl((prescale <<  TMR_TCLR_PTV_SHIFT), &ctlr->tclr);

	/* set PRE bit to enable prscaling */
	if(timer_wait_to_write(ctlr, TMR_TWPS_TCLR)==0) return 0;
	else or_regl(TMR_TCLR_PRE, &ctlr->tclr);
    return 1;
}

/* -- timer_set_polarity ------------------------------------------------
 * 
 * --------------------------------------------------------------------*/
int32_t timer_set_polarity(struct tmrCtlr *ctlr, uint32_t pol)
{
	/* Clear polarity bit */
	if(timer_wait_to_write(ctlr,TMR_TWPS_TCLR)==0) return 0;
	else and_regl(~TMR_TCLR_SCPWM_NEG, &ctlr->tclr);

	/* write it polarity bit */
	if(timer_wait_to_write(ctlr, TMR_TWPS_TCLR)==0) return 0;
	else or_regl((pol << 7), &ctlr->tclr);
    return 1;
}	

/* -- timer_set_period ---------------------------------------------------
 *
 * ---------------------------------------------------------------------*/
int32_t timer_set_period(struct tmrCtlr *ctlr, float period)
{
	uint32_t tldr;
	uint32_t count;
    int32_t rv = 1;

	/* calculate reload value.  ASSUMES NO DIVISOR */
	count = (uint32_t)(period/1000/TMR_SYS_CLK_PERIOD);
	tldr = 0xFFFFFFFF - count + 1;
	
	/* write value to register */
	if(timer_set_reload(ctlr, tldr)==0) rv = 0;
    return rv;
}


/* -- timer_set_reload ---------------------------------------------------
 * 
 * ---------------------------------------------------------------------*/
int32_t timer_set_reload(struct tmrCtlr *ctlr, uint32_t load_val)
{
	if(timer_wait_to_write(ctlr, TMR_TWPS_TLDR)==0) return 0;
	else writel(load_val, &ctlr->tldr);
    return 1;
}

/* -- timer_set_cnt ------------------------------------------------------
 *
 * ---------------------------------------------------------------------*/
int32_t timer_set_cnt(struct tmrCtlr *ctlr, uint32_t cnt)
{
	if(timer_wait_to_write(ctlr, TMR_TWPS_TCRR)==0) return 0;
    else writel(cnt, &ctlr->tcrr);
    return 1;
}

/* -- timer_trigger_reload -----------------------------------------------
 *
 * ---------------------------------------------------------------------*/
int32_t timer_trigger_reload(struct tmrCtlr *ctlr)
{
	if(timer_wait_to_write(ctlr, TMR_TWPS_TTGR)==0) return 0;
    else writel(0xFFFF,&ctlr->ttgr);
    return 1;
}

