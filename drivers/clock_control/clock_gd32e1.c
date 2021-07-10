/*
 *
 * Copyright (c) 2017 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <soc.h>
#include <drivers/clock_control.h>
#include <sys/util.h>
#include <drivers/clock_control/gd32_clock_control.h>
#include <gd32e10x_rcu.h>
#include <gd32e10x.h>


#define RCC_BASE DT_REG_ADDR_BY_IDX(DT_NODELABEL(rcc), 0)
#undef RCU_REG_VAL
#define RCU_REG_VAL(periph)             (REG32(RCC_BASE | periph ))

static inline int gd32_clock_reset()
{
	/* reset the RCU clock configuration to the default reset state */
	/* Set IRC8MEN bit */
	RCU_CTL |= RCU_CTL_IRC8MEN;

	/* Reset CFG0 and CFG1 registers */
	RCU_CFG0 = 0x00000000U;
	RCU_CFG1 = 0x00000000U;

	/* Reset HXTALEN, CKMEN, PLLEN, PLL1EN and PLL2EN bits */
	RCU_CTL &= ~(RCU_CTL_PLLEN |RCU_CTL_PLL1EN | RCU_CTL_PLL2EN | RCU_CTL_CKMEN | RCU_CTL_HXTALEN);
	/* disable all interrupts */
	RCU_INT = 0x00ff0000U;

	/* reset HXTALBPS bit */
	RCU_CTL &= ~(RCU_CTL_HXTALBPS);

	return 0;
}

static inline void system_clock_120m_hxtal(void)
{
    uint32_t timeout = 0U;
    uint32_t stab_flag = 0U;

    /* enable HXTAL */
    RCU_CTL |= RCU_CTL_HXTALEN;

    /* wait until HXTAL is stable or the startup time is longer than HXTAL_STARTUP_TIMEOUT */
    do{
        timeout++;
        stab_flag = (RCU_CTL & RCU_CTL_HXTALSTB);
    }while((0U == stab_flag) && (HXTAL_STARTUP_TIMEOUT != timeout));

    /* if fail */
    if(0U == (RCU_CTL & RCU_CTL_HXTALSTB)){
        while(1){
        }
    }

    FMC_WS = (FMC_WS & (~FMC_WS_WSCNT)) | FMC_WAIT_STATE_3;

    /* HXTAL is stable */
    /* AHB = SYSCLK */
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
    /* APB2 = AHB/1 */
    RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
    /* APB1 = AHB/2 */
    RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;

    /* CK_PLL = (CK_PREDIV0) * 30 = 120 MHz */
    RCU_CFG0 &= ~(RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4);
    RCU_CFG0 |= (RCU_PLLSRC_HXTAL_IRC48M | RCU_PLL_MUL30);

    RCU_CFG1 &= ~(RCU_CFG1_PLLPRESEL | RCU_CFG1_PREDV0SEL | RCU_CFG1_PLL1MF | RCU_CFG1_PREDV1 | RCU_CFG1_PREDV0);
#ifdef HXTAL_VALUE_8M
    /* CK_PREDIV0 = (CK_HXTAL)/2 *10 /10 = 4 MHz */
    RCU_CFG1 |= (RCU_PLLPRESRC_HXTAL | RCU_PREDV0SRC_CKPLL1 | RCU_PLL1_MUL10 | RCU_PREDV1_DIV2 | RCU_PREDV0_DIV10);
#elif defined (HXTAL_VALUE_25M)
    /* CK_PREDIV0 = (CK_HXTAL)/5 *8/10 = 4 MHz */
    RCU_CFG1 |= (RCU_PLLPRESRC_HXTAL | RCU_PREDV0SRC_CKPLL1 | RCU_PLL1_MUL8 | RCU_PREDV1_DIV5 | RCU_PREDV0_DIV10);
#endif

    /* enable PLL1 */
    RCU_CTL |= RCU_CTL_PLL1EN;
    /* wait till PLL1 is ready */
    while((RCU_CTL & RCU_CTL_PLL1STB) == 0U){
    }

    /* enable PLL */
    RCU_CTL |= RCU_CTL_PLLEN;

    /* wait until PLL is stable */
    while(0U == (RCU_CTL & RCU_CTL_PLLSTB)){
    }

    /* select PLL as system clock */
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_PLL;

    /* wait until PLL is selected as system clock */
    while(0U == (RCU_CFG0 & RCU_SCSS_PLL)){
    }
}

static inline int gd32_clock_init()
{
	system_clock_120m_hxtal();

	return 0;
}

/**
 * @brief Initialize clocks for the gd32
 *
 * This routine is called to enable and configure the clocks and PLL
 * of the soc on the board. It depends on the board definition.
 * This function is called on the startup and also to restore the config
 * when exiting for low power mode.
 *
 * @param dev clock device struct
 *
 * @return 0
 */
int gd32_clock_control_init(const struct device *dev)
{
	gd32_clock_reset();
	gd32_clock_init();

	return 0;
}

static inline int gd32_clock_control_on(const struct device *dev,
					 clock_control_subsys_t sub_system)
{
	ARG_UNUSED(dev);
	struct gd32_pclken *pclken = (struct gd32_pclken *)(sub_system);

	switch (pclken->bus)
	{
	case GD32_CLOCK_BUS_APB1:
    		RCU_REG_VAL(APB1EN_REG_OFFSET) |= pclken->enr;
		break;

	case GD32_CLOCK_BUS_APB2:
    		RCU_REG_VAL(APB2EN_REG_OFFSET) |= pclken->enr;
		break;

	default:
		break;
	}

	return 0;
}

static inline int gd32_clock_control_off(const struct device *dev,
					  clock_control_subsys_t sub_system)
{
	return 0;
}

static int gd32_clock_control_get_subsys_rate(const struct device *clock,
						clock_control_subsys_t sub_system,
						uint32_t *rate)
{
	return 0;
}

static struct clock_control_driver_api gd32_clock_control_api = {
	.on = gd32_clock_control_on,
	.off = gd32_clock_control_off,
	.get_rate = gd32_clock_control_get_subsys_rate,
};

/**
 * @brief RCC device, note that priority is intentionally set to 1 so
 * that the device init runs just after SOC init
 */
DEVICE_DT_DEFINE(DT_NODELABEL(rcc),
		    &gd32_clock_control_init,
		    NULL,
		    NULL, NULL,
		    PRE_KERNEL_1,
		    CONFIG_CLOCK_CONTROL_GD32_DEVICE_INIT_PRIORITY,
		    &gd32_clock_control_api);
