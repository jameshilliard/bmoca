/*
    <:copyright-BRCM:2013:DUAL/GPL:standard
    
       Copyright (c) 2013 Broadcom Corporation
       All Rights Reserved
    
    Unless you and Broadcom execute a separate written software license
    agreement governing use of this software, this software is licensed
    to you under the terms of the GNU General Public License version 2
    (the "GPL"), available at http://www.broadcom.com/licenses/GPLv2.php,
    with the following added to such license:
    
       As a special exception, the copyright holders of this software give
       you permission to link this software with independent modules, and
       to copy and distribute the resulting executable under terms of your
       choice, provided that you also meet, for each linked independent
       module, the terms and conditions of the license of that module.
       An independent module is a module which is not derived from this
       software.  The special exception does not apply to any modifications
       of the software.
    
    Not withstanding the above, under no circumstances may you combine
    this software in any way with any other Broadcom software provided
    under a license other than the GPL, without Broadcom's express prior
    written consent.
    
    :> 

*/

#include "bbsi.h"
#include <linux/spi/spi.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30)
#else
typedef unsigned long uintptr_t;
#endif // LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,30)

#define MOCA_RD(x)    ((((struct moca_platform_data *)priv->pdev->dev.platform_data)->use_spi == 0) ? \
                       (*((volatile uint32_t *)((unsigned long)(x)))) : \
                       ((uint32_t)kerSysBcmSpiSlaveReadReg32(((struct moca_platform_data *)priv->pdev->dev.platform_data)->spi, (uint32_t)(x))))

#define MOCA_RD8(x, y) ((((struct moca_platform_data *)priv->pdev->dev.platform_data)->use_spi == 0) ? \
                        (*(y) = *((volatile unsigned char *)((unsigned long)(x)))) : \
                        (kerSysBcmSpiSlaveRead(((struct moca_platform_data *)priv->pdev->dev.platform_data)->spi, (unsigned long)(x), y, 1)))

#define MOCA_WR(x,y)   do { ((((struct moca_platform_data *)priv->pdev->dev.platform_data)->use_spi == 0) ? \
                            (*((volatile uint32_t *)((unsigned long)(x)))) = (y) : \
                            kerSysBcmSpiSlaveWriteReg32(((struct moca_platform_data *)priv->pdev->dev.platform_data)->spi, (uint32_t)(x), (y))); } while(0)

#define MOCA_WR8(x,y)    do { ((((struct moca_platform_data *)priv->pdev->dev.platform_data)->use_spi == 0) ? \
                               (*((volatile unsigned char *)((unsigned long)(x)))) = (unsigned char)(y) : \
                               kerSysBcmSpiSlaveWrite(((struct moca_platform_data *)priv->pdev->dev.platform_data)->spi, (unsigned long)(x), (y), 1)); } while(0)

#define MOCA_WR16(x,y)   do { ((((struct moca_platform_data *)priv->pdev->dev.platform_data)->use_spi == 0) ? \
                               (*((volatile unsigned short *)((unsigned long)(x)))) = (unsigned short)(y) : \
                               kerSysBcmSpiSlaveWrite(((struct moca_platform_data *)priv->pdev->dev.platform_data)->spi, (unsigned long)(x), (y), 2)); } while(0)

#define MOCA_WR_BLOCK(addr, src, len) do { kerSysBcmSpiSlaveWriteBuf(((struct moca_platform_data *)priv->pdev->dev.platform_data)->spi, addr, src, len, 4); } while(0)
#define MOCA_RD_BLOCK(addr, dst, len) do { kerSysBcmSpiSlaveReadBuf(((struct moca_platform_data *)priv->pdev->dev.platform_data)->spi, addr, dst, len, 4); } while(0)


#define I2C_RD(x)		MOCA_RD(x)
#define I2C_WR(x, y)		MOCA_WR(x, y)

#define MOCA_BPCM_NUM         5
#define MOCA_BPCM_ZONES_NUM   8

typedef enum _PMB_COMMAND_E_
{
   PMB_COMMAND_ALL_OFF = 0,
   PMB_COMMAND_ALL_ON,

   PMB_COMMAND_LAST
} PMB_COMMAND_E;

static uint32_t zone_off_bitmask[MOCA_BPCM_NUM] = { 0xFF, 0x03, 0x30, 0x00, 0x00 };
static uint32_t zone_on_bitmask[MOCA_BPCM_NUM]  = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static void bogus_release(struct device *dev)
{
}

static struct moca_platform_data moca_lan_data = {
	.macaddr_hi =		0x00000102,
	.macaddr_lo =		0x03040000,

	.bcm3450_i2c_base =  0x10406200,
	.bcm3450_i2c_addr =  0x70,
	.hw_rev  =     HWREV_MOCA_20_GEN22,
	.rf_band =     MOCA_BAND_EXT_D,
	.chip_id =     0,
	.use_dma           = 0,
	.use_spi           = 1,
	.devId            = MOCA_DEVICE_ID_UNREGISTERED, // Filled in dynamically
#ifdef CONFIG_SMP
	.smp_processor_id = 1,
#endif
};

static struct resource moca_lan_resources[] = {
	[0] = {
		.start = 0x10600000,
		.end =   0x107ffd97,
		.flags = IORESOURCE_MEM,
	},
	[1] = { /* Not used for 6802, define for bmoca */
		.start = 0,
		.end = 0,
		.flags = IORESOURCE_IRQ,
	}
};

static struct platform_device moca_lan_plat_dev = {
	.name = "bmoca",
	.id = 0,
	.num_resources = ARRAY_SIZE(moca_lan_resources),
	.resource = moca_lan_resources,
	.dev = {
		.platform_data = &moca_lan_data,
		.release = bogus_release,
	},
};

static struct moca_platform_data moca_wan_data = {
	.macaddr_hi       = 0x00000102,
	.macaddr_lo       = 0x03040000,

	.bcm3450_i2c_base =  0x10406200,
	.bcm3450_i2c_addr =  0x70,
	.hw_rev  = HWREV_MOCA_20_GEN22,
	.chip_id = 0,
	
	.rf_band = MOCA_BAND_EXT_D,

	.use_dma           = 0,
	.use_spi           = 1,
	.devId            = MOCA_DEVICE_ID_UNREGISTERED, // Filled in dynamically

#ifdef CONFIG_SMP
	.smp_processor_id = 1,
#endif
};

static struct resource moca_wan_resources[] = {
	[0] = {
		.start = 0x10600000,
		.end =   0x107ffd97,
		.flags = IORESOURCE_MEM,
	},
	[1] = { /* Not used for 6802, define for bmoca */
		.start = 0,
		.end = 0,
		.flags = IORESOURCE_IRQ,
	}
};

static struct platform_device moca_wan_plat_dev = {
	.name          = "bmoca",
	.id            = 1,
	.num_resources = ARRAY_SIZE(moca_wan_resources),
	.resource      = moca_wan_resources,
	.dev           = {
		.platform_data = &moca_wan_data,
		.release       = bogus_release,
	},
};

static void moca_enable_irq(struct moca_priv_data *priv)
{
	kerSysMocaHostIntrEnable(((struct moca_platform_data *)priv->pdev->dev.platform_data)->spi);
}

static void moca_disable_irq(struct moca_priv_data *priv)
{
	kerSysMocaHostIntrDisable(((struct moca_platform_data *)priv->pdev->dev.platform_data)->spi);
}

static void moca_pmb_busy_wait(struct moca_priv_data *priv)
{
	#if 0
	uint32_t data;

	/* Possible time saver: The register access time over SPI may 
	   always be enough to guarantee that the write will complete 
	   in time without having to check the status. */
	do
	{
		data = MOCA_RD(priv->base + priv->regs->pmb_master_status);
	} while (data & 0x1);
	#endif
}

static void moca_pmb_control(struct moca_priv_data *priv, PMB_COMMAND_E cmd)
{
	int i, j;
	uint32_t * p_zone_control;
	uint32_t data;

	switch (cmd)
	{
		case PMB_COMMAND_ALL_OFF:
			// Turn off zone command
			MOCA_WR(priv->base + priv->regs->pmb_master_wdata_offset, 0xA00);
			p_zone_control = &zone_off_bitmask[0];
			break;
		case PMB_COMMAND_ALL_ON:
			// Turn on zone command
			MOCA_WR(priv->base + priv->regs->pmb_master_wdata_offset, 0xC00);
			p_zone_control = &zone_on_bitmask[0];
			break;
		default:
			printk(KERN_WARNING "%s: illegal cmd: %08x\n",
				__FUNCTION__, cmd);
			return;
	}

	for (i = 0; i < MOCA_BPCM_NUM; i++)
	{
		for (j = 0; j < MOCA_BPCM_ZONES_NUM; j++)
		{
			if (*p_zone_control & (1 << j))
			{
				// zone address in bpcms
				data = (0x1 << 20) + 16 + (i * 4096) + (j * 4);
				MOCA_WR(priv->base + priv->regs->pmb_master_cmd_offset, data);
				moca_pmb_busy_wait(priv);
			}
		}
		p_zone_control++;
	}

}

static void moca_pmb_give_fw_cntrl(struct moca_priv_data *priv)
{
	/* Pass control over the memories to the FW */
	MOCA_WR(priv->base + priv->regs->pmb_master_wdata_offset, 0x1);
	MOCA_WR(priv->base + priv->regs->pmb_master_cmd_offset, 0x100002);
	moca_pmb_busy_wait(priv);
}

static void moca_hw_reset(struct moca_priv_data *priv)
{
//	unsigned long flags;
//   uint32_t chipid;

	/* disable and clear all interrupts */
	MOCA_WR(priv->base + priv->regs->l2_mask_set_offset, 0xffffffff);
	MOCA_RD(priv->base + priv->regs->l2_mask_set_offset);

	/* assert resets */

	/* reset CPU first, both CPUs for MoCA 20 HW */
	if (priv->hw_rev == HWREV_MOCA_20_GEN22)
		MOCA_SET(priv->base + priv->regs->sw_reset_offset, 5);
	else
		MOCA_SET(priv->base + priv->regs->sw_reset_offset, 1);

	MOCA_RD(priv->base + priv->regs->sw_reset_offset);

	udelay(20);

	/* reset everything else except clocks */
	MOCA_SET(priv->base + priv->regs->sw_reset_offset, ~((1 << 3) | (1 << 7)));
	MOCA_RD(priv->base + priv->regs->sw_reset_offset);

	udelay(20);

	/* disable clocks */
	MOCA_SET(priv->base + priv->regs->sw_reset_offset, ~(1 << 3));
	MOCA_RD(priv->base + priv->regs->sw_reset_offset);

	MOCA_WR(priv->base + priv->regs->l2_clear_offset, 0xffffffff);
	MOCA_RD(priv->base + priv->regs->l2_clear_offset);

	/* Power down all zones */
	moca_pmb_control(priv, PMB_COMMAND_ALL_OFF);

	/* Power down all SYS_CTRL memories */
	MOCA_WR(0x10100068, 1);   // CLKGEN_PLL_SYS1_PLL_PWRDN
	MOCA_SET(0x1010000c, 1);  // CLKGEN_PLL_SYS0_PLL_CHANNEL_CTRL_CH_3

}

static unsigned int moca_get_phy_freq(struct moca_priv_data *priv)
{
	unsigned int x = MOCA_RD(0x10100044); // CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL_CH_2

	x = (x >> 1) & 0xFF; // Get the MDIV_CH2 field

	return( x ? 2400 / x : 0);
}

/* called any time we start/restart/stop MoCA */
static void moca_hw_init(struct moca_priv_data *priv, int action)
{
	u32 mask;
	u32 temp;
	u32 data;
	u32 count = 0;
	struct moca_platform_data * pMocaData = (struct moca_platform_data *)priv->pdev->dev.platform_data;

	if (action == MOCA_ENABLE && !priv->enabled) {
		clk_enable(priv->clk);

		MOCA_WR(0x10404318, 0xfffffffd); // SUN_TOP_CTRL_SW_INIT_0_SET
		udelay(20);
		MOCA_WR(0x1040431c, 0xffffffff); // SUN_TOP_CTRL_SW_INIT_0_CLEAR --> Do this at start of sequence
		udelay(20);
   
		priv->enabled = 1;
	}

	/* clock not enabled, register accesses will fail with bus error */
	if (!priv->enabled)
		return;

	moca_hw_reset(priv);
	udelay(1);

	MOCA_WR(0x10800000, 0x03);       // EMUX_CNTRL
	MOCA_WR(0x1080000c, 0x11);       // RGMII_0_CNTRL
	MOCA_WR(0x10800014, 0xc0);       // RGMII_0_RX_CLK_DELAY_CNTRL

	MOCA_WR(0x104040a4, 0x01);       // GENERAL_CTRL_NO_SCAN_0
	MOCA_WR(0x10404100, 0x11110011); // PIN_MUX_CTRL_0
	MOCA_WR(0x10404104, 0x11111111); // PIN_MUX_CTRL_1

	if (action == MOCA_ENABLE) {

		/* Power up all zones */
		moca_pmb_control(priv, PMB_COMMAND_ALL_ON);
		moca_pmb_give_fw_cntrl(priv);

		MOCA_UNSET(0x1010000c, 1);  // CLKGEN_PLL_SYS0_PLL_CHANNEL_CTRL_CH_3 

		MOCA_WR(0x1010006C, 1);  // CLKGEN_PLL_SYS1_PLL_RESET 
		MOCA_WR(0x10100068, 0);  // CLKGEN_PLL_SYS1_PLL_PWRDN 
		data = 0;
		while ((data & 0x1) == 0)
		{
			/* This typically is only read once */
			data = MOCA_RD(0x10100060); // CLKGEN_PLL_SYS1_PLL_LOCK_STATUS

			if (count++ > 10)
				break;
		}
		MOCA_WR(0x1010006C, 0);  // CLKGEN_PLL_SYS1_PLL_RESET 

		if (priv->bonded_mode) {
			MOCA_UNSET(0x10100048, 1);  // CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL_CH_3 
			MOCA_UNSET(0x10100050, 1);  // CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL_CH_5 
		} else {
			MOCA_SET(0x10100048, 1);  // CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL_CH_3 
			MOCA_SET(0x10100050, 1);  // CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL_CH_5 
		}
		udelay(1);

		/* deassert moca_sys_reset, system clock, phy0 and phy0 clock */
		mask = (1 << 1) | (1 << 7) | (1 << 4) | (1 << 8);

		/* deassert phy1 and phy1 clock in bonded mode */
		if (priv->bonded_mode)
			mask |= (1 << 5) | (1 << 9);

		MOCA_UNSET(priv->base + priv->regs->sw_reset_offset, mask);
		MOCA_RD(priv->base + priv->regs->sw_reset_offset);


		/* Check for 6802/6803 A0 chip only with Xtal mod */
		if ((pMocaData->chip_id & 0xFFFEFFFF) == 0x680200A0)
		{
			data = MOCA_RD(0x1040401c);
			if ((data & 0x7) == 0x2) {
				/* 25MHz */
				printk("MoCA running with 25MHz XTAL\n");
				MOCA_WR(priv->base + priv->regs->host2moca_mmp_outbox_0_offset, 1);
			} else {
				printk("MoCA == 50MHz XTAL\n");
				/* 50MHz clock change only */
				MOCA_WR(priv->base + priv->regs->host2moca_mmp_outbox_0_offset, 0);
				//Note: The re-configuration is in NDIV_INT, not PDIV.
				//`CLKGEN_REG_START + `CLKGEN_PLL_SYS1_PLL_DIV (32'h10100058) [09:00] = 10�d48
				temp = MOCA_RD(0x10100058);
				temp = (temp & 0xFFFFFC00) + 48;
				MOCA_WR(0x10100058, temp);

				//`CLKGEN_REG_START + `CLKGEN_PLL_SYS0_PLL_DIV (32'h10100018) [09:00] = 10�d40
				temp = MOCA_RD(0x10100018);
				temp = (temp & 0xFFFFFC00) + 40;
				MOCA_WR(0x10100018, temp);

				//`CLKGEN_REG_START + `CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL_CH_4 (32'h1010004C) [08:01] = 8�d48
				temp = MOCA_RD(0x1010004c);
				temp = (temp & 0xFFFFFE01) + (48 << 1);
				MOCA_WR(0x1010004c, temp);

				//`CLKGEN_REG_START + `CLKGEN_PLL_SYS1_PLL_CHANNEL_CTRL_CH_5 (32'h10100050) [08:01] = 8�d48
				temp = MOCA_RD(0x10100050);
				temp = (temp & 0xFFFFFE01) + (48 << 1);
				MOCA_WR(0x10100050, temp);

				// Then Restart the PLL.

				//`CLKGEN_REG_START + `CLKGEN_PLL_SYS0_PLL_RESET (32'h1010002C) [0] = 1�b1
				MOCA_SET(0x1010002c, 1);
				//`CLKGEN_REG_START + `CLKGEN_PLL_SYS1_PLL_RESET (32'h1010006C) [0] = 1�b1
				MOCA_SET(0x1010006c, 1);

				udelay(1);

				//`CLKGEN_REG_START + `CLKGEN_PLL_SYS0_PLL_RESET (32'h1010002C) [0] = 1�b0
				MOCA_UNSET(0x1010002c, 1);
				//`CLKGEN_REG_START + `CLKGEN_PLL_SYS1_PLL_RESET (32'h1010006C) [0] = 1�b0
				MOCA_UNSET(0x1010006c, 1);
			}
		}

		// CLKGEN_PLL_SYS1_PLL_SSC_MODE_CONTROL_HIGH
		data = MOCA_RD(0x10100070);
		data = (data & 0xFFFF0000) | 0x7dd;
		MOCA_WR(0x10100070, data);

		// CLKGEN_PLL_SYS1_PLL_SSC_MODE_CONTROL_LOW
		data = MOCA_RD(0x10100074);
		data = (data & 0xffc00000) | 0x3d71;
		MOCA_WR(0x10100074, data);

		// CLKGEN_PLL_SYS1_PLL_SSC_MODE_CONTROL_LOW
		MOCA_SET(0x10100074, (1 << 22));
		
		printk("Set PLL SSC mode\n");
	}


	if (priv->hw_rev <= HWREV_MOCA_20_GEN21) {
	/* clear junk out of GP0/GP1 */
		MOCA_WR(priv->base + priv->regs->gp0_offset, 0xffffffff);
		MOCA_WR(priv->base + priv->regs->gp1_offset, 0x0);
		/* set up activity LED for 50% duty cycle */
		MOCA_WR(priv->base + priv->regs->led_ctrl_offset,
			0x40004000);
	}

	/* enable DMA completion interrupts */
	mask = M2H_REQ | M2H_RESP | M2H_ASSERT | M2H_WDT_CPU1 |
		M2H_NEXTCHUNK | M2H_DMA;

	if (priv->hw_rev >= HWREV_MOCA_20_GEN21)
		mask |= M2H_WDT_CPU0 | M2H_NEXTCHUNK_CPU0 |
			M2H_REQ_CPU0 | M2H_RESP_CPU0 | M2H_ASSERT_CPU0;

	MOCA_WR(priv->base + priv->regs->ringbell_offset, 0);
	MOCA_WR(priv->base + priv->regs->l2_mask_clear_offset, mask);
	MOCA_RD(priv->base + priv->regs->l2_mask_clear_offset);


	/* Set pinmuxing for MoCA interrupt and flow control */
	MOCA_UNSET(0x10404110, 0xF00000FF);
	MOCA_SET(0x10404110, 0x10000022);
 
	MOCA_WR(0x100b0318, 2);

	if (action == MOCA_DISABLE && priv->enabled) {
		priv->enabled = 0;
		clk_disable(priv->clk);
	}
}

static void moca_ringbell(struct moca_priv_data *priv, uint32_t mask)
{
	MOCA_WR(priv->base + priv->regs->ringbell_offset, mask);
}

static uint32_t moca_start_mips(struct moca_priv_data *priv, unsigned int cpu)
{
	if (priv->hw_rev == HWREV_MOCA_20_GEN22) {
		if (cpu == 1)
			MOCA_UNSET(priv->base + priv->regs->sw_reset_offset,
				(1 << 0));
		else {
			moca_mmp_init(priv, 1);
			MOCA_UNSET(priv->base + priv->regs->sw_reset_offset,
				(1 << 2));
		}
	} else
		MOCA_UNSET(priv->base + priv->regs->sw_reset_offset, (1 << 0));
	MOCA_RD(priv->base + priv->regs->sw_reset_offset);

	return(0);
}

static void moca_m2m_xfer(struct moca_priv_data *priv,
	uint32_t dst, uint32_t src, uint32_t ctl)
{
	uint32_t status;

	MOCA_WR(priv->base + priv->regs->m2m_src_offset, src);
	MOCA_WR(priv->base + priv->regs->m2m_dst_offset, dst);
	MOCA_WR(priv->base + priv->regs->m2m_status_offset, 0);
	MOCA_RD(priv->base + priv->regs->m2m_status_offset);
	MOCA_WR(priv->base + priv->regs->m2m_cmd_offset, ctl);

	do {
		status = MOCA_RD(priv->base + priv->regs->m2m_status_offset);
	} while(status == 0);

}

static void moca_write_mem(struct moca_priv_data *priv,
	uint32_t dst_offset, void *src, unsigned int len)
{
	struct moca_platform_data *pd = priv->pdev->dev.platform_data;

	if((dst_offset >= priv->regs->cntl_mem_offset+priv->regs->cntl_mem_size) ||
		((dst_offset + len) > priv->regs->cntl_mem_offset+priv->regs->cntl_mem_size)) {
		printk(KERN_WARNING "%s: copy past end of cntl memory: %08x\n",
			__FUNCTION__, dst_offset);
		return;
	}

	if ( 1 == pd->use_dma )
	{
		dma_addr_t pa;

		pa = dma_map_single(&priv->pdev->dev, src, len, DMA_TO_DEVICE);
		mutex_lock(&priv->copy_mutex);
		moca_m2m_xfer(priv, dst_offset + priv->regs->data_mem_offset, (uint32_t)pa, len | M2M_WRITE);
		mutex_unlock(&priv->copy_mutex);
		dma_unmap_single(&priv->pdev->dev, pa, len, DMA_TO_DEVICE);
	}
	else
	{
		uintptr_t addr = (uintptr_t)priv->base + priv->regs->data_mem_offset + dst_offset;
		uint32_t *data = src;
		int i;

		mutex_lock(&priv->copy_mutex);
		if (((struct moca_platform_data *)priv->pdev->dev.platform_data)->use_spi == 1)
		{
			src = data;
			MOCA_WR_BLOCK(addr, src, len);
		}
		else
		{
			for(i = 0; i < len; i += 4, addr += 4, data++)
				MOCA_WR(addr, *data);
			MOCA_RD(addr - 4);	/* flush write */
		}

		mutex_unlock(&priv->copy_mutex);
	}
}

static void moca_read_mem(struct moca_priv_data *priv,
	void *dst, uint32_t src_offset, unsigned int len)
{
	struct moca_platform_data *pd = priv->pdev->dev.platform_data;
    
	if((src_offset >= priv->regs->cntl_mem_offset+priv->regs->cntl_mem_size) ||
		((src_offset + len) > priv->regs->cntl_mem_offset+priv->regs->cntl_mem_size)) {
		printk(KERN_WARNING "%s: copy past end of cntl memory: %08x\n",
			__FUNCTION__, src_offset);
		return;
	}

	if ( 1 == pd->use_dma )
	{
		dma_addr_t pa;

		pa = dma_map_single(&priv->pdev->dev, dst, len, DMA_FROM_DEVICE);
		mutex_lock(&priv->copy_mutex);
		moca_m2m_xfer(priv, (uint32_t)pa, src_offset + priv->regs->data_mem_offset, len | M2M_READ);
		mutex_unlock(&priv->copy_mutex);
		dma_unmap_single(&priv->pdev->dev, pa, len, DMA_FROM_DEVICE);
	}
	else
	{
		uintptr_t addr = priv->regs->data_mem_offset + src_offset;
		uint32_t *data = dst;
		int i;

		mutex_lock(&priv->copy_mutex);
		if (((struct moca_platform_data *)priv->pdev->dev.platform_data)->use_spi == 1)
		{
			MOCA_RD_BLOCK((uintptr_t)priv->base + addr, dst, len);
		}
		else
		{
			for(i = 0; i < len; i += 4, addr += 4, data++)
	 			*data = MOCA_RD((uintptr_t)priv->base + addr);
		}
		mutex_unlock(&priv->copy_mutex);
	}
}

static void moca_write_sg(struct moca_priv_data *priv,
	uint32_t dst_offset, struct scatterlist *sg, int nents)
{
	int j;
	uintptr_t addr = priv->regs->data_mem_offset + dst_offset;
	struct moca_platform_data *pd = priv->pdev->dev.platform_data;

	dma_map_sg(&priv->pdev->dev, sg, nents, DMA_TO_DEVICE);

	mutex_lock(&priv->copy_mutex);
	for(j = 0; j < nents; j++)
	{
		if ( 1 == pd->use_dma )
		{
		    // printk("XXX copying page %d, PA %08x\n", j, (int)sg[j].dma_address);
			moca_m2m_xfer(priv, addr, (uint32_t)sg[j].dma_address, 
				sg[j].length | M2M_WRITE);

			addr += sg[j].length;
		}
		else
		{
			unsigned long *data = (void *)phys_to_virt(sg[j].dma_address);
         //printk("%s: Writing 0x%lx to addr 0x%08lx (len = %d)\n", __FUNCTION__, *data, ((unsigned long)priv->base) + addr, sg[j].length);
			MOCA_WR_BLOCK(((unsigned long)priv->base) + addr, data, sg[j].length);
			addr += sg[j].length;
		}
	}
	mutex_unlock(&priv->copy_mutex);

	dma_unmap_sg(&priv->pdev->dev, sg, nents, DMA_TO_DEVICE);
}

/* NOTE: this function is not tested */
#if 0
static void moca_read_sg(struct moca_priv_data *priv,
	uint32_t src_offset, struct scatterlist *sg, int nents)
{
	int j;
	uintptr_t addr = priv->data_mem_offset + src_offset;

	dma_map_sg(&priv->pdev->dev, sg, nents, DMA_FROM_DEVICE);

	mutex_lock(&priv->copy_mutex);
	for(j = 0; j < nents; j++) {
#if 0 //USE_DMA
		 printk("XXX copying page %d, PA %08x\n", j, (int)sg[j].dma_address);
		moca_m2m_xfer(priv, addr, (uint32_t)sg[j].dma_address,
			sg[j].length | M2M_READ);

		addr += sg[j].length;
#else
		uint32_t *data = (void *)phys_to_virt(sg[j].dma_address);
		unsigned int len = sg[j].length;
		int i;

		for(i = 0; i < len; i += 4, addr += 4, data++) {
			*data = cpu_to_be32(
				MOCA_RD((uintptr_t)priv->base + addr));
			//printk("MoCA READ: AD 0x%x  = 0x%x (0x%x)\n", (priv->base + addr), MOCA_RD((uintptr_t)priv->base + addr), *data);
		 }
#endif
	}
	mutex_unlock(&priv->copy_mutex);

	dma_unmap_sg(&priv->pdev->dev, sg, nents, DMA_FROM_DEVICE);
}
#endif

static void moca_read_mac_addr(struct moca_priv_data *priv, uint32_t * hi, uint32_t * lo)
{
	struct net_device * pdev ;
	char					 mocaName[7] ;

	if (priv == NULL)
		sprintf (mocaName, "moca%u", 0) ;
	else
		sprintf (mocaName, "moca%u", ((struct moca_platform_data *)priv->pdev->dev.platform_data)->devId) ;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 29)
	pdev = dev_get_by_name ( &init_net, mocaName ) ;
#else
	pdev = dev_get_by_name ( mocaName ) ;
#endif

	if ((pdev != NULL) && (lo != NULL) && (hi != NULL)) {
		mac_to_u32(hi, lo, pdev->dev_addr);
	}
}


#if defined(DSL_MOCA)

/*
 * This helper function was added to allow the enet driver to compile in
 * consumer environment for 68xx profiles.
 */
void moca_get_fc_bits(void * arg, unsigned long *moca_fc_reg)
{
	struct moca_priv_data *     priv;
	struct moca_platform_data * pMocaData;
	unsigned long               flags;

	if (arg == NULL) {
		return;
	}

	priv = (struct moca_priv_data *) arg;
	pMocaData = (struct moca_platform_data *)priv->pdev->dev.platform_data;

	*moca_fc_reg = 0;
	if (priv != NULL)
	{
		/* We can't read moca core regs unless the core's clocks are on. */
		spin_lock_irqsave(&priv->clock_lock, flags);
		if (priv->running) {
			*moca_fc_reg = MOCA_RD(priv->base+priv->regs->sideband_gmii_fc_offset);
		}
		spin_unlock_irqrestore(&priv->clock_lock, flags);
	}
}

#endif /* DSL_MOCA */

static int __devinit bmoca_spi_probe(struct spi_device *spi) {
	// TODO(apenwarr): match one spi device to one moca device struct.
	// I happen to know that right now the system only registers one of
	// moca_lan or moca_wan, never both, and there is never more than
	// one moca chip present on our systems, so this is okay for now.
	uint32_t val = kerSysBcmSpiSlaveReadReg32(spi, 0x10404000);
	pr_info("bmoca_spi_probe bus=%d chip_select=%d: id=%08x %s\n",
		spi->master->bus_num, spi->chip_select, val,
		val != 0 ? "yes" : "no");
	if (val == 0) return -ENODEV;
	moca_lan_data.spi = spi;
	moca_wan_data.spi = spi;
	return 0; // success
}

static int __devexit bmoca_spi_remove(struct spi_device *spi) {
	pr_info("bmoca_spi_remove\n");
	if (moca_lan_data.spi == spi) moca_lan_data.spi = NULL;
	if (moca_wan_data.spi == spi) moca_wan_data.spi = NULL;
	return 0; // success
}

static struct spi_driver bmoca_spi_driver = {
  .driver = {
    .name = "bmoca",
    .owner = THIS_MODULE,
  },
  .probe = bmoca_spi_probe,
  .remove = __devexit_p(bmoca_spi_remove),
};

//extern void bcmenet_register_moca_fc_bits_cb(void cb(void *, unsigned long *), int isWan, void * arg);

static int  hw_specific_init( struct moca_priv_data *priv )
{
#ifdef DSL_MOCA
	struct moca_platform_data *pMocaData;

	pMocaData = (struct moca_platform_data *)priv->pdev->dev.platform_data;

	/* fill in the hw_rev field */
	pMocaData->chip_id = MOCA_RD(0x10404004) + 0xA0;
        pr_info("read moca chip id: %08x\n", pMocaData->chip_id);

	pMocaData->hw_rev = HWREV_MOCA_20_GEN22;

	/* Power down all LEAP memories */
	MOCA_WR(0x101000e4, 0x6); // CLKGEN_LEAP_TOP_INST_DATA   
	MOCA_WR(0x101000e8, 0x6); // CLKGEN_LEAP_TOP_INST_HAB 
	MOCA_WR(0x101000ec, 0x6); // CLKGEN_LEAP_TOP_INST_PROG0
	MOCA_WR(0x101000f0, 0x6); // CLKGEN_LEAP_TOP_INST_PROG1   
	MOCA_WR(0x101000f4, 0x6); // CLKGEN_LEAP_TOP_INST_PROG2  
	MOCA_WR(0x101000f8, 0x6); // CLKGEN_LEAP_TOP_INST_ROM
	MOCA_WR(0x101000fc, 0x6); // CLKGEN_LEAP_TOP_INST_SHARED  
	MOCA_WR(0x10100164, 0x3); // CLKGEN_SYS_CTRL_INST_POWER_SWITCH_MEMORY 

//	bcmenet_register_moca_fc_bits_cb(
//		moca_get_fc_bits, pMocaData->use_spi ? 1 : 0, (void *)priv);
#endif

	return 0;
}

static int moca_platform_dev_register(void)
{
	struct moca_platform_data *pMocaData;
	struct platform_device *pPlatformDev;
	BP_MOCA_INFO mocaInfo[BP_MOCA_MAX_NUM];
	int mocaChipNum = BP_MOCA_MAX_NUM;
	int i;
	int ret = 0;   

	BpGetMocaInfo(mocaInfo, &mocaChipNum);

	ret = spi_register_driver(&bmoca_spi_driver);
	if (ret < 0) return ret;

	for (i = 0; i < mocaChipNum; i++) {
		switch (mocaInfo[i].type) {
			case BP_MOCA_TYPE_WAN:
				pMocaData = &moca_wan_data;
				pPlatformDev = &moca_wan_plat_dev;
				break;

			case BP_MOCA_TYPE_LAN:
				pMocaData = &moca_lan_data;
				pPlatformDev = &moca_lan_plat_dev;
				break;

			default:
				printk(KERN_ERR "bmoca: unrecognized MoCA type %d\n",
					mocaInfo[i].type);
				return(-1);
				break;
		}

		ret = platform_device_register(pPlatformDev);
		if (ret < 0) {
			spi_unregister_driver(&bmoca_spi_driver);
			return(ret);
		}
		else {
			pMocaData->devId = i;

			/* Map the board params RF Band to the bmoca.h value */
			switch (mocaInfo[i].rfBand)
			{
				case BP_MOCA_RF_BAND_D_LOW:
					pMocaData->rf_band = MOCA_BAND_D_LOW;
					break;
				case BP_MOCA_RF_BAND_D_HIGH:
					pMocaData->rf_band = MOCA_BAND_D_HIGH;
					break;
				case BP_MOCA_RF_BAND_EXT_D:
					pMocaData->rf_band = MOCA_BAND_EXT_D;
					break;
				case BP_MOCA_RF_BAND_E:
					pMocaData->rf_band = MOCA_BAND_E;
					break;
				case BP_MOCA_RF_BAND_F:    
					pMocaData->rf_band = MOCA_BAND_F;
					break;
				default:
					/* Do nothing */
					break;
			}
			printk(KERN_INFO "bmoca: Found MoCA device %d/%d  RF Band %d\n",
				i, mocaChipNum, mocaInfo[i].rfBand);
		}
	}

	return(ret);
}

static void moca_platform_dev_unregister(void)
{
	spi_unregister_driver(&bmoca_spi_driver);

	if (moca_lan_data.devId != MOCA_DEVICE_ID_UNREGISTERED)
		platform_device_unregister(&moca_lan_plat_dev);

	if (moca_wan_data.devId != MOCA_DEVICE_ID_UNREGISTERED)
		platform_device_unregister(&moca_wan_plat_dev);
}

static void moca_3450_write(struct moca_priv_data *priv, u8 addr, u32 data)
{
	/* comment out for now. We don't use i2c on the 63268BHR board */
#ifdef MOCA_3450_USE_I2C
	if (((struct moca_platform_data *)priv->pdev->dev.platform_data)->use_spi == 0)
		bcm3450_write_reg(addr, data);
	else
#endif
		moca_3450_write_i2c(priv, addr, data);
}

static u32 moca_3450_read(struct moca_priv_data *priv, u8 addr)
{
	/* comment out for now. We don't use i2c on the 63268BHR board */
#ifdef MOCA_3450_USE_I2C
	if (((struct moca_platform_data *)priv->pdev->dev.platform_data)->use_spi == 0)
		return(bcm3450_read_reg(addr));
	else
#endif
		return(moca_3450_read_i2c(priv, addr));
}

/*
 * PM STUBS
 */

struct clk *clk_get(struct device *dev, const char *id)
{
	return NULL;
}

int clk_enable(struct clk *clk)
{

	return 0;
}

void clk_disable(struct clk *clk)
{
}

void clk_put(struct clk *clk)
{
}

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	return 0;
}

