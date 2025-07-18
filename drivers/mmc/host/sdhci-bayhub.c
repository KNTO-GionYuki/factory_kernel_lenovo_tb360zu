// SPDX-License-Identifier: GPL-2.0-only
/*
 * Bayhub Technologies, Inc. BH201 SDHCI bridge IC for
 * VENDOR SDHCI platform driver source file
 *
 * Copyright (c) 2012-2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sd.h>
#include <linux/io.h>
#include <linux/delay.h>

#ifdef MSM_HOST_USED
#define sdhci_vendor_host			sdhci_msm_host
#define sdhci_vendor_execute_tuning sdhci_msm_execute_tuning
#define TRUE 1
#define FALSE 0
#elif defined MTK_HOST_USED
#include "mtk-mmc.h"
#define sdhci_vendor_host			msdc_host
#define sdhci_vendor_execute_tuning msdc_execute_tuning_bh
#define vendor_priv msdc_priv
#elif defined SPRD_HOST_USED
#define sdhci_vendor_host			sdhci_sprd_host
#define sdhci_vendor_execute_tuning sdhci_sprd_execute_tuning
#else
#define sdhci_vendor_host
#define sdhci_vendor_execute_tuning
#endif

static const unsigned int tran_exp[] = {
	10000,		100000,		1000000,	10000000,
	0,		0,		0,		0
};

static const unsigned char tran_mant[] = {
	0,	10,	12,	13,	15,	20,	25,	30,
	35,	40,	45,	50,	55,	60,	70,	80,
};

static const unsigned int taac_exp[] = {
	1,	10,	100,	1000,	10000,	100000,	1000000, 10000000,
};

static const unsigned int taac_mant[] = {
	0,	10,	12,	13,	15,	20,	25,	30,
	35,	40,	45,	50,	55,	60,	70,	80,
};

#define UNSTUFF_BITS(resp,start,size)					\
	({								\
		const int __size = size;				\
		const u32 __mask = (__size < 32 ? 1 << __size : 0) - 1;	\
		const int __off = 3 - ((start) / 32);			\
		const int __shft = (start) & 31;			\
		u32 __res;						\
									\
		__res = resp[__off] >> __shft;				\
		if (__size + __shft > 32)				\
			__res |= resp[__off-1] << ((32 - __shft) % 32);	\
		__res & __mask;						\
	})
#define	SD_FNC_AM_SDR50		0x2
#define	SD_FNC_AM_SDR104	0x3
#define BIT_PASS_MASK  (0x7ff)
#define SDR104_MANUAL_INJECT 0x3ff
#define SDR50_MANUAL_INJECT  0x77f

#define	TRUNING_RING_IDX(x)  ((x) % TUNING_PHASE_SIZE)
#define	GET_IDX_VALUE(tb, x)  (tb & (1 << (x)))
#define	GENERATE_IDX_VALUE(x)  (1 << (x))
#define	GET_TRUNING_RING_IDX_VALUE(tb, x)  (tb & (1 << TRUNING_RING_IDX(x)))
#define	GENERATE_TRUNING_RING_IDX_VALUE(x)  (1 << TRUNING_RING_IDX(x))
#define	MAX_CFG_BIT_VAL (383)
#define MMC_CMD_RETRIES 3
#ifndef MMC_STATE_SUSPENDED
#define MMC_STATE_SUSPENDED	(1<<5)		/* card is suspended */
#define mmc_card_suspended(c)	((c)->state & MMC_STATE_SUSPENDED)
#define mmc_card_set_suspended(c) ((c)->state |= MMC_STATE_SUSPENDED)
#define mmc_card_clr_suspended(c) ((c)->state &= ~MMC_STATE_SUSPENDED)
#endif
static int gg_select_card_spec(struct sdhci_host *host);
extern int mmc_app_cmd(struct mmc_host *host, struct mmc_card *card);
int sd_tuning_sw(struct sdhci_host *host);
//SDHCI_PLTFM_HOST_ALLOC will be defined when use sdhci_pltfm_init at host probe function.
#ifdef SDHCI_PLTFM_HOST_ALLOC
static struct sdhci_vendor_host* get_vendor_host(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
#if PLATFORM_845
	struct sdhci_vendor_host *vendor_host = pltfm_host->priv;
#else
	struct sdhci_vendor_host *vendor_host = sdhci_pltfm_priv(pltfm_host);
#endif
	return vendor_host;
}

static struct sdhci_host* get_sdhci_host(struct mmc_host *mmc)
{
	return mmc_priv(mmc);
}
#elif defined MMC_HOST_ALLOC
static struct sdhci_vendor_host* get_vendor_host(struct sdhci_host *host)
{
	struct mmc_host *mmc = host->mmc;
	struct sdhci_vendor_host *vendor_host = mmc_priv(mmc);
	return vendor_host;
}

static struct sdhci_host* get_sdhci_host(struct mmc_host *mmc)
{
	struct sdhci_vendor_host *vendor_host = mmc_priv(mmc);

	return vendor_priv(vendor_host);
}
#else

#endif

int init_flg = 0;
static u32 g_card_cid[4] = {0};
static void cfg_bit_2_bt(int max_bit, int tar, int *byt, int *bit)
{
	struct rl_bit_lct cfg_bit_map[6] = {
		{0, 6},	{1, 5},	{2, 4},
		{3, 2},	{4, 1},	{5, 0},
	};

	*byt = (max_bit - tar) / 6;
	*bit = cfg_bit_map[(max_bit - tar) % 6].rl_bits;
}

static u32 cfg_read_bits_ofs_mask(u8 *cfg, struct t_gg_reg_strt *bts)
{
	u32 rv = 0;
	u32 msk = bts->mask;
	int byt = 0, bit = 0;
	int i = 0;

	do {
		cfg_bit_2_bt(MAX_CFG_BIT_VAL, bts->ofs + i, &byt, &bit);
		if (cfg[byt] & (1 << bit))
			rv |= 1 << i;

		i++;
		msk >>= 1;
		if (msk == 0)
			break;
	} while (1);
	return rv;
}

static void cfg_write_bits_ofs_mask(u8 *cfg, struct t_gg_reg_strt *bts, u32 w_value)
{
	u32 wv = w_value & bts->mask;
	u32 msk = bts->mask;
	int byt = 0, bit = 0;
	int i = 0;

	do {
		cfg_bit_2_bt(MAX_CFG_BIT_VAL, bts->ofs + i, &byt, &bit);
		if (wv & 1)
			cfg[byt] |= (u8) (1 << bit);
		else
			cfg[byt] &= (u8) (~(1 << bit));
		//
		i++;
		wv >>= 1;
		msk >>= 1;
		if (msk == 0)
			break;
	} while (1);
}

void ram_bit_2_bt(int tar, int *byt, int *bit)
{
	*byt = tar / 8;
	*bit = tar % 8;
}

void set_gg_reg_cur_val(struct ggc_platform_t  *ggc, u8 *data, u8 len)
{
	//int i = 0;
	memcpy(&ggc->_gg_reg_cur[0], data, len);
	/*for (i = 0;i < 16; i ++) {
		pr_info("_gg_reg_cur[%d]: 0x%x\n", i, ggc->_gg_reg_cur[i]);
	}*/
}

void get_gg_reg_cur_val(struct ggc_platform_t  *ggc, u8 *data, u8 len)
{
	//int i = 0;
	memcpy(data, &ggc->_gg_reg_cur[0], len);
	/*for (i = 0;i < 16; i ++) {
		pr_info("_gg_reg_cur[%d]: 0x%x\n", i, ggc->_gg_reg_cur[i]);
	}*/
}

static u32 read_ram_bits_ofs_mask(u8 *cfg, struct t_gg_reg_strt *bts)
{
	u32 rv = 0;
	u32 msk = bts->mask;
	int byt = 0, bit = 0;
	int i = 0;

	do {
		ram_bit_2_bt(bts->ofs + i, &byt, &bit);
		if (cfg[byt] & (1 << bit))
			rv |= 1 << i;

		i++;
		msk >>= 1;
		if (msk == 0)
			break;

	} while (1);
	return rv;
}

void ggc_dll_voltage_init(struct sdhci_host *host)
{
	int i = 0;
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	for (i = 0; i < 4; i++) {
		vendor_host->ggc.dll_voltage_scan_map[i] = 0;
		vendor_host->ggc.sdr50.dll_voltage_unlock_cnt[i] = 0;
		vendor_host->ggc.sdr104.dll_voltage_unlock_cnt[i] = 0;
	}
}

void host_cmddat_line_reset(struct sdhci_host *host)
{
#ifdef MTK_HOST_USED	
	msdc_reset_hw_bh(get_vendor_host(host));
#else
	if (host->ops && host->ops->reset)
		host->ops->reset(host, SDHCI_RESET_CMD | SDHCI_RESET_DATA);
#endif
}

static int driver_send_command(struct sdhci_host *host)
{
	int ret = 0;
	int err;
	struct mmc_host *mmc = host->mmc;
	struct mmc_command cmd = {0};

	cmd.opcode = MMC_SELECT_CARD;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_NONE | MMC_CMD_AC;
	err = mmc_wait_for_cmd(mmc, &cmd, 3);
	if (err)
		pr_err("---- CMD7 FAILE0 ----\n");
	else
		ret = 1;

	return ret;
}

static void driver_send_command24(struct sdhci_host *host, u32 *cfg_data, int data_len)
{
	struct mmc_host *mmc = host->mmc;

	u8 *data1 = kzalloc(PAGE_SIZE, GFP_KERNEL);
	struct mmc_request mrq = {0};
	struct mmc_command cmd = { 0 };
	struct mmc_data data = { 0 };
	struct scatterlist sg;

	memcpy(data1, (u8 *)&(cfg_data[0]), data_len);
	sg_init_one(&sg, data1, 512);

	cmd.opcode = MMC_WRITE_BLOCK;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
	data.blksz = 512;
	data.blocks = 1;
	data.flags = MMC_DATA_WRITE;
	data.timeout_ns = 1000 * 1000 * 1000; /* 1 sec */
	data.sg = &sg;
	data.sg_len = 1;
	mrq.cmd = &cmd;
	mrq.data = &data;
	mrq.stop = NULL;
	mmc_wait_for_req(mmc, &mrq);

	kfree(data1);
}

int bht_mmc_send_cxd_native(struct mmc_host *host, u32 arg, u32 *cxd, int opcode)
{
	int err;
	struct mmc_command cmd = {};

	cmd.opcode = opcode;
	cmd.arg = arg;
	cmd.flags = MMC_RSP_R2 | MMC_CMD_AC;

	err = mmc_wait_for_cmd(host, &cmd, MMC_CMD_RETRIES);
	if (err)
		return err;

	memcpy(cxd, cmd.resp, sizeof(u32) * 4);

	return 0;
}

int bht_mmc_send_csd(struct mmc_card *card, u32 *csd)
{
	return bht_mmc_send_cxd_native(card->host, card->rca << 16,	csd,
				MMC_SEND_CSD);
}

int bht_mmc_decode_csd(struct mmc_card *card)
{
	struct mmc_csd *csd = &card->csd;
	unsigned int e, m, a, b;
	u32 *resp = card->raw_csd;

	/*
	 * We only understand CSD structure v1.1 and v1.2.
	 * v1.2 has extra information in bits 15, 11 and 10.
	 * We also support eMMC v4.4 & v4.41.
	 */
	csd->structure = UNSTUFF_BITS(resp, 126, 2);
	if (csd->structure == 0) {
		pr_err("%s: unrecognised CSD structure version %d\n",
			mmc_hostname(card->host), csd->structure);
		return -EINVAL;
	}

	csd->mmca_vsn	 = UNSTUFF_BITS(resp, 122, 4);
	m = UNSTUFF_BITS(resp, 115, 4);
	e = UNSTUFF_BITS(resp, 112, 3);
	csd->taac_ns	 = (taac_exp[e] * taac_mant[m] + 9) / 10;
	csd->taac_clks	 = UNSTUFF_BITS(resp, 104, 8) * 100;

	m = UNSTUFF_BITS(resp, 99, 4);
	e = UNSTUFF_BITS(resp, 96, 3);
	csd->max_dtr	  = tran_exp[e] * tran_mant[m];
	csd->cmdclass	  = UNSTUFF_BITS(resp, 84, 12);

	e = UNSTUFF_BITS(resp, 47, 3);
	m = UNSTUFF_BITS(resp, 62, 12);
	csd->capacity	  = (1 + m) << (e + 2);

	csd->read_blkbits = UNSTUFF_BITS(resp, 80, 4);
	csd->read_partial = UNSTUFF_BITS(resp, 79, 1);
	csd->write_misalign = UNSTUFF_BITS(resp, 78, 1);
	csd->read_misalign = UNSTUFF_BITS(resp, 77, 1);
	csd->dsr_imp = UNSTUFF_BITS(resp, 76, 1);
	csd->r2w_factor = UNSTUFF_BITS(resp, 26, 3);
	csd->write_blkbits = UNSTUFF_BITS(resp, 22, 4);
	csd->write_partial = UNSTUFF_BITS(resp, 21, 1);

	if (csd->write_blkbits >= 9) {
		a = UNSTUFF_BITS(resp, 42, 5);
		b = UNSTUFF_BITS(resp, 37, 5);
		csd->erase_size = (a + 1) * (b + 1);
		csd->erase_size <<= csd->write_blkbits - 9;
	}

	return 0;
}

int bht_mmc_sd_get_csd(struct mmc_host *host, struct mmc_card *card)
{
	int err;

	/*
	 * Fetch CSD from card.
	 */
	err = bht_mmc_send_csd(card, card->raw_csd);//thomas add 
	if (err)
		return err;

	err = bht_mmc_decode_csd(card);//thomas add
	if (err)
		return err;

	return 0;
}

static int bht_mmc_send_relative_addr(struct mmc_host *host, unsigned int *rca)
{
	int err;
	struct mmc_command cmd = {};

	cmd.opcode = 3;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R6 | MMC_CMD_BCR;

	err = mmc_wait_for_cmd(host, &cmd, 3);
	if (err)
		return err;

	*rca = cmd.resp[0] >> 16;

	return 0;
}

int bht_mmc_app_send_scr(struct mmc_card *card)
{
	int err;
	struct mmc_request mrq = {};
	struct mmc_command cmd = {};
	struct mmc_data data = {};
	struct scatterlist sg;
	__be32 *scr;

	/* NOTE: caller guarantees scr is heap-allocated */

	err = mmc_app_cmd(card->host, card);
	if (err)
		return err;

	/* dma onto stack is unsafe/nonportable, but callers to this
	 * routine normally provide temporary on-stack buffers ...
	 */
	scr = kmalloc(sizeof(card->raw_scr), GFP_KERNEL);
	if (!scr)
		return -ENOMEM;

	mrq.cmd = &cmd;
	mrq.data = &data;

	cmd.opcode = SD_APP_SEND_SCR;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = 8;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;

	sg_init_one(&sg, scr, 8);

	mmc_set_data_timeout(&data, card);

	mmc_wait_for_req(card->host, &mrq);

	card->raw_scr[0] = be32_to_cpu(scr[0]);
	card->raw_scr[1] = be32_to_cpu(scr[1]);

	kfree(scr);

	if (cmd.error)
		return cmd.error;
	if (data.error)
		return data.error;

	return 0;
}

/*
 * Given a 64-bit response, decode to our card SCR structure.
 */
static int bht_mmc_decode_scr(struct mmc_card *card)
{
	struct sd_scr *scr = &card->scr;
	unsigned int scr_struct;
	u32 resp[4];

	resp[3] = card->raw_scr[1];
	resp[2] = card->raw_scr[0];

	scr_struct = UNSTUFF_BITS(resp, 60, 4);
	if (scr_struct != 0) {
		pr_err("%s: unrecognised SCR structure version %d\n",
			mmc_hostname(card->host), scr_struct);
		return -EINVAL;
	}

	scr->sda_vsn = UNSTUFF_BITS(resp, 56, 4);
	scr->bus_widths = UNSTUFF_BITS(resp, 48, 4);
	if (scr->sda_vsn == SCR_SPEC_VER_2)
		/* Check if Physical Layer Spec v3.0 is supported */
		scr->sda_spec3 = UNSTUFF_BITS(resp, 47, 1);

	if (scr->sda_spec3) {
		scr->sda_spec4 = UNSTUFF_BITS(resp, 42, 1);
		scr->sda_specx = UNSTUFF_BITS(resp, 38, 4);
	}

	if (UNSTUFF_BITS(resp, 55, 1))
		card->erased_byte = 0xFF;
	else
		card->erased_byte = 0x0;

	if (scr->sda_spec3)
		scr->cmds = UNSTUFF_BITS(resp, 32, 2);

	/* SD Spec says: any SD Card shall set at least bits 0 and 2 */
	if (!(scr->bus_widths & SD_SCR_BUS_WIDTH_1) ||
	    !(scr->bus_widths & SD_SCR_BUS_WIDTH_4)) {
		pr_err("%s: invalid bus width\n", mmc_hostname(card->host));
		return -EINVAL;
	}

	return 0;
}

static int bht_mmc_sd_switch(struct mmc_card *card, int mode, int group,
	u8 value, u8 *resp)
{
	struct mmc_request mrq = {};
	struct mmc_command cmd = {};
	struct mmc_data data = {};
	struct scatterlist sg;

	/* NOTE: caller guarantees resp is heap-allocated */

	mode = !!mode;
	value &= 0xF;

	mrq.cmd = &cmd;
	mrq.data = &data;

	cmd.opcode = SD_SWITCH;
	cmd.arg = mode << 31 | 0x00FFFFFF;
	cmd.arg &= ~(0xF << (group * 4));
	cmd.arg |= value << (group * 4);
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = 64;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;

	sg_init_one(&sg, resp, 64);

	mmc_set_data_timeout(&data, card);

	mmc_wait_for_req(card->host, &mrq);

	if (cmd.error)
		return cmd.error;
	if (data.error)
		return data.error;

	return 0;
}

static int bht_mmc_read_switch(struct mmc_card *card)
{
	int err;
	u8 *status;

	if (card->scr.sda_vsn < SCR_SPEC_VER_1)
		return 0;

	if (!(card->csd.cmdclass & CCC_SWITCH)) {
		pr_warn("%s: card lacks mandatory switch function, performance might suffer\n",
			mmc_hostname(card->host));
		return 0;
	}

	status = kmalloc(64, GFP_KERNEL);
	if (!status)
		return -ENOMEM;

	/*
	 * Find out the card's support bits with a mode 0 operation.
	 * The argument does not matter, as the support bits do not
	 * change with the arguments.
	 */
	err = bht_mmc_sd_switch(card, 0, 0, 0, status);//fred add
	if (err) {
		/*
		 * If the host or the card can't do the switch,
		 * fail more gracefully.
		 */
		if (err != -EINVAL && err != -ENOSYS && err != -EFAULT)
			goto out;

		pr_warn("%s: problem reading Bus Speed modes\n",
			mmc_hostname(card->host));
		err = 0;

		goto out;
	}

	if (status[13] & SD_MODE_HIGH_SPEED)
		card->sw_caps.hs_max_dtr = HIGH_SPEED_MAX_DTR;

	if (card->scr.sda_spec3) {
		card->sw_caps.sd3_bus_mode = status[13];
		/* Driver Strengths supported by the card */
		card->sw_caps.sd3_drv_type = status[9];
		card->sw_caps.sd3_curr_limit = status[7] | status[6] << 8;
	}

out:
	kfree(status);

	return err;
}

static int bht_mmc_wait_for_app_cmd(struct mmc_host *host, struct mmc_card *card,
				struct mmc_command *cmd)
{
	struct mmc_request mrq = {};
	int i, err = -EIO;

	/*
	 * We have to resend MMC_APP_CMD for each attempt so
	 * we cannot use the retries field in mmc_command.
	 */
	for (i = 0; i <= MMC_CMD_RETRIES; i++) {
		err = mmc_app_cmd(host, card);
		if (err) {
			/* no point in retrying; no APP commands allowed */
			if (mmc_host_is_spi(host)) {
				if (cmd->resp[0] & R1_SPI_ILLEGAL_COMMAND)
					break;
			}
			continue;
		}

		memset(&mrq, 0, sizeof(struct mmc_request));

		memset(cmd->resp, 0, sizeof(cmd->resp));
		cmd->retries = 0;

		mrq.cmd = cmd;
		cmd->data = NULL;

		mmc_wait_for_req(host, &mrq);

		err = cmd->error;
		if (!cmd->error)
			break;

		/* no point in retrying illegal APP commands */
		if (mmc_host_is_spi(host)) {
			if (cmd->resp[0] & R1_SPI_ILLEGAL_COMMAND)
				break;
		}
	}

	return err;
}

static int bht_mmc_app_set_bus_width(struct mmc_card *card, int width)
{
	struct mmc_command cmd = {};

	cmd.opcode = SD_APP_SET_BUS_WIDTH;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

	switch (width) {
	case MMC_BUS_WIDTH_1:
		cmd.arg = SD_BUS_WIDTH_1;
		break;
	case MMC_BUS_WIDTH_4:
		cmd.arg = SD_BUS_WIDTH_4;
		break;
	default:
		return -EINVAL;
	}

	return bht_mmc_wait_for_app_cmd(card->host, card, &cmd);
}

static int bht_mmc_app_set_clr_card_detect(struct mmc_card *card)
{
       struct mmc_command cmd = {};

       cmd.opcode = 42;
       cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

       return bht_mmc_wait_for_app_cmd(card->host, card, &cmd);
}

static int bht_mmc_send_io_op_cond(struct mmc_host *host, u32 ocr, u32 *rocr)
{
	struct mmc_command cmd = {};
	int i, err = 0;

	cmd.opcode = 5;
	cmd.arg = ocr;
	cmd.flags = MMC_RSP_SPI_R4 | MMC_RSP_R4 | MMC_CMD_BCR;

	for (i = 100; i; i--) {
		err = mmc_wait_for_cmd(host, &cmd, MMC_CMD_RETRIES);
		if (err)
			break;

		/* if we're just probing, do a single pass */
		if (ocr == 0)
			break;

		/* otherwise wait until reset completes */
		if (mmc_host_is_spi(host)) {
			/*
			 * Both R1_SPI_IDLE and MMC_CARD_BUSY indicate
			 * an initialized card under SPI, but some cards
			 * (Marvell's) only behave when looking at this
			 * one.
			 */
			if (cmd.resp[1] & MMC_CARD_BUSY)
				break;
		} else {
			if (cmd.resp[0] & MMC_CARD_BUSY)
				break;
		}

		err = -ETIMEDOUT;
	}

	if (rocr)
		*rocr = cmd.resp[mmc_host_is_spi(host) ? 1 : 0];

	return err;
}
static int wait_dll_lock(struct sdhci_host *host)
{
	return sd_tuning_sw(host);
}

void bht_update_cfg(struct mmc_host *mmc_host, struct mmc_card *card, u32 *cfg_data, int data_len)
{
	int ret = 0;
	struct sdhci_host *host;

	host = get_sdhci_host(mmc_host);
#ifdef MTK_HOST_USED
	msdc_set_buswidth_bh(get_vendor_host(host), MMC_BUS_WIDTH_4);
	msdc_reset_hw_bh(get_vendor_host(host));
#else
	if(host->ops && host->ops->set_bus_width)
		host->ops->set_bus_width(host, MMC_BUS_WIDTH_4);
	else
		sdhci_set_bus_width(host, MMC_BUS_WIDTH_4);
	
	if (host->ops && host->ops->reset)
		host->ops->reset(host, SDHCI_RESET_CMD|SDHCI_RESET_DATA);
#endif

	ret = driver_send_command(host);
	if (!ret)
		pr_err("--send cmd7   error--\n");

	driver_send_command24(host, cfg_data, data_len);

	ret = driver_send_command(host);
	if (!ret)
		pr_err("--send cmd7  error--\n");

#ifdef MTK_HOST_USED
	msdc_set_buswidth_bh(get_vendor_host(host), MMC_BUS_WIDTH_1);
#else
	if(host->ops && host->ops->set_bus_width)
		host->ops->set_bus_width(host, MMC_BUS_WIDTH_1);
	else
		sdhci_set_bus_width(host, MMC_BUS_WIDTH_1);
#endif
}

void bht_load_hw_inject(struct mmc_host *mmc_host, struct mmc_card *card,
						u32 *cfg_data, int data_len, u32 sel200, u32 sel100)
{
	struct sdhci_host *host = get_sdhci_host(mmc_host);
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);
	u32 gg_hw_inj[16] = GGC_CFG_DATA;

	gg_hw_inj[1] = 0x7364032;

	if (vendor_host->ggc.bh201_sdr104_selb_hw_inject)
		gg_hw_inj[11] = vendor_host->ggc.bh201_sdr104_selb_hw_inject;
	else
		gg_hw_inj[11] = 0x77316200;

	if (vendor_host->ggc.bh201_sdr50_selb_hw_inject)
		gg_hw_inj[12] = vendor_host->ggc.bh201_sdr50_selb_hw_inject;
	else
		gg_hw_inj[12] = 0x00725777;

	pr_info("gg_hw_inj[11] = 0x%08x, gg_hw_inj[12] = 0x%08x, gg_hw_inj[15] = 0x%08x \n",gg_hw_inj[11], gg_hw_inj[12], gg_hw_inj[15]);

	bht_update_cfg(mmc_host, card, gg_hw_inj, data_len);
}


void bh201_signal_voltage_on_off(struct sdhci_host *host, u32 on_off)
{
	int card_present_status = 0;
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	if (vendor_host->ggc.bh201_used) {
		if (gpio_is_valid(vendor_host->ggc.det_gpio)) {
			card_present_status = gpio_get_value(vendor_host->ggc.det_gpio);
			pr_info("%s: detect_gpio pin %d status is %d\n",
			mmc_hostname(host->mmc), vendor_host->ggc.det_gpio, card_present_status);
		} else {
			pr_err("%s: no det_gpio provided\n", mmc_hostname(host->mmc));
		}

		if (on_off) {
			pr_info("%s: apply bht power on patch\n", mmc_hostname(host->mmc));

			ggc_dll_voltage_init(host);

			ggc_chip_init(host);

			if (gpio_is_valid(vendor_host->ggc.pwr_gpio)) {
				gpio_set_value(vendor_host->ggc.pwr_gpio, 1);
				msleep(100);
				pr_info("%s: pwr_gpio pin %d status is %d\n",
					mmc_hostname(host->mmc), vendor_host->ggc.pwr_gpio,
					gpio_get_value(vendor_host->ggc.pwr_gpio));
			} else {
				pr_err("%s: no pwr_gpio provided\n", mmc_hostname(host->mmc));
			}
		} else {
			pr_info("%s: apply bht power off patch\n", mmc_hostname(host->mmc));

			ggc_dll_voltage_init(host);

			if (card_present_status <= 0) {
				pr_info("%s: clear tuning result for power off and card removed\n",
					mmc_hostname(host->mmc));
				if(vendor_host->ggc.origin_cap)
				{
					host->mmc->caps = vendor_host->ggc.origin_cap;
					pr_info("%s: restore mmc caps to 0x%x\n",
					mmc_hostname(host->mmc), host->mmc->caps);
				}
				vendor_host->ggc.tuning_fail_count = 0;
				ggc_tuning_result_reset(host);
			}
			ggc_chip_init(host);

			if (gpio_is_valid(vendor_host->ggc.pwr_gpio)) {

				gpio_set_value(vendor_host->ggc.pwr_gpio, 0);
				pr_info("%s: pwr_gpio pin %d status is %d\n",
				mmc_hostname(host->mmc), vendor_host->ggc.pwr_gpio,
				gpio_get_value(vendor_host->ggc.pwr_gpio));
			} else {
				pr_err("%s: no pwr_gpio provided\n", mmc_hostname(host->mmc));
			}
		}
	}
}

void sdhci_bh201_parse(struct mmc_host *mmc_host)
{
	struct sdhci_host *host = get_sdhci_host(mmc_host);
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);
	struct device_node *np = vendor_host->pdev->dev.of_node;
	struct t_gg_reg_strt index_array[] ={
		{ 14, 0xffffffff, 0 }, { 46, 0xffffffff, 0 },
		{ 205, 0xffffffff, 0 }, { 237, 0xffffffff, 0 },
		{ 141, 0xf, 0 }, { 145, 0xf, 0 },
		{ 83, 0xfff, 0 }, { 95, 0xfff, 0 },
		{ 126, 0xf, 0 }, { 130, 0xf, 0 },
		{ 140, 0xf, 0 }, { 144, 0xf, 0 },
		{ 183, 0x1, 0 }, { 184, 0x1, 0 },
		{ 171, 0x1, 0 }, { 172, 0x1, 0 },
		{ 173, 0x3f, 0 }, { 357, 0x1, 0 },
		{ 93, 0x7ff, 0 }, { 81, 0x7ff, 0 },
	};
	// read
	vendor_host->ggc.pha_stas_rx_low32 = index_array[0];
	vendor_host->ggc.pha_stas_rx_high32 = index_array[1];
	vendor_host->ggc.pha_stas_tx_low32 = index_array[2];
	vendor_host->ggc.pha_stas_tx_high32 = index_array[3];
	vendor_host->ggc.dll_sela_after_mask = index_array[4];
	vendor_host->ggc.dll_selb_after_mask = index_array[5];

	vendor_host->ggc.dll_delay_100m_backup = index_array[6];
	vendor_host->ggc.dll_delay_200m_backup = index_array[7];

	// write
	vendor_host->ggc.dll_sela_100m_cfg = index_array[8];
	vendor_host->ggc.dll_sela_200m_cfg = index_array[9];
	vendor_host->ggc.dll_selb_100m_cfg = index_array[10];
	vendor_host->ggc.dll_selb_200m_cfg = index_array[11];
	vendor_host->ggc.dll_selb_100m_cfg_en = index_array[12];
	vendor_host->ggc.dll_selb_200m_cfg_en = index_array[13];
	vendor_host->ggc.internl_tuning_en_100m = index_array[14];
	vendor_host->ggc.internl_tuning_en_200m = index_array[15];
	vendor_host->ggc.cmd19_cnt_cfg = index_array[16];

	vendor_host->ggc.inject_failure_for_tuning_enable_cfg = index_array[17];
	vendor_host->ggc.inject_failure_for_200m_tuning_cfg = index_array[18];
	vendor_host->ggc.inject_failure_for_100m_tuning_cfg = index_array[19];

	vendor_host->ggc.bh201_used = 1;
	vendor_host->ggc.cur_sd_bus_speed = 0;
	vendor_host->ggc.bh201_drive_strength = 0;
	vendor_host->ggc.bh201_sdr50_sela_sw_inject = 0;
	vendor_host->ggc.bh201_sdr50_selb_hw_inject = 0;
	vendor_host->ggc.bh201_sdr104_selb_hw_inject = 0;

	host->flags |= SDHCI_SDR50_NEEDS_TUNING;
	host->mmc_host_ops.init_card = bht_load;

	if (of_find_property(np, "bh201_drive_strength", NULL))
		of_property_read_u32_index(np, "bh201_drive_strength", 0,
			&vendor_host->ggc.bh201_drive_strength);
	if (of_find_property(np, "bh201_sdr50_sela_sw_inject", NULL))
		of_property_read_u32_index(np, "bh201_sdr50_sela_sw_inject", 0,
			&vendor_host->ggc.bh201_sdr50_sela_sw_inject);
	if (of_find_property(np, "bh201_sdr50_selb_hw_inject", NULL))
		of_property_read_u32_index(np, "bh201_sdr50_selb_hw_inject", 0,
			&vendor_host->ggc.bh201_sdr50_selb_hw_inject);
	if (of_find_property(np, "bh201_sdr104_selb_hw_inject", NULL))
		of_property_read_u32_index(np, "bh201_sdr104_selb_hw_inject", 0,
			&vendor_host->ggc.bh201_sdr104_selb_hw_inject);

	vendor_host->ggc.pwr_gpio = of_get_named_gpio(np, "pwr-gpios", 0);
	if (!gpio_is_valid(vendor_host->ggc.pwr_gpio))
		dev_err(&vendor_host->pdev->dev, "no pwr-gpio provided !\n");
	else
		dev_info(&vendor_host->pdev->dev, "pwr-gpio provided\n");

	vendor_host->ggc.det_gpio = of_get_named_gpio(np, "cd-gpios", 0);
	if (!gpio_is_valid(vendor_host->ggc.det_gpio))
		dev_err(&vendor_host->pdev->dev, "no det-gpio provided !\n");
	else
		dev_info(&vendor_host->pdev->dev, "det-gpio provided\n");

	if (gpio_is_valid(vendor_host->ggc.pwr_gpio)) {
		devm_gpio_request_one(&vendor_host->pdev->dev, vendor_host->ggc.pwr_gpio,
			GPIOF_OUT_INIT_LOW, "sprd-1-pwr");
		pr_info("%s: pwr_gpio pin %d\n",
			mmc_hostname(host->mmc), vendor_host->ggc.pwr_gpio);
	} else {
		pr_err("%s: no pwr_gpio provided\n",
			mmc_hostname(host->mmc));
	}

	if (gpio_is_valid(vendor_host->ggc.det_gpio)) {
		devm_gpio_request_one(&vendor_host->pdev->dev, vendor_host->ggc.det_gpio,
		GPIOF_DIR_IN, "sprd-1-det");
		pr_info("%s: detect_gpio pin %d\n",
			mmc_hostname(host->mmc), vendor_host->ggc.det_gpio);
	} else {
		pr_err("%s: no detect_gpio provided\n",
			mmc_hostname(host->mmc));
	}
}

int card_deselect_card(struct sdhci_host *host)
{
	int ret = -1;
	int err;
	struct mmc_host *mmc = host->mmc;
	struct mmc_command cmd = { 0 };

	cmd.opcode = MMC_SELECT_CARD;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_NONE | MMC_CMD_AC;

	err = mmc_wait_for_cmd(mmc, &cmd, 3);
	if (err)
		pr_err("BHT ERR:%s: ---- CMD7 FAIL: err = %d ----\n", __func__, err);
	else
		ret = 0;

	return ret;
}

bool enter_exit_emulator_mode(struct sdhci_host *host, bool b_enter)
{
	bool ret = FALSE;
	u8 times = b_enter ? 2 : 1;
	u8 i = 0;

	for (i = 0; i < times; i++) {
		ret = card_deselect_card(host);
		if (ret)
			break;
	}
	return ret;
}

static bool _gg_emulator_read_only(struct sdhci_host *host,
				   u8 *in_data, u32 datalen)
{
	struct mmc_host *mmc = host->mmc;
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);
	int rc = 0;
	u8 *data1 = kzalloc(PAGE_SIZE, GFP_KERNEL);
	struct mmc_request mrq = { 0 };
	struct mmc_command cmd = { 0 };
	struct mmc_data data = { 0 };
	struct scatterlist sg;

	if (!data1) {
		pr_info("BHT MSG:gg read no memory\n");
		rc = -ENOMEM;
		goto out;
	}

	sg_init_one(&sg, data1, 512);

	cmd.opcode = MMC_READ_SINGLE_BLOCK;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
	data.blksz = 512;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;
	data.timeout_ns = 1000 * 1000 * 1000;	/* 1 sec */
	data.sg = &sg;
	data.sg_len = 1;
	mrq.cmd = &cmd;
	mrq.data = &data;
	mrq.stop = NULL;

	mmc_wait_for_req(mmc, &mrq);
	memcpy(in_data, data1, datalen);

	kfree(data1);

	if (cmd.error || data.error) {

		if (cmd.error) {
			if (cmd.error == -EILSEQ) {
				pr_err("BHT ERR:cmd error 0x%x\n", cmd.error);
				vendor_host->ggc.sdr50_notuning_crc_error_flag = 1;
			}
		}

		if (data.error) {
			if (data.error == -EILSEQ) {
				pr_err("BHT ERR:data error 0x%x\n", data.error);
				vendor_host->ggc.sdr50_notuning_crc_error_flag = 1;
			}
		}
		rc = -1;
		goto out;
	}

out:
	return rc;
}

static int gg_select_card_spec(struct sdhci_host *host)
{
	int err;
	struct mmc_command cmd = { 0 };
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	cmd.opcode = MMC_SELECT_CARD;
	/* Linden code for JLINDEN-5088 by wanghan14 at 2023/03/15 start */
	if (host->mmc->card) {
		cmd.arg = host->mmc->card->rca << 16;
		cmd.flags = MMC_RSP_R1B | MMC_CMD_AC;
	} else if (vendor_host->ggc.card){
		cmd.arg = vendor_host->ggc.card->rca << 16;
		cmd.flags = MMC_RSP_R1B | MMC_CMD_AC;
	} else {
		pr_err("Can't get valid RCA, deselect card instead of select card\n", __func__);
	}
	/* Linden code for JLINDEN-5088 by wanghan14 at 2023/03/15 end */
	err = mmc_wait_for_cmd(host->mmc, &cmd, 0);
	if (err) {
		if (-EILSEQ == err) {
			host_cmddat_line_reset(host);
			{
				struct mmc_command cmd = { 0 };

				cmd.opcode = 5;
				cmd.arg = 0;
				cmd.flags =
				    MMC_RSP_SPI_R4 | MMC_RSP_R4 | MMC_CMD_BCR;

				mmc_wait_for_cmd(host->mmc, &cmd, 0);
			}
			pr_err("BHT ERR:%s: CMD7 CRC\n", __func__);
			host_cmddat_line_reset(host);
			return 0;
		}
		if (-ETIMEDOUT == err) {
			pr_err("BHT ERR:%s: CMD7 timeout\n", __func__);
			host_cmddat_line_reset(host);
			return err;
		}
		return 0;
	}

	return 0;
}

void dump_u32_buf(u8 *ptb, int len)
{
	int i = 0;
	u32 *tb = (u32 *)ptb;

	for (i = 0; i < len / 4; i++)
		pr_info("BHT MSG: [%d]:%08xh\n", i, tb[i]);
}

bool gg_emulator_read_ext(struct sdhci_host *host, bool *card_status,
					bool *read_status, u8 *data, u32 datalen)
{
	bool ret = FALSE;
	bool card_ret = TRUE;
	bool rd_ret = FALSE;

	ret = (enter_exit_emulator_mode(host, TRUE) == 0) ? TRUE : FALSE;
	if (!ret)
		goto exit;

	rd_ret = (_gg_emulator_read_only(host, data, datalen) == 0) ? TRUE : FALSE;

	ret = (enter_exit_emulator_mode(host, FALSE) == 0) ? TRUE : FALSE;
	if (!ret)
		goto exit;

	card_ret = (gg_select_card_spec(host) == 0) ? TRUE : FALSE;

	if (!rd_ret)
		pr_err("BHT ERR:GGC read status error\n");

exit:
	if (!card_ret) {
		pr_err("BHT ERR:GGC Emulator exit Fail!!\n");
		ret = FALSE;
	}

	if (card_status)
		*card_status = ret;

	if (read_status)
		*read_status = rd_ret;

	if (rd_ret && !ret)
		pr_err("BHT ERR:data read ok, but exit NG\n");
	else if (!rd_ret && ret)
		pr_err("BHT ERR:data read NG, but exit ok\n");

	return ret;
}

static void _status_bit_2_bt(int tar, int *byt, int *bit)
{
	*byt = tar / 8;
	*bit = tar % 8;
}

static u32 _read_status_data_read_register(u8 *cfg, struct t_gg_reg_strt *bts)
{
	u32 rv = 0;
	u32 msk = bts->mask;
	int byt = 0, bit = 0;
	int i = 0;

	do {
		_status_bit_2_bt(bts->ofs + i, &byt, &bit);
		if (cfg[byt] & (1 << bit))
			rv |= 1 << i;

		i++;
		msk >>= 1;
		if (msk == 0)
			break;
	} while (1);
	return rv;
}

bool ggc_read_registers_ext(struct sdhci_host *host,
		bool *card_status, bool *read_status,
		struct t_gg_reg_strt *gg_reg_arr, u8 num)
{
	u8 get_idx = 0;
	bool ret = FALSE;
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);
	struct ggc_platform_t *ggc = &vendor_host->ggc;

	if (read_status)
		*read_status = FALSE;
	if (card_status)
		*card_status = FALSE;
    // read ggc register
	memset(ggc->_cur_read_buf, 0, 512);
	ret = gg_emulator_read_ext(host, card_status, read_status, ggc->_cur_read_buf, 512);
	if (read_status == FALSE)
		goto exit;

    // read the offset bits value
	for (get_idx = 0; get_idx < num; get_idx++)
		(gg_reg_arr + get_idx)->value =
			_read_status_data_read_register(ggc->_cur_read_buf, (gg_reg_arr + get_idx));

exit:
	return ret;
}

bool gg_emulator_read(struct sdhci_host *host, u8 *data, u32 datalen)
{
	bool ret = FALSE;
	bool rd_ret = FALSE;

	ret = enter_exit_emulator_mode(host, TRUE);
	if (ret)
		goto exit;

	rd_ret = _gg_emulator_read_only(host, data, datalen);

	ret = enter_exit_emulator_mode(host, FALSE);
	if (ret)
		goto exit;

	ret = gg_select_card_spec(host);

exit:
	if (rd_ret)
		pr_err("BHT ERR:GGC read status error\n");

	if (ret)
		pr_err("BHT ERR:GGC Emulator exit Fail!!\n");

	if (rd_ret == 0 && ret) {
		pr_err("BHT ERR:data read ok, but exit NG\n");
		ret = 0;
	}

	if (rd_ret && ret == 0) {
		pr_err("BHT ERR:data read NG, but exit ok\n");
		ret = -1;
	}

	return ret ? FALSE : TRUE;
}

static bool _ggc_emulator_write_only(struct sdhci_host *host,
				     u8 *in_data, u32 datalen)
{
	struct mmc_host *mmc = host->mmc;
	int rc = 0;
	u8 *data1 = kzalloc(PAGE_SIZE, GFP_KERNEL);
	struct mmc_request mrq = { 0 };
	struct mmc_command cmd = { 0 };
	struct mmc_data data = { 0 };
	struct scatterlist sg;
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	if (!data1) {
		pr_info("BHT MSG:gg write no memory\n");
		rc = -ENOMEM;
		goto out;
	}

	memcpy(data1, in_data, datalen);
	sg_init_one(&sg, data1, 512);

	cmd.opcode = MMC_WRITE_BLOCK;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
	data.blksz = 512;
	data.blocks = 1;
	data.flags = MMC_DATA_WRITE;
	data.timeout_ns = 1000 * 1000 * 1000;	/* 1 sec */
	data.sg = &sg;
	data.sg_len = 1;
	mrq.cmd = &cmd;
	mrq.data = &data;
	mrq.stop = NULL;

	mmc_wait_for_req(mmc, &mrq);

	if (cmd.error == -EILSEQ)
		vendor_host->ggc.sdr50_notuning_crc_error_flag = 1;

	kfree(data1);
out:
	return rc;
}

bool gg_emulator_write(struct sdhci_host *host, u8 *data, u32 datalen)
{
	bool ret = FALSE;
	bool wr_ret = FALSE;
	u32 i = 0;
	u32 reg;

	ret = enter_exit_emulator_mode(host, TRUE);
	if (ret)
		goto exit;

	pr_info("BHT MSG:%s: dump config data\n", __func__);
	for (i = 0; i < (datalen/sizeof(u32)); i++)	{
		memcpy(&reg, data+i*sizeof(u32), sizeof(u32));
		pr_info("BHT MSG:\tggc_reg32[%03d]=0x%08x\n", i, reg);
	}

	_ggc_emulator_write_only(host, data, datalen);
	wr_ret = TRUE;

	ret = enter_exit_emulator_mode(host, FALSE);
	if (ret)
		goto exit;

	//ret = gg_select_card_spec(host);
	ret = (gg_select_card_spec(host) == 0) ? TRUE : FALSE;
exit:
	if (wr_ret == FALSE)
		ret = FALSE;

	if (ret == FALSE)
		pr_err("BHT ERR:%s: GGC Emulator Write Fail!!\n", __func__);

	return ret;
}

void get_gg_reg_def(struct sdhci_host *host, u8 *data)
{
	u32 gg_sw_def[16] = GGC_CFG_DATA;

	memcpy(data, (u8 *)&(gg_sw_def[0]), sizeof(gg_sw_def));
}

bool get_gg_reg_cur(struct sdhci_host *host, u8 *data,
		    struct t_gg_reg_strt *gg_reg_arr, u8 num)
{
	u8 get_idx = 0;
	bool ret = FALSE;

	// read ggc register
	memset(data, 0, 512);
	ret = gg_emulator_read(host, data, 512);

	if (ret == FALSE)
		goto exit;

	// read the offset bits value
	for (get_idx = 0; get_idx < num; get_idx++) {
		(gg_reg_arr + get_idx)->value =
		    read_ram_bits_ofs_mask(data, (gg_reg_arr + get_idx));
	}
exit:
	return ret;
}

bool chg_gg_reg(struct sdhci_host *host, u8 *data, struct t_gg_reg_strt *gg_reg_arr,
		u8 num)
{
	u8 chg_idx = 0;

	memset(data, 0, 512);
	get_gg_reg_def(host, data);

	for (chg_idx = 0; chg_idx < num; chg_idx++) {
		// modify the ggc register bit value
		cfg_write_bits_ofs_mask(data, (gg_reg_arr + chg_idx),
					(gg_reg_arr + chg_idx)->value);
	}

	// write ggc register
	return gg_emulator_write(host, data, 512);
}

void chg_gg_reg_cur_val(struct ggc_platform_t  *ggc, u8 *data,
	struct t_gg_reg_strt *gg_reg_arr, u8 num, bool b_sav_chg)
{
	u8 chg_idx = 0;

	for (chg_idx = 0; chg_idx < num; chg_idx++) {
		// modify the ggc register bit value
		cfg_write_bits_ofs_mask(data, (gg_reg_arr + chg_idx),
					(gg_reg_arr + chg_idx)->value);
	}

	if (b_sav_chg)
		set_gg_reg_cur_val(ggc, data, 64);
}

void generate_gg_reg_val(u8 *data, struct t_gg_reg_strt *gg_reg_arr, u8 num)
{
	u8 chg_idx = 0;

	for (chg_idx = 0; chg_idx < num; chg_idx++) {
		// modify the ggc register bit value
		cfg_write_bits_ofs_mask(data, (gg_reg_arr + chg_idx),
					(gg_reg_arr + chg_idx)->value);
	}
}

void log_bin(u32 n)
{
	int i = 0;
	u8 tb[33] = { 0 };

	for (i = 0; i < 32; i++) {
		if (n & (1 << i))
			tb[i] = '1';
		else
			tb[i] = '0';
	}
	pr_info("BHT MSG:bin:%s\n", tb);
}

void phase_str(u8 *tb, u32 n)
{
	int i = 0;

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (n & (1 << i))
			tb[i] = '1';
		else
			tb[i] = '0';
	}
	tb[TUNING_PHASE_SIZE] = 0;
}

int get_bit_number(u32 n)
{
	int i = 0;
	int cnt = 0;

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (n & (1 << i))
			cnt++;
	}
	return cnt;
}

bool gg_emulator_write_ext(struct sdhci_host *host, bool *card_status, u8 *data, u32 datalen)
{
	bool ret = FALSE;
	bool wr_ret = FALSE;
	//u32 reg = 0;
	//int i = 0;

	ret = enter_exit_emulator_mode(host, TRUE);
	if (ret)
		goto exit;

	/*pr_info("BHT MSG :%s: dump config data \n", __func__);
	for(i = 0;i < (datalen/sizeof(u32)); i ++) {
		memcpy(&reg, data + i*sizeof(u32),  sizeof(u32));
		pr_info("BHT MSG:\tgcc_reg32[%03d]=0x%08x\n",i, reg);
	}*/

	_ggc_emulator_write_only(host, data, datalen);
	wr_ret = TRUE;

	ret = enter_exit_emulator_mode(host, FALSE);
	if (ret)
		goto exit;

	ret = (gg_select_card_spec(host) == 0) ? TRUE : FALSE;
	if (ret == FALSE) {
		if (card_status)
			*card_status = FALSE;
	}

exit:
	if (wr_ret == FALSE)
		ret = FALSE;

	if (ret == FALSE)
		pr_err("BHT ERR:%s: GGC Emulator Write Fail!!\n", __func__);

	return ret;
}

bool ggc_set_output_tuning_phase_ext(struct sdhci_host *host, bool *card_status, int sela, int selb)
{
	bool ret = TRUE;
	u8 data[512] = { 0 };
	struct t_gg_reg_strt gg_reg_arr[8];
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	get_gg_reg_cur_val(&vendor_host->ggc, data, 64);
	memcpy(&gg_reg_arr[0], &vendor_host->ggc.dll_sela_100m_cfg, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[1], &vendor_host->ggc.dll_sela_200m_cfg, sizeof(struct t_gg_reg_strt));
	gg_reg_arr[0].value = sela;
	gg_reg_arr[1].value = sela;
	memcpy(&gg_reg_arr[2], &vendor_host->ggc.dll_selb_100m_cfg, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[3], &vendor_host->ggc.dll_selb_200m_cfg, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[4], &vendor_host->ggc.dll_selb_100m_cfg_en, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[5], &vendor_host->ggc.dll_selb_200m_cfg_en, sizeof(struct t_gg_reg_strt));
	gg_reg_arr[2].value = selb;
	gg_reg_arr[3].value = selb;
	gg_reg_arr[4].value = 1;
	gg_reg_arr[5].value = 1;
	memcpy(&gg_reg_arr[6], &vendor_host->ggc.internl_tuning_en_100m,
		  sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[7], &vendor_host->ggc.internl_tuning_en_200m,
		  sizeof(struct t_gg_reg_strt));
	gg_reg_arr[6].value = 1;
	gg_reg_arr[7].value = 1;
	if (card_status)
		*card_status = TRUE;
	chg_gg_reg_cur_val(&vendor_host->ggc, data, gg_reg_arr, 8, TRUE);
	ret = gg_emulator_write_ext(host, card_status, data, 512);
	return ret;
}

bool ggc_set_output_tuning_phase(struct sdhci_host *host, int sela, int selb)
{
	bool ret = TRUE;
	u8 data[512] = { 0 };
	struct t_gg_reg_strt gg_reg_arr[8];
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	get_gg_reg_cur_val(&vendor_host->ggc, data, 64);
	memcpy(&gg_reg_arr[0], &vendor_host->ggc.dll_sela_100m_cfg, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[1], &vendor_host->ggc.dll_sela_200m_cfg, sizeof(struct t_gg_reg_strt));
	gg_reg_arr[0].value = sela;
	gg_reg_arr[1].value = sela;
	memcpy(&gg_reg_arr[2], &vendor_host->ggc.dll_selb_100m_cfg, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[3], &vendor_host->ggc.dll_selb_200m_cfg, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[4], &vendor_host->ggc.dll_selb_100m_cfg_en, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[5], &vendor_host->ggc.dll_selb_200m_cfg_en, sizeof(struct t_gg_reg_strt));
	gg_reg_arr[2].value = selb;
	gg_reg_arr[3].value = selb;
	gg_reg_arr[4].value = 1;
	gg_reg_arr[5].value = 1;
	memcpy(&gg_reg_arr[6], &vendor_host->ggc.internl_tuning_en_100m,
		  sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[7], &vendor_host->ggc.internl_tuning_en_200m,
		  sizeof(struct t_gg_reg_strt));
	gg_reg_arr[6].value = 1;
	gg_reg_arr[7].value = 1;
	chg_gg_reg_cur_val(&vendor_host->ggc, data, gg_reg_arr, 8, TRUE);
	ret = gg_emulator_write(host, data, 512);
	return ret;
}

bool gg_fix_output_tuning_phase(struct sdhci_host *host, int sela, int selb)
{
	u8 data[512] = { 0 };
	struct t_gg_reg_strt gg_reg_arr[10];
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	pr_info("BHT MSG:### %s - sela dll: %x, selb dll: %x\n", __func__, sela,
		selb);

	get_gg_reg_cur_val(&vendor_host->ggc, data, 64);

	memcpy(&gg_reg_arr[0], &vendor_host->ggc.dll_sela_100m_cfg, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[1], &vendor_host->ggc.dll_sela_200m_cfg, sizeof(struct t_gg_reg_strt));
	gg_reg_arr[0].value = sela;
	gg_reg_arr[1].value = sela;
	memcpy(&gg_reg_arr[2], &vendor_host->ggc.dll_selb_100m_cfg, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[3], &vendor_host->ggc.dll_selb_200m_cfg, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[4], &vendor_host->ggc.dll_selb_100m_cfg_en, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[5], &vendor_host->ggc.dll_selb_200m_cfg_en, sizeof(struct t_gg_reg_strt));
	gg_reg_arr[2].value = selb;
	gg_reg_arr[3].value = selb;
	gg_reg_arr[4].value = 1;
	gg_reg_arr[5].value = 1;
	memcpy(&gg_reg_arr[6], &vendor_host->ggc.internl_tuning_en_100m,
		  sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[7], &vendor_host->ggc.internl_tuning_en_200m,
		  sizeof(struct t_gg_reg_strt));
	gg_reg_arr[6].value = 0;
	gg_reg_arr[7].value = 0;

	chg_gg_reg_cur_val(&vendor_host->ggc, data, gg_reg_arr, 8, TRUE);

	return gg_emulator_write(host, data, 512);
}

void gen_array_data(u32 low32, u32 high32, u32 *ptw)
{
	u8 tu_res_per[6][TUNING_PHASE_SIZE];
	u8 i = 0, j = 0;
	u8 i_mode = 0;
	u32 tw = 0;

	memset(tu_res_per, 1, sizeof(tu_res_per));
	for (i = 0; i < 64; i++) {
		u32 tmp_data = (i < 32) ? low32 : high32;

		tu_res_per[i / TUNING_PHASE_SIZE][i % TUNING_PHASE_SIZE] =
		    (tmp_data & (1 << (i % 32))) >> (i % 32);
	}

	for (i_mode = 0; i_mode < TUNING_PHASE_SIZE; i_mode++) {
		for (j = 0; j < 6; j++) {
			if (tu_res_per[j][i_mode] != 0)
				tw |= (1 << i_mode);
			else {
				tw &= ~(1 << i_mode);
				break;
			}
		}
	}
	if (ptw)
		*ptw = tw;
}

bool sw_calc_tuning_result(struct sdhci_host *host, u32 *tx_selb,
			   u32 *all_selb, u64 *raw_tx_selb)
{
	bool ret = FALSE;
	u8 data[512] = { 0 };
	u32 selb_status_tx_low32 = 0, selb_status_tx_high32 = 0;
	u32 selb_status_ggc_low32 = 0, selb_status_ggc_high32 = 0;
	struct t_gg_reg_strt gg_reg_arr[6];
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	memcpy(&gg_reg_arr[0], &vendor_host->ggc.pha_stas_tx_low32, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[1], &vendor_host->ggc.pha_stas_tx_high32, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[2], &vendor_host->ggc.pha_stas_rx_low32, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[3], &vendor_host->ggc.pha_stas_rx_high32, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[4], &vendor_host->ggc.dll_sela_after_mask, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[5], &vendor_host->ggc.dll_selb_after_mask, sizeof(struct t_gg_reg_strt));

	ret = get_gg_reg_cur(host, data, gg_reg_arr, 6);

	if (ret == TRUE) {
		selb_status_tx_low32 = gg_reg_arr[0].value;
		pr_info("BHT MSG:[205-236]:\n");
		log_bin(selb_status_tx_low32);
		pr_info("BHT MSG:[237-268]:\n");
		selb_status_tx_high32 = gg_reg_arr[1].value;
		log_bin(selb_status_tx_high32);

		pr_info("BHT MSG:[14-45]:\n");
		selb_status_ggc_low32 = gg_reg_arr[2].value;
		log_bin(selb_status_ggc_low32);
		pr_info("BHT MSG:[46-77]:\n");
		selb_status_ggc_high32 = gg_reg_arr[3].value;
		log_bin(selb_status_ggc_high32);
		pr_info("BHT MSG:dll sela after mask=%xh\n", gg_reg_arr[4].value);
		pr_info("BHT MSG:dll selb after mask=%xh\n", gg_reg_arr[5].value);

		if (raw_tx_selb) {
			*raw_tx_selb = gg_reg_arr[1].value;
			(*raw_tx_selb) <<= 32;
			*raw_tx_selb += gg_reg_arr[0].value;
			pr_info("BHT MSG:raw_tx_selb:%llxh\n", *raw_tx_selb);
		}

		if (tx_selb) {
			gen_array_data(gg_reg_arr[0].value, gg_reg_arr[1].value,
				       tx_selb);
			pr_info("BHT MSG:tx_selb:%xh\n", *tx_selb);
		}
		if (all_selb) {
			gen_array_data(gg_reg_arr[2].value, gg_reg_arr[3].value,
				       all_selb);
			pr_info("BHT MSG:all_selb:%xh\n", *all_selb);
		}
	}

	return ret;
}

bool gg_tuning_result(struct sdhci_host *host, u32 *tx_selb, u32 *all_selb,
			u64 *raw_tx_selb)
{

	host_cmddat_line_reset(host);
	return sw_calc_tuning_result(host, tx_selb, all_selb, raw_tx_selb);
}

u64 GENERATE_64_IDX_VALUE(int sft)
{
	u64 val = 1;

	return val << sft;
}

bool is_bus_mode_sdr104(struct sdhci_host *host)
{
	bool ret = FALSE;

	if (host->timing == MMC_TIMING_UHS_SDR104)
		ret = TRUE;

	return ret;
}

bool _check_bus_mode(struct sdhci_host *host)
{
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);
	struct ggc_platform_t *ggc = &vendor_host->ggc;

	if (is_bus_mode_sdr104(host))
		ggc->cur_bus_mode = &vendor_host->ggc.sdr104;
	else
		ggc->cur_bus_mode = &vendor_host->ggc.sdr50;

	return true;
}

void tx_selb_failed_history_update(struct sdhci_host *host, u32 val)
{
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	_check_bus_mode(host);

	vendor_host->ggc.cur_bus_mode->tx_selb_failed_history &= val;
}

void tx_selb_failed_history_reset(struct sdhci_host *host)
{
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	vendor_host->ggc.sdr50.tx_selb_failed_history = BIT_PASS_MASK;
	vendor_host->ggc.sdr104.tx_selb_failed_history = BIT_PASS_MASK;
}

u32 tx_selb_failed_history_get(struct sdhci_host *host)
{
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	_check_bus_mode(host);

	return vendor_host->ggc.cur_bus_mode->tx_selb_failed_history;
}

void tx_selb_failed_tb_update(struct sdhci_host *host, int sela, u32 val)
{
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	_check_bus_mode(host);
	vendor_host->ggc.cur_bus_mode->tx_selb_tb[sela] &= val;
}

void tx_selb_failed_tb_reset(struct sdhci_host *host)
{
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	memset(&vendor_host->ggc.sdr104.tx_selb_tb, 0xff,
			sizeof(vendor_host->ggc.sdr104.tx_selb_tb));
	memset(&vendor_host->ggc.sdr50.tx_selb_tb, 0xff,
			sizeof(vendor_host->ggc.sdr50.tx_selb_tb));
}

u32 tx_selb_failed_tb_get(struct sdhci_host *host, int sela)
{
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);
	u32 value = 0;

	_check_bus_mode(host);

	if (is_bus_mode_sdr104(host))
		value = vendor_host->ggc.sdr104.tx_selb_tb[sela];
	else
		value = vendor_host->ggc.sdr50.tx_selb_tb[sela];

	return value;
}

void all_selb_failed_tb_update(struct sdhci_host *host, int sela, u32 val)
{
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	_check_bus_mode(host);
	vendor_host->ggc.cur_bus_mode->all_selb_tb[sela] &= val;
}

void all_selb_failed_tb_reset(struct sdhci_host *host)
{
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	memset(vendor_host->ggc.sdr104.all_selb_tb, 0xff,
			sizeof(vendor_host->ggc.sdr104.all_selb_tb));
	memset(vendor_host->ggc.sdr50.all_selb_tb, 0xff,
			sizeof(vendor_host->ggc.sdr50.all_selb_tb));
}

u32 all_selb_failed_tb_get(struct sdhci_host *host, int sela)
{
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);
	u32 val;

	_check_bus_mode(host);

	val = vendor_host->ggc.cur_bus_mode->all_selb_tb[sela];

	return val;
}

void chk_phase_window(u8 *tuning_win, u8 *mid_val, u8 *max_pass_win)
{
	u8 tuning_pass[TUNING_PHASE_SIZE + 32];
	u8 tuning_pass_start[TUNING_PHASE_SIZE + 32];
	u8 tuning_pass_num_max = 0;
	u8 first_0 = 0;
	u8 i = 0, j = 0;
	u8 i_mode = 0, selb_mode = 0;

	memset(tuning_pass, 1, sizeof(tuning_pass));
	memset(tuning_pass_start, 1, sizeof(tuning_pass_start));

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (tuning_win[i] == 0) {
			first_0 = i;
			break;
		}
	}

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		i_mode = (first_0 + i) % TUNING_PHASE_SIZE;
		if (tuning_win[i_mode] == 1)
			tuning_pass[j]++;
		else if (tuning_pass[j])
			j++;
		if (tuning_pass[j] == 1)
			tuning_pass_start[j] = i_mode;
	}

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (tuning_pass_num_max < tuning_pass[i]) {
			tuning_pass_num_max = tuning_pass[i];
			i_mode = i;
		}
	}

	if (tuning_pass_num_max == 0)
		pr_err
		    ("###### Get max pass window fail, there is no any pass phase!!\n");
	else {
		*max_pass_win = tuning_pass_num_max - 1;
		tuning_pass_num_max /= 2;
		selb_mode = tuning_pass_start[i_mode] + tuning_pass_num_max;
		if ((*max_pass_win % 2 == 0))
			selb_mode += 1;
		selb_mode %= TUNING_PHASE_SIZE;
	}

	*mid_val = selb_mode;
}

void dump_array(u8 *tb)
{
	int i = 0;
	u8 str[12] = { 0 };

	for (i = 0; i < TUNING_PHASE_SIZE; i++)
		str[i] = tb[i] + '0';

}

void bits_generate_array(u8 *tb, u32 v)
{
	int i = 0;

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if ((v & (1 << i)))
			tb[i] = 1;
		else
			tb[i] = 0;
	}
	dump_array(tb);
}

void chk_arr_max_win(u8 *tuning_win, u8 first_i, u8 *mid_val,
				u8 *first_val, u8 *max_pass_win, struct chk_type_t type)
{
	u8 tuning_pass[TUNING_PHASE_SIZE];
	u8 tuning_pass_start[TUNING_PHASE_SIZE];
	u8 tuning_pass_num_max = 0;
	u8 first_0 = 0;
	u8 i = 0, j = 0;
	u8 i_mode = 0, selb_mode = 0;

	memset(tuning_pass, 1, sizeof(tuning_pass));
	memset(tuning_pass_start, 1, sizeof(tuning_pass_start));

	if (type.first_valid)
		first_0 = first_i;
	else {
		for (i = 0; i < TUNING_PHASE_SIZE; i++) {
			if (tuning_win[i] == 0) {
				first_0 = i;
				break;
			}
		}
	}

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		i_mode = (first_0 + i) % TUNING_PHASE_SIZE;
		if (tuning_win[i_mode] == 1)
			tuning_pass[j]++;
		else if (tuning_pass[j])
			j++;
		if (tuning_pass[j] == 1)
			tuning_pass_start[j] = i_mode;
	}

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (tuning_pass_num_max < tuning_pass[i]) {
			tuning_pass_num_max = tuning_pass[i];
			i_mode = i;
		}
	}

	if (tuning_pass_num_max == 0)
		pr_err
		    ("###### Get max pass window fail, there is no any pass phase!!\n");
	else {
		*max_pass_win = tuning_pass_num_max - 1;
		tuning_pass_num_max /= 2;
		if (first_val)
			*first_val = tuning_pass_start[i_mode];
		selb_mode = tuning_pass_start[i_mode] + tuning_pass_num_max;
		if ((*max_pass_win % 2 == 0) && (type.right_valid)
		    )
			selb_mode += 1;
		selb_mode %= TUNING_PHASE_SIZE;
	}

	*mid_val = selb_mode;
}

void no_fail_p(u8 *tuning_win, u8 *mid_val, u8 *max_pass_win, u8 *first_val)
{
	struct chk_type_t type;
	u8 first_0 = 0;

	memset((u8 *)&type, 0, sizeof(struct chk_type_t));

	type.first_valid = 0;
	type.right_valid = 1;
	type.record_valid = 0;

	chk_arr_max_win(tuning_win, first_0, mid_val, first_val, max_pass_win,
			type);

}

static int ggc_get_selx_weight(u32 val)
{
	int i = 0;
	int cnt = 0;

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (GET_IDX_VALUE(val, i))
			cnt++;
	}
	return cnt;
}

void tx_selb_calculate_valid_phase_range(u32 val, int *start,
						int *pass_cnt)
{
	int i = 0, flg = 0;
	*pass_cnt = ggc_get_selx_weight(val);
	for (i = 0; i < (TUNING_PHASE_SIZE * 2); i++) {
		if ((GET_TRUNING_RING_IDX_VALUE(val, i)) == 0 && (flg == 0))
			flg = 1;
		if ((flg == 1) && GET_TRUNING_RING_IDX_VALUE(val, i)) {
			*start = TRUNING_RING_IDX(i);
			break;
		}
	}
}

bool ggc_update_default_selb_phase_tuning_cnt(struct sdhci_host *host, int selb,
					      int tuning_cnt)
{
	struct t_gg_reg_strt gg_reg_arr[3];
	u8 data[512];
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	get_gg_reg_cur_val(&vendor_host->ggc, data, 64);

	pr_info("BHT MSG:%s selb:%xh,tuning_cnt:%xh\n", __func__, selb,
		 tuning_cnt);
	memcpy(&gg_reg_arr[0], &vendor_host->ggc.dll_selb_100m_cfg, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[1], &vendor_host->ggc.dll_selb_200m_cfg, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[2], &vendor_host->ggc.cmd19_cnt_cfg, sizeof(struct t_gg_reg_strt));

	gg_reg_arr[0].value = selb;
	gg_reg_arr[1].value = selb;
	gg_reg_arr[2].value = tuning_cnt;
	chg_gg_reg_cur_val(&vendor_host->ggc, data, gg_reg_arr, 3, TRUE);

	return TRUE;
}

static void _ggc_update_cur_setting_for_sw_selb_tuning(struct sdhci_host *host,
						       u32 val)
{
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);
	int start = 0, pass_cnt = 0;

	tx_selb_calculate_valid_phase_range(val, &start, &pass_cnt);
	pr_info("BHT MSG:%s %x %x %x\n", __func__, val, start, pass_cnt);
	ggc_update_default_selb_phase_tuning_cnt(host, start, pass_cnt);	//update
	vendor_host->ggc.ggc_sw_selb_tuning_first_selb = start;
}


int sdhci_bht_sdr104_execute_tuning(struct sdhci_host *host, u32 opcode)
{
#if PLATFORM_845
	return sdhci_vendor_execute_tuning(host, opcode);
#else
	struct mmc_host *mmc = host->mmc;

	return sdhci_vendor_execute_tuning(mmc, opcode);
#endif
}

int sdhci_bht_sdr50_execute_tuning(struct sdhci_host *host, u32 opcode)
{
#ifdef MSM_HOST_USED
	u8 phase, *data_buf;
	int size = 64;
	int rc = 0;
	struct mmc_host *mmc = host->mmc;
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	pr_debug("%s: Enter %s\n", mmc_hostname(mmc), __func__);

	data_buf = kmalloc(size, GFP_KERNEL);
	if (!data_buf) {
		pr_info("BHT MSG:tuning no memory\n");
		rc = -ENOMEM;
		goto out;
	}

	phase = 0;
	do {
		struct mmc_command cmd = { 0 };
		struct mmc_data data = { 0 };
		struct mmc_request mrq = {
			.cmd = &cmd,
			.data = &data
		};
		struct scatterlist sg;

		cmd.opcode = opcode;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;

		data.blksz = size;
		data.blocks = 1;
		data.flags = MMC_DATA_READ;
		data.timeout_ns = 30 * 1000 * 1000;	/* 30ms */

		data.sg = &sg;
		data.sg_len = 1;
		sg_init_one(&sg, data_buf, size);
		memset(data_buf, 0, size);
		host_cmddat_line_reset(host);
		mmc_wait_for_req(mmc, &mrq);
		usleep_range(1000,1200);
		if (cmd.error) {
			if (cmd.error == -EILSEQ)
				vendor_host->ggc.sdr50_notuning_crc_error_flag = 1;
			if (cmd.error == -ETIMEDOUT && phase == 0) {
				pr_err("BHT ERR:cmd19 timeout\n");
				rc = -ETIMEDOUT;
				goto kfree;
			}
		}

		if (data.error) {
			if (data.error == -EILSEQ)
				vendor_host->ggc.sdr50_notuning_crc_error_flag = 1;
		}
	} while (++phase < 16);

kfree:
	kfree(data_buf);
out:

	return rc;
#else
	return sdhci_bht_sdr104_execute_tuning(host, opcode);
#endif
}

int sd_tuning_sw(struct sdhci_host *host)
{
	int ret = 0;

	if (is_bus_mode_sdr104(host))
		ret = sdhci_bht_sdr104_execute_tuning(host, 0x13);
	else
		ret = sdhci_bht_sdr50_execute_tuning(host, 0x13);

	return ret;
}

bool sd_gg_tuning_status(struct sdhci_host *host,
			 u32 *tx_selb, u32 *all_selb, u64 *raw_tx_selb,
			 bool *status_ret, bool *first_cmd19_status)
{
	bool ret = TRUE;
	int err = sd_tuning_sw(host);

	ret = err == 0 ? TRUE : FALSE;
	if (err == -ETIMEDOUT) {
		ret = FALSE;
		if (first_cmd19_status)
			*first_cmd19_status = false;
		goto exit;
	}

	if (status_ret) {
		*status_ret =
			gg_tuning_result(host, tx_selb, all_selb,
						raw_tx_selb);
	} else {
		gg_tuning_result(host, 0, 0, 0);
	}

exit:
	return ret;
}

bool ggc_sd_tuning(struct sdhci_host *host,
			 bool *first_cmd19_status)
{
	bool ret = TRUE;
	int err = sd_tuning_sw(host);

	ret = err == 0 ? TRUE : FALSE;
	if (err == -ETIMEDOUT) {
		ret = FALSE;
		if (first_cmd19_status)
			*first_cmd19_status = false;
		goto exit;
	}

exit:
	return ret;
}

int get_config_sela_setting(struct sdhci_host *host)
{
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	if (is_bus_mode_sdr104(host))
		return vendor_host->ggc.def_sela_200m;
	else
		return vendor_host->ggc.def_sela_100m;
}

int get_config_selb_setting(struct sdhci_host *host)
{
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	if (is_bus_mode_sdr104(host))
		return vendor_host->ggc.def_selb_200m;
	else
		return vendor_host->ggc.def_selb_100m;
}

void get_default_setting(struct sdhci_host *host, u8 *data)
{
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	vendor_host->ggc.def_sela_100m =
		cfg_read_bits_ofs_mask(data, &vendor_host->ggc.dll_sela_100m_cfg);
	vendor_host->ggc.def_sela_200m =
		cfg_read_bits_ofs_mask(data, &vendor_host->ggc.dll_sela_200m_cfg);
	vendor_host->ggc.def_selb_100m =
		cfg_read_bits_ofs_mask(data, &vendor_host->ggc.dll_selb_100m_cfg);
	vendor_host->ggc.def_selb_200m =
		cfg_read_bits_ofs_mask(data, &vendor_host->ggc.dll_selb_200m_cfg);
}

u32 get_all_sela_status(struct sdhci_host *host, u32 target_selb)
{
	u32 all_sela = 0;
	u32 all_selb = 0;
	int i = 0;

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		all_selb = all_selb_failed_tb_get(host, i);
		if (all_selb & (1 << target_selb))
			all_sela |= 1 << i;
	}
	return all_sela;
}

int get_pass_window_weight(u32 val)
{
	int i = 0;
	int cnt = 0;

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (GET_IDX_VALUE(val, i))
			cnt++;
	}
	return cnt;
}

int get_sela_nearby_pass_window(u32 sela, u32 base)
{

	int i = 0;
	int idx = base;
	int cnt = 0;

	if (GET_IDX_VALUE(sela, idx) == 0)
		return 0;

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (GET_IDX_VALUE(sela, idx)) {
			idx++;
			idx %= TUNING_PHASE_SIZE;
		} else {
			break;
		}
	}

	if (idx == 0)
		idx = 0xa;
	else
		idx--;

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (GET_IDX_VALUE(sela, idx)) {
			cnt++;
			if (idx == 0)
				idx = 0xa;
			else
				idx--;
		} else {
			break;
		}

	}
	return cnt;
}

int get_left_one_sel(int base)
{
	if (base == 0)
		return 0xa;
	else
		return base - 1;
}

int get_right_one_sel(int base)
{
	if (base == 0xa)
		return 0x0;
	else
		return base + 1;
}

int get_dif(int x, int y)
{
	int dif = 0;

	if (y > x)
		dif = y - x;
	else
		dif = x - y;

	return dif;
}

bool get_refine_sel(struct sdhci_host *host, u32 tga, u32 tgb, u32 *rfa,
		    u32 *rfb)
{
	int len_tb[TUNING_PHASE_SIZE] = { 0 };
	u32 sela = 0;
	int i = 0;
	u32 selb = 0;

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		sela = get_all_sela_status(host, selb);
		len_tb[selb] = get_sela_nearby_pass_window(sela, tga);
		selb++;
		selb %= TUNING_PHASE_SIZE;
	}
	pr_info("BHT MSG:tgb:%xh, tgb len:%x\n", tgb, len_tb[tgb]);
	if (len_tb[tgb] < 6) {
		int lft1 = get_left_one_sel(tgb);
		int lft2 = get_left_one_sel(lft1);
		int rt1 = get_right_one_sel(tgb);
		int rt2 = get_right_one_sel(rt1);

		if (0 == len_tb[lft1] || 0 == len_tb[lft2]) {
			len_tb[lft1] = 0;
			len_tb[lft2] = 0;
			pr_info("BHT MSG:over boundary case\n");
			goto next;
		}
		if (0 == len_tb[rt1] || 0 == len_tb[rt2]) {
			len_tb[rt1] = 0;
			len_tb[rt2] = 0;
			pr_info("BHT MSG:over boundary case\n");
			goto next;
		}
		{
			int dif = get_dif(len_tb[lft1], len_tb[tgb]);

			if (dif > 2)
				len_tb[lft1] = len_tb[tgb];

			dif = get_dif(len_tb[rt1], len_tb[tgb]);
			if (dif > 2)
				len_tb[rt1] = len_tb[tgb];

			dif = get_dif(len_tb[lft2], len_tb[tgb]);
			if (dif > 3)
				len_tb[lft2] = len_tb[tgb];

			dif = get_dif(len_tb[rt2], len_tb[tgb]);
			if (dif > 3)
				len_tb[rt2] = len_tb[tgb];

		}
next:
		if ((len_tb[lft1] + len_tb[lft2]) >=
		    (len_tb[rt1] + len_tb[rt2])) {
			*rfb = lft2;
			*rfa = get_left_one_sel(tga);
		} else {
			*rfb = rt2;
			*rfa = get_right_one_sel(tga);
		}

		sela = get_all_sela_status(host, *rfb);
		if (GET_IDX_VALUE(sela, *rfa) == 0) {
			pr_info
			    ("refine point is failed point, so no change\n");
			*rfa = tga;
			*rfb = tgb;
		}
	} else {
		*rfa = tga;
		*rfb = tgb;
	}
	pr_info("BHT MSG:tg sela:%xh, selb:%x\n", tga, tgb);
	pr_info("BHT MSG:rf sela:%xh, selb:%x\n", *rfa, *rfb);
	return TRUE;
}

int update_selb(struct sdhci_host *host, int target_selb)
{
	return target_selb;
}

static int ggc_get_10case_0_index(u32 val)
{
	int i = 0;

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (GET_IDX_VALUE(val, i) == 0
		    && GET_IDX_VALUE(val,
				     TRUNING_RING_IDX(i + TUNING_PHASE_SIZE -
						      1))) {
			return i;
		}
	}

	return -1;
}

static u32 ggc_get_01case_0_index(u32 val)
{
	int i = 0;

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (GET_IDX_VALUE(val, i) == 0
		    && GET_IDX_VALUE(val, TRUNING_RING_IDX(i + 1))) {
			return i;
		}
	}

	return -1;
}

static int ggc_get_next_1_index(u32 val, int pos)
{
	int i = 0;

	pos = pos % TUNING_PHASE_SIZE;
	for (i = 0; i < TUNING_PHASE_SIZE; i++)	{
		if (GET_IDX_VALUE(val, (pos+i)%TUNING_PHASE_SIZE))
			break;
	}
	if (GET_IDX_VALUE(val, (pos+i)%TUNING_PHASE_SIZE))
		return (pos+i)%TUNING_PHASE_SIZE;
	else
		return -1;
}

static u32 ggc_get_01case_1_index(u32 val)
{
	int i = 0;

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (GET_IDX_VALUE(val, i) == 0
			&& GET_IDX_VALUE(val, TRUNING_RING_IDX(i + 1))) {
			return TRUNING_RING_IDX(i + 1);
		}
	}

	return -1;
}

static int ggc_get_first_0_index(u32 val)
{
	int i = 0;

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (GET_IDX_VALUE(val, i) == 0)
			return i;
	}
	pr_info("BHT MSG:oops-not find 0 index\n");
	return 0;
}

static int _tx_selb_inject_policy(int tx_selb, int org_selb)
{
	int group_pos[TUNING_PHASE_SIZE+1][3];
	int group_cnt = 0;
	int max_len_group = 0;
	int max_len = 0;
	int i, j, cnt;
	int zero_start, zero_end, sel;

	if ((org_selb & BIT_PASS_MASK) != BIT_PASS_MASK) {
		sel = tx_selb;
		zero_start = ggc_get_10case_0_index(sel);
		sel &=
		    ~GENERATE_TRUNING_RING_IDX_VALUE(get_left_one_sel
						     (zero_start));
		zero_end = ggc_get_01case_0_index(sel);
		sel &=
		    ~GENERATE_TRUNING_RING_IDX_VALUE(get_right_one_sel
						     (zero_end));
		if (sel != (sel & tx_selb)) {
			pr_err("BHT ERR:!!!=============\n\n\n");
			pr_err
			    ("tx selb reinject exception case :not adjacent phase\n");
			pr_err("BHT ERR:selb_failed range:%xh  ,new tx_selb:%x\n",
			       org_selb, tx_selb);
			pr_err("BHT ERR:\n\n!!!=============\n");
		}
		org_selb &= tx_selb;
	} else {
		cnt = ggc_get_selx_weight(~tx_selb);
		pr_info("BHT MSG:%d\n", cnt);
		switch (cnt) {
		case 1:
			i = ggc_get_first_0_index(tx_selb);
			tx_selb &=
			    ~GENERATE_TRUNING_RING_IDX_VALUE(get_right_one_sel
							     (i));
			tx_selb &=
			    ~GENERATE_TRUNING_RING_IDX_VALUE(get_left_one_sel
							     (i));

			break;
		case 2:
			i = ggc_get_10case_0_index(tx_selb);
			tx_selb &=
			    ~GENERATE_TRUNING_RING_IDX_VALUE(get_left_one_sel
							     (i));
			i = ggc_get_01case_0_index(tx_selb);
			tx_selb &=
			    ~GENERATE_TRUNING_RING_IDX_VALUE(get_right_one_sel
							     (i));
			break;
		default:
			pr_info("BHT MSG:>= 3 point case\n");
		}
		org_selb &= tx_selb;
	}

	pr_info("BHT MSG:will check continuous 0bits: 0x%x\n", org_selb);

	memset(group_pos, 0, sizeof(group_pos));
	for (i = ggc_get_01case_1_index(org_selb);
		i < TUNING_PHASE_SIZE && i >= 0 && group_cnt < TUNING_PHASE_SIZE;) {
		for (j = 1; j < TUNING_PHASE_SIZE; j++)	{
			if (GET_TRUNING_RING_IDX_VALUE(org_selb, i+j) != 0)
				continue;
			else
				break;
		}
		group_pos[group_cnt][0] = i;
		group_pos[group_cnt][1] = (i + j - 1) % TUNING_PHASE_SIZE;
		group_pos[group_cnt][2] = j;
		group_cnt++;
		if (group_pos[group_cnt-1][0] > group_pos[group_cnt-1][1])
			break;
		i = ggc_get_next_1_index(org_selb, (i+j)%TUNING_PHASE_SIZE);
		for (j = 0; j < group_cnt; j++)	{
			if (i == group_pos[j][0])
				break;
		}
		if (j < group_cnt)
			break;
	}

	if (group_cnt > 1) {
		pr_err("BHT ERR:After inject, selb 0x%x has %d continuous 0 bits\n",
				org_selb, group_cnt);

		for (i = 0; i < group_cnt; i++) {
			if (max_len < group_pos[i][2]) {
				max_len = group_pos[i][2];
				max_len_group = i;
			}
		}
		for (i = (group_pos[max_len_group][1] + 1) % TUNING_PHASE_SIZE;
			i != group_pos[max_len_group][0]; i = (i+1)%TUNING_PHASE_SIZE) {
			org_selb &= ~(1 << i);
		}
		pr_err("BHT ERR:After merge incontious 0 group, selb changed to 0x%x\n", org_selb);
	} else if (group_cnt > 0) {
		pr_err("BHT ERR:After merge incontious 0 group, selb = 0x%x\n", org_selb);
	} else {
		pr_err("BHT ERR:selb 0x%x has no bit is 0\n", org_selb);
	}

	return org_selb;
}

void tx_selb_inject_policy(struct sdhci_host *host, int tx_selb)
{

	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	pr_info("BHT MSG:before inject, failed ragen 0x%x, tx_selb 0x%x\n",
			vendor_host->ggc.ggc_cmd_tx_selb_failed_range, tx_selb);
	vendor_host->ggc.ggc_cmd_tx_selb_failed_range =
		_tx_selb_inject_policy(tx_selb, vendor_host->ggc.ggc_cmd_tx_selb_failed_range);
	tx_selb_failed_history_update(host, vendor_host->ggc.ggc_cmd_tx_selb_failed_range);
	pr_info("BHT MSG:after inject %xh range:%xh\n", tx_selb,
		 vendor_host->ggc.ggc_cmd_tx_selb_failed_range);
	if (is_bus_mode_sdr104(host))
		vendor_host->ggc.sdr104.fail_phase = vendor_host->ggc.ggc_cmd_tx_selb_failed_range;
	else
		vendor_host->ggc.sdr50.fail_phase = vendor_host->ggc.ggc_cmd_tx_selb_failed_range;
}

int get_selb_failure_point(int start, u64 raw_tx_selb, int tuning_cnt)
{
	int last = start + (tuning_cnt - 1);
	int i = 0;
	int j = 0;
	int phase = start;
	int vct = BIT_PASS_MASK;

	pr_info("BHT MSG:%s start:%d tuning_cnt:%d\n", __func__, start,
			 tuning_cnt);

	for (i = 0; i < tuning_cnt; i++) {
		if ((raw_tx_selb & GENERATE_64_IDX_VALUE(last - i)) == 0)
			break;
	}
	if (i == tuning_cnt) {
		phase = last % TUNING_PHASE_SIZE;
		vct &= (~(1 << phase));
		goto exit;
	}

	for (i = 0; i < tuning_cnt; i++) {
		if ((raw_tx_selb & GENERATE_64_IDX_VALUE(last - i)) != 0)
			break;
	}
	for (j = i - 2; j >= 0; j--)
		raw_tx_selb |= (1 << (last - j));

	for (j = 0; j < tuning_cnt; j++) {
		if (0 == (raw_tx_selb & GENERATE_64_IDX_VALUE(last - j)))
			vct &= (~(1 << (last-j)));
	}

exit:
	pr_info("BHT MSG:%s: after adjust raw_tx_selb: 0x%llx, vct 0x%x\n",
			 __func__, raw_tx_selb, vct);

	return vct;
}

bool selx_failure_point_exist(u32 val)
{
	return (val & BIT_PASS_MASK) != BIT_PASS_MASK;
}

static int _bits_vct_get_left_index(int base)
{
	return TRUNING_RING_IDX(base + TUNING_PHASE_SIZE - 1);
}

int _get_best_window_phase(u32 vct, int *pmax_pass_win, int shif_left_flg)
{
	u8 tuning_win[TUNING_PHASE_SIZE] = { 0 };
	u8 tuning_pass[TUNING_PHASE_SIZE];
	int tuning_pass_start[TUNING_PHASE_SIZE];
	int tuning_pass_num_max = 0;
	int first_0 = 0;
	int i = 0, j = 0;
	int i_mode = 0, selb_mode = 0;

	memset(tuning_pass, 0, sizeof(tuning_pass));
	memset(tuning_pass_start, 0, sizeof(tuning_pass_start));

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (GET_IDX_VALUE(vct, i))
			tuning_win[i] = 1;
		else
			tuning_win[i] = 0;
	}

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (tuning_win[i] == 0) {
			first_0 = i;
			break;
		}
	}

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		i_mode = TRUNING_RING_IDX(first_0 + i);
		if (tuning_win[i_mode] == 1)
			tuning_pass[j]++;
		else if (tuning_pass[j])
			j++;
		if (tuning_pass[j] == 1)
			tuning_pass_start[j] = i_mode;
	}

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (tuning_pass_num_max < tuning_pass[i]) {
			tuning_pass_num_max = tuning_pass[i];
			i_mode = i;
		}
	}

	if (tuning_pass_num_max == 0) {
		pr_err("BHT ERR:###### Get max pass window fail, there is no any pass phase!!\n");
		selb_mode = 0;
	} else {
		if (tuning_pass_num_max % 2)	{
			selb_mode = tuning_pass_start[i_mode] + (tuning_pass_num_max - 1) / 2;
		} else {
			selb_mode = tuning_pass_start[i_mode] + (tuning_pass_num_max) / 2;
			if (shif_left_flg) {
				selb_mode = _bits_vct_get_left_index(selb_mode);
				pr_info("BHT MSG:shift left index\n");
			}
		}
		selb_mode = TRUNING_RING_IDX(selb_mode);
	}
	if (pmax_pass_win)
		*pmax_pass_win = tuning_pass_num_max;

	return selb_mode;
}


int get_best_window_phase(u32 vct, int *pmax_pass_win)
{
	return _get_best_window_phase(vct, pmax_pass_win, 0);
}

static int _ggc_get_suitable_selb_for_next_tuning(struct sdhci_host *host)
{
	int selb = 0;
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	if (selx_failure_point_exist(vendor_host->ggc.ggc_cmd_tx_selb_failed_range)) {
		selb = vendor_host->ggc.ggc_sw_selb_tuning_first_selb;
	} else {
		u32 inj_tx_selb = BIT_PASS_MASK;

		pr_info("BHT MSG:manual inject for all pass case\n");
		if (is_bus_mode_sdr104(host))
			inj_tx_selb &= SDR104_MANUAL_INJECT;
		else
			inj_tx_selb &= SDR50_MANUAL_INJECT;

		pr_info("BHT MSG:manual inject for all pass case, inj_tx_selb=0x%x\n",
				inj_tx_selb);
		selb = get_best_window_phase(inj_tx_selb, NULL);
		pr_info("BHT MSG:select selb %d for all pass case\n", selb);
	}
	return selb;
}

void ggc_reset_selx_failed_tb(struct sdhci_host *host)
{
	tx_selb_failed_tb_reset(host);
	all_selb_failed_tb_reset(host);
	tx_selb_failed_history_reset(host);
}

static void _ggc_reset_sela_tuning_result(struct sdhci_vendor_host *host)
{
	int i = 0;

	for (i = 0; i < TUNING_PHASE_SIZE; i++)
		host->ggc.ggc_sela_tuning_result[i] = NO_TUNING;
}

void _ggc_reset_tuning_result_for_dll(struct sdhci_host *host)
{
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	ggc_reset_selx_failed_tb(host);
	vendor_host->ggc.ggc_cmd_tx_selb_failed_range = BIT_PASS_MASK;
	vendor_host->ggc.selx_tuning_done_flag = 0;
	_ggc_reset_sela_tuning_result(vendor_host);
}

static int ggc_get_tuning_cnt_from_buffer(struct sdhci_host *host)
{
	int cnt = 0;
	u8 data[512];
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	get_gg_reg_cur_val(&vendor_host->ggc, data, 64);
	cnt = (int)cfg_read_bits_ofs_mask(data, &vendor_host->ggc.cmd19_cnt_cfg);

	pr_info("BHT MSG:tuning cnt=%d\n", cnt);
	return cnt;
}

bool ggc_hw_inject_ext(struct sdhci_host *host, bool *card_status,
			u32 sel200, u32 sel100, bool writetobh201)
{
	bool ret = TRUE;
	u8 data[512];
	struct t_gg_reg_strt gg_reg_arr[10];
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	pr_info("BHT MSG:%s sel200:%xh,sel100:%xh\n", __func__, sel200, sel100);
	get_gg_reg_cur_val(&vendor_host->ggc, data, 64);
	memcpy(&gg_reg_arr[0], &vendor_host->ggc.inject_failure_for_tuning_enable_cfg,
				sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[1], &vendor_host->ggc.inject_failure_for_200m_tuning_cfg,
				sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[2], &vendor_host->ggc.inject_failure_for_100m_tuning_cfg,
				sizeof(struct t_gg_reg_strt));
	gg_reg_arr[0].value = 1;
	gg_reg_arr[1].value = sel200;
	gg_reg_arr[2].value = sel100;

	chg_gg_reg_cur_val(&vendor_host->ggc, data, gg_reg_arr, 3, TRUE);
	if (writetobh201)
		ret = gg_emulator_write_ext(host, card_status, data, 512);
	else {
		u32 i = 0;
		u32 reg;

		pr_info("BHT MSG:%s: dump config data instead write to bh201\n", __func__);
		for (i = 0; i < 16; i++) {
			memcpy(&reg, data+i*sizeof(u32), sizeof(u32));
			pr_info("BHT MSG:    ggc_reg32[%03d]=0x%08x\n", i, reg);
		}
	}
	return ret;
}

bool _ggc_hw_inject_may_recursive(struct sdhci_host *host, u32 sel200,
			u32 sel100, int max_recur, bool writetobh201)
{
	bool ret = TRUE, card_status = TRUE;
	int selb = BIT_PASS_MASK;
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	ret = ggc_hw_inject_ext(host, &card_status, vendor_host->ggc.ggc_cmd_tx_selb_failed_range,
			vendor_host->ggc.ggc_cmd_tx_selb_failed_range, writetobh201);
	pr_info("BHT MSG:ret:%x\n", ret);
	if ((ret == FALSE) && (card_status == FALSE)) {
		pr_info("BHT MSG:inject again when hw inject\n");
		selb &= ~GENERATE_IDX_VALUE(vendor_host->ggc.ggc_sw_selb_tuning_first_selb);
		tx_selb_inject_policy(host, selb);
		_ggc_update_cur_setting_for_sw_selb_tuning(host,
			vendor_host->ggc.ggc_cmd_tx_selb_failed_range);

		if (((11 - get_bit_number(vendor_host->ggc.ggc_cmd_tx_selb_failed_range)) >= 5)) {
			pr_err("BHT ERR:pass windows too small,reinit recursive\n");
			return FALSE;
		}

		if (max_recur--)
			return _ggc_hw_inject_may_recursive(host,
			vendor_host->ggc.ggc_cmd_tx_selb_failed_range,
			vendor_host->ggc.ggc_cmd_tx_selb_failed_range, max_recur, writetobh201);
		else
			return FALSE;
	} else
		return TRUE;
}

bool ggc_hw_inject_may_recursive(struct sdhci_host *host, u32 sel200,
			u32 sel100, bool writetobh201)
{
	return _ggc_hw_inject_may_recursive(host, sel200, sel100, 4, writetobh201);
}

bool get_next_dll_voltage(int cur, int *next, u32 *dll_voltage_unlock_cnt,
			int *dll_voltage_scan_map)
{
	int min_idx = 0, cur_cnt = 0, next_cnt = 0;
	int cur_flg = 0;
	int i = 0;
	u8 ret = 0;

	pr_err("BHT ERR:dll_voltage_unlock_cnt:%x %x %x %x\n",
			dll_voltage_unlock_cnt[0], dll_voltage_unlock_cnt[1],
			dll_voltage_unlock_cnt[2], dll_voltage_unlock_cnt[3]);
	pr_err("BHT ERR:dll_voltage_scan_map:%x %x %x %x\n",
			dll_voltage_scan_map[0], dll_voltage_scan_map[1],
			dll_voltage_scan_map[2], dll_voltage_scan_map[3]);
	for (i = 1; i < 4; i++) {
		if (cur_flg == 0) {
			if (dll_voltage_scan_map[(cur + i) % 4] != 0)
				continue;
			cur_cnt = dll_voltage_unlock_cnt[(cur + i) % 4];
			cur_flg = 1;
			min_idx = (cur + i) % 4;
			continue;
		} else {
			if (dll_voltage_scan_map[(cur + i) % 4] != 0)
				continue;
			next_cnt = dll_voltage_unlock_cnt[(cur + i) % 4];
			if (cur_cnt > next_cnt) {
				cur_cnt = next_cnt;
				min_idx = (cur + i) % 4;
			}
		}
	}
	if (cur_flg == 0) {
		pr_err("BHT ERR:no find available voltage\n");
		ret = FALSE;
	} else {
		*next = min_idx;
		pr_err("BHT ERR:next available voltage %d\n", min_idx);
		ret = TRUE;
	}
	return ret;
}

bool ggc_sw_calc_tuning_result(struct sdhci_host *host, bool *card_status,
			       bool *read_status, u32 *tx_selb, u32 *all_selb, u64 *raw_tx_selb)
{
	bool ret = FALSE;
	bool card_ret = FALSE;
	bool read_ret = FALSE;
	u32 selb_status_tx_low32 = 0, selb_status_tx_high32 = 0;
	u32 selb_status_ggc_low32 = 0, selb_status_ggc_high32 = 0;
	struct t_gg_reg_strt gg_reg_arr[8] = {{0}};
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	memcpy(&gg_reg_arr[0], &vendor_host->ggc.pha_stas_tx_low32, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[1], &vendor_host->ggc.pha_stas_tx_high32, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[2], &vendor_host->ggc.pha_stas_rx_low32, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[3], &vendor_host->ggc.pha_stas_rx_high32, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[4], &vendor_host->ggc.dll_sela_after_mask, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[5], &vendor_host->ggc.dll_selb_after_mask, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[6], &vendor_host->ggc.dll_delay_100m_backup, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[7], &vendor_host->ggc.dll_delay_200m_backup, sizeof(struct t_gg_reg_strt));

	ret = ggc_read_registers_ext(host, &card_ret, &read_ret, gg_reg_arr, 8);
	if (read_ret == TRUE) {
		selb_status_tx_low32 = gg_reg_arr[0].value;
		pr_info("BHT MSG:[205-236]:\n");
		log_bin(selb_status_tx_low32);
		pr_info("BHT MSG:[237-268]:\n");
		selb_status_tx_high32 = gg_reg_arr[1].value;
		log_bin(selb_status_tx_high32);

		pr_info("BHT MSG:[14-45]:\n");
		selb_status_ggc_low32 = gg_reg_arr[2].value;
		log_bin(selb_status_ggc_low32);
		pr_info("BHT MSG:[46-77]:\n");
		selb_status_ggc_high32 = gg_reg_arr[3].value;
		log_bin(selb_status_ggc_high32);
		pr_info("BHT MSG:dll  sela after mask=%xh", gg_reg_arr[4].value);
		pr_info("BHT MSG:dll  selb after mask=%xh", gg_reg_arr[5].value);

		if (raw_tx_selb) {
			*raw_tx_selb = gg_reg_arr[1].value;
			(*raw_tx_selb) <<= 32;
			*raw_tx_selb += gg_reg_arr[0].value;
			pr_info("BHT MSG:raw_tx_selb:%llxh\n", *raw_tx_selb);
		}

		if (tx_selb) {
			gen_array_data(gg_reg_arr[0].value, gg_reg_arr[1].value,
				       tx_selb);
			pr_info("BHT MSG:tx_selb:%xh\n", *tx_selb);
		}
		if (all_selb) {
			gen_array_data(gg_reg_arr[2].value, gg_reg_arr[3].value,
				       all_selb);
			pr_info("BHT MSG:all_selb:%xh\n", *all_selb);
		}
	}

	if (read_status)
		(*read_status) = read_ret;
	if (card_status)
		(*card_status) = card_ret;

	if (card_status && read_status)
		pr_info("BHT MSG:card_status,read_status:%x %x\n", *card_status, *read_status);
	return ret;
}

bool _ggc_calc_cur_sela_tuning_result(struct sdhci_host *host, int cur_sela, int start_selb)
{
	bool read_status = FALSE;
	bool card_status = FALSE;
	bool ret = TRUE;
	u32 tx_selb, all_selb;
	u64 raw_tx_selb = 0;
	bool retuning_flg = FALSE;
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);
	int selb;
	struct ggc_platform_t *ggc = &vendor_host->ggc;
	enum tuning_stat_et *psela_tuning_result = ggc->ggc_sela_tuning_result;

	ret = ggc_sw_calc_tuning_result(host, &card_status,
			&read_status, &tx_selb, &all_selb, &raw_tx_selb);

	if (card_status == FALSE) {
		if (read_status == TRUE) {
			selb = get_selb_failure_point(start_selb, raw_tx_selb,
					ggc_get_tuning_cnt_from_buffer(host));
			pr_info("BHT MSG:inject selb %03x for CMD7 read timeout\n", selb);
			tx_selb_inject_policy(host, selb);
		} else {
			pr_info("BHT MSG:read status failedA!!\n");
			pr_info("BHT MSG:============ %s dll:%xh failed ============\n",
					__func__, cur_sela);
		}
		ret = FALSE;
		goto exit;
	} else {
		if (read_status == TRUE) {
			if (selx_failure_point_exist(tx_selb)) {
				if ((11-get_bit_number(tx_selb)) <= 3) {
					tx_selb_inject_policy(host, tx_selb);
					all_selb_failed_tb_update(host, cur_sela, all_selb);
					tx_selb_failed_tb_update(host, cur_sela, tx_selb);
					tx_selb_failed_history_update(host, tx_selb);
				} else if (get_bit_number(tx_selb) == 0) {
					selb = get_selb_failure_point(start_selb, raw_tx_selb,
							ggc_get_tuning_cnt_from_buffer(host));
					tx_selb_inject_policy(host, selb);
					all_selb_failed_tb_update(host, cur_sela, all_selb);
					tx_selb_failed_tb_update(host, cur_sela, selb);
					tx_selb_failed_history_update(host, selb);
					retuning_flg = TRUE;
				} else {
					tx_selb_inject_policy(host, tx_selb);
					all_selb_failed_tb_update(host, cur_sela, all_selb);
					tx_selb_failed_tb_update(host, cur_sela, tx_selb);
					tx_selb_failed_history_update(host, tx_selb);
					retuning_flg = TRUE;
				}

				_ggc_update_cur_setting_for_sw_selb_tuning(host,
						ggc->ggc_cmd_tx_selb_failed_range);
				ggc_hw_inject_may_recursive(host, ggc->ggc_cmd_tx_selb_failed_range,
						ggc->ggc_cmd_tx_selb_failed_range, TRUE);
			} else {
				all_selb_failed_tb_update(host, cur_sela, all_selb);
				tx_selb_failed_tb_update(host, cur_sela, tx_selb);
				tx_selb_failed_history_update(host, tx_selb);
			}

			if (retuning_flg == TRUE) {
				pr_info("BHT MSG:== %s dll:%xh need retuning ==\n",
						__func__, cur_sela);
				psela_tuning_result[cur_sela] = RETUNING_CASE;
			} else {
				pr_info("BHT MSG:== %s dll:%xh pass ==\n",
						__func__, cur_sela);
				psela_tuning_result[cur_sela] = OUTPUT_PASS_TYPE;
			}
		} else {
			pr_info("BHT MSG:read status failed!!\n");
			psela_tuning_result[cur_sela] = READ_STATUS_FAIL_TYPE;
			all_selb_failed_tb_update(host, cur_sela, 0);
			pr_info("BHT MSG:== %s dll:%xh failed ==\n",
					__func__, cur_sela);
		}
	}
exit:
	return ret;
}

bool _ggc_output_tuning(struct sdhci_host *host, u8 *selb_pass_win)
{
	int cur_sela = 0, dll_sela_cnt = 0;
	int dll_sela_basis = 0;
	bool ret = FALSE;
	u8 win_tb[12] = { 0 };
	u8 win_mid = 0;
	u8 win_max = 0;
	u32 tx_tmp = 0;
	int target_sela = 0;
	int target_selb = 0;
	u32 all_sela, tx_selb, all_selb;
	u64 raw_tx_selb;
	bool status_ret = FALSE;
	int cur_selb = 0;
	int tuning_error_type[16] = { 0 };
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);
	struct ggc_platform_t *ggc = &vendor_host->ggc;
	enum tuning_stat_et *psela_tuning_result = ggc->ggc_sela_tuning_result;
	int i = 0;
	u32 idx_r, idx_c;
	u32 min_pos = 0;
	u32 all_selb_ar[TUNING_PHASE_SIZE] = { 0 };
	u32 pass_cnt[TUNING_PHASE_SIZE] = { 0 };
	u32 cfg = 0;
	int rescan_selb_cnt = 0;
	int returning_selb_cnt = 0;
	bool first_cmd19_sta = TRUE;
	int next = 0;
	bool card_status = TRUE;
	int selb =	BIT_PASS_MASK;
	u8 all_str[TUNING_PHASE_SIZE + 1],tx_str[TUNING_PHASE_SIZE + 1];

	vendor_host->ggc.driver_strength_reinit_flg = 0;
	vendor_host->ggc.dll_unlock_reinit_flg = 0;
	vendor_host->ggc.sdr50_notuning_crc_error_flag = 0;
	dll_sela_basis = get_config_sela_setting(host);
	cur_selb = get_config_selb_setting(host);

	// wait for DLL lock
	{
		if(wait_dll_lock(host))
			pr_err("BHT MSG:wait for DLL lock failed\n");
		bht_mmc_send_io_op_cond(host->mmc,0,NULL);
	}

	if (ggc->tuning_cmd7_timeout_reinit_flg) {
		ggc->tuning_cmd7_timeout_reinit_flg = 0;
		dll_sela_basis = vendor_host->ggc.ggc_cur_sela;
		cur_selb = vendor_host->ggc.ggc_sw_selb_tuning_first_selb;
		pr_info
		("BHT MSG:Tuning start sela: 0x%x, selb: 0x%x where CMD7 timeout\n",
				dll_sela_basis, cur_selb);
	}

	for (dll_sela_cnt = 0; dll_sela_cnt < TUNING_PHASE_SIZE; dll_sela_cnt++) {
		rescan_selb_cnt = 0;
		returning_selb_cnt = 0;
		cur_sela =
				(dll_sela_cnt + dll_sela_basis) % TUNING_PHASE_SIZE;
		ggc->ggc_cur_sela = cur_sela;
		pr_info("BHT MSG:== %s select sela dll: %x, selb dll: %x ==\n",
				__func__, cur_sela, cur_selb);
		if (psela_tuning_result[cur_sela] != NO_TUNING) {
			pr_info("BHT MSG:sela tuning=%d already tuning,so skip it\n", cur_sela);
			continue;
		}
rescan_selb:
		host_cmddat_line_reset(host);

		if (dll_sela_cnt == 0) {
			if (!selx_failure_point_exist
					(vendor_host->ggc.ggc_cmd_tx_selb_failed_range)) {
				rescan_selb_cnt = 3;
				pr_info("BHT MSG:no need rescan case\n");
			}
			status_ret = FALSE;
			ret = ggc_sd_tuning(host, &first_cmd19_sta);

			if (first_cmd19_sta == FALSE) {
				next = 0;
				_check_bus_mode(host);
				ggc->cur_bus_mode->dll_voltage_unlock_cnt
							[ggc->cur_dll_voltage_idx]++;
				ggc->dll_voltage_scan_map[ggc->cur_dll_voltage_idx] = 1;
				if (get_next_dll_voltage(ggc->cur_dll_voltage_idx, &next,
					ggc->cur_bus_mode->dll_voltage_unlock_cnt,
					ggc->dll_voltage_scan_map) == TRUE)
					ggc->cur_dll_voltage_idx = next;
				else
					ggc->cur_dll_voltage_idx =
							(ggc->cur_dll_voltage_idx + 1) % 4;

				pr_err("BHT ERR:first cmd19 timeout\n");
				vendor_host->ggc.dll_unlock_reinit_flg = 1;
				_ggc_reset_tuning_result_for_dll(host);
				ret = FALSE;
				goto exit;
			}
		} else if ((is_bus_mode_sdr104(host) == FALSE)
				&& vendor_host->ggc.sdr50_notuning_sela_inject_flag == 1
				&& !GET_IDX_VALUE(vendor_host->ggc.sdr50_notuning_sela_rx_inject,
				cur_sela)) {
			pr_info("BHT MSG:skip %d\n", cur_sela);
			tuning_error_type[cur_sela] = READ_STATUS_FAIL_TYPE;
			goto cur_sela_failed;
		} else {
			ret = ggc_set_output_tuning_phase_ext(host, &card_status,
				cur_sela, update_selb(host, cur_selb));
			if (ret == FALSE || card_status == FALSE) {
				pr_err
				("BHT ERR: output_tuning fail at phase %d,ret=%d,card_status=%d\n",
						cur_sela, ret, card_status);
				if (card_status == FALSE) {
					selb = BIT_PASS_MASK;
					selb &= ~GENERATE_IDX_VALUE(cur_selb);
					pr_err("BHT ERR:inject selb %d for update sela/selb fail\n",
							selb);
					tx_selb_inject_policy(host, selb);
					_ggc_update_cur_setting_for_sw_selb_tuning(host,
						ggc->ggc_cmd_tx_selb_failed_range);
					ggc_hw_inject_may_recursive(host,
							ggc->ggc_cmd_tx_selb_failed_range,
							ggc->ggc_cmd_tx_selb_failed_range, TRUE);

					if (((11 - get_bit_number(
						ggc->ggc_cmd_tx_selb_failed_range))	>= 5)) {
						u8 current_ds = (u8)(ggc->_gg_reg_cur[15] >> 28);

						pr_err("BHT ERR:pass windows too small\n");

						if(vendor_host->ggc.tuning_fail_count == 0)
						{
							if (current_ds < 7)
								vendor_host->ggc.driver_strength_reinit_flg = 6;
								// vendor_host->ggc.driver_strength_reinit_flg = current_ds + 1;
							else
								vendor_host->ggc.driver_strength_reinit_flg = 7;

							ggc->_gg_reg_cur[15] &= 0x0F0FFFFF;
							ggc->_gg_reg_cur[15] |=
							(vendor_host->ggc.driver_strength_reinit_flg << 28)
							| (vendor_host->ggc.driver_strength_reinit_flg << 20);
							pr_err("BHT ERR:will change driver strength from %d to %d\n",
							current_ds,
							vendor_host->ggc.driver_strength_reinit_flg);
						}
						else if(is_bus_mode_sdr104(host))
						{
							vendor_host->ggc.degrade = 1;	
							host->mmc->caps &= ~(MMC_CAP_UHS_DDR50 | MMC_CAP_UHS_SDR104);
							pr_info("BHT MSG: disable SDR104 and DDR50, caps 0x%x\n",host->mmc->caps);
						} else
						{
							vendor_host->ggc.degrade = 1;
							host->mmc->caps &= ~(MMC_CAP_UHS);
							pr_info("BHT MSG: disable UHS mode, caps 0x%x\n",host->mmc->caps);
						}
						ret = FALSE;
						goto exit;
					}
					cur_selb = _ggc_get_suitable_selb_for_next_tuning(host);
				}
				psela_tuning_result[cur_sela] = RETUNING_CASE;
				goto retuning_case;
			}
			ret = ggc_sd_tuning(host, NULL);
		}

		if (ret == FALSE) {
			pr_err("BHT ERR:Error when output_tuning, fail at phase %d\n",
					cur_sela);
			psela_tuning_result[cur_sela] = TUNING_FAIL_TYPE;
			all_selb_failed_tb_update(host, cur_sela, 0);
			continue;
		}

		ret = _ggc_calc_cur_sela_tuning_result(host, cur_sela, cur_selb);

		if ((11 - get_bit_number(vendor_host->ggc.ggc_cmd_tx_selb_failed_range)) >= 5) 
		{
			u8 current_ds = (u8)(ggc->_gg_reg_cur[15] >> 28);

			pr_err("BHT ERR:pass windows too small after result calculate, reinit\n");

			if(vendor_host->ggc.tuning_fail_count == 0)
			{
				if (current_ds < 7)
					vendor_host->ggc.driver_strength_reinit_flg = 6;
					// vendor_host->ggc.driver_strength_reinit_flg = current_ds + 1;
				else
					vendor_host->ggc.driver_strength_reinit_flg = 7;

				ggc->_gg_reg_cur[15] &= 0x0F0FFFFF;
				ggc->_gg_reg_cur[15] |=
					(vendor_host->ggc.driver_strength_reinit_flg << 28)
					| (vendor_host->ggc.driver_strength_reinit_flg << 20);

				pr_err("BHT ERR:will change driver strength from %d to %d\n",
							current_ds,
							vendor_host->ggc.driver_strength_reinit_flg);
			}
			else if(is_bus_mode_sdr104(host))
			{
				vendor_host->ggc.degrade = 1;	
				host->mmc->caps &= ~(MMC_CAP_UHS_DDR50 | MMC_CAP_UHS_SDR104);
				pr_info("BHT MSG: disable SDR104 and DDR50, caps 0x%x\n",host->mmc->caps);
			} else
			{
				vendor_host->ggc.degrade = 1;
				host->mmc->caps &= ~(MMC_CAP_UHS);
				pr_info("BHT MSG: disable UHS mode, caps 0x%x\n",host->mmc->caps);
			}
			ret = FALSE;
			goto exit;
		}

		if (ret == FALSE) {
			pr_err("BHT ERR:cmd7 timeout fail,reinit\n");
			vendor_host->ggc.tuning_cmd7_timeout_reinit_flg = 1;

			_ggc_update_cur_setting_for_sw_selb_tuning(host,
							ggc->ggc_cmd_tx_selb_failed_range);
			ggc_hw_inject_may_recursive(host, ggc->ggc_cmd_tx_selb_failed_range,
							ggc->ggc_cmd_tx_selb_failed_range, FALSE);
			if ((11 - get_bit_number(vendor_host->ggc.ggc_cmd_tx_selb_failed_range))
										>= 5) {
				u8 current_ds = (u8)(ggc->_gg_reg_cur[15] >> 28);

				pr_err("BHT ERR:pass windows too small, driver strength reinit\n");

				vendor_host->ggc.tuning_cmd7_timeout_reinit_flg = 0;

				if(vendor_host->ggc.tuning_fail_count == 0)
				{
					if (current_ds < 7)
						vendor_host->ggc.driver_strength_reinit_flg = 6;
						// vendor_host->ggc.driver_strength_reinit_flg = current_ds + 1;
					else
						vendor_host->ggc.driver_strength_reinit_flg = 7;

					ggc->_gg_reg_cur[15] &= 0x0F0FFFFF;
					ggc->_gg_reg_cur[15] |=
						(vendor_host->ggc.driver_strength_reinit_flg << 28)
						| (vendor_host->ggc.driver_strength_reinit_flg << 20);
					pr_err("BHT ERR:will change driver strength from %d to %d\n",
							current_ds,
							vendor_host->ggc.driver_strength_reinit_flg);
				}
				else if(is_bus_mode_sdr104(host))
				{
					vendor_host->ggc.degrade = 1;	
					host->mmc->caps &= ~(MMC_CAP_UHS_DDR50 | MMC_CAP_UHS_SDR104);
					pr_info("BHT MSG: disable SDR104 and DDR50, caps 0x%x\n",host->mmc->caps);
				} else
				{
					vendor_host->ggc.degrade = 1;
					host->mmc->caps &= ~(MMC_CAP_UHS);
					pr_info("BHT MSG: disable UHS mode, caps 0x%x\n",host->mmc->caps);
				}
				ret = FALSE;
				goto exit;
			}
			goto exit;
		}

		cur_selb = _ggc_get_suitable_selb_for_next_tuning(host);

		pr_info("BHT MSG:===ot  sela:%xh pass ===\n", cur_sela);
		rescan_selb_cnt++;
		if ((rescan_selb_cnt < 3) &&
			(selx_failure_point_exist(vendor_host->ggc.ggc_cmd_tx_selb_failed_range))) {
			pr_info("BHT MSG:rescan cnt %d, ggc_cmd_tx_selb_failed_range=0x%x\n",
						rescan_selb_cnt,
						vendor_host->ggc.ggc_cmd_tx_selb_failed_range);
			goto rescan_selb;
		}

retuning_case:
		if (psela_tuning_result[cur_sela] == RETUNING_CASE) {
			returning_selb_cnt++;
			if (returning_selb_cnt < 3) {
				rescan_selb_cnt = 0;
				pr_info("BHT MSG:retuning %d\n", rescan_selb_cnt);
				goto rescan_selb;
			} else {
				psela_tuning_result[cur_sela] = SET_PHASE_FAIL_TYPE;
				all_selb_failed_tb_update(host, cur_sela, 0);
				continue;
			}
		}

		goto next_dll_sela;

cur_sela_failed:
		pr_info("BHT MSG:read status failedB\n");
		all_selb = 0;
		all_selb_failed_tb_update(host, cur_sela, all_selb);
		pr_info("BHT MSG:===ot  sela:%xh failed ===\n", cur_sela);
next_dll_sela:
		if ((is_bus_mode_sdr104(host) == FALSE)
				&& vendor_host->ggc.sdr50_notuning_crc_error_flag) {
			u32 fp = 0;

			fp = GENERATE_IDX_VALUE(cur_sela);
			fp |= GENERATE_IDX_VALUE((cur_sela + 1) % TUNING_PHASE_SIZE);
			fp |= GENERATE_IDX_VALUE((cur_sela + 10) % TUNING_PHASE_SIZE);
			vendor_host->ggc.sdr50_notuning_sela_rx_inject &= ~fp;
			vendor_host->ggc.sdr50_notuning_sela_inject_flag = 1;
			pr_info("BHT MSG:sdr50_notuning_sela_rx_inject:%x\n",
						vendor_host->ggc.sdr50_notuning_sela_rx_inject);
			ret = FALSE;
			goto exit;
		};
	}

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		phase_str(all_str, all_selb_failed_tb_get(host, i));
		phase_str(tx_str, tx_selb_failed_tb_get(host, i));
		pr_info
				("BHT MSG:DLL sela[%x]  all selb: %s   tx selb: %s [%xh,%xh] %s\n",
					i, all_str, tx_str,
					all_selb_failed_tb_get(host, i),
					tx_selb_failed_tb_get(host, i),
					op_dbg_str[tuning_error_type[i]]);

	}

	// remove margin passed all selb phase by ernest 2019/6/6
	for (i = 0; i < TUNING_PHASE_SIZE; i++)
		all_selb_ar[i] = all_selb_failed_tb_get(host, i);

	// calculate cumulation of diagonal bits
	for (idx_c = 0; idx_c < TUNING_PHASE_SIZE; idx_c++) {
		for (idx_r = 0; idx_r < TUNING_PHASE_SIZE;
					idx_r++) {
			pass_cnt[idx_c] +=	((all_selb_ar[idx_r] >>
					((idx_r + idx_c) % TUNING_PHASE_SIZE)) & 0x01);
		}
		if (idx_c == 0)
			min_pos = 0;
		else if (pass_cnt[idx_c] < pass_cnt[min_pos])
			min_pos = idx_c;
	}
	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		all_selb_ar[i] &= ~(1 << (min_pos + i) % TUNING_PHASE_SIZE);
		all_selb_failed_tb_update(host, i, all_selb_ar[i]);
	}

	tx_selb = tx_selb_failed_history_get(host);

	pr_info("inject sw selb & merge tx_selb failed point to all_selb\n");
	for (i = 0; i < TUNING_PHASE_SIZE; i++)
		all_selb_failed_tb_update(host, i, tx_selb);

	pr_info("BHT MSG:inject sw sela failed point to all_selb\n");
	if (is_bus_mode_sdr104(host))
		cfg = 0x7ff;
	else
		cfg = 0x7ff;

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (GET_IDX_VALUE(cfg, i) == 0)
			all_selb_failed_tb_update(host, i, 0);
	}

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		phase_str(all_str, all_selb_failed_tb_get(host, i));
		phase_str(tx_str, tx_selb_failed_tb_get(host, i));
		pr_info("BHT MSG:DLL sela[%x]  all selb: %s tx selb: %s [%xh,%xh] %s\n",
				i, all_str, tx_str,
				all_selb_failed_tb_get(host, i),
				tx_selb_failed_tb_get(host, i),
				op_dbg_str[tuning_error_type[i]]);
	}

	tx_selb = tx_selb_failed_history_get(host);
	tx_selb &= 0x7ff;
	tx_tmp = tx_selb;

	pr_info("BHT MSG:---selb merge---\n");
	if ((tx_selb&0x7ff) == 0x7ff) {
		if (is_bus_mode_sdr104(host)) {
			u32 cfg = SDR104_MANUAL_INJECT;

			tx_selb &= cfg;
			pr_info("tx selb:%xh SDR104 inject:%xh merge tx_selb:%xh\n",
						tx_tmp, cfg, tx_selb);
		} else {
			u32 cfg = SDR50_MANUAL_INJECT;

			tx_selb &= cfg;
			pr_info("tx selb:%xh SDR50 inject:%xh merge tx_selb:%xh\n",
						tx_tmp, cfg, tx_selb);
		}
	}

	if (tx_selb == 0) {
		pr_err("all failed, force fixed phase sela selb to default\n");
		target_selb =
				get_config_selb_setting(host);
		target_sela =
				get_config_sela_setting(host);
		goto final;
	}
	phase_str(win_tb, tx_selb);
	pr_info("BHT MSG:######### tx selb[%xh] 11 bit: %s\n",
						tx_selb, win_tb);
	bits_generate_array(win_tb, tx_selb);
	chk_phase_window(win_tb, &win_mid, &win_max);
	target_selb = win_mid;

	//sela
	all_sela = 0;

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		u32 all_selb = all_selb_failed_tb_get(host, i);

		phase_str(win_tb, all_selb);
		pr_info("######### all_selb[%xh] 11 bit: %s\n",
				all_selb, win_tb);
		bits_generate_array(win_tb, all_selb);
		chk_phase_window(win_tb, &win_mid,
					&win_max);
		*selb_pass_win = win_max;
		if (all_selb & (1 << target_selb))
			all_sela |= 1 << i;
	}

	phase_str(win_tb, all_sela);
	pr_info("######### all_sela[%xh] 11 bit: %s\n",
			all_sela, win_tb);
	bits_generate_array(win_tb, all_sela);
	chk_phase_window(win_tb, &win_mid, &win_max);
	target_sela = win_mid;

final:

	gg_fix_output_tuning_phase(host, target_sela,
						target_selb);

	ret = sd_gg_tuning_status(host, &tx_selb,
			&all_selb, &raw_tx_selb, &status_ret, NULL);
	if (ret == FALSE) {
		pr_err("Error when output_tuning,  sd_tuning fail\n");
		ret = FALSE;
		goto exit;
	}

	//use final pass windows
	phase_str(win_tb, all_selb);
	pr_info("######### all_selb[%xh] 11 bit: %s\n",
			all_selb, win_tb);
	bits_generate_array(win_tb, all_selb);
	chk_phase_window(win_tb, &win_mid, &win_max);
	*selb_pass_win = win_max;

	vendor_host->ggc.selx_tuning_done_flag = 1;

exit:
	pr_info("BHT MSG:exit:%s  %d\n", __func__, ret);
	return ret;
}

void ggc_tuning_result_reset(struct sdhci_host *host)
{
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);

	pr_info("BHT MSG:%s: will clear all tuning results\n", __func__);
	_ggc_reset_tuning_result_for_dll(host);

	// init_flg = 0;
	vendor_host->ggc.sdr50.bus_mode = SD_FNC_AM_SDR50;
	vendor_host->ggc.sdr104.bus_mode = SD_FNC_AM_SDR104;
	vendor_host->ggc.driver_strength_reinit_flg = 0;
	vendor_host->ggc.cur_bus_mode = NULL;
	vendor_host->ggc.dll_unlock_reinit_flg = 0;
	vendor_host->ggc.dll_unlock_reinit_flg = 0;
	vendor_host->ggc.tuning_cmd7_timeout_reinit_flg = 0;
	vendor_host->ggc.tuning_cmd7_timeout_reinit_cnt = 0;
	vendor_host->ggc.sdr50_notuning_sela_inject_flag = 1;
	vendor_host->ggc.sdr50_notuning_crc_error_flag = 0;
	vendor_host->ggc.degrade = 0;

	if (vendor_host->ggc.bh201_sdr50_sela_sw_inject)
		vendor_host->ggc.sdr50_notuning_sela_rx_inject =
						vendor_host->ggc.bh201_sdr50_sela_sw_inject;
	else
		vendor_host->ggc.sdr50_notuning_sela_rx_inject = 0x47F;
}

void ggc_chip_init(struct sdhci_host *host)
{
	u8 data[512] = { 0 };
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);
	if (init_flg == 0) {
		ggc_tuning_result_reset(host);
		get_gg_reg_def(host, data);
		get_default_setting(host, data);
		set_gg_reg_cur_val(&vendor_host->ggc, data, 64);
		init_flg = 1;
	}
}

#if PLATFORM_845
int sdhci_bht_execute_tuning(struct sdhci_host *host, u32 opcode)
{
#else
int sdhci_bht_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct sdhci_host *host = get_sdhci_host(mmc);
#endif
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);
	u8 tw = 0;
	int ret = 0;

#ifdef MTK_HOST_USED
	host->clock = vendor_host->mclk;
	host->timing = vendor_host->timing;
#endif

	if (vendor_host->ggc.bh201_used) {
		pr_info("BHT MSG:enter bht tuning\n");
		if (host->clock < CORE_FREQ_100MHZ) {
			pr_info("BHT MSG:%d less 100Mhz,no tuning\n", host->clock);
			goto exit;
		}

		if (vendor_host->ggc.tuning_in_progress) {
			pr_info("BHT MSG:tuning_in_progress\n");
			goto exit;
		}
		vendor_host->ggc.tuning_in_progress = true;

		if(vendor_host->ggc.origin_cap == 0)
			vendor_host->ggc.origin_cap = host->mmc->caps;

		if (vendor_host->ggc.selx_tuning_done_flag) {
			pr_info("BHT MSG:GGC tuning is done, just do vendor host tuning\n");
			if (is_bus_mode_sdr104(host))
				ret = sdhci_bht_sdr104_execute_tuning(host, 0x13);
			else
				ret = sdhci_bht_sdr50_execute_tuning(host, 0x13);

			if(ret)
			{
				pr_info("BHT MSG:GGC tuning is done, but do vendor host tuning failed\n");
				ggc_tuning_result_reset(host);
			}
		} else {
			ret = !_ggc_output_tuning(host, &tw);
		}
		vendor_host->ggc.tuning_in_progress = false;

	} else
#if PLATFORM_845
		ret = sdhci_vendor_execute_tuning(host, opcode);
#else
		ret = sdhci_vendor_execute_tuning(mmc, opcode);
#endif	
	pr_info("BHT MSG:GGC tuning_fail_count %d\n",vendor_host->ggc.tuning_fail_count);
	if(vendor_host->ggc.tuning_fail_count >= 2)
	{
		vendor_host->ggc.degrade = 1;
		host->mmc->caps &= ~(MMC_CAP_UHS);
		pr_info("BHT MSG: disable UHS mode, caps 0x%x\n",host->mmc->caps);
	}
	if(ret)
		vendor_host->ggc.tuning_fail_count++;
	else
		vendor_host->ggc.tuning_fail_count = 0;

	if(ret && vendor_host->ggc.tuning_fail_count < 5) {
		 /*
		 in order to invoke card initialization can be executed
		 again
		 */
		pr_info("BHT MSG: finit is %dHz\n",host->mmc->f_init);
		if (host->ops->get_min_clock && host->ops->get_min_clock(host) == host->mmc->f_init)
        {
			pr_info("BHT MSG: Invoke card init from bayhub tuning routine\n");
			queue_delayed_work(system_freezable_wq, &mmc->detect, HZ);
		}
	}

exit:
	// Free the temporary pointer
	vendor_host->ggc.card = NULL;
	return ret;
}

void bht_load(struct mmc_host *mmc_host, struct mmc_card *card)
{
	static const int s_dll_voltage_cfg[4][2] = {
		{0x30503106, 0x64141711},
		{0x31503106, 0x64141711},
		{0x30503106, 0x64141751},
		{0x31503106, 0x64141751},
	};

	struct sdhci_host *host = get_sdhci_host(mmc_host);
	struct sdhci_vendor_host *vendor_host = get_vendor_host(host);
	u32 i = 0;
	u32 reg;
	u32 gg_sw_def[16] = GGC_CFG_DATA;
	u8 data[512];

	if (vendor_host->ggc.bh201_used) {
		pr_info("%s: Load BHT patch\n", mmc_hostname(mmc_host));
		if (vendor_host->ggc.degrade) {
			ggc_tuning_result_reset(host);
		}

		bht_mmc_send_relative_addr(mmc_host, &card->rca);

		// Use temporary pointer to access card structure
		vendor_host->ggc.card = card;

		// store card cid for card change detect
		if(memcmp(g_card_cid, card->raw_cid, sizeof(g_card_cid)) != 0)
		{
			pr_info("card cid is different, card is changed\n");
			ggc_tuning_result_reset(host);
			memcpy(g_card_cid,card->raw_cid,sizeof(g_card_cid));
		}


		if (vendor_host->ggc.dll_unlock_reinit_flg) {
			int cur_dll_voltage_idx = vendor_host->ggc.cur_dll_voltage_idx;

			pr_info("update dll voltage cfg for dll unlock reinit flg: idx=%d\n",
					cur_dll_voltage_idx);
			ggc_tuning_result_reset(host);
			gg_sw_def[8] = s_dll_voltage_cfg[cur_dll_voltage_idx][0];
			gg_sw_def[9] = s_dll_voltage_cfg[cur_dll_voltage_idx][1];
		}
		if (vendor_host->ggc.driver_strength_reinit_flg) {
			u8 driver_strength_reinit_flg = vendor_host->ggc.driver_strength_reinit_flg;

			pr_info("%s: driver strength should be init to %d\n",
				mmc_hostname(mmc_host), driver_strength_reinit_flg);
			ggc_tuning_result_reset(host);
			if (vendor_host->ggc.driver_strength_reinit_flg <= 7) {
				gg_sw_def[15] &= 0x0f0fffff;
				gg_sw_def[15] |= (driver_strength_reinit_flg << 28)
						| (driver_strength_reinit_flg << 20);
			}
		} else {
			if (vendor_host->ggc.bh201_drive_strength)
				gg_sw_def[15] = vendor_host->ggc.bh201_drive_strength;
		}
		driver_send_command(host);
		if (vendor_host->ggc.tuning_cmd7_timeout_reinit_flg == 0
					&& vendor_host->ggc.selx_tuning_done_flag == 0) {
			pr_debug("gg_sw_def[8] = 0x%08x, gg_sw_def[9]=0x%08x, gg_sw_def[15]=%08x\n",
					gg_sw_def[8], gg_sw_def[9], gg_sw_def[15]);
			bht_load_hw_inject(mmc_host, card, gg_sw_def,
				sizeof(gg_sw_def), 0x3ff, 0x77f);
			bht_update_cfg(mmc_host, card, gg_sw_def, sizeof(gg_sw_def));
			set_gg_reg_cur_val(&vendor_host->ggc, (u8 *)gg_sw_def, sizeof(gg_sw_def));
			pr_debug("gg_sw_def[8] = 0x%08x, gg_sw_def[9]=0x%08x, gg_sw_def[15]=%08x\n",
					gg_sw_def[8], gg_sw_def[9], gg_sw_def[15]);
		} else {
			if (vendor_host->ggc.selx_tuning_done_flag)
				pr_info("%s: write previous inject results to bh201 for tuning done\n",
					mmc_hostname(mmc_host));
			if (vendor_host->ggc.tuning_cmd7_timeout_reinit_flg)
				pr_info("%s: write previous inject results to bh201 for cmd7 timeout flag is set\n",
					mmc_hostname(mmc_host));

			get_gg_reg_cur_val(&vendor_host->ggc, data, sizeof(gg_sw_def));

			pr_info("%s: dump config data before write to bh201\n", __func__);
			for (i = 0; i < 16; i++) {
				memcpy(&reg, data+i*sizeof(u32), sizeof(u32));
				pr_info("ggc_reg32[%03d]=0x%08x\n", i, reg);
			}
			bht_update_cfg(mmc_host, card, (u32 *)data, sizeof(data));

		}

		/*
		 * Prepare condition for BH201 chip mode switch
		 *
		 */
		bht_mmc_sd_get_csd(mmc_host, card);
		gg_select_card_spec(host);
		bht_mmc_app_send_scr(card);
		bht_mmc_decode_scr(card);
		bht_mmc_read_switch(card);
		bht_mmc_app_set_bus_width(card, MMC_BUS_WIDTH_1);
		bht_mmc_app_set_clr_card_detect(card);
		driver_send_command(host);
	}
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("BH Driver");

