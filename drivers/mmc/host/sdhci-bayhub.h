/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Bayhub Technologies, Inc. BH201 SDHCI bridge IC for
 * VENDOR SDHCI platform driver head file
 *
 * Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 */

#include <linux/of_gpio.h>
#include "sdhci-pltfm.h"

/*Use Bayhub 845 or not
 * 0: not used
 * 1: used
 */
#define PLATFORM_845 0

/* selec a vendor host to define according to project in bwlow:
 * 1. MSM_HOST_USED; 2. MTK_HOST_USED; 3. SPRD_HOST_USED; etc
 */
#define MSM_HOST_USED
/* selec a vendor host allocate methdd according to project in bwlow:
 * 1. SDHCI_PLTFM_HOST_ALLOC; 2. MMC_HOST_ALLOC;
 */
#define SDHCI_PLTFM_HOST_ALLOC

#define TUNING_PHASE_SIZE	11
#ifndef CORE_FREQ_100MHZ
#define CORE_FREQ_100MHZ	(100 * 1000 * 1000)
#endif

struct ggc_bus_mode_cfg_t {
	u32 tx_selb_tb[TUNING_PHASE_SIZE];
	u32 all_selb_tb[TUNING_PHASE_SIZE];
	u32 tx_selb_failed_history;
	int bus_mode;
	int default_sela;
	int default_selb;
	u32 default_delaycode;
	u32 dll_voltage_unlock_cnt[4];
	u32 max_delaycode;
	u32 min_delaycode;
	u32 delaycode_narrowdown_index;
	u32 fail_phase;
};

struct t_gg_reg_strt {
	u32 ofs;
	u32 mask;
	u32 value;
};

enum tuning_stat_et {
	NO_TUNING = 0,
	OUTPUT_PASS_TYPE = 1,
	SET_PHASE_FAIL_TYPE = 2,
	TUNING_FAIL_TYPE = 3,
	READ_STATUS_FAIL_TYPE = 4,
	TUNING_CMD7_TIMEOUT = 5,
	RETUNING_CASE = 6,
};

struct rl_bit_lct {
	u8 bits;
	u8 rl_bits;
};

struct chk_type_t {
	u8 right_valid:1;
	u8 first_valid:1;
	u8 record_valid:1;
	u8 reserved:5;
};

static const char *const op_dbg_str[] = {
	"no tuning",
	"pass",
	"set_phase_fail",
	"tuning fail",
	"read status fail",
	"tuning CMD7 timeout",
	"retuning case"
};

struct ggc_platform_t {
	struct ggc_bus_mode_cfg_t sdr50;
	struct ggc_bus_mode_cfg_t sdr104;
	struct ggc_bus_mode_cfg_t *cur_bus_mode;
	struct t_gg_reg_strt pha_stas_rx_low32;
	struct t_gg_reg_strt pha_stas_rx_high32;
	struct t_gg_reg_strt pha_stas_tx_low32;
	struct t_gg_reg_strt pha_stas_tx_high32;
	struct t_gg_reg_strt dll_sela_after_mask;
	struct t_gg_reg_strt dll_selb_after_mask;

	struct t_gg_reg_strt dll_delay_100m_backup;
	struct t_gg_reg_strt dll_delay_200m_backup;

	struct t_gg_reg_strt dll_sela_100m_cfg;
	struct t_gg_reg_strt dll_sela_200m_cfg;
	struct t_gg_reg_strt dll_selb_100m_cfg;
	struct t_gg_reg_strt dll_selb_200m_cfg;
	struct t_gg_reg_strt dll_selb_100m_cfg_en;
	struct t_gg_reg_strt dll_selb_200m_cfg_en;
	struct t_gg_reg_strt internl_tuning_en_100m;
	struct t_gg_reg_strt internl_tuning_en_200m;
	struct t_gg_reg_strt cmd19_cnt_cfg;

	struct t_gg_reg_strt inject_failure_for_tuning_enable_cfg;
	struct t_gg_reg_strt inject_failure_for_200m_tuning_cfg;
	struct t_gg_reg_strt inject_failure_for_100m_tuning_cfg;
	//Used to access card structure during Bayhub tuning stage
	struct mmc_card *card;

	int def_sela_100m;
	int def_sela_200m;
	int def_selb_100m;
	int def_selb_200m;

	u32 _gg_reg_cur[16];
	u8 _cur_read_buf[512];//only for read
	u32 ggc_cur_ssc_level;
	int ssc_crc_recovery_retry_cnt;
	u32 crc_retry_cnt;
	u32 cclk_ds_18v;
	u32 cdata_ds_18v;
	u32 ds_inc_cnt;
	bool ggc_400k_update_ds_flg;
	bool ggc_400k_update_ssc_arg_flg;
	bool ggc_ocb_in_check;
	bool dll_unlock_reinit_flg;
	u8 driver_strength_reinit_flg;
	bool tuning_cmd7_timeout_reinit_flg;
	u32 tuning_cmd7_timeout_reinit_cnt;
	u32 ggc_400k_setting_change_flg: 1;
	u32 ggc_cur_sela;
	u32 ggc_target_selb;
	u32 target_tuning_pass_win;
	bool selx_tuning_done_flag;
	u32 ggc_cmd_tx_selb_failed_range;
	int ggc_sw_selb_tuning_first_selb;
	enum tuning_stat_et ggc_sela_tuning_result[11];
	int dll_voltage_scan_map[4];
	int cur_dll_voltage_idx;

	int sdr50_notuning_sela_inject_flag;
	int sdr50_notuning_crc_error_flag;
	u32 sdr50_notuning_sela_rx_inject;
	u32 bh201_sdr50_sela_sw_inject;
	u32 bh201_sdr50_selb_hw_inject;
	u32 bh201_sdr104_selb_hw_inject;
	u32 bh201_drive_strength;
	bool tuning_in_progress;
	int bh201_used;
	int pwr_gpio; /* External power enable pin for Redriver IC */
	int det_gpio;

	u8			v18_disable; /* flag used for degrde to sd2.0 */
	u8			legacy_mode; /* flag used to keep sd2.0 mode */
	u8			card_removed_flag; /* flag used for recovery from degrade mode after insert card */
	u8			degrade;
	u8			degrade_count;
	u8			tuning_fail_count;
	u32			origin_cap;
	unsigned int  cur_sd_bus_speed;
};

void bht_load(struct mmc_host *mmc_host, struct mmc_card *card);
void bh201_signal_voltage_on_off(struct sdhci_host *host, u32 on_off);
#if PLATFORM_845
int sdhci_bht_execute_tuning(struct sdhci_host *host, u32 opcode);
#else
int sdhci_bht_execute_tuning(struct mmc_host *mmc, u32 opcode);
#endif
void ggc_chip_init(struct sdhci_host *host);
void ggc_tuning_result_reset(struct sdhci_host *host);
void sdhci_bh201_parse(struct mmc_host *mmc_host);

static inline bool bht_target_host(struct sdhci_host *host)
{
#if PLATFORM_845
	return 0 == strcmp(host->hw_name, "8804000.sdhci");
#else
	return 0 == strcmp(host->hw_name, "4784000.sdhci");
#endif
}


#define   GGC_CFG_DATA {0x07000000, 0x07364022, 0x01015412, 0x01062400,\
	0x10400076, 0x00025432, 0x01046076, 0x62011000,\
	0x30503106, 0x64141711, 0x10057513, 0x00336200,\
	0x00020006, 0x40001400, 0x12200310, 0x4A414177}
