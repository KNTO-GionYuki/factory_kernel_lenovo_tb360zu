// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 */

#include <linux/module.h>

#include "cam_sensor_cmn_header.h"
#include "cam_flash_core.h"
#include "cam_res_mgr_api.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"
#include <linux/math64.h>
#ifdef CONFIG_CAMERA_FLASH_PWM
#include "cam_sensor_util.h"
#include "pm6125_flash_gpio.h"
#endif

static uint default_on_timer = 2;
module_param(default_on_timer, uint, 0644);

int cam_flash_led_prepare(struct led_trigger *trigger, int options,
	int *max_current, bool is_wled)
{
	int rc = 0;

	if (is_wled) {
#if IS_REACHABLE(CONFIG_BACKLIGHT_QCOM_SPMI_WLED)
		rc = wled_flash_led_prepare(trigger, options, max_current);
		if (rc) {
			CAM_ERR(CAM_FLASH, "enable reg failed: rc: %d",
				rc);
			return rc;
		}
#else
	return -EPERM;
#endif
	} else {
#if IS_REACHABLE(CONFIG_LEDS_QPNP_FLASH_V2)
		rc = qpnp_flash_led_prepare(trigger, options, max_current);
#elif IS_REACHABLE(CONFIG_LEDS_QTI_FLASH)
		rc = qti_flash_led_prepare(trigger, options, max_current);
#endif
		if (rc) {
			CAM_ERR(CAM_FLASH,
				"Regulator enable failed rc = %d", rc);
			return rc;
		}
	}

	return rc;
}

static int cam_flash_pmic_flush_nrt(struct cam_flash_ctrl *fctrl)
{
	int j = 0;
	struct cam_flash_frame_setting *nrt_settings;

	if (!fctrl)
		return -EINVAL;

	nrt_settings = &fctrl->nrt_info;

	if (nrt_settings->cmn_attr.cmd_type ==
		CAMERA_SENSOR_FLASH_CMD_TYPE_INIT_INFO) {
		fctrl->flash_init_setting.cmn_attr.is_settings_valid = false;
	} else if ((nrt_settings->cmn_attr.cmd_type ==
		CAMERA_SENSOR_FLASH_CMD_TYPE_WIDGET) ||
		(nrt_settings->cmn_attr.cmd_type ==
		CAMERA_SENSOR_FLASH_CMD_TYPE_RER) ||
		(nrt_settings->cmn_attr.cmd_type ==
		CAMERA_SENSOR_FLASH_CMD_TYPE_INIT_FIRE)) {
		fctrl->nrt_info.cmn_attr.is_settings_valid = false;
		fctrl->nrt_info.cmn_attr.count = 0;
		fctrl->nrt_info.num_iterations = 0;
		fctrl->nrt_info.led_on_delay_ms = 0;
		fctrl->nrt_info.led_off_delay_ms = 0;
		for (j = 0; j < CAM_FLASH_MAX_LED_TRIGGERS; j++)
			fctrl->nrt_info.led_current_ma[j] = 0;
	}

	return 0;
}

static int cam_flash_i2c_flush_nrt(struct cam_flash_ctrl *fctrl)
{
	int rc = 0;

	if (fctrl->i2c_data.init_settings.is_settings_valid == true) {
		rc = delete_request(&fctrl->i2c_data.init_settings);
		if (rc) {
			CAM_WARN(CAM_FLASH,
				"Failed to delete Init i2c_setting: %d",
				rc);
			return rc;
		}
	}
	if (fctrl->i2c_data.config_settings.is_settings_valid == true) {
		rc = delete_request(&fctrl->i2c_data.config_settings);
		if (rc) {
			CAM_WARN(CAM_FLASH,
				"Failed to delete NRT i2c_setting: %d",
				rc);
			return rc;
		}
	}

	return rc;
}

static int cam_flash_construct_default_power_setting(
	struct cam_sensor_power_ctrl_t *power_info)
{
	int rc = 0;

	power_info->power_setting_size = 1;
	power_info->power_setting =
		kzalloc(sizeof(struct cam_sensor_power_setting),
			GFP_KERNEL);
	if (!power_info->power_setting)
		return -ENOMEM;

	power_info->power_setting[0].seq_type = SENSOR_CUSTOM_REG1;
	power_info->power_setting[0].seq_val = CAM_V_CUSTOM1;
	power_info->power_setting[0].config_val = 0;
	power_info->power_setting[0].delay = 2;

	power_info->power_down_setting_size = 1;
	power_info->power_down_setting =
		kzalloc(sizeof(struct cam_sensor_power_setting),
			GFP_KERNEL);
	if (!power_info->power_down_setting) {
		rc = -ENOMEM;
		goto free_power_settings;
	}

	power_info->power_down_setting[0].seq_type = SENSOR_CUSTOM_REG1;
	power_info->power_down_setting[0].seq_val = CAM_V_CUSTOM1;
	power_info->power_down_setting[0].config_val = 0;

	return rc;

free_power_settings:
	kfree(power_info->power_setting);
	power_info->power_setting = NULL;
	power_info->power_setting_size = 0;
	return rc;
}

int cam_flash_i2c_power_ops(struct cam_flash_ctrl *fctrl,
	bool regulator_enable)
{
	int rc = 0;
	struct cam_hw_soc_info *soc_info = &fctrl->soc_info;
	struct cam_sensor_power_ctrl_t *power_info =
		&fctrl->power_info;

	if (!power_info || !soc_info) {
		CAM_ERR(CAM_FLASH, "Power Info is NULL");
		return -EINVAL;
	}
	power_info->dev = soc_info->dev;

	if (regulator_enable && (fctrl->is_regulator_enabled == false)) {
		if ((power_info->power_setting == NULL) &&
			(power_info->power_down_setting == NULL)) {
			CAM_INFO(CAM_FLASH,
				"Using default power settings");
			rc = cam_flash_construct_default_power_setting(
					power_info);
			if (rc < 0) {
				CAM_ERR(CAM_FLASH,
				"Construct default pwr setting failed rc: %d",
				rc);
				return rc;
			}
		}

		rc = cam_sensor_core_power_up(power_info, soc_info);
		if (rc) {
			CAM_ERR(CAM_FLASH, "power up the core is failed:%d",
				rc);
			goto free_pwr_settings;
		}

		rc = camera_io_init(&(fctrl->io_master_info));
		if (rc) {
			CAM_ERR(CAM_FLASH, "cci_init failed: rc: %d", rc);
			cam_sensor_util_power_down(power_info, soc_info);
			goto free_pwr_settings;
		}
		fctrl->is_regulator_enabled = true;
	} else if ((!regulator_enable) &&
		(fctrl->is_regulator_enabled == true)) {
		rc = cam_sensor_util_power_down(power_info, soc_info);
		if (rc) {
			CAM_ERR(CAM_FLASH, "power down the core is failed:%d",
				rc);
			return rc;
		}
		camera_io_release(&(fctrl->io_master_info));
		fctrl->is_regulator_enabled = false;
		goto free_pwr_settings;
	}
	return rc;

free_pwr_settings:
	kfree(power_info->power_setting);
	kfree(power_info->power_down_setting);
	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	power_info->power_setting_size = 0;
	power_info->power_down_setting_size = 0;

	return rc;
}

int cam_flash_pmic_flush_request(struct cam_flash_ctrl *fctrl,
	enum cam_flash_flush_type type, uint64_t req_id)
{
	int rc = 0;
	int i = 0, j = 0;
	int frame_offset = 0;
	bool is_off_needed = false;
	struct cam_flash_frame_setting *flash_data = NULL;

	if (!fctrl) {
		CAM_ERR(CAM_FLASH, "Device data is NULL");
		return -EINVAL;
	}

	if (type == FLUSH_ALL) {
	/* flush all requests*/
		for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
			flash_data =
				&fctrl->per_frame[i];
			if ((flash_data->opcode ==
				CAMERA_SENSOR_FLASH_OP_OFF) &&
				(flash_data->cmn_attr.request_id > 0) &&
				flash_data->cmn_attr.is_settings_valid) {
				is_off_needed = true;
				CAM_DBG(CAM_FLASH,
					"FLASH_ALL: Turn off the flash for req %llu",
					flash_data->cmn_attr.request_id);
			}

			flash_data->cmn_attr.request_id = 0;
			flash_data->cmn_attr.is_settings_valid = false;
			flash_data->cmn_attr.count = 0;
			for (j = 0; j < CAM_FLASH_MAX_LED_TRIGGERS; j++)
				flash_data->led_current_ma[j] = 0;
		}

		cam_flash_pmic_flush_nrt(fctrl);
	} else if ((type == FLUSH_REQ) && (req_id != 0)) {
	/* flush request with req_id*/
		frame_offset = req_id % MAX_PER_FRAME_ARRAY;
		flash_data =
			&fctrl->per_frame[frame_offset];

		if (flash_data->opcode ==
			CAMERA_SENSOR_FLASH_OP_OFF) {
			is_off_needed = true;
			CAM_DBG(CAM_FLASH,
				"FLASH_REQ: Turn off the flash for req %llu",
				flash_data->cmn_attr.request_id);
		}

		flash_data->cmn_attr.request_id = 0;
		flash_data->cmn_attr.is_settings_valid =
			false;
		flash_data->cmn_attr.count = 0;
		for (i = 0; i < CAM_FLASH_MAX_LED_TRIGGERS; i++)
			flash_data->led_current_ma[i] = 0;
	} else if ((type == FLUSH_REQ) && (req_id == 0)) {
		/* Handels NonRealTime usecase */
		cam_flash_pmic_flush_nrt(fctrl);
	} else {
		CAM_ERR(CAM_FLASH, "Invalid arguments");
		return -EINVAL;
	}

	if (is_off_needed)
		cam_flash_off(fctrl);

	return rc;
}

int cam_flash_i2c_flush_request(struct cam_flash_ctrl *fctrl,
	enum cam_flash_flush_type type, uint64_t req_id)
{
	int rc = 0;
	int i = 0;
	uint32_t cancel_req_id_found = 0;
	struct i2c_settings_array *i2c_set = NULL;

	if (!fctrl) {
		CAM_ERR(CAM_FLASH, "Device data is NULL");
		return -EINVAL;
	}
	if ((type == FLUSH_REQ) && (req_id == 0)) {
		/* This setting will be called only when NonRealTime
		 * settings needs to clean.
		 */
		cam_flash_i2c_flush_nrt(fctrl);
	} else {
		/* All other usecase will be handle here */
		for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
			i2c_set = &(fctrl->i2c_data.per_frame[i]);

			if ((type == FLUSH_REQ) &&
				(i2c_set->request_id != req_id))
				continue;

			if (i2c_set->is_settings_valid == 1) {
				rc = delete_request(i2c_set);
				if (rc < 0)
					CAM_ERR(CAM_FLASH,
						"delete request: %lld rc: %d",
						i2c_set->request_id, rc);

				if (type == FLUSH_REQ) {
					cancel_req_id_found = 1;
					break;
				}
			}
		}
	}

	if ((type == FLUSH_REQ) && (req_id != 0) &&
			(!cancel_req_id_found))
		CAM_DBG(CAM_FLASH,
			"Flush request id:%lld not found in the pending list",
			req_id);

	return rc;
}

int cam_flash_flush_request(struct cam_req_mgr_flush_request *flush)
{
	int rc = 0;
	struct cam_flash_ctrl *fctrl = NULL;

	fctrl = (struct cam_flash_ctrl *) cam_get_device_priv(flush->dev_hdl);
	if (!fctrl) {
		CAM_ERR(CAM_FLASH, "Device data is NULL");
		return -EINVAL;
	}

	mutex_lock(&fctrl->flash_mutex);
	if (fctrl->flash_state == CAM_FLASH_STATE_INIT)
		goto end;

	if (flush->type == CAM_REQ_MGR_FLUSH_TYPE_ALL) {
		fctrl->last_flush_req = flush->req_id;
		CAM_DBG(CAM_FLASH, "last reqest to flush is %lld",
			flush->req_id);
		rc = fctrl->func_tbl.flush_req(fctrl, FLUSH_ALL, 0);
		if (rc) {
			CAM_ERR(CAM_FLASH, "FLUSH_TYPE_ALL failed rc: %d", rc);
			goto end;
		}
	} else if (flush->type == CAM_REQ_MGR_FLUSH_TYPE_CANCEL_REQ) {
		rc = fctrl->func_tbl.flush_req(fctrl,
				FLUSH_REQ, flush->req_id);
		if (rc) {
			CAM_ERR(CAM_FLASH, "FLUSH_REQ failed rc: %d", rc);
			goto end;
		}
	}
end:
	mutex_unlock(&fctrl->flash_mutex);
	return rc;
}

static int cam_flash_ops(struct cam_flash_ctrl *flash_ctrl,
	struct cam_flash_frame_setting *flash_data, enum camera_flash_opcode op)
{
	uint32_t curr = 0, max_current = 0;
	struct cam_flash_private_soc *soc_private = NULL;
	int i = 0;

	if (!flash_ctrl || !flash_data) {
		CAM_ERR(CAM_FLASH, "Fctrl or Data NULL");
		return -EINVAL;
	}

	soc_private = (struct cam_flash_private_soc *)
		flash_ctrl->soc_info.soc_private;

	if (op == CAMERA_SENSOR_FLASH_OP_FIRELOW) {
		for (i = 0; i < flash_ctrl->torch_num_sources; i++) {
			if (flash_ctrl->torch_trigger[i]) {
				max_current = soc_private->torch_max_current[i];
				if (flash_data->led_current_ma[i] <=
					max_current)
					curr = flash_data->led_current_ma[i];
				else
					curr = max_current;
			}
			CAM_DBG(CAM_FLASH, "Led_Torch[%d]: Current: %d",
				i, curr);
			cam_res_mgr_led_trigger_event(
				flash_ctrl->torch_trigger[i], curr);
		}
	} else if ((op == CAMERA_SENSOR_FLASH_OP_FIREHIGH) ||
		(op == CAMERA_SENSOR_FLASH_OP_FIREDURATION)) {
		for (i = 0; i < flash_ctrl->flash_num_sources; i++) {
			if (flash_ctrl->flash_trigger[i]) {
				max_current = soc_private->flash_max_current[i];
				if (flash_data->led_current_ma[i] <=
					max_current)
					curr = flash_data->led_current_ma[i];
				else
					curr = max_current;
			}
			CAM_DBG(CAM_FLASH, "LED_Flash[%d]: Current: %d",
				i, curr);
			cam_res_mgr_led_trigger_event(
				flash_ctrl->flash_trigger[i], curr);
		}
	} else {
		CAM_ERR(CAM_FLASH, "Wrong Operation: %d", op);
		return -EINVAL;
	}

	if (flash_ctrl->switch_trigger) {
#if IS_ENABLED(CONFIG_LEDS_QTI_FLASH)
		int rc = 0;

		if (op == CAMERA_SENSOR_FLASH_OP_FIREDURATION) {
			struct flash_led_param param;

			param.off_time_ms =
				flash_data->flash_active_time_ms;
			/* This is to dynamically change the turn on time */
			param.on_time_ms = default_on_timer;
			CAM_DBG(CAM_FLASH,
				"Precise flash_on time: %u, Precise flash_off time: %u",
				param.on_time_ms, param.off_time_ms);
			rc = qti_flash_led_set_param(
				flash_ctrl->switch_trigger,
				param);
			if (rc) {
				CAM_ERR(CAM_FLASH,
					"LED set param fail rc= %d", rc);
				return rc;
			}
		}
#else
		if (op == CAMERA_SENSOR_FLASH_OP_FIREDURATION) {
			CAM_ERR(CAM_FLASH, "FIREDURATION op not supported");
			return -EINVAL;
		}
#endif
		cam_res_mgr_led_trigger_event(
			flash_ctrl->switch_trigger,
			(enum led_brightness)LED_SWITCH_ON);
	}

	return 0;
}

int cam_flash_off(struct cam_flash_ctrl *flash_ctrl)
{
	int rc = 0;
#ifdef CONFIG_CAMERA_FLASH_PWM
	struct cam_hw_soc_info  soc_info = flash_ctrl->soc_info;
	struct cam_flash_private_soc *soc_private = (struct cam_flash_private_soc *)soc_info.soc_private;
#endif

	if (!flash_ctrl) {
		CAM_ERR(CAM_FLASH, "Flash control Null");
		return -EINVAL;
	}
	CAM_DBG(CAM_FLASH, "Flash OFF Triggered");
	if (flash_ctrl->switch_trigger)
		cam_res_mgr_led_trigger_event(flash_ctrl->switch_trigger,
			(enum led_brightness)LED_SWITCH_OFF);

	if ((flash_ctrl->i2c_data.streamoff_settings.is_settings_valid) &&
		(flash_ctrl->i2c_data.streamoff_settings.request_id == 0)) {
		flash_ctrl->apply_streamoff = true;
		rc = cam_flash_i2c_apply_setting(flash_ctrl, 0);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
			"cannot apply streamoff settings");
		}
	}
#ifdef CONFIG_CAMERA_FLASH_PWM
	CAM_DBG(CAM_SENSOR, "cam_flash_off");
	cam_res_mgr_gpio_set_value(soc_private->flash_gpio_enable, 0);
	cam_res_mgr_gpio_free(soc_info.dev, soc_private->flash_gpio_enable);
	pm6125_flash_gpio_select_state(PM6125_FLASH_GPIO_STATE_SUSPEND, CAMERA_SENSOR_FLASH_OP_OFF, 0);
	msleep(5);
#endif
	return 0;
}

static int cam_flash_low(
	struct cam_flash_ctrl *flash_ctrl,
	struct cam_flash_frame_setting *flash_data)
{
	int i = 0, rc = 0;
#ifdef CONFIG_CAMERA_FLASH_PWM
	struct cam_hw_soc_info  soc_info = flash_ctrl->soc_info;
	struct cam_flash_private_soc *soc_private = (struct cam_flash_private_soc *)soc_info.soc_private;
#endif

	if (!flash_data) {
		CAM_ERR(CAM_FLASH, "Flash Data Null");
		return -EINVAL;
	}

	for (i = 0; i < flash_ctrl->flash_num_sources; i++)
		if (flash_ctrl->flash_trigger[i])
			cam_res_mgr_led_trigger_event(
				flash_ctrl->flash_trigger[i],
				LED_OFF);

	rc = cam_flash_ops(flash_ctrl, flash_data,
		CAMERA_SENSOR_FLASH_OP_FIRELOW);
	if (rc)
		CAM_ERR(CAM_FLASH, "Fire Torch failed: %d", rc);

#ifdef CONFIG_CAMERA_FLASH_PWM
	CAM_DBG(CAM_FLASH, "Flash low Triggered flash_data->led_current_ma[0] = %u", flash_data->led_current_ma[0]);
	rc = cam_res_mgr_gpio_request(soc_info.dev, soc_private->flash_gpio_enable, 0, "CUSTOM_GPIO1");
	if(rc) {
		CAM_ERR(CAM_FLASH, "gpio %d request fails", soc_private->flash_gpio_enable);
		return rc;
	}
	cam_res_mgr_gpio_set_value(soc_private->flash_gpio_enable, 0);
	pm6125_flash_gpio_select_state(PM6125_FLASH_GPIO_STATE_SUSPEND, CAMERA_SENSOR_FLASH_OP_OFF, 0);
	pm6125_flash_gpio_select_state(PM6125_FLASH_GPIO_STATE_ACTIVE,CAMERA_SENSOR_FLASH_OP_FIRELOW,(u64)FLASH_FIRE_LOW_MAXCURRENT);
	msleep(5);
	pm6125_flash_gpio_select_state(PM6125_FLASH_GPIO_STATE_ACTIVE,CAMERA_SENSOR_FLASH_OP_FIRELOW,(u64)flash_data->led_current_ma[0]);
#endif
	return rc;
}

static int cam_flash_high(
	struct cam_flash_ctrl *flash_ctrl,
	struct cam_flash_frame_setting *flash_data)
{
	int i = 0, rc = 0;
#ifdef CONFIG_CAMERA_FLASH_PWM
	struct cam_hw_soc_info  soc_info = flash_ctrl->soc_info;
	struct cam_flash_private_soc *soc_private = (struct cam_flash_private_soc *)soc_info.soc_private;
#endif

	if (!flash_data) {
		CAM_ERR(CAM_FLASH, "Flash Data Null");
		return -EINVAL;
	}

	for (i = 0; i < flash_ctrl->torch_num_sources; i++)
		if (flash_ctrl->torch_trigger[i])
			cam_res_mgr_led_trigger_event(
				flash_ctrl->torch_trigger[i],
				LED_OFF);

	rc = cam_flash_ops(flash_ctrl, flash_data,
		CAMERA_SENSOR_FLASH_OP_FIREHIGH);
	if (rc)
		CAM_ERR(CAM_FLASH, "Fire Flash Failed: %d", rc);

#ifdef CONFIG_CAMERA_FLASH_PWM
	CAM_DBG(CAM_FLASH, "Flash high Triggered flash_data->led_current_ma[0] = %u", flash_data->led_current_ma[0]);
	rc = cam_res_mgr_gpio_request(soc_info.dev, soc_private->flash_gpio_enable, 0, "CUSTOM_GPIO1");
	if(rc) {
		CAM_ERR(CAM_FLASH, "gpio %d request fails", soc_private->flash_gpio_enable);
		return rc;
	}
	pm6125_flash_gpio_select_state(PM6125_FLASH_GPIO_STATE_ACTIVE,CAMERA_SENSOR_FLASH_OP_FIREHIGH,(u64)flash_data->led_current_ma[0]);
	msleep(1);
	cam_res_mgr_gpio_set_value(soc_private->flash_gpio_enable, 1);
#endif
	return rc;
}

static int cam_flash_duration(struct cam_flash_ctrl *fctrl,
	struct cam_flash_frame_setting *flash_data)
{
	int i = 0, rc = 0;

	if (!flash_data) {
		CAM_ERR(CAM_FLASH, "Flash Data NULL");
		return -EINVAL;
	}

	for (i = 0; i < fctrl->torch_num_sources; i++)
		if (fctrl->torch_trigger[i])
			cam_res_mgr_led_trigger_event(
				fctrl->torch_trigger[i],
				LED_OFF);

	rc = cam_flash_ops(fctrl, flash_data,
		CAMERA_SENSOR_FLASH_OP_FIREDURATION);
	if (rc)
		CAM_ERR(CAM_FLASH, "Fire PreciseFlash Failed: %d", rc);

	return rc;
}

static int cam_flash_i2c_delete_req(struct cam_flash_ctrl *fctrl,
	uint64_t req_id)
{
	int i = 0, rc = 0;
	uint64_t top = 0, del_req_id = 0;

	if (req_id != 0) {
		for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
			if ((req_id >=
				fctrl->i2c_data.per_frame[i].request_id) &&
				(top <
				fctrl->i2c_data.per_frame[i].request_id) &&
				(fctrl->i2c_data.per_frame[i].is_settings_valid
					== 1)) {
				del_req_id = top;
				top = fctrl->i2c_data.per_frame[i].request_id;
			}
		}

		if (top < req_id) {
			if ((((top % MAX_PER_FRAME_ARRAY) - (req_id %
				MAX_PER_FRAME_ARRAY)) >= BATCH_SIZE_MAX) ||
				(((top % MAX_PER_FRAME_ARRAY) - (req_id %
				MAX_PER_FRAME_ARRAY)) <= -BATCH_SIZE_MAX))
				del_req_id = req_id;
		}

		if (!del_req_id)
			return rc;

		CAM_DBG(CAM_FLASH, "top: %llu, del_req_id:%llu",
			top, del_req_id);

		for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
			if ((del_req_id >
				 fctrl->i2c_data.per_frame[i].request_id) && (
				 fctrl->i2c_data.per_frame[i].is_settings_valid
					== 1)) {
				fctrl->i2c_data.per_frame[i].request_id = 0;
				rc = delete_request(
					&(fctrl->i2c_data.per_frame[i]));
				if (rc < 0)
					CAM_ERR(CAM_SENSOR,
						"Delete request Fail:%lld rc:%d",
						del_req_id, rc);
			}
		}
	}

	cam_flash_i2c_flush_nrt(fctrl);

	return 0;
}

static int cam_flash_pmic_delete_req(struct cam_flash_ctrl *fctrl,
	uint64_t req_id)
{
	int i = 0;
	struct cam_flash_frame_setting *flash_data = NULL;
	uint64_t top = 0, del_req_id = 0;
	int frame_offset = 0;

	if (req_id != 0) {
		for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
			flash_data = &fctrl->per_frame[i];
			if (req_id >= flash_data->cmn_attr.request_id &&
				flash_data->cmn_attr.is_settings_valid
				== 1) {
				if (top < flash_data->cmn_attr.request_id) {
					del_req_id = top;
					top = flash_data->cmn_attr.request_id;
				} else if (top >
					flash_data->cmn_attr.request_id &&
					del_req_id <
					flash_data->cmn_attr.request_id) {
					del_req_id =
						flash_data->cmn_attr.request_id;
				}
			}
		}

		if (top < req_id) {
			if ((((top % MAX_PER_FRAME_ARRAY) - (req_id %
				MAX_PER_FRAME_ARRAY)) >= BATCH_SIZE_MAX) ||
				(((top % MAX_PER_FRAME_ARRAY) - (req_id %
				MAX_PER_FRAME_ARRAY)) <= -BATCH_SIZE_MAX))
				del_req_id = req_id;
		}

		if (!del_req_id)
			return 0;

		CAM_DBG(CAM_FLASH, "top: %llu, del_req_id:%llu",
			top, del_req_id);
	}

	/* delete the request */
	frame_offset = del_req_id % MAX_PER_FRAME_ARRAY;
	flash_data = &fctrl->per_frame[frame_offset];
	flash_data->cmn_attr.request_id = 0;
	flash_data->cmn_attr.is_settings_valid = false;
	flash_data->cmn_attr.count = 0;

	for (i = 0; i < CAM_FLASH_MAX_LED_TRIGGERS; i++)
		flash_data->led_current_ma[i] = 0;

	return 0;
}

static int32_t cam_flash_slaveInfo_pkt_parser(struct cam_flash_ctrl *fctrl,
	uint32_t *cmd_buf, size_t len)
{
	int32_t rc = 0;
	struct cam_cmd_i2c_info *i2c_info = (struct cam_cmd_i2c_info *)cmd_buf;

	if (len < sizeof(struct cam_cmd_i2c_info)) {
		CAM_ERR(CAM_FLASH, "Not enough buffer");
		return -EINVAL;
	}
	if (fctrl->io_master_info.master_type == CCI_MASTER) {
		fctrl->io_master_info.cci_client->cci_i2c_master =
			fctrl->cci_i2c_master;
		fctrl->io_master_info.cci_client->i2c_freq_mode =
			i2c_info->i2c_freq_mode;
		fctrl->io_master_info.cci_client->sid =
			i2c_info->slave_addr >> 1;
		CAM_DBG(CAM_FLASH, "Slave addr: 0x%x Freq Mode: %d",
			i2c_info->slave_addr, i2c_info->i2c_freq_mode);
	} else if (fctrl->io_master_info.master_type == I2C_MASTER) {
		fctrl->io_master_info.client->addr = i2c_info->slave_addr;
		CAM_DBG(CAM_FLASH, "Slave addr: 0x%x", i2c_info->slave_addr);
	} else {
		CAM_ERR(CAM_FLASH, "Invalid Master type: %d",
			fctrl->io_master_info.master_type);
		 rc = -EINVAL;
	}

	return rc;
}

int cam_flash_i2c_apply_setting(struct cam_flash_ctrl *fctrl,
	uint64_t req_id)
{
	struct i2c_settings_list *i2c_list;
	struct i2c_settings_array *i2c_set = NULL;
	int frame_offset = 0, rc = 0;
	CAM_DBG(CAM_FLASH, "req_id=%llu", req_id);
	if (req_id == 0) {
		/* NonRealTime Init settings*/
		if (fctrl->apply_streamoff == true) {
			fctrl->apply_streamoff = false;
			i2c_set = &fctrl->i2c_data.streamoff_settings;
			list_for_each_entry(i2c_list,
				&(i2c_set->list_head),
				list) {
				rc = cam_sensor_util_i2c_apply_setting
					(&(fctrl->io_master_info), i2c_list);
				if (rc) {
					CAM_ERR(CAM_FLASH,
					"Failed to apply stream on settings: %d", rc);
					return rc;
				}
				break;
			}
		} else if (fctrl->i2c_data.init_settings.is_settings_valid == true) {
			list_for_each_entry(i2c_list,
				&(fctrl->i2c_data.init_settings.list_head),
				list) {
				rc = cam_sensor_util_i2c_apply_setting
					(&(fctrl->io_master_info), i2c_list);
				if ((rc == -EAGAIN) &&
					(fctrl->io_master_info.master_type ==
					CCI_MASTER)) {
					CAM_WARN(CAM_FLASH,
						"CCI HW is in reset mode: Reapplying Init settings");
					usleep_range(1000, 1010);
					rc = cam_sensor_util_i2c_apply_setting
					(&(fctrl->io_master_info), i2c_list);
				}

				if (rc) {
					CAM_ERR(CAM_FLASH,
					"Failed to apply init settings: %d",
					rc);
					return rc;
				}
			}
		}
		/* NonRealTime (Widget/RER/INIT_FIRE settings) */
		if (fctrl->i2c_data.config_settings.is_settings_valid == true) {
			list_for_each_entry(i2c_list,
				&(fctrl->i2c_data.config_settings.list_head),
				list) {
				rc = cam_sensor_util_i2c_apply_setting
					(&(fctrl->io_master_info), i2c_list);
				if (rc) {
					CAM_ERR(CAM_FLASH,
					"Failed to apply NRT settings: %d", rc);
					return rc;
				}
			}
		}
	} else {
		/* RealTime */
		frame_offset = req_id % MAX_PER_FRAME_ARRAY;
		i2c_set = &fctrl->i2c_data.per_frame[frame_offset];
		if ((i2c_set->is_settings_valid == true) &&
			(i2c_set->request_id == req_id)) {
			list_for_each_entry(i2c_list,
				&(i2c_set->list_head), list) {
				rc = cam_sensor_util_i2c_apply_setting(
					&(fctrl->io_master_info), i2c_list);
				if (rc) {
					CAM_ERR(CAM_FLASH,
					"Failed to apply settings: %d", rc);
					return rc;
				}
			}
		}
	}

	cam_flash_i2c_delete_req(fctrl, req_id);
	return rc;
}

int cam_flash_pmic_apply_setting(struct cam_flash_ctrl *fctrl,
	uint64_t req_id)
{
	int rc = 0, i = 0;
	int frame_offset = 0;
	uint16_t num_iterations;
	struct cam_flash_frame_setting *flash_data = NULL;

	if (req_id == 0) {
		if (fctrl->nrt_info.cmn_attr.cmd_type ==
			CAMERA_SENSOR_FLASH_CMD_TYPE_INIT_FIRE) {
			flash_data = &fctrl->nrt_info;
			CAM_DBG(CAM_REQ,
				"FLASH_INIT_FIRE req_id: %u flash_opcode: %d",
				req_id, flash_data->opcode);

			if (flash_data->opcode ==
				CAMERA_SENSOR_FLASH_OP_FIREHIGH) {
				if (fctrl->flash_state ==
					CAM_FLASH_STATE_START) {
					CAM_WARN(CAM_FLASH,
					"Wrong state :Prev state: %d",
					fctrl->flash_state);
					return -EINVAL;
				}

				rc = cam_flash_high(fctrl, flash_data);
				if (rc)
					CAM_ERR(CAM_FLASH,
						"FLASH ON failed : %d", rc);
			}
			if (flash_data->opcode ==
				CAMERA_SENSOR_FLASH_OP_FIRELOW) {
				if (fctrl->flash_state ==
					CAM_FLASH_STATE_START) {
					CAM_WARN(CAM_FLASH,
					"Wrong state :Prev state: %d",
					fctrl->flash_state);
					return -EINVAL;
				}

				rc = cam_flash_low(fctrl, flash_data);
				if (rc)
					CAM_ERR(CAM_FLASH,
						"TORCH ON failed : %d", rc);
			}
			if (flash_data->opcode ==
				CAMERA_SENSOR_FLASH_OP_OFF) {
				rc = cam_flash_off(fctrl);
				if (rc) {
					CAM_ERR(CAM_FLASH,
					"LED OFF FAILED: %d",
					rc);
					return rc;
				}
			}
		} else if (fctrl->nrt_info.cmn_attr.cmd_type ==
			CAMERA_SENSOR_FLASH_CMD_TYPE_WIDGET) {
			flash_data = &fctrl->nrt_info;
			CAM_DBG(CAM_REQ,
				"FLASH_WIDGET req_id: %u flash_opcode: %d",
				req_id, flash_data->opcode);

			if (flash_data->opcode ==
				CAMERA_SENSOR_FLASH_OP_FIRELOW) {
				rc = cam_flash_low(fctrl, flash_data);
				if (rc) {
					CAM_ERR(CAM_FLASH,
						"Torch ON failed : %d",
						rc);
					goto nrt_del_req;
				}
			} else if (flash_data->opcode ==
				CAMERA_SENSOR_FLASH_OP_OFF) {
				rc = cam_flash_off(fctrl);
				if (rc)
					CAM_ERR(CAM_FLASH,
					"LED off failed: %d",
					rc);
			}
		} else if (fctrl->nrt_info.cmn_attr.cmd_type ==
			CAMERA_SENSOR_FLASH_CMD_TYPE_RER) {
			flash_data = &fctrl->nrt_info;
			if (fctrl->flash_state != CAM_FLASH_STATE_START) {
				rc = cam_flash_off(fctrl);
				if (rc) {
					CAM_ERR(CAM_FLASH,
						"Flash off failed: %d",
						rc);
					goto nrt_del_req;
				}
			}
			CAM_DBG(CAM_REQ, "FLASH_RER req_id: %u", req_id);

			num_iterations = flash_data->num_iterations;
			for (i = 0; i < num_iterations; i++) {
				/* Turn On Torch */
				if (fctrl->flash_state ==
					CAM_FLASH_STATE_START) {
					rc = cam_flash_low(fctrl, flash_data);
					if (rc) {
						CAM_ERR(CAM_FLASH,
							"Fire Torch Failed");
						goto nrt_del_req;
					}

					usleep_range(
					flash_data->led_on_delay_ms * 1000,
					flash_data->led_on_delay_ms * 1000 +
						100);
				}
				/* Turn Off Torch */
				rc = cam_flash_off(fctrl);
				if (rc) {
					CAM_ERR(CAM_FLASH,
						"Flash off failed: %d", rc);
					continue;
				}
				fctrl->flash_state = CAM_FLASH_STATE_START;
				usleep_range(
				flash_data->led_off_delay_ms * 1000,
				flash_data->led_off_delay_ms * 1000 + 100);
			}
		}
	} else {
		frame_offset = req_id % MAX_PER_FRAME_ARRAY;
		flash_data = &fctrl->per_frame[frame_offset];
		CAM_DBG(CAM_REQ, "FLASH_RT req_id: %u flash_opcode: %d",
			req_id, flash_data->opcode);

		if ((flash_data->opcode == CAMERA_SENSOR_FLASH_OP_FIREHIGH) &&
			(flash_data->cmn_attr.is_settings_valid) &&
			(flash_data->cmn_attr.request_id == req_id)) {
			/* Turn On Flash */
			if (fctrl->flash_state == CAM_FLASH_STATE_START) {
				rc = cam_flash_high(fctrl, flash_data);
				if (rc) {
					CAM_ERR(CAM_FLASH,
						"Flash ON failed: rc= %d",
						rc);
					goto apply_setting_err;
				}
			}
		} else if ((flash_data->opcode ==
			CAMERA_SENSOR_FLASH_OP_FIRELOW) &&
			(flash_data->cmn_attr.is_settings_valid) &&
			(flash_data->cmn_attr.request_id == req_id)) {
			/* Turn On Torch */
			if (fctrl->flash_state == CAM_FLASH_STATE_START) {
				rc = cam_flash_low(fctrl, flash_data);
				if (rc) {
					CAM_ERR(CAM_FLASH,
						"Torch ON failed: rc= %d",
						rc);
					goto apply_setting_err;
				}
			}
		} else if ((flash_data->opcode == CAMERA_SENSOR_FLASH_OP_OFF) &&
			(flash_data->cmn_attr.is_settings_valid) &&
			(flash_data->cmn_attr.request_id == req_id)) {
			rc = cam_flash_off(fctrl);
			if (rc) {
				CAM_ERR(CAM_FLASH,
					"Flash off failed %d", rc);
				goto apply_setting_err;
			}
		} else if ((flash_data->opcode ==
			CAMERA_SENSOR_FLASH_OP_FIREDURATION) &&
			(flash_data->cmn_attr.is_settings_valid) &&
			(flash_data->cmn_attr.request_id == req_id)) {
			if (fctrl->flash_state == CAM_FLASH_STATE_START) {
				rc = cam_flash_duration(fctrl, flash_data);
				if (rc) {
					CAM_ERR(CAM_FLASH,
						"PreciaseFlash op failed:%d",
						rc);
					goto apply_setting_err;
				}
			}
		} else if (flash_data->opcode == CAM_PKT_NOP_OPCODE) {
			CAM_DBG(CAM_FLASH, "NOP Packet");
		} else {
			rc = -EINVAL;
			CAM_ERR(CAM_FLASH, "Invalid opcode: %d req_id: %llu",
				flash_data->opcode, req_id);
			goto apply_setting_err;
		}
	}

nrt_del_req:
	cam_flash_pmic_delete_req(fctrl, req_id);
apply_setting_err:
	return rc;
}

int cam_flash_i2c_pkt_parser(struct cam_flash_ctrl *fctrl, void *arg)
{
	int rc = 0, i = 0;
	uintptr_t generic_ptr;
	uint32_t total_cmd_buf_in_bytes = 0;
	uint32_t processed_cmd_buf_in_bytes = 0;
	uint16_t cmd_length_in_bytes = 0;
	uint32_t *cmd_buf =  NULL;
	uint32_t *offset = NULL;
	uint32_t frm_offset = 0;
	size_t len_of_buffer;
	size_t remain_len;
	struct cam_flash_init *flash_init = NULL;
	struct common_header  *cmn_hdr = NULL;
	struct cam_control *ioctl_ctrl = NULL;
	struct cam_packet *csl_packet = NULL;
	struct cam_cmd_buf_desc *cmd_desc = NULL;
	struct cam_config_dev_cmd config;
	struct cam_req_mgr_add_request add_req;
	struct i2c_data_settings *i2c_data = NULL;
	struct i2c_settings_array *i2c_reg_settings = NULL;
	struct cam_sensor_power_ctrl_t *power_info = NULL;

	if (!fctrl || !arg) {
		CAM_ERR(CAM_FLASH, "fctrl/arg is NULL");
		return -EINVAL;
	}
	/* getting CSL Packet */
	ioctl_ctrl = (struct cam_control *)arg;

	if (copy_from_user((&config), (void __user *) ioctl_ctrl->handle,
		sizeof(config))) {
		CAM_ERR(CAM_FLASH, "Copy cmd handle from user failed");
		return -EFAULT;
	}

	rc = cam_mem_get_cpu_buf(config.packet_handle,
		&generic_ptr, &len_of_buffer);
	if (rc) {
		CAM_ERR(CAM_FLASH, "Failed in getting the packet : %d", rc);
		return rc;
	}
	remain_len = len_of_buffer;
	if ((sizeof(struct cam_packet) > len_of_buffer) ||
		((size_t)config.offset >= len_of_buffer -
		sizeof(struct cam_packet))) {
		CAM_ERR(CAM_FLASH,
			"Inval cam_packet strut size: %zu, len_of_buff: %zu",
			 sizeof(struct cam_packet), len_of_buffer);
		return -EINVAL;
	}

	remain_len -= (size_t)config.offset;
	/* Add offset to the flash csl header */
	csl_packet = (struct cam_packet *)(generic_ptr + config.offset);

	if (cam_packet_util_validate_packet(csl_packet,
		remain_len)) {
		CAM_ERR(CAM_FLASH, "Invalid packet params");
		return -EINVAL;
	}

	if ((csl_packet->header.op_code & 0xFFFFFF) !=
		CAM_FLASH_PACKET_OPCODE_INIT &&
		csl_packet->header.request_id <= fctrl->last_flush_req
		&& fctrl->last_flush_req != 0) {
		CAM_DBG(CAM_FLASH,
			"reject request %lld, last request to flush %lld",
			csl_packet->header.request_id, fctrl->last_flush_req);
		return -EINVAL;
	}

	if (csl_packet->header.request_id > fctrl->last_flush_req)
		fctrl->last_flush_req = 0;

	switch (csl_packet->header.op_code & 0xFFFFFF) {
	case CAM_FLASH_PACKET_OPCODE_INIT: {
		/* INIT packet*/
		offset = (uint32_t *)((uint8_t *)&csl_packet->payload +
			csl_packet->cmd_buf_offset);
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);

		/* Loop through multiple command buffers */
		for (i = 1; i < csl_packet->num_cmd_buf; i++) {
			total_cmd_buf_in_bytes = cmd_desc[i].length;
			processed_cmd_buf_in_bytes = 0;
			if (!total_cmd_buf_in_bytes)
				continue;
			rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
				&generic_ptr, &len_of_buffer);
			if (rc < 0) {
				CAM_ERR(CAM_FLASH, "Failed to get cpu buf");
				return rc;
			}
			cmd_buf = (uint32_t *)generic_ptr;
			if (!cmd_buf) {
				CAM_ERR(CAM_FLASH, "invalid cmd buf");
				return -EINVAL;
			}

			if ((len_of_buffer < sizeof(struct common_header)) ||
				(cmd_desc[i].offset >
				(len_of_buffer -
				sizeof(struct common_header)))) {
				CAM_ERR(CAM_FLASH, "invalid cmd buf length");
				return -EINVAL;
			}
			remain_len = len_of_buffer - cmd_desc[i].offset;
			cmd_buf += cmd_desc[i].offset / sizeof(uint32_t);
			cmn_hdr = (struct common_header *)cmd_buf;

			/* Loop through cmd formats in one cmd buffer */
			CAM_DBG(CAM_FLASH,
				"command Type: %d,Processed: %d,Total: %d",
				cmn_hdr->cmd_type, processed_cmd_buf_in_bytes,
				total_cmd_buf_in_bytes);
			switch (cmn_hdr->cmd_type) {
			case CAMERA_SENSOR_FLASH_CMD_TYPE_INIT_INFO:
				if (len_of_buffer <
					sizeof(struct cam_flash_init)) {
					CAM_ERR(CAM_FLASH, "Not enough buffer");
					return -EINVAL;
				}

				flash_init = (struct cam_flash_init *)cmd_buf;
				fctrl->flash_type = flash_init->flash_type;
				cmd_length_in_bytes =
					sizeof(struct cam_flash_init);
				processed_cmd_buf_in_bytes +=
					cmd_length_in_bytes;
				cmd_buf += cmd_length_in_bytes/
						sizeof(uint32_t);
				break;
			case CAMERA_SENSOR_CMD_TYPE_I2C_INFO:
				rc = cam_flash_slaveInfo_pkt_parser(
					fctrl, cmd_buf, remain_len);
				if (rc < 0) {
					CAM_ERR(CAM_FLASH,
					"Failed parsing slave info: rc: %d",
					rc);
					return rc;
				}
				cmd_length_in_bytes =
					sizeof(struct cam_cmd_i2c_info);
				processed_cmd_buf_in_bytes +=
					cmd_length_in_bytes;
				cmd_buf += cmd_length_in_bytes/
						sizeof(uint32_t);
				break;
			case CAMERA_SENSOR_CMD_TYPE_PWR_UP:
			case CAMERA_SENSOR_CMD_TYPE_PWR_DOWN:
				CAM_DBG(CAM_FLASH,
					"Received power settings");
				cmd_length_in_bytes =
					total_cmd_buf_in_bytes;
				rc = cam_sensor_update_power_settings(
					cmd_buf,
					total_cmd_buf_in_bytes,
					&fctrl->power_info, remain_len);
				processed_cmd_buf_in_bytes +=
					cmd_length_in_bytes;
				cmd_buf += cmd_length_in_bytes/
						sizeof(uint32_t);
				if (rc) {
					CAM_ERR(CAM_FLASH,
					"Failed update power settings");
					return rc;
				}
				break;
			default:
				CAM_DBG(CAM_FLASH,
					"Received initSettings");
				i2c_data = &(fctrl->i2c_data);
				i2c_reg_settings =
					&fctrl->i2c_data.init_settings;

				i2c_reg_settings->request_id = 0;
				i2c_reg_settings->is_settings_valid = 1;
				rc = cam_sensor_i2c_command_parser(
					&fctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1, NULL);
				if (rc < 0) {
					CAM_ERR(CAM_FLASH,
					"pkt parsing failed: %d", rc);
					return rc;
				}
				cmd_length_in_bytes =
					cmd_desc[i].length;
				processed_cmd_buf_in_bytes +=
					cmd_length_in_bytes;
				cmd_buf += cmd_length_in_bytes/
						sizeof(uint32_t);

				break;
			}
		}
		power_info = &fctrl->power_info;
		if (!power_info) {
			CAM_ERR(CAM_FLASH, "Power_info is NULL");
			return -EINVAL;
		}

		/* Parse and fill vreg params for power up settings */
		rc = msm_camera_fill_vreg_params(&fctrl->soc_info,
			power_info->power_setting,
			power_info->power_setting_size);
		if (rc) {
			CAM_ERR(CAM_FLASH,
				"failed to fill vreg params for power up rc:%d",
				rc);
			return rc;
		}

		/* Parse and fill vreg params for power down settings*/
		rc = msm_camera_fill_vreg_params(
			&fctrl->soc_info,
			power_info->power_down_setting,
			power_info->power_down_setting_size);
		if (rc) {
			CAM_ERR(CAM_FLASH,
				"failed to fill vreg params power down rc:%d",
				rc);
			return rc;
		}

		rc = fctrl->func_tbl.power_ops(fctrl, true);
		if (rc) {
			CAM_ERR(CAM_FLASH,
				"Enable Regulator Failed rc = %d", rc);
			return rc;
		}

		rc = fctrl->func_tbl.apply_setting(fctrl, 0);
		if (rc) {
			CAM_ERR(CAM_FLASH, "cannot apply settings rc = %d", rc);
			return rc;
		}

		fctrl->flash_state = CAM_FLASH_STATE_CONFIG;
		break;
	}
	case CAM_FLASH_PACKET_OPCODE_SET_OPS: {
		offset = (uint32_t *)((uint8_t *)&csl_packet->payload +
			csl_packet->cmd_buf_offset);
		frm_offset = csl_packet->header.request_id %
			MAX_PER_FRAME_ARRAY;
		/* add support for handling i2c_data*/
		i2c_reg_settings =
			&fctrl->i2c_data.per_frame[frm_offset];
		if (i2c_reg_settings->is_settings_valid == true) {
			CAM_DBG(CAM_FLASH, "settings already valid");
			i2c_reg_settings->request_id = 0;
			i2c_reg_settings->is_settings_valid = false;
			goto update_req_mgr;
		}
		i2c_reg_settings->is_settings_valid = true;
		i2c_reg_settings->request_id =
			csl_packet->header.request_id;
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		rc = cam_sensor_i2c_command_parser(
			&fctrl->io_master_info,
			i2c_reg_settings, cmd_desc, 1, NULL);
		if (rc) {
			CAM_ERR(CAM_FLASH,
			"Failed in parsing i2c packets");
			return rc;
		}
		if ((fctrl->flash_state == CAM_FLASH_STATE_ACQUIRE) ||
			(fctrl->flash_state == CAM_FLASH_STATE_CONFIG)) {
			fctrl->flash_state = CAM_FLASH_STATE_CONFIG;
			rc = fctrl->func_tbl.apply_setting(fctrl, 1);
			if (rc) {
				CAM_ERR(CAM_FLASH, "cannot apply fire settings rc = %d", rc);
				return rc;
			}
			return rc;
		}
		break;
	}
	case CAM_FLASH_PACKET_OPCODE_NON_REALTIME_SET_OPS: {
		offset = (uint32_t *)((uint8_t *)&csl_packet->payload +
			csl_packet->cmd_buf_offset);

		/* add support for handling i2c_data*/
		i2c_reg_settings = &fctrl->i2c_data.config_settings;
		if (i2c_reg_settings->is_settings_valid == true) {
			i2c_reg_settings->request_id = 0;
			i2c_reg_settings->is_settings_valid = false;

			rc = delete_request(i2c_reg_settings);
			if (rc) {
				CAM_ERR(CAM_FLASH,
				"Failed in Deleting the err: %d", rc);
				return rc;
			}
		}
		i2c_reg_settings->is_settings_valid = true;
		i2c_reg_settings->request_id =
			csl_packet->header.request_id;
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		rc = cam_sensor_i2c_command_parser(
			&fctrl->io_master_info,
			i2c_reg_settings, cmd_desc, 1, NULL);
		if (rc) {
			CAM_ERR(CAM_FLASH,
			"Failed in parsing i2c NRT packets");
			return rc;
		}
		rc = fctrl->func_tbl.apply_setting(fctrl, 0);
		if (rc)
			CAM_ERR(CAM_FLASH,
			"Apply setting failed: %d", rc);
		return rc;
	}
	case CAM_PKT_NOP_OPCODE: {
		frm_offset = csl_packet->header.request_id %
			MAX_PER_FRAME_ARRAY;
		if ((fctrl->flash_state == CAM_FLASH_STATE_INIT) ||
			(fctrl->flash_state == CAM_FLASH_STATE_ACQUIRE)) {
			CAM_WARN(CAM_FLASH,
				"Rxed NOP packets without linking");
			fctrl->i2c_data.per_frame[frm_offset].is_settings_valid
				= false;
			return 0;
		}
		i2c_reg_settings =
			&fctrl->i2c_data.per_frame[frm_offset];
		i2c_reg_settings->is_settings_valid = true;
		i2c_reg_settings->request_id =
			csl_packet->header.request_id;
		CAM_DBG(CAM_FLASH, "NOP Packet is Received: req_id: %u",
			csl_packet->header.request_id);
		goto update_req_mgr;
	}
	case CAM_FLASH_PACKET_OPCODE_STREAM_OFF: {
		if (fctrl->streamoff_count > 0)
			return rc;

		CAM_DBG(CAM_FLASH, "Received Stream off Settings");
		i2c_data = &(fctrl->i2c_data);
		fctrl->streamoff_count = fctrl->streamoff_count + 1;
		i2c_reg_settings       = &i2c_data->streamoff_settings;
		i2c_reg_settings->request_id = 0;
		i2c_reg_settings->is_settings_valid = 1;
		offset = (uint32_t *)((uint8_t *)&csl_packet->payload +
			csl_packet->cmd_buf_offset);
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		rc = cam_sensor_i2c_command_parser(&fctrl->io_master_info,
				i2c_reg_settings,
				cmd_desc, 1, NULL);
		if (rc) {
			CAM_ERR(CAM_FLASH,
			"Failed in parsing i2c Stream off packets");
			return rc;
		}
		break;
	}
	default:
		CAM_ERR(CAM_FLASH, "Wrong Opcode : %d",
			(csl_packet->header.op_code & 0xFFFFFF));
		return -EINVAL;
	}
update_req_mgr:
	if (((csl_packet->header.op_code  & 0xFFFFF) ==
		CAM_PKT_NOP_OPCODE) ||
		((csl_packet->header.op_code & 0xFFFFF) ==
		CAM_FLASH_PACKET_OPCODE_SET_OPS)) {
		memset(&add_req, 0, sizeof(add_req));
		add_req.link_hdl = fctrl->bridge_intf.link_hdl;
		add_req.req_id = csl_packet->header.request_id;
		add_req.dev_hdl = fctrl->bridge_intf.device_hdl;

		if ((csl_packet->header.op_code & 0xFFFFF) ==
			CAM_FLASH_PACKET_OPCODE_SET_OPS) {
			add_req.trigger_eof = true;
			add_req.skip_at_sof = 1;
		}

		if (fctrl->bridge_intf.crm_cb &&
			fctrl->bridge_intf.crm_cb->add_req) {
			rc = fctrl->bridge_intf.crm_cb->add_req(&add_req);
			if  (rc) {
				CAM_ERR(CAM_FLASH,
					"Failed in adding request: %llu to request manager",
					csl_packet->header.request_id);
				return rc;
			}
			CAM_DBG(CAM_FLASH,
				"add req %lld to req_mgr, trigger_eof %d",
				add_req.req_id, add_req.trigger_eof);
		}
	}
	return rc;
}

int cam_flash_pmic_pkt_parser(struct cam_flash_ctrl *fctrl, void *arg)
{
	int rc = 0, i = 0;
	uintptr_t generic_ptr, cmd_buf_ptr;
	uint32_t *cmd_buf =  NULL;
	uint32_t *offset = NULL;
	uint32_t frm_offset = 0;
	size_t len_of_buffer;
	size_t remain_len;
	struct cam_control *ioctl_ctrl = NULL;
	struct cam_packet *csl_packet = NULL;
	struct cam_cmd_buf_desc *cmd_desc = NULL;
	struct common_header *cmn_hdr;
	struct cam_config_dev_cmd config;
	struct cam_req_mgr_add_request add_req = {0};
	struct cam_flash_init *cam_flash_info = NULL;
	struct cam_flash_set_rer *flash_rer_info = NULL;
	struct cam_flash_set_on_off *flash_operation_info = NULL;
	struct cam_flash_query_curr *flash_query_info = NULL;
	struct cam_flash_frame_setting *flash_data = NULL;
	struct cam_flash_private_soc *soc_private = NULL;

	if (!fctrl || !arg) {
		CAM_ERR(CAM_FLASH, "fctrl/arg is NULL");
		return -EINVAL;
	}

	soc_private = (struct cam_flash_private_soc *)
		fctrl->soc_info.soc_private;

	/* getting CSL Packet */
	ioctl_ctrl = (struct cam_control *)arg;

	if (copy_from_user((&config),
		u64_to_user_ptr(ioctl_ctrl->handle),
		sizeof(config))) {
		CAM_ERR(CAM_FLASH, "Copy cmd handle from user failed");
		rc = -EFAULT;
		return rc;
	}

	rc = cam_mem_get_cpu_buf(config.packet_handle,
		&generic_ptr, &len_of_buffer);
	if (rc) {
		CAM_ERR(CAM_FLASH, "Failed in getting the packet: %d", rc);
		return rc;
	}

	remain_len = len_of_buffer;
	if ((sizeof(struct cam_packet) > len_of_buffer) ||
		((size_t)config.offset >= len_of_buffer -
		sizeof(struct cam_packet))) {
		CAM_ERR(CAM_FLASH,
			"Inval cam_packet strut size: %zu, len_of_buff: %zu",
			 sizeof(struct cam_packet), len_of_buffer);
		rc = -EINVAL;
		return rc;
	}

	remain_len -= (size_t)config.offset;
	/* Add offset to the flash csl header */
	csl_packet = (struct cam_packet *)(generic_ptr + config.offset);

	if (cam_packet_util_validate_packet(csl_packet,
		remain_len)) {
		CAM_ERR(CAM_FLASH, "Invalid packet params");
		rc = -EINVAL;
		return rc;
	}

	if ((csl_packet->header.op_code & 0xFFFFFF) !=
		CAM_FLASH_PACKET_OPCODE_INIT &&
		csl_packet->header.request_id <= fctrl->last_flush_req
		&& fctrl->last_flush_req != 0) {
		CAM_WARN(CAM_FLASH,
			"reject request %lld, last request to flush %d",
			csl_packet->header.request_id, fctrl->last_flush_req);
		rc = -EINVAL;
		return rc;
	}

	if (csl_packet->header.request_id > fctrl->last_flush_req)
		fctrl->last_flush_req = 0;

	switch (csl_packet->header.op_code & 0xFFFFFF) {
	case CAM_FLASH_PACKET_OPCODE_INIT: {
		/* INIT packet*/
		offset = (uint32_t *)((uint8_t *)&csl_packet->payload +
			csl_packet->cmd_buf_offset);
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		rc = cam_mem_get_cpu_buf(cmd_desc->mem_handle,
			&cmd_buf_ptr, &len_of_buffer);
		if (rc) {
			CAM_ERR(CAM_FLASH, "Fail in get buffer: %d", rc);
			return rc;
		}
		if ((len_of_buffer < sizeof(struct cam_flash_init)) ||
			(cmd_desc->offset >
			(len_of_buffer - sizeof(struct cam_flash_init)))) {
			CAM_ERR(CAM_FLASH, "Not enough buffer");
			rc = -EINVAL;
			return rc;
		}
		remain_len = len_of_buffer - cmd_desc->offset;
		cmd_buf = (uint32_t *)((uint8_t *)cmd_buf_ptr +
			cmd_desc->offset);
		cam_flash_info = (struct cam_flash_init *)cmd_buf;

		switch (cam_flash_info->cmd_type) {
		case CAMERA_SENSOR_FLASH_CMD_TYPE_INIT_INFO: {
			CAM_DBG(CAM_FLASH, "INIT_INFO CMD CALLED");
			fctrl->flash_init_setting.cmn_attr.request_id = 0;
			fctrl->flash_init_setting.cmn_attr.is_settings_valid =
				true;
			fctrl->flash_type = cam_flash_info->flash_type;
			fctrl->is_regulator_enabled = false;
			fctrl->nrt_info.cmn_attr.cmd_type =
				CAMERA_SENSOR_FLASH_CMD_TYPE_INIT_INFO;

			fctrl->flash_state =
				CAM_FLASH_STATE_CONFIG;
			break;
		}
		case CAMERA_SENSOR_FLASH_CMD_TYPE_INIT_FIRE: {
			CAM_DBG(CAM_FLASH, "INIT_FIRE Operation");

			if (remain_len < sizeof(struct cam_flash_set_on_off)) {
				CAM_ERR(CAM_FLASH, "Not enough buffer");
				rc = -EINVAL;
				return rc;
			}

			flash_operation_info =
				(struct cam_flash_set_on_off *) cmd_buf;
			if (!flash_operation_info) {
				CAM_ERR(CAM_FLASH,
					"flash_operation_info Null");
				rc = -EINVAL;
				return rc;
			}
			if (flash_operation_info->count >
				CAM_FLASH_MAX_LED_TRIGGERS) {
				CAM_ERR(CAM_FLASH, "led count out of limit");
				rc = -EINVAL;
				return rc;
			}
			fctrl->nrt_info.cmn_attr.count =
				flash_operation_info->count;
			fctrl->nrt_info.cmn_attr.request_id = 0;
			fctrl->nrt_info.opcode =
				flash_operation_info->opcode;
			fctrl->nrt_info.cmn_attr.cmd_type =
				CAMERA_SENSOR_FLASH_CMD_TYPE_INIT_FIRE;
			for (i = 0;
				i < flash_operation_info->count; i++)
				fctrl->nrt_info.led_current_ma[i] =
				flash_operation_info->led_current_ma[i];

			rc = fctrl->func_tbl.apply_setting(fctrl, 0);
			if (rc)
				CAM_ERR(CAM_FLASH,
					"Apply setting failed: %d",
					rc);

			fctrl->flash_state = CAM_FLASH_STATE_CONFIG;
			break;
		}
		default:
			CAM_ERR(CAM_FLASH, "Wrong cmd_type = %d",
				cam_flash_info->cmd_type);
			rc = -EINVAL;
			return rc;
		}
		break;
	}
	case CAM_FLASH_PACKET_OPCODE_SET_OPS: {
		offset = (uint32_t *)((uint8_t *)&csl_packet->payload +
			csl_packet->cmd_buf_offset);
		frm_offset = csl_packet->header.request_id %
			MAX_PER_FRAME_ARRAY;
		flash_data = &fctrl->per_frame[frm_offset];

		if (flash_data->cmn_attr.is_settings_valid == true) {
			flash_data->cmn_attr.request_id = 0;
			flash_data->cmn_attr.is_settings_valid = false;
			for (i = 0; i < flash_data->cmn_attr.count; i++)
				flash_data->led_current_ma[i] = 0;
		}

		flash_data->cmn_attr.request_id = csl_packet->header.request_id;
		flash_data->cmn_attr.is_settings_valid = true;
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		rc = cam_mem_get_cpu_buf(cmd_desc->mem_handle,
			&cmd_buf_ptr, &len_of_buffer);
		if (rc) {
			CAM_ERR(CAM_FLASH, "Fail in get buffer: 0x%x",
				cmd_desc->mem_handle);
			return rc;
		}

		if ((len_of_buffer < sizeof(struct common_header)) ||
			(cmd_desc->offset >
			(len_of_buffer - sizeof(struct common_header)))) {
			CAM_ERR(CAM_FLASH, "not enough buffer");
			rc = -EINVAL;
			return rc;
		}
		remain_len = len_of_buffer - cmd_desc->offset;

		cmd_buf = (uint32_t *)((uint8_t *)cmd_buf_ptr +
			cmd_desc->offset);
		if (!cmd_buf) {
			rc = -EINVAL;
			return rc;
		}
		cmn_hdr = (struct common_header *)cmd_buf;

		switch (cmn_hdr->cmd_type) {
		case CAMERA_SENSOR_FLASH_CMD_TYPE_FIRE: {
			CAM_DBG(CAM_FLASH,
				"CAMERA_SENSOR_FLASH_CMD_TYPE_FIRE cmd called, req:%lld",
				csl_packet->header.request_id);
			if ((fctrl->flash_state == CAM_FLASH_STATE_INIT) ||
				(fctrl->flash_state ==
					CAM_FLASH_STATE_ACQUIRE)) {
				CAM_WARN(CAM_FLASH,
					"Rxed Flash fire ops without linking");
				flash_data->cmn_attr.is_settings_valid = false;
				return -EINVAL;
			}
			if (remain_len < sizeof(struct cam_flash_set_on_off)) {
				CAM_ERR(CAM_FLASH, "Not enough buffer");
				rc = -EINVAL;
				return rc;
			}

			flash_operation_info =
				(struct cam_flash_set_on_off *) cmd_buf;
			if (!flash_operation_info) {
				CAM_ERR(CAM_FLASH,
					"flash_operation_info Null");
				rc = -EINVAL;
				return rc;
			}
			if (flash_operation_info->count >
				CAM_FLASH_MAX_LED_TRIGGERS) {
				CAM_ERR(CAM_FLASH, "led count out of limit");
				rc = -EINVAL;
				return rc;
			}

			flash_data->opcode = flash_operation_info->opcode;
			flash_data->cmn_attr.count =
				flash_operation_info->count;
			for (i = 0; i < flash_operation_info->count; i++)
				flash_data->led_current_ma[i]
				= flash_operation_info->led_current_ma[i];

			CAM_DBG(CAM_FLASH,
				"FLASH_CMD_TYPE op:%d, req:%lld",
				flash_data->opcode, csl_packet->header.request_id);

			if (flash_data->opcode ==
				CAMERA_SENSOR_FLASH_OP_FIREDURATION) {
				add_req.trigger_eof = true;
				/* Active time for the preflash */
				flash_data->flash_active_time_ms =
				div_u64(flash_operation_info->time_on_duration_ns, 1000000);
				CAM_DBG(CAM_FLASH,
					"PRECISE FLASH: active_time: %llu",
					flash_data->flash_active_time_ms);
			}
		}
		break;
		default:
			CAM_ERR(CAM_FLASH, "Wrong cmd_type = %d",
				cmn_hdr->cmd_type);
			rc = -EINVAL;
			return rc;
		}
		break;
	}
	case CAM_FLASH_PACKET_OPCODE_NON_REALTIME_SET_OPS: {
		offset = (uint32_t *)((uint8_t *)&csl_packet->payload +
			csl_packet->cmd_buf_offset);
		fctrl->nrt_info.cmn_attr.is_settings_valid = true;
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		rc = cam_mem_get_cpu_buf(cmd_desc->mem_handle,
			&cmd_buf_ptr, &len_of_buffer);
		if (rc) {
			CAM_ERR(CAM_FLASH, "Fail in get buffer: %d", rc);
			return rc;
		}

		if ((len_of_buffer < sizeof(struct common_header)) ||
			(cmd_desc->offset >
			(len_of_buffer - sizeof(struct common_header)))) {
			CAM_ERR(CAM_FLASH, "Not enough buffer");
			rc = -EINVAL;
			return rc;
		}
		remain_len = len_of_buffer - cmd_desc->offset;
		cmd_buf = (uint32_t *)((uint8_t *)cmd_buf_ptr +
			cmd_desc->offset);
		cmn_hdr = (struct common_header *)cmd_buf;

		switch (cmn_hdr->cmd_type) {
		case CAMERA_SENSOR_FLASH_CMD_TYPE_WIDGET: {
			CAM_DBG(CAM_FLASH, "Widget Flash Operation");
			if (remain_len < sizeof(struct cam_flash_set_on_off)) {
				CAM_ERR(CAM_FLASH, "Not enough buffer");
				rc = -EINVAL;
				return rc;
			}
			flash_operation_info =
				(struct cam_flash_set_on_off *) cmd_buf;
			if (!flash_operation_info) {
				CAM_ERR(CAM_FLASH,
					"flash_operation_info Null");
				rc = -EINVAL;
				return rc;
			}
			if (flash_operation_info->count >
				CAM_FLASH_MAX_LED_TRIGGERS) {
				CAM_ERR(CAM_FLASH, "led count out of limit");
				rc = -EINVAL;
				return rc;
			}

			fctrl->nrt_info.cmn_attr.count =
				flash_operation_info->count;
			fctrl->nrt_info.cmn_attr.request_id = 0;
			fctrl->nrt_info.opcode =
				flash_operation_info->opcode;
			fctrl->nrt_info.cmn_attr.cmd_type =
				CAMERA_SENSOR_FLASH_CMD_TYPE_WIDGET;

			for (i = 0; i < flash_operation_info->count; i++)
				fctrl->nrt_info.led_current_ma[i] =
					flash_operation_info->led_current_ma[i];

			rc = fctrl->func_tbl.apply_setting(fctrl, 0);
			if (rc)
				CAM_ERR(CAM_FLASH, "Apply setting failed: %d",
					rc);
			return rc;
		}
		case CAMERA_SENSOR_FLASH_CMD_TYPE_QUERYCURR: {
			int query_curr_ma = 0;

			if (remain_len < sizeof(struct cam_flash_query_curr)) {
				CAM_ERR(CAM_FLASH, "Not enough buffer");
				rc = -EINVAL;
				return rc;
			}
			flash_query_info =
				(struct cam_flash_query_curr *)cmd_buf;

			rc = cam_flash_led_prepare(fctrl->switch_trigger,
				QUERY_MAX_AVAIL_CURRENT, &query_curr_ma,
				soc_private->is_wled_flash);

			CAM_DBG(CAM_FLASH, "query_curr_ma = %d",
				query_curr_ma);
			if (rc) {
				CAM_ERR(CAM_FLASH,
				"Query current failed with rc=%d", rc);
				return rc;
			}
			flash_query_info->query_current_ma = query_curr_ma;
			break;
		}
		case CAMERA_SENSOR_FLASH_CMD_TYPE_RER: {
			rc = 0;
			if (remain_len < sizeof(struct cam_flash_set_rer)) {
				CAM_ERR(CAM_FLASH, "Not enough buffer");
				rc = -EINVAL;
				return rc;
			}
			flash_rer_info = (struct cam_flash_set_rer *)cmd_buf;
			if (!flash_rer_info) {
				CAM_ERR(CAM_FLASH,
					"flash_rer_info Null");
				rc = -EINVAL;
				return rc;
			}
			if (flash_rer_info->count >
				CAM_FLASH_MAX_LED_TRIGGERS) {
				CAM_ERR(CAM_FLASH, "led count out of limit");
				rc = -EINVAL;
				return rc;
			}

			fctrl->nrt_info.cmn_attr.cmd_type =
				CAMERA_SENSOR_FLASH_CMD_TYPE_RER;
			fctrl->nrt_info.opcode = flash_rer_info->opcode;
			fctrl->nrt_info.cmn_attr.count = flash_rer_info->count;
			fctrl->nrt_info.cmn_attr.request_id = 0;
			fctrl->nrt_info.num_iterations =
				flash_rer_info->num_iteration;
			fctrl->nrt_info.led_on_delay_ms =
				flash_rer_info->led_on_delay_ms;
			fctrl->nrt_info.led_off_delay_ms =
				flash_rer_info->led_off_delay_ms;

			for (i = 0; i < flash_rer_info->count; i++)
				fctrl->nrt_info.led_current_ma[i] =
					flash_rer_info->led_current_ma[i];

			rc = fctrl->func_tbl.apply_setting(fctrl, 0);
			if (rc)
				CAM_ERR(CAM_FLASH, "apply_setting failed: %d",
					rc);
			return rc;
		}
		default:
			CAM_ERR(CAM_FLASH, "Wrong cmd_type : %d",
				cmn_hdr->cmd_type);
			rc = -EINVAL;
			return rc;
		}

		break;
	}
	case CAM_PKT_NOP_OPCODE: {
		frm_offset = csl_packet->header.request_id %
			MAX_PER_FRAME_ARRAY;
		if ((fctrl->flash_state == CAM_FLASH_STATE_INIT) ||
			(fctrl->flash_state == CAM_FLASH_STATE_ACQUIRE)) {
			CAM_WARN(CAM_FLASH,
				"Rxed NOP packets without linking");
			fctrl->per_frame[frm_offset].cmn_attr.is_settings_valid
				= false;
			return -EINVAL;
		}

		fctrl->per_frame[frm_offset].cmn_attr.is_settings_valid = false;
		fctrl->per_frame[frm_offset].cmn_attr.request_id = 0;
		fctrl->per_frame[frm_offset].opcode = CAM_PKT_NOP_OPCODE;
		CAM_DBG(CAM_FLASH, "NOP Packet is Received: req_id: %llu",
			csl_packet->header.request_id);
		break;
	}
	default:
		CAM_ERR(CAM_FLASH, "Wrong Opcode : %d",
			(csl_packet->header.op_code & 0xFFFFFF));
		rc = -EINVAL;
		return rc;
	}

	if (((csl_packet->header.op_code  & 0xFFFFF) ==
		CAM_PKT_NOP_OPCODE) ||
		((csl_packet->header.op_code & 0xFFFFF) ==
		CAM_FLASH_PACKET_OPCODE_SET_OPS)) {
		memset(&add_req, 0, sizeof(add_req));
		add_req.link_hdl = fctrl->bridge_intf.link_hdl;
		add_req.req_id = csl_packet->header.request_id;
		add_req.dev_hdl = fctrl->bridge_intf.device_hdl;

		if ((csl_packet->header.op_code & 0xFFFFF) ==
			CAM_FLASH_PACKET_OPCODE_SET_OPS) {
			add_req.trigger_eof = true;
			if (flash_data->opcode == CAMERA_SENSOR_FLASH_OP_OFF) {
				add_req.skip_at_sof = 1;
				add_req.skip_at_eof = 1;
			} else
				add_req.skip_at_sof = 1;
		}

		if (fctrl->bridge_intf.crm_cb &&
			fctrl->bridge_intf.crm_cb->add_req) {
			rc = fctrl->bridge_intf.crm_cb->add_req(&add_req);
			if  (rc) {
				CAM_ERR(CAM_FLASH,
					"Failed in adding request: %llu to request manager",
					csl_packet->header.request_id);
				return rc;
			}
			CAM_DBG(CAM_FLASH,
				"add req %lld to req_mgr, trigger_eof %d",
				add_req.req_id, add_req.trigger_eof);
		}
	}

	return rc;
}

int cam_flash_publish_dev_info(struct cam_req_mgr_device_info *info)
{
	info->dev_id = CAM_REQ_MGR_DEVICE_FLASH;
	strlcpy(info->name, CAM_FLASH_NAME, sizeof(info->name));
	info->p_delay = CAM_FLASH_PIPELINE_DELAY;
	info->trigger = CAM_TRIGGER_POINT_SOF;
	return 0;
}

int cam_flash_establish_link(struct cam_req_mgr_core_dev_link_setup *link)
{
	struct cam_flash_ctrl *fctrl = NULL;

	if (!link)
		return -EINVAL;

	fctrl = (struct cam_flash_ctrl *)cam_get_device_priv(link->dev_hdl);
	if (!fctrl) {
		CAM_ERR(CAM_FLASH, " Device data is NULL");
		return -EINVAL;
	}
	mutex_lock(&fctrl->flash_mutex);
	if (link->link_enable) {
		fctrl->bridge_intf.link_hdl = link->link_hdl;
		fctrl->bridge_intf.crm_cb = link->crm_cb;
	} else {
		fctrl->bridge_intf.link_hdl = -1;
		fctrl->bridge_intf.crm_cb = NULL;
	}
	mutex_unlock(&fctrl->flash_mutex);

	return 0;
}

int cam_flash_release_dev(struct cam_flash_ctrl *fctrl)
{
	int rc = 0;

	if (fctrl->i2c_data.streamoff_settings.is_settings_valid == true) {
		fctrl->i2c_data.streamoff_settings.is_settings_valid = false;
		rc = delete_request(&fctrl->i2c_data.streamoff_settings);
		if (rc) {
			CAM_WARN(CAM_FLASH,
				"Failed to delete Stream off i2c_setting: %d",
				rc);
		}
	}

	if (fctrl->bridge_intf.device_hdl != 1) {
		rc = cam_destroy_device_hdl(fctrl->bridge_intf.device_hdl);
		if (rc)
			CAM_ERR(CAM_FLASH,
				"Failed in destroying device handle rc = %d",
				rc);
		fctrl->bridge_intf.device_hdl = -1;
		fctrl->bridge_intf.link_hdl = -1;
		fctrl->bridge_intf.session_hdl = -1;
		fctrl->last_flush_req = 0;
		fctrl->streamoff_count = 0;
	}

	return rc;
}

void cam_flash_shutdown(struct cam_flash_ctrl *fctrl)
{
	int rc;

	if (fctrl->flash_state == CAM_FLASH_STATE_INIT)
		return;

	if ((fctrl->flash_state == CAM_FLASH_STATE_CONFIG) ||
		(fctrl->flash_state == CAM_FLASH_STATE_START)) {
		fctrl->func_tbl.flush_req(fctrl, FLUSH_ALL, 0);
		rc = cam_flash_off(fctrl);
		if (rc) {
			CAM_ERR(CAM_FLASH,
				"LED OFF FAILED: %d", rc);
		}
		if (fctrl->func_tbl.power_ops) {
			rc = fctrl->func_tbl.power_ops(fctrl, false);
			if (rc)
				CAM_ERR(CAM_FLASH, "Power Down Failed rc: %d",
					rc);
		}
	}

	rc = cam_flash_release_dev(fctrl);
	if (rc)
		CAM_ERR(CAM_FLASH, "Release failed rc: %d", rc);

	fctrl->flash_state = CAM_FLASH_STATE_INIT;
}

int cam_flash_apply_request(struct cam_req_mgr_apply_request *apply)
{
	int rc = 0;
	struct cam_flash_ctrl *fctrl = NULL;

	if (!apply)
		return -EINVAL;

	fctrl = (struct cam_flash_ctrl *) cam_get_device_priv(apply->dev_hdl);
	if (!fctrl) {
		CAM_ERR(CAM_FLASH, "Device data is NULL");
		return -EINVAL;
	}

	mutex_lock(&fctrl->flash_mutex);
	rc = fctrl->func_tbl.apply_setting(fctrl, apply->request_id);
	if (rc)
		CAM_ERR(CAM_FLASH, "apply_setting failed with rc=%d",
			rc);
	mutex_unlock(&fctrl->flash_mutex);

	return rc;
}
