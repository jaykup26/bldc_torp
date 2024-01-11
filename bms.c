/*
	Copyright 2020 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

/**
 * This is the BMS module of the VESC firmware. It mainly supports the VESC BMS, but
 * the intention is to have it extendible to other BMSs too. The first step is
 * to add the BMS you want to support to the BMS_TYPE enum, and then you need to update
 * this module to interpret CAN-messages from it properly.
 */

#include "bms.h"
#include "buffer.h"
#include "utils.h"
#include "datatypes.h"
#include "comm_can.h"
#include "app.h"
#include "mc_interface.h"
#include <string.h>
#include <math.h>

// Settings
#define MAX_CAN_AGE_SEC				2.0

// Private variables
static volatile bms_config m_conf;
static volatile bms_values m_values;
static volatile bms_info m_info;
static volatile bms_peak m_peak;
static volatile bms_soc_soh_temp_stat m_stat_temp_max;
static volatile bms_soc_soh_temp_stat m_stat_soc_min;
static volatile bms_soc_soh_temp_stat m_stat_soc_max;

static volatile bool init_done = false;

void bms_init(bms_config *conf) {
	m_conf = *conf;

	if (!init_done) {
		memset((void*)&m_values, 0, sizeof(m_values));
		memset((void*)&m_info, 0, sizeof(m_info));
		memset((void*)&m_stat_temp_max, 0, sizeof(m_stat_temp_max));
		memset((void*)&m_stat_soc_min, 0, sizeof(m_stat_soc_min));
		memset((void*)&m_stat_soc_max, 0, sizeof(m_stat_soc_max));
		memset((void*)&m_values, 0, sizeof(m_values));
		memset((void*)&m_peak, 0, sizeof(m_peak));
		m_values.can_id = -1;
		m_stat_temp_max.id = -1;
		m_stat_soc_min.id = -1;
		m_stat_soc_max.id = -1;
		init_done = true;
	}
}

bool bms_process_can_frame(uint32_t can_id, uint8_t *data8, int len, bool is_ext) {
	bool used_data = false;

	if (m_conf.type == BMS_TYPE_VESC) {
		if (is_ext) {
			uint8_t id = can_id & 0xFF;
			CAN_PACKET_ID cmd = can_id >> 8;

			switch (cmd) {
			case CAN_PACKET_BMS_SOC_SOH_TEMP_STAT: {
				used_data = true;

				int32_t ind = 0;
				bms_soc_soh_temp_stat msg;
				msg.id = id;
				msg.rx_time = chVTGetSystemTime();
				msg.v_cell_min = buffer_get_float16(data8, 1e3, &ind);
				msg.v_cell_max = buffer_get_float16(data8, 1e3, &ind);
				msg.soc = ((float)((uint8_t)data8[ind++])) / 255.0;
				msg.soh = ((float)((uint8_t)data8[ind++])) / 255.0;
				msg.t_cell_max = (float)((int8_t)data8[ind++]);
				uint8_t stat = data8[ind++];
				msg.is_charging = (stat >> 0) & 1;
				msg.is_balancing = (stat >> 1) & 1;
				msg.is_charge_allowed = (stat >> 2) & 1;

				if (id == m_values.can_id || UTILS_AGE_S(m_values.update_time) > MAX_CAN_AGE_SEC) {
					m_values.can_id = id;
					m_values.update_time = chVTGetSystemTimeX();
					m_values.soc = msg.soc;
					m_values.soh = msg.soh;
					m_values.temp_max_cell = msg.t_cell_max;
				}

				// In case there is more than one BMS, keep track of the limiting
				// values for all of them.

				if (m_stat_temp_max.id < 0 ||
						UTILS_AGE_S(m_stat_temp_max.rx_time) > MAX_CAN_AGE_SEC ||
						m_stat_temp_max.t_cell_max < msg.t_cell_max) {
					m_stat_temp_max = msg;
				} else if (m_stat_temp_max.id == msg.id) {
					m_stat_temp_max = msg;
				}

				if (m_stat_soc_min.id < 0 ||
						UTILS_AGE_S(m_stat_soc_min.rx_time) > MAX_CAN_AGE_SEC ||
						m_stat_soc_min.soc > msg.soc) {
					m_stat_soc_min = msg;
				} else if (m_stat_soc_min.id == msg.id) {
					m_stat_soc_min = msg;
				}

				if (m_stat_soc_max.id < 0 ||
						UTILS_AGE_S(m_stat_soc_max.rx_time) > MAX_CAN_AGE_SEC ||
						m_stat_soc_max.soc < msg.soc) {
					m_stat_soc_max = msg;
				} else if (m_stat_soc_max.id == msg.id) {
					m_stat_soc_max = msg;
				}
			} break;

			case CAN_PACKET_BMS_V_TOT: {
				used_data = true;

				if (id == m_values.can_id || UTILS_AGE_S(m_values.update_time) > MAX_CAN_AGE_SEC) {
					int32_t ind = 0;
					m_values.can_id = id;
					m_values.update_time = chVTGetSystemTimeX();
					m_values.v_tot = buffer_get_float32_auto(data8, &ind);
					m_values.v_charge = buffer_get_float32_auto(data8, &ind);
				}
			} break;

			case CAN_PACKET_BMS_I: {
				used_data = true;

				if (id == m_values.can_id || UTILS_AGE_S(m_values.update_time) > MAX_CAN_AGE_SEC) {
					int32_t ind = 0;
					m_values.can_id = id;
					m_values.update_time = chVTGetSystemTimeX();
					m_values.i_in = buffer_get_float32_auto(data8, &ind);
					m_values.i_in_ic = buffer_get_float32_auto(data8, &ind);
				}
			} break;

			case CAN_PACKET_BMS_AH_WH: {
				used_data = true;

				if (id == m_values.can_id || UTILS_AGE_S(m_values.update_time) > MAX_CAN_AGE_SEC) {
					int32_t ind = 0;
					m_values.can_id = id;
					m_values.update_time = chVTGetSystemTimeX();
					m_values.ah_cnt = buffer_get_float32_auto(data8, &ind);
					m_values.wh_cnt = buffer_get_float32_auto(data8, &ind);
				}
			} break;

			case CAN_PACKET_BMS_V_CELL: {
				used_data = true;

				if (id == m_values.can_id || UTILS_AGE_S(m_values.update_time) > MAX_CAN_AGE_SEC) {
					int32_t ind = 0;
					m_values.can_id = id;
					m_values.update_time = chVTGetSystemTimeX();
					unsigned int ofs = data8[ind++];
					m_values.cell_num = data8[ind++];

					while(ind < len) {
						if (ofs >= (sizeof(m_values.v_cell) / sizeof(float))) {
							// Out of buffer space
							break;
						}

						m_values.v_cell[ofs++] = buffer_get_float16(data8, 1e3, &ind);
					}
				}
			} break;

			case CAN_PACKET_BMS_BAL: {
				used_data = true;

				if (id == m_values.can_id || UTILS_AGE_S(m_values.update_time) > MAX_CAN_AGE_SEC) {
					int32_t ind = 0;
					m_values.can_id = id;
					m_values.update_time = chVTGetSystemTimeX();

					int cell_num = data8[0];
					uint64_t bal_state_0 = buffer_get_uint32(data8, &ind);
					bal_state_0 &= 0x00FFFFFF;
					uint64_t bal_state_1 = buffer_get_uint32(data8, &ind);
					uint64_t bal_state = bal_state_0 << 32 | bal_state_1;
					ind = 0;

					while (ind < (int)(sizeof(m_values.bal_state) / sizeof(bool)) && ind < cell_num) {
						m_values.bal_state[ind] = (bal_state >> ind) & 1;
						ind++;
					}
				}
			} break;

			case CAN_PACKET_BMS_TEMPS: {
				used_data = true;

				if (id == m_values.can_id || UTILS_AGE_S(m_values.update_time) > MAX_CAN_AGE_SEC) {
					int32_t ind = 0;
					m_values.can_id = id;
					m_values.update_time = chVTGetSystemTimeX();
					unsigned int ofs = data8[ind++];
					m_values.temp_adc_num = data8[ind++];

					while(ind < len) {
						if (ofs >= (sizeof(m_values.temps_adc) / sizeof(float))) {
							// Out of buffer space
							break;
						}

						m_values.temps_adc[ofs++] = buffer_get_float16(data8, 1e2, &ind);
					}
				}
			} break;

			case CAN_PACKET_BMS_HUM: {
				used_data = true;

				if (id == m_values.can_id || UTILS_AGE_S(m_values.update_time) > MAX_CAN_AGE_SEC) {
					int32_t ind = 0;
					m_values.can_id = id;
					m_values.update_time = chVTGetSystemTimeX();
					m_values.temp_hum = buffer_get_float16(data8, 1e2, &ind);
					m_values.hum = buffer_get_float16(data8, 1e2, &ind);
					m_values.temp_ic = buffer_get_float16(data8, 1e2, &ind);
				}
			} break;

			default:
				break;
			}
		}
	}

	return used_data;
}

void bms_update_limits(float *i_in_min, float *i_in_max,
		float i_in_min_conf, float i_in_max_conf) {
	float i_in_min_bms = i_in_min_conf;
	float i_in_max_bms = i_in_max_conf;

	if (m_conf.type == BMS_TYPE_VESC) {

		// Temperature
		if (UTILS_AGE_S(m_stat_temp_max.rx_time) < MAX_CAN_AGE_SEC) {
			float temp = m_stat_temp_max.t_cell_max;

			if (temp < m_conf.t_limit_start) {
				// OK
			}
			else if (temp > m_conf.t_limit_end) {
				i_in_min_bms = 0.0;
				i_in_max_bms = 0.0;
				// Maybe add fault code?
	//			mc_interface_fault_stop(FAULT_CODE_OVER_TEMP_FET, false, false);
			}
			else {
				float maxc = fabsf(i_in_max_conf);
				if (fabsf(i_in_min_conf) > maxc) {
					maxc = fabsf(i_in_min_conf);
				}

				maxc = utils_map(temp, m_conf.t_limit_start, m_conf.t_limit_end, maxc, 0.0);

				if (fabsf(i_in_min_bms) > maxc) {
					i_in_min_bms = SIGN(i_in_min_bms) * maxc;
				}

				if (fabsf(i_in_max_bms) > maxc) {
					i_in_max_bms = SIGN(i_in_max_bms) * maxc;
				}
			}
		}

		// TODO: add support for conf->l_temp_accel_dec to still have braking.

		// SOC
		if (UTILS_AGE_S(m_stat_soc_min.rx_time) < MAX_CAN_AGE_SEC) {
			float soc = m_stat_soc_min.soc;

			if (soc > m_conf.soc_limit_start) {
				// OK
			}
			else if (soc < m_conf.soc_limit_end) {
				i_in_max_bms = 0.0;
			}
			else {
				i_in_max_bms = utils_map(soc, m_conf.soc_limit_start,
					m_conf.soc_limit_end, i_in_max_conf, 0.0);
			}
		}
	}

	mc_interface_clear_warning(WARNING_CODE_34);
	if (m_conf.type == BMS_TYPE_SURRON) { 
		suron_conf sur_conf = app_get_configuration()->app_suron_conf;
		mc_interface_clear_warning(WARNING_CODE_15);
		mc_interface_clear_warning(WARNING_CODE_9);
		mc_interface_clear_warning(WARNING_CODE_30);
		mc_interface_clear_warning(WARNING_CODE_31);
		mc_interface_clear_warning(WARNING_CODE_32);

		if (m_values.update_time && UTILS_AGE_S(m_values.update_time) < 5.0) {
			float t2 = m_values.temps_adc[1];
			float t3 = m_values.temps_adc[2];
			float t_dis_fet = m_values.temps_adc[3];
			float max_temp;
			
			if (t2 >= t3) {
				max_temp = t2;
			} else {
				max_temp = t3;
			}

			float i_in_max_bms_t_cell = i_in_max_bms;
			if (max_temp < m_conf.t_limit_start) {
				// OK
			} else if (max_temp > m_conf.t_limit_end) {
				i_in_max_bms_t_cell = 0.0;
				mc_interface_set_warning(WARNING_CODE_15);
			} else {
				i_in_max_bms_t_cell = utils_map(max_temp, m_conf.t_limit_start, m_conf.t_limit_end, i_in_max_conf, 0.0);
				mc_interface_set_warning(WARNING_CODE_9);
			}

			float i_in_max_bms_t_fet = i_in_max_bms;
			if (t_dis_fet < m_conf.t_dis_limit_start) {
				// OK
			} else if (t_dis_fet > m_conf.t_dis_limit_end) {
				i_in_max_bms_t_fet = 0.0;
				mc_interface_set_warning(WARNING_CODE_31);
			} else {
				i_in_max_bms_t_fet = utils_map(t_dis_fet, m_conf.t_dis_limit_start, m_conf.t_dis_limit_end, i_in_max_conf, 0.0);
				mc_interface_set_warning(WARNING_CODE_30);
			}

			float i_in_min_bms_t_max = i_in_min_bms;
			if (max_temp >= 58.0 || ((m_values.live_error >> 19) & 1)) {
				i_in_min_bms_t_max = 0.0;
				mc_interface_set_warning(WARNING_CODE_32);
			}

			float i_in_max_bms_war11 = i_in_max_bms;
			float i_in_min_bms_war13 = i_in_min_bms;
			if (sur_conf.app_battery_type == SURON_BATTERY_TYPE_STOCK) {

				static systime_t timer_start = 0;
				static systime_t last_time = 0;
				static systime_t last_time_down = 0;
				static bool was_in_war11_limit = false;
				static float pwr_ramp_up = 0.0;
				static float pwr_ramp_down = 0.0;

				uint32_t war11_time = 4000;
				float pwr_ramp_down_target = 85.0;
				float pwr_ramp_up_case = 70.0;

				bool war_11 = (m_values.live_warning >> 11) & 1;

				if (m_info.battery_type == 1) {
					war11_time = 3500;
					pwr_ramp_down_target = 100.0;
					pwr_ramp_up_case = 95.0;

					if ((timer_start && -m_values.i_in < 50.0f) || (timer_start && -m_values.i_in < 110.0f && (chVTTimeElapsedSinceX(timer_start) < MS2ST(war11_time)))) {
						war_11 = false;
					} else if ((!timer_start && -m_values.i_in > 110.0f) || (timer_start && -m_values.i_in > 50.0f)) {
						war_11 = true;
					} 
				}

				if (!timer_start && war_11) {
					timer_start = chVTGetSystemTime();
				}

				if ((timer_start && chVTTimeElapsedSinceX(timer_start) > MS2ST(war11_time)) || (timer_start && !war_11)) {
					if (!war_11) {
						timer_start = 0;
						last_time_down = 0;
					} else {
						if (!last_time_down) {
							last_time_down = chVTGetSystemTimeX();
							pwr_ramp_down = i_in_max_conf;
						}

						const float ramp_step_down = (float)ST2MS(chVTTimeElapsedSinceX(last_time_down)) / (15.0);
						utils_step_towards(&pwr_ramp_down, pwr_ramp_down_target, ramp_step_down);
						last_time_down = chVTGetSystemTimeX();
						i_in_max_bms_war11 = pwr_ramp_down;

						was_in_war11_limit = true;
						pwr_ramp_up = pwr_ramp_down_target;
						last_time = chVTGetSystemTimeX();
					}
				}

				if (!timer_start && was_in_war11_limit && mc_interface_get_tot_current_in_filtered() > pwr_ramp_up_case) {
					const float ramp_step = (float)ST2MS(chVTTimeElapsedSinceX(last_time)) / (25.0);
					utils_step_towards(&pwr_ramp_up, i_in_max_conf, ramp_step);
					last_time = chVTGetSystemTimeX();
					i_in_max_bms_war11 = pwr_ramp_up;
				} else if (!timer_start) {
					was_in_war11_limit = false;
				}

				static systime_t timer_start_regen = 0;
				static systime_t last_time_regen = 0;
				static systime_t last_time_down_regen = 0;
				static bool was_in_war13_regen_limit = false;
				static float pwr_ramp_up_regen = 0.0;
				static float pwr_ramp_down_regen = 0.0;

				uint32_t war13_time_regen = 7000;
				float pwr_ramp_down_regen_target = -18.0;
				float pwr_ramp_up_regen_case = -15.0;

				bool war_13 = (m_values.live_warning >> 13) & 1;

				if (m_info.battery_type == 1) { 
					if ((timer_start_regen && -m_values.i_in > -12.0f) || (timer_start_regen && -m_values.i_in > -20.0f && (chVTTimeElapsedSinceX(timer_start_regen) < MS2ST(war13_time_regen)))) {
						war_13 = false;
					} else if ((!timer_start_regen && -m_values.i_in < -20.0f) || (timer_start_regen && -m_values.i_in < -12.0f)) {
						war_13 = true;
					}
				}

				if (!timer_start_regen && war_13) {
					timer_start_regen = chVTGetSystemTime();
				}

				if ((timer_start_regen && chVTTimeElapsedSinceX(timer_start_regen) > MS2ST(war13_time_regen)) || (timer_start_regen && !war_13)) {
					if (!war_13) {
						timer_start_regen = 0;
						last_time_down_regen = 0;
					} else {
						if (!last_time_down_regen) {
							last_time_down_regen = chVTGetSystemTimeX();
							pwr_ramp_down_regen = i_in_min_conf;
						}

						const float ramp_step_down = (float)ST2MS(chVTTimeElapsedSinceX(last_time_down_regen)) / (15.0);
						utils_step_towards(&pwr_ramp_down_regen, pwr_ramp_down_regen_target, ramp_step_down);
						last_time_down_regen = chVTGetSystemTimeX();
						i_in_min_bms_war13 = pwr_ramp_down_regen;

						was_in_war13_regen_limit = true;
						pwr_ramp_up_regen = pwr_ramp_down_regen_target;
						last_time_regen = chVTGetSystemTimeX();
					}
				}

				if (!timer_start_regen && was_in_war13_regen_limit && mc_interface_get_tot_current_in_filtered() < pwr_ramp_up_regen_case) {
					const float ramp_step = (float)ST2MS(chVTTimeElapsedSinceX(last_time_regen)) / (25.0);
					utils_step_towards(&pwr_ramp_up_regen, i_in_min_conf, ramp_step);
					last_time_regen = chVTGetSystemTimeX();
					i_in_min_bms_war13 = pwr_ramp_up_regen;
				} else if (!timer_start_regen) {
					was_in_war13_regen_limit = false;
				}
			}

			i_in_max_bms = utils_min_abs(i_in_max_bms, i_in_max_bms_t_cell);
			i_in_max_bms = utils_min_abs(i_in_max_bms, i_in_max_bms_t_fet);
			i_in_max_bms = utils_min_abs(i_in_max_bms, i_in_max_bms_war11);
			i_in_min_bms = utils_min_abs(i_in_min_bms, i_in_min_bms_t_max);
			i_in_min_bms = utils_min_abs(i_in_min_bms, i_in_min_bms_war13);
		} else if (sur_conf.app_battery_type == SURON_BATTERY_TYPE_STOCK) {
			if (m_info.battery_type != 1) {
				if (i_in_max_conf > MCCONF_L_IN_CURRENT_MAX) {
					i_in_max_bms = MCCONF_L_IN_CURRENT_MAX;
					mc_interface_set_warning(WARNING_CODE_34);
				}
			} else {
				if (i_in_max_conf > 100.0) {
					i_in_max_bms = 100.0;
					mc_interface_set_warning(WARNING_CODE_34);
				}
			}
			i_in_min_bms = -20.0;
		}
	}

	if (fabsf(i_in_min_bms) < fabsf(*i_in_min)) {
		*i_in_min = i_in_min_bms;
	}

	if (fabsf(i_in_max_bms) < fabsf(*i_in_max)) {
		*i_in_max = i_in_max_bms;
	}
}

void bms_process_cmd(unsigned char *data, unsigned int len,
		void(*reply_func)(unsigned char *data, unsigned int len)) {
	COMM_PACKET_ID packet_id;

	packet_id = data[0];
	data++;
	len--;

	switch (packet_id) {
	case COMM_BMS_GET_VALUES: {
		int32_t ind = 0;
		uint8_t send_buffer[128];

		send_buffer[ind++] = packet_id;

		buffer_append_float32(send_buffer, GET_INPUT_VOLTAGE(), 1e6, &ind);
		buffer_append_float32(send_buffer, m_values.v_charge, 1e6, &ind);
		buffer_append_float32(send_buffer, m_values.i_in, 1e6, &ind);
		buffer_append_float32(send_buffer, m_values.i_in_ic, 1e6, &ind);
		buffer_append_float32(send_buffer, m_values.ah_cnt, 1e3, &ind);
		buffer_append_float32(send_buffer, m_values.wh_cnt, 1e3, &ind);

		// Cell voltages
		send_buffer[ind++] = m_values.cell_num;
		for (int i = 0;i < m_values.cell_num;i++) {
			buffer_append_float16(send_buffer, m_values.v_cell[i], 1e3, &ind);
		}

		// Balancing state
		for (int i = 0;i < m_values.cell_num;i++) {
			send_buffer[ind++] = m_values.bal_state[i];
		}

		// Temperatures
		send_buffer[ind++] = m_values.temp_adc_num;
		for (int i = 0;i < m_values.temp_adc_num;i++) {
			buffer_append_float16(send_buffer, m_values.temps_adc[i], 1e2, &ind);
		}
		buffer_append_float16(send_buffer, m_values.temp_ic, 1e2, &ind);

		// Humidity
		buffer_append_float16(send_buffer, m_values.temp_hum, 1e2, &ind);
		buffer_append_float16(send_buffer, m_values.hum, 1e2, &ind);

		// Highest cell temperature
		buffer_append_float16(send_buffer, m_values.temp_max_cell, 1e2, &ind);

		// State of charge and state of health
		buffer_append_float16(send_buffer, m_values.soc, 1e3, &ind);
		buffer_append_float16(send_buffer, m_values.soh, 1e3, &ind);

		buffer_append_uint32(send_buffer, m_values.live_error, &ind);
		buffer_append_uint32(send_buffer, m_values.live_warning, &ind);
		buffer_append_float32(send_buffer, UTILS_AGE_S(m_values.update_time), 1e1, &ind);
		send_buffer[ind++] = app_get_configuration()->app_suron_conf.app_battery_type;
		send_buffer[ind++] = m_info.battery_type;

		reply_func(send_buffer, ind);
	} break;
	case COMM_BMS_GET_INFO:{
	    int32_t ind = 0;
	    uint8_t send_buffer[130];
	    send_buffer[ind++] = packet_id;

		buffer_append_uint16(send_buffer, m_info.full_cap, &ind);
		buffer_append_uint16(send_buffer, m_info.remain_cap, &ind);
		buffer_append_uint16(send_buffer, m_info.cycle_count, &ind);
		send_buffer[ind++] = m_info.hardware[0];
		send_buffer[ind++] = m_info.hardware[1];
		send_buffer[ind++] = m_info.software[0];
		send_buffer[ind++] = m_info.software[1];
		send_buffer[ind++] = m_info.max_temp;
		send_buffer[ind++] = m_info.min_temp;
		buffer_append_uint16(send_buffer, m_info.max_cell_volt, &ind);
		buffer_append_uint16(send_buffer, m_info.min_cell_volt, &ind);
		buffer_append_int32(send_buffer, m_info.max_dis_current, &ind);
		buffer_append_int32(send_buffer, m_info.max_ch_current, &ind);
		for (int i =0; i<32; i++) {
			send_buffer[ind++] = m_info.sn[i];
		}
		send_buffer[ind++] = m_info.manu_date[0];
		send_buffer[ind++] = m_info.manu_date[1];
		send_buffer[ind++] = m_info.manu_date[2];
		send_buffer[ind++] = m_info.rtc[0];
		send_buffer[ind++] = m_info.rtc[1];
		send_buffer[ind++] = m_info.rtc[2];
		send_buffer[ind++] = m_info.rtc[3];
		send_buffer[ind++] = m_info.rtc[4];
		send_buffer[ind++] = m_info.rtc[5];
		for (int i = 0; i < 64; i++) {
			send_buffer[ind++] = m_info.error_counter[i];
		}

		reply_func(send_buffer, ind);
	} break;
	case COMM_BMS_GET_READ_ADDRESS: {
		app_surron_bms_commands_process_packet(data, len, 3, reply_func);
	} break;
	default:
		break;
	}

	if (m_conf.type == BMS_TYPE_VESC && UTILS_AGE_S(m_values.update_time) < MAX_CAN_AGE_SEC) {
		switch (packet_id) {
		case COMM_BMS_SET_CHARGE_ALLOWED:
		case COMM_BMS_SET_BALANCE_OVERRIDE:
		case COMM_BMS_RESET_COUNTERS:
		case COMM_BMS_FORCE_BALANCE:
		case COMM_BMS_ZERO_CURRENT_OFFSET: {
			comm_can_send_buffer(m_values.can_id, data - 1, len + 1, 0);
		} break;

		default:
			break;
		}
	}

}

bms_values *bms_get_values(void) {
	return (bms_values*)&m_values;
}

bms_info* bms_get_info(void) {
	return (bms_info*)&m_info;
}

bms_peak* bms_get_peak(bool reset) {
	if (reset) {
		memset((void*)&m_peak, 0, sizeof(m_peak));
	}
	return (bms_peak*)&m_peak;
}
