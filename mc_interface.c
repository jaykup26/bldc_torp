/*
	Copyright 2016 - 2020 Benjamin Vedder	benjamin@vedder.se

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

#include "mc_interface.h"
#include "mcpwm.h"
#include "mcpwm_foc.h"
#include "ledpwm.h"
#include "stm32f4xx_conf.h"
#include "hw.h"
#include "terminal.h"
#include "utils.h"
#include "ch.h"
#include "hal.h"
#include "commands.h"
#include "encoder.h"
#include "drv8301.h"
#include "drv8320s.h"
#include "drv8323s.h"
#include "buffer.h"
#include "gpdrive.h"
#include "comm_can.h"
#include "shutdown.h"
#include "app.h"
#include "utils.h"
#include "mempools.h"
#include "crc.h"
#include "bms.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

// Macros
#define DIR_MULT		(motor_now()->m_conf.m_invert_direction ? -1.0 : 1.0)

// Global variables
volatile uint16_t ADC_Value[HW_ADC_CHANNELS + HW_ADC_CHANNELS_EXTRA];
volatile int ADC_curr_norm_value[6];

typedef struct {
	volatile mc_configuration m_conf;
	mc_fault_code m_fault_now;
	int m_ignore_iterations;
	unsigned int m_cycles_running;
	bool m_lock_enabled;
	bool m_lock_override_once;
	float m_motor_current_sum;
	float m_input_current_sum;
	float m_motor_current_iterations;
	float m_input_current_iterations;
	float m_motor_id_sum;
	float m_motor_iq_sum;
	float m_motor_id_iterations;
	float m_motor_iq_iterations;
	float m_motor_vd_sum;
	float m_motor_vq_sum;
	float m_motor_vd_iterations;
	float m_motor_vq_iterations;
	float m_amp_seconds;
	float m_amp_seconds_charged;
	float m_watt_seconds;
	float m_watt_seconds_charged;
	float m_position_set;
	float m_temp_fet;
	float m_temp_motor;
	float m_gate_driver_voltage;
	float m_motor_current_unbalance;
	float m_motor_current_unbalance_error_rate;
	float m_f_samp_now;
	uint64_t m_warning;
	uint64_t m_warning_old;

	// Backup data counters
	uint32_t m_odometer_last;
} motor_if_state_t;

// Private variables
static volatile motor_if_state_t m_motor_1;
#ifdef HW_HAS_DUAL_MOTORS
static volatile motor_if_state_t m_motor_2;
#endif

// Sampling variables
#define ADC_SAMPLE_MAX_LEN		2000
__attribute__((section(".ram4"))) static volatile int16_t m_curr0_samples[ADC_SAMPLE_MAX_LEN];
__attribute__((section(".ram4"))) static volatile int16_t m_curr1_samples[ADC_SAMPLE_MAX_LEN];
__attribute__((section(".ram4"))) static volatile int16_t m_ph1_samples[ADC_SAMPLE_MAX_LEN];
__attribute__((section(".ram4"))) static volatile int16_t m_ph2_samples[ADC_SAMPLE_MAX_LEN];
__attribute__((section(".ram4"))) static volatile int16_t m_ph3_samples[ADC_SAMPLE_MAX_LEN];
__attribute__((section(".ram4"))) static volatile int16_t m_vzero_samples[ADC_SAMPLE_MAX_LEN];
__attribute__((section(".ram4"))) static volatile uint8_t m_status_samples[ADC_SAMPLE_MAX_LEN];
__attribute__((section(".ram4"))) static volatile int16_t m_curr_fir_samples[ADC_SAMPLE_MAX_LEN];
__attribute__((section(".ram4"))) static volatile int16_t m_f_sw_samples[ADC_SAMPLE_MAX_LEN];
__attribute__((section(".ram4"))) static volatile int8_t m_phase_samples[ADC_SAMPLE_MAX_LEN];

__attribute__((section(".ram5"))) volatile backup_data g_backup;
__attribute__((section(".ram5"))) volatile uint8_t peak_histoy_bkp_sram_num;
__attribute__((section(".ram5"))) volatile uint16_t peak_histoy_bkp_sram_cnt;
__attribute__((section(".ram5"))) volatile mc_peak_history_t mc_peak_history[MC_PEAK_HISTORY_LEN];

static volatile int m_sample_len;
static volatile int m_sample_int;
static volatile debug_sampling_mode m_sample_mode;
static volatile debug_sampling_mode m_sample_mode_last;
static volatile int m_sample_now;
static volatile int m_sample_trigger;
static volatile float m_last_adc_duration_sample;
static volatile bool m_sample_is_second_motor;
static volatile mc_fault_code m_fault_stop_fault;
static volatile bool m_fault_stop_is_second_motor;

static volatile float max_throttle_rise = 0.0;
static volatile int throttle_skip_num = 15;
static volatile bool fram_comm_ok = true;

volatile mc_peak_t mc_peak;
volatile mc_peak_history_t mc_peak_history_tmp;
volatile fram_data_t fram_data;
volatile fram_data_t fram_data_temp;

// Private functions
static void update_override_limits(volatile motor_if_state_t *motor, volatile mc_configuration *conf);
static void run_timer_tasks(volatile motor_if_state_t *motor);
static volatile motor_if_state_t *motor_now(void);

// Function pointers
static void(*pwn_done_func)(void) = 0;
static void(* volatile send_func_sample)(unsigned char* data, unsigned int len) = 0;

// Threads
static THD_WORKING_AREA(timer_thread_wa, 1024);
static THD_FUNCTION(timer_thread, arg);
static THD_WORKING_AREA(sample_send_thread_wa, 512);
static THD_FUNCTION(sample_send_thread, arg);
static thread_t *sample_send_tp;
static THD_WORKING_AREA(fault_stop_thread_wa, 512);
static THD_FUNCTION(fault_stop_thread, arg);
static thread_t *fault_stop_tp;
static THD_WORKING_AREA(fram_thread_wa, 512);
static THD_FUNCTION(fram_thread, arg);

void mc_interface_init(void) {
	memset((void*)&m_motor_1, 0, sizeof(motor_if_state_t));
#ifdef HW_HAS_DUAL_MOTORS
	memset((void*)&m_motor_2, 0, sizeof(motor_if_state_t));
#endif

	conf_general_read_mc_configuration((mc_configuration*)&m_motor_1.m_conf, false);
#ifdef HW_HAS_DUAL_MOTORS
	conf_general_read_mc_configuration((mc_configuration*)&m_motor_2.m_conf, true);
#endif

#ifdef HW_HAS_DUAL_MOTORS
	m_motor_1.m_conf.motor_type = MOTOR_TYPE_FOC;
	m_motor_2.m_conf.motor_type = MOTOR_TYPE_FOC;
#endif

	m_last_adc_duration_sample = 0.0;
	m_sample_len = 1000;
	m_sample_int = 1;
	m_sample_now = 0;
	m_sample_trigger = 0;
	m_sample_mode = DEBUG_SAMPLING_OFF;
	m_sample_mode_last = DEBUG_SAMPLING_OFF;
	m_sample_is_second_motor = false;

	//check odometer in sram
	if (g_backup.odometer_init_flag != BACKUP_VAR_INIT_CODE) {
		g_backup.odometer = 0;
		g_backup.odometer_init_flag = BACKUP_VAR_INIT_CODE;
		fault_vesc_write_sram = 0;
		fault_vesc_write_sram_cnt = 0;

		peak_histoy_bkp_sram_num = 0;
		peak_histoy_bkp_sram_cnt = 0;

		if (RNG->SR == (uint32_t)1) {
			fault_vesc_write_sram_rnd_id = RNG->DR;
		} else {
			fault_vesc_write_sram_rnd_id = 0;
		}
	} else {
		peak_histoy_bkp_sram_num++;
		peak_histoy_bkp_sram_cnt++;

		if (peak_histoy_bkp_sram_num >= MC_PEAK_HISTORY_LEN) {
			peak_histoy_bkp_sram_num = 0;
		}
		if (peak_histoy_bkp_sram_cnt > 1000) {
			peak_histoy_bkp_sram_cnt = 0;
		}
	}

	memset((void*)&mc_peak_history[peak_histoy_bkp_sram_num], 0, sizeof(mc_peak_history_t));
	mc_peak_history[peak_histoy_bkp_sram_num].cnt = peak_histoy_bkp_sram_cnt;

	RNG->CR &= RNG_CR_RNGEN;
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, DISABLE);

	// Start threads
	chThdCreateStatic(timer_thread_wa, sizeof(timer_thread_wa), NORMALPRIO, timer_thread, NULL);
	chThdCreateStatic(sample_send_thread_wa, sizeof(sample_send_thread_wa), NORMALPRIO - 1, sample_send_thread, NULL);
	chThdCreateStatic(fault_stop_thread_wa, sizeof(fault_stop_thread_wa), HIGHPRIO - 3, fault_stop_thread, NULL);
	chThdCreateStatic(fram_thread_wa, sizeof(fram_thread_wa), NORMALPRIO - 3, fram_thread, NULL);

	int motor_old = mc_interface_get_motor_thread();
	mc_interface_select_motor_thread(1);
#ifdef HW_HAS_DRV8301
	drv8301_set_oc_mode(motor_now()->m_conf.m_drv8301_oc_mode);
	drv8301_set_oc_adj(motor_now()->m_conf.m_drv8301_oc_adj);
#elif defined(HW_HAS_DRV8320S)
	drv8320s_set_oc_mode(motor_now()->m_conf.m_drv8301_oc_mode);
	drv8320s_set_oc_adj(motor_now()->m_conf.m_drv8301_oc_adj);
#elif defined(HW_HAS_DRV8323S)
	drv8323s_set_oc_mode(motor_now()->m_conf.m_drv8301_oc_mode);
	drv8323s_set_oc_adj(motor_now()->m_conf.m_drv8301_oc_adj);
	DRV8323S_CUSTOM_SETTINGS();
#endif

#if defined HW_HAS_DUAL_MOTORS || defined HW_HAS_DUAL_PARALLEL
	mc_interface_select_motor_thread(2);
#ifdef HW_HAS_DRV8301
	drv8301_set_oc_mode(motor_now()->m_conf.m_drv8301_oc_mode);
	drv8301_set_oc_adj(motor_now()->m_conf.m_drv8301_oc_adj);
#elif defined(HW_HAS_DRV8320S)
	drv8320s_set_oc_mode(motor_now()->m_conf.m_drv8301_oc_mode);
	drv8320s_set_oc_adj(motor_now()->m_conf.m_drv8301_oc_adj);
#elif defined(HW_HAS_DRV8323S)
	drv8323s_set_oc_mode(motor_now()->m_conf.m_drv8301_oc_mode);
	drv8323s_set_oc_adj(motor_now()->m_conf.m_drv8301_oc_adj);
	DRV8323S_CUSTOM_SETTINGS();
#endif
#endif
	mc_interface_select_motor_thread(motor_old);

	// Initialize encoder
#if !WS2811_ENABLE
	switch (motor_now()->m_conf.m_sensor_port_mode) {
	case SENSOR_PORT_MODE_ABI:
		encoder_init_abi(motor_now()->m_conf.m_encoder_counts);
		break;

	case SENSOR_PORT_MODE_AS5047_SPI:
		encoder_init_as5047p_spi();
		break;

	case SENSOR_PORT_MODE_MT6816_SPI:
		encoder_init_mt6816_spi();
		break;

	case SENSOR_PORT_MODE_AD2S1205:
		encoder_init_ad2s1205_spi();
		break;

	case SENSOR_PORT_MODE_SINCOS:
		encoder_init_sincos(motor_now()->m_conf.foc_encoder_sin_gain, motor_now()->m_conf.foc_encoder_sin_offset,
							motor_now()->m_conf.foc_encoder_cos_gain, motor_now()->m_conf.foc_encoder_cos_offset,
							motor_now()->m_conf.foc_encoder_sincos_filter_constant);
		break;

	case SENSOR_PORT_MODE_TS5700N8501:
	case SENSOR_PORT_MODE_TS5700N8501_MULTITURN: {
		app_configuration *appconf = mempools_alloc_appconf();
		conf_general_read_app_configuration(appconf);
		if (appconf->app_to_use == APP_ADC ||
				appconf->app_to_use == APP_UART ||
				appconf->app_to_use == APP_PPM_UART ||
				appconf->app_to_use == APP_ADC_UART) {
			appconf->app_to_use = APP_NONE;
			conf_general_store_app_configuration(appconf);
		}
		mempools_free_appconf(appconf);
		encoder_init_ts5700n8501();
	} break;

	default:
		break;
	}
#endif

	// Initialize selected implementation
	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		mcpwm_init(&motor_now()->m_conf);
		break;

	case MOTOR_TYPE_FOC:
#ifdef HW_HAS_DUAL_MOTORS
		mcpwm_foc_init(&m_motor_1.m_conf, &m_motor_2.m_conf);
#else
		mcpwm_foc_init(&m_motor_1.m_conf, &m_motor_1.m_conf);
#endif
		break;

	case MOTOR_TYPE_GPD:
		gpdrive_init(&motor_now()->m_conf);
		break;

	default:
		break;
	}

	bms_init((bms_config*)&m_motor_1.m_conf.bms);
	mc_interface_reset_peak();
}

int mc_interface_motor_now(void) {
#if defined HW_HAS_DUAL_MOTORS || defined HW_HAS_DUAL_PARALLEL
	int isr_motor = mcpwm_foc_isr_motor();
	int thd_motor = chThdGetSelfX()->motor_selected;

	if (isr_motor > 0) {
		return isr_motor;
	} else if (thd_motor > 0) {
		return thd_motor;
	} else {
		return 1;
	}
#else
	return 1;
#endif
}

/**
 * Select motor for current thread. When a thread has a motor selected,
 * the mc_interface functions will use that motor for that thread. This
 * is only relevant for dual motor hardware.
 *
 * @param motor
 * 0: no specific motor selected, the last motor will be used.
 * 1: motor 1 selected (default).
 * 2: motor 2 selected.
 */
void mc_interface_select_motor_thread(int motor) {
#if defined HW_HAS_DUAL_MOTORS || defined HW_HAS_DUAL_PARALLEL
	if (motor == 0 || motor == 1 || motor == 2) {
		chThdGetSelfX()->motor_selected = motor;
	}
#else
	(void)motor;
#endif
}

/**
 * Get the motor selected for the current thread.
 *
 * @return
 * 0: no specific motor selected, the last motor will be used.
 * 1: motor 1 selected (default).
 * 2: motor 2 selected.
 */
int mc_interface_get_motor_thread(void) {
	return chThdGetSelfX()->motor_selected;
}

const volatile mc_configuration* mc_interface_get_configuration(void) {
	return &motor_now()->m_conf;
}

void mc_interface_set_configuration(mc_configuration *configuration) {
	volatile motor_if_state_t *motor = motor_now();

#if defined HW_HAS_DUAL_MOTORS || defined HW_HAS_DUAL_PARALLEL
	configuration->motor_type = MOTOR_TYPE_FOC;
#endif

#if !WS2811_ENABLE
	if (motor->m_conf.m_sensor_port_mode != configuration->m_sensor_port_mode) {
		encoder_deinit();
		switch (configuration->m_sensor_port_mode) {
		case SENSOR_PORT_MODE_ABI:
			encoder_init_abi(configuration->m_encoder_counts);
			break;

		case SENSOR_PORT_MODE_AS5047_SPI:
			encoder_init_as5047p_spi();
			break;

		case SENSOR_PORT_MODE_AD2S1205:
			encoder_init_ad2s1205_spi();
			break;

		case SENSOR_PORT_MODE_SINCOS:
			encoder_init_sincos(motor->m_conf.foc_encoder_sin_gain, motor->m_conf.foc_encoder_sin_offset,
								motor->m_conf.foc_encoder_cos_gain, motor->m_conf.foc_encoder_cos_offset,
								motor->m_conf.foc_encoder_sincos_filter_constant);
			break;

		case SENSOR_PORT_MODE_TS5700N8501:
		case SENSOR_PORT_MODE_TS5700N8501_MULTITURN: {
			app_configuration *appconf = mempools_alloc_appconf();
			*appconf = *app_get_configuration();
			if (appconf->app_to_use == APP_ADC ||
					appconf->app_to_use == APP_UART ||
					appconf->app_to_use == APP_PPM_UART ||
					appconf->app_to_use == APP_ADC_UART) {
				appconf->app_to_use = APP_NONE;
				conf_general_store_app_configuration(appconf);
				app_set_configuration(appconf);
			}
			mempools_free_appconf(appconf);
			encoder_init_ts5700n8501();
		} break;

		default:
			break;
		}
	}

	if (configuration->m_sensor_port_mode == SENSOR_PORT_MODE_ABI) {
		encoder_set_counts(configuration->m_encoder_counts);
	}
#endif

#ifdef HW_HAS_DRV8301
	drv8301_set_oc_mode(configuration->m_drv8301_oc_mode);
	drv8301_set_oc_adj(configuration->m_drv8301_oc_adj);
#elif defined(HW_HAS_DRV8320S)
	drv8320s_set_oc_mode(configuration->m_drv8301_oc_mode);
	drv8320s_set_oc_adj(configuration->m_drv8301_oc_adj);
#elif defined(HW_HAS_DRV8323S)
	drv8323s_set_oc_mode(configuration->m_drv8301_oc_mode);
	drv8323s_set_oc_adj(configuration->m_drv8301_oc_adj);
#endif

#ifdef HW_HAS_DUAL_PARALLEL
	mc_interface_select_motor_thread(2);
#ifdef HW_HAS_DRV8301
	drv8301_set_oc_mode(configuration->m_drv8301_oc_mode);
	drv8301_set_oc_adj(configuration->m_drv8301_oc_adj);
#elif defined(HW_HAS_DRV8320S)
	drv8320s_set_oc_mode(configuration->m_drv8301_oc_mode);
	drv8320s_set_oc_adj(configuration->m_drv8301_oc_adj);
#elif defined(HW_HAS_DRV8323S)
	drv8323s_set_oc_mode(configuration->m_drv8301_oc_mode);
	drv8323s_set_oc_adj(configuration->m_drv8301_oc_adj);
#endif
	mc_interface_select_motor_thread(1);
#endif

	if (motor->m_conf.motor_type != configuration->motor_type) {
		mcpwm_deinit();
		mcpwm_foc_deinit();
		gpdrive_deinit();

		motor->m_conf = *configuration;

		switch (motor->m_conf.motor_type) {
		case MOTOR_TYPE_BLDC:
		case MOTOR_TYPE_DC:
			mcpwm_init(&motor->m_conf);
			break;

		case MOTOR_TYPE_FOC:
#ifdef HW_HAS_DUAL_MOTORS
			mcpwm_foc_init(&m_motor_1.m_conf, &m_motor_2.m_conf);
#else
			mcpwm_foc_init(&m_motor_1.m_conf, &m_motor_1.m_conf);
#endif
			break;

		case MOTOR_TYPE_GPD:
			gpdrive_init(&motor->m_conf);
			break;

		default:
			break;
		}
	} else {
		motor->m_conf = *configuration;
	}

	update_override_limits(motor, &motor->m_conf);

	switch (motor->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		mcpwm_set_configuration(&motor->m_conf);
		break;

	case MOTOR_TYPE_FOC:
#ifdef HW_HAS_DUAL_MOTORS
		if (motor == &m_motor_1) {
			m_motor_2.m_conf.foc_f_sw = motor->m_conf.foc_f_sw;
		} else {
			m_motor_1.m_conf.foc_f_sw = motor->m_conf.foc_f_sw;
		}
#endif
		mcpwm_foc_set_configuration(&motor->m_conf);
		break;

	case MOTOR_TYPE_GPD:
		gpdrive_set_configuration(&motor->m_conf);
		break;

	default:
		break;
	}

	bms_init(&configuration->bms);
}

bool mc_interface_dccal_done(void) {
	bool ret = false;
	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		ret = mcpwm_is_dccal_done();
		break;

	case MOTOR_TYPE_FOC:
		ret = mcpwm_foc_is_dccal_done();
		break;

	case MOTOR_TYPE_GPD:
		ret = gpdrive_is_dccal_done();
		break;

	default:
		break;
	}

	return ret;
}

/**
 * Set a function that should be called after each PWM cycle.
 *
 * Note: this function is called from an interrupt.
 *
 * @param p_func
 * The function to be called. 0 will not call any function.
 */
void mc_interface_set_pwm_callback(void (*p_func)(void)) {
	pwn_done_func = p_func;
}

/**
 * Lock the control by disabling all control commands.
 */
void mc_interface_lock(void) {
	motor_now()->m_lock_enabled = true;
}

/**
 * Unlock all control commands.
 */
void mc_interface_unlock(void) {
	motor_now()->m_lock_enabled = false;
}

/**
 * Allow just one motor control command in the locked state.
 */
void mc_interface_lock_override_once(void) {
	motor_now()->m_lock_override_once = true;
}

mc_fault_code mc_interface_get_fault(void) {
	return motor_now()->m_fault_now;
}

mc_fault_code mc_interface_get_fault_old(void) {
	return m_fault_stop_fault;
}

const char* mc_interface_fault_to_string(mc_fault_code fault) {
	switch (fault) {
	case FAULT_CODE_NONE: return "FAULT_CODE_NONE"; break;
	case FAULT_CODE_OVER_VOLTAGE: return "FAULT_CODE_OVER_VOLTAGE"; break;
	case FAULT_CODE_UNDER_VOLTAGE: return "FAULT_CODE_UNDER_VOLTAGE"; break;
	case FAULT_CODE_DRV: return "FAULT_CODE_DRV"; break;
	case FAULT_CODE_ABS_OVER_CURRENT: return "FAULT_CODE_ABS_OVER_CURRENT"; break;
	case FAULT_CODE_OVER_TEMP_FET: return "FAULT_CODE_OVER_TEMP_FET"; break;
	case FAULT_CODE_OVER_TEMP_MOTOR: return "FAULT_CODE_OVER_TEMP_MOTOR"; break;
	case FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE: return "FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE"; break;
	case FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE: return "FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE"; break;
	case FAULT_CODE_MCU_UNDER_VOLTAGE: return "FAULT_CODE_MCU_UNDER_VOLTAGE"; break;
	case FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET: return "FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET"; break;
	case FAULT_CODE_ENCODER_SPI: return "FAULT_CODE_ENCODER_SPI"; break;
	case FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE: return "FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE"; break;
	case FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE: return "FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE"; break;
    case FAULT_CODE_FLASH_CORRUPTION: return "FAULT_CODE_FLASH_CORRUPTION";
    case FAULT_CODE_FLASH_CORRUPTION_APP_CFG: return "FAULT_CODE_FLASH_CORRUPTION_APP_CFG";
    case FAULT_CODE_FLASH_CORRUPTION_MC_CFG: return "FAULT_CODE_FLASH_CORRUPTION_MC_CFG";
    case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1: return "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1";
    case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2: return "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2";
    case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3: return "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3";
    case FAULT_CODE_UNBALANCED_CURRENTS: return "FAULT_CODE_UNBALANCED_CURRENTS";
    case FAULT_CODE_BRK: return "FAULT_CODE_BRK";
    case FAULT_CODE_PHASE_1_FET: return "FAULT_CODE_PHASE_1_FET";
    case FAULT_CODE_PHASE_2_FET: return "FAULT_CODE_PHASE_2_FET";
    case FAULT_CODE_PHASE_3_FET: return "FAULT_CODE_PHASE_3_FET";
    case FAULT_CODE_TORP_3: return "FAULT_CODE_HALL_SENSOR";
	default: return "FAULT_UNKNOWN"; break;
	}
}

uint64_t mc_interface_get_warning(void) {
	return motor_now()->m_warning;
}

mc_state mc_interface_get_state(void) {
	mc_state ret = MC_STATE_OFF;
	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		ret = mcpwm_get_state();
		break;

	case MOTOR_TYPE_FOC:
		ret = mcpwm_foc_get_state();
		break;

	default:
		break;
	}

	return ret;
}

void mc_interface_set_duty(float dutyCycle) {
	if (fabsf(dutyCycle) > 0.001) {
		SHUTDOWN_RESET();
	}

	if (mc_interface_try_input()) {
		return;
	}

	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		mcpwm_set_duty(DIR_MULT * dutyCycle);
		break;

	case MOTOR_TYPE_FOC:
		mcpwm_foc_set_duty(DIR_MULT * dutyCycle);
		break;

	default:
		break;
	}
}

void mc_interface_set_duty_noramp(float dutyCycle) {
	if (fabsf(dutyCycle) > 0.001) {
		SHUTDOWN_RESET();
	}

	if (mc_interface_try_input()) {
		return;
	}

	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		mcpwm_set_duty_noramp(DIR_MULT * dutyCycle);
		break;

	case MOTOR_TYPE_FOC:
		mcpwm_foc_set_duty_noramp(DIR_MULT * dutyCycle);
		break;

	default:
		break;
	}
}

void mc_interface_set_pid_speed(float rpm) {
	if (fabsf(rpm) > 0.001) {
		SHUTDOWN_RESET();
	}

	if (mc_interface_try_input()) {
		return;
	}

	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		mcpwm_set_pid_speed(DIR_MULT * rpm);
		break;

	case MOTOR_TYPE_FOC:
		mcpwm_foc_set_pid_speed(DIR_MULT * rpm);
		break;

	default:
		break;
	}
}

void mc_interface_set_pid_pos(float pos) {
	SHUTDOWN_RESET();

	if (mc_interface_try_input()) {
		return;
	}

	motor_now()->m_position_set = pos;

	pos *= DIR_MULT;
	utils_norm_angle(&pos);

	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		mcpwm_set_pid_pos(pos);
		break;

	case MOTOR_TYPE_FOC:
		mcpwm_foc_set_pid_pos(pos);
		break;

	default:
		break;
	}
}

void mc_interface_set_current(float current) {
	if (fabsf(current) > 0.001) {
		SHUTDOWN_RESET();
	}

	if (mc_interface_try_input()) {
		return;
	}

	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		mcpwm_set_current(DIR_MULT * current);
		break;

	case MOTOR_TYPE_FOC:
		mcpwm_foc_set_current(DIR_MULT * current);
		break;

	default:
		break;
	}
}

void mc_interface_set_brake_current(float current) {
	if (fabsf(current) > 0.001) {
		SHUTDOWN_RESET();
	}

	if (mc_interface_try_input()) {
		return;
	}

	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		mcpwm_set_brake_current(DIR_MULT * current);
		break;

	case MOTOR_TYPE_FOC:
		mcpwm_foc_set_brake_current(DIR_MULT * current);
		break;

	case MOTOR_TYPE_GPD:
		// For timeout to stop the output
		gpdrive_set_mode(GPD_OUTPUT_MODE_NONE);
		break;

	default:
		break;
	}
}

/**
 * Set current relative to the minimum and maximum current limits.
 *
 * @param current
 * The relative current value, range [-1.0 1.0]
 */
void mc_interface_set_current_rel(float val) {
	if (fabsf(val) > 0.001) {
		SHUTDOWN_RESET();
	}

	mc_interface_set_current(val * motor_now()->m_conf.lo_current_motor_max_now);
}

/**
 * Set brake current relative to the minimum current limit.
 *
 * @param current
 * The relative current value, range [0.0 1.0]
 */
void mc_interface_set_brake_current_rel(float val) {
	if (fabsf(val) > 0.001) {
		SHUTDOWN_RESET();
	}

	mc_interface_set_brake_current(val * fabsf(motor_now()->m_conf.lo_current_motor_min_now));
}

/**
 * Set open loop current vector to brake motor.
 *
 * @param current
 * The current value.
 */
void mc_interface_set_handbrake(float current) {
	if (fabsf(current) > 0.001) {
		SHUTDOWN_RESET();
	}

	if (mc_interface_try_input()) {
		return;
	}

	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		// TODO: Not implemented yet, use brake mode for now.
		mcpwm_set_brake_current(current);
		break;

	case MOTOR_TYPE_FOC:
		mcpwm_foc_set_handbrake(current);
		break;

	default:
		break;
	}
}

/**
 * Set handbrake brake current relative to the minimum current limit.
 *
 * @param current
 * The relative current value, range [0.0 1.0]
 */
void mc_interface_set_handbrake_rel(float val) {
	if (fabsf(val) > 0.001) {
		SHUTDOWN_RESET();
	}

	mc_interface_set_handbrake(val * fabsf(motor_now()->m_conf.lo_current_motor_min_now));
}

void mc_interface_brake_now(void) {
	SHUTDOWN_RESET();

	mc_interface_set_duty(0.0);
}

/**
 * Disconnect the motor and let it turn freely.
 */
void mc_interface_release_motor(void) {
	mc_interface_set_current(0.0);
}

/**
 * Stop the motor and use braking.
 */
float mc_interface_get_duty_cycle_set(void) {
	float ret = 0.0;

	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		ret = mcpwm_get_duty_cycle_set();
		break;

	case MOTOR_TYPE_FOC:
		ret = mcpwm_foc_get_duty_cycle_set();
		break;

	default:
		break;
	}

	return DIR_MULT * ret;
}

float mc_interface_get_duty_cycle_now(void) {
	float ret = 0.0;

	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		ret = mcpwm_get_duty_cycle_now();
		break;

	case MOTOR_TYPE_FOC:
		ret = mcpwm_foc_get_duty_cycle_now();
		break;

	default:
		break;
	}

	return DIR_MULT * ret;
}

float mc_interface_get_sampling_frequency_now(void) {
	float ret = 0.0;

	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		ret = mcpwm_get_switching_frequency_now();
		break;

	case MOTOR_TYPE_FOC:
		ret = mcpwm_foc_get_sampling_frequency_now();
		break;

	case MOTOR_TYPE_GPD:
		ret = gpdrive_get_switching_frequency_now();
		break;

	default:
		break;
	}

	return ret;
}

float mc_interface_get_rpm(void) {
	float ret = 0.0;

	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		ret = mcpwm_get_rpm();
		break;

	case MOTOR_TYPE_FOC:
		ret = mcpwm_foc_get_rpm();
		break;

	default:
		break;
	}

	return DIR_MULT * ret;
}

/**
 * Get the amount of amp hours drawn from the input source.
 *
 * @param reset
 * If true, the counter will be reset after this call.
 *
 * @return
 * The amount of amp hours drawn.
 */
float mc_interface_get_amp_hours(bool reset) {
	float val = motor_now()->m_amp_seconds / 3600;

	if (reset) {
		motor_now()->m_amp_seconds = 0.0;
	}

	return val;
}

/**
 * Get the amount of amp hours fed back into the input source.
 *
 * @param reset
 * If true, the counter will be reset after this call.
 *
 * @return
 * The amount of amp hours fed back.
 */
float mc_interface_get_amp_hours_charged(bool reset) {
	float val = motor_now()->m_amp_seconds_charged / 3600;

	if (reset) {
		motor_now()->m_amp_seconds_charged = 0.0;
	}

	return val;
}

/**
 * Get the amount of watt hours drawn from the input source.
 *
 * @param reset
 * If true, the counter will be reset after this call.
 *
 * @return
 * The amount of watt hours drawn.
 */
float mc_interface_get_watt_hours(bool reset) {
	float val = motor_now()->m_watt_seconds / 3600;

	if (reset) {
		motor_now()->m_watt_seconds = 0.0;
	}

	return val;
}

/**
 * Get the amount of watt hours fed back into the input source.
 *
 * @param reset
 * If true, the counter will be reset after this call.
 *
 * @return
 * The amount of watt hours fed back.
 */
float mc_interface_get_watt_hours_charged(bool reset) {
	float val = motor_now()->m_watt_seconds_charged / 3600;

	if (reset) {
		motor_now()->m_watt_seconds_charged = 0.0;
	}

	return val;
}

float mc_interface_get_tot_current(void) {
	float ret = 0.0;

	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		ret = mcpwm_get_tot_current();
		break;

	case MOTOR_TYPE_FOC:
		ret = mcpwm_foc_get_tot_current();
		break;

	default:
		break;
	}

	return ret;
}

float mc_interface_get_tot_current_filtered(void) {
	float ret = 0.0;

	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		ret = mcpwm_get_tot_current_filtered();
		break;

	case MOTOR_TYPE_FOC:
		ret = mcpwm_foc_get_tot_current_filtered();
		break;

	default:
		break;
	}

	return ret;
}

float mc_interface_get_tot_current_directional(void) {
	float ret = 0.0;

	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		ret = mcpwm_get_tot_current_directional();
		break;

	case MOTOR_TYPE_FOC:
		ret = mcpwm_foc_get_tot_current_directional();
		break;

	default:
		break;
	}

	return DIR_MULT * ret;
}

float mc_interface_get_tot_current_directional_filtered(void) {
	float ret = 0.0;

	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		ret = mcpwm_get_tot_current_directional_filtered();
		break;

	case MOTOR_TYPE_FOC:
		ret = mcpwm_foc_get_tot_current_directional_filtered();
		break;

	default:
		break;
	}

	return DIR_MULT * ret;
}

float mc_interface_get_tot_current_in(void) {
	float ret = 0.0;

	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		ret = mcpwm_get_tot_current_in();
		break;

	case MOTOR_TYPE_FOC:
		ret = mcpwm_foc_get_tot_current_in();
		break;

	default:
		break;
	}

	return ret;
}

float mc_interface_get_tot_current_in_filtered(void) {
	float ret = 0.0;

	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		ret = mcpwm_get_tot_current_in_filtered();
		break;

	case MOTOR_TYPE_FOC:
		ret = mcpwm_foc_get_tot_current_in_filtered();
		break;

	default:
		break;
	}

	return ret;
}

float mc_interface_get_abs_motor_current_unbalance(void) {
	float ret = 0.0;

#ifdef HW_HAS_3_SHUNTS
	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		break;

	case MOTOR_TYPE_FOC:
		ret = mcpwm_foc_get_abs_motor_current_unbalance();
		break;

	default:
		break;
	}
#endif
	return ret;
}

int mc_interface_set_tachometer_value(int steps) {
	int ret = 0;
	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		ret = mcpwm_set_tachometer_value(DIR_MULT * steps);
		break;

	case MOTOR_TYPE_FOC:
		ret = mcpwm_foc_set_tachometer_value(DIR_MULT * steps);
		break;

	default:
		break;
	}

	return DIR_MULT * ret;
}

int mc_interface_get_tachometer_value(bool reset) {
	int ret = 0;

	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		ret = mcpwm_get_tachometer_value(reset);
		break;

	case MOTOR_TYPE_FOC:
		ret = mcpwm_foc_get_tachometer_value(reset);
		break;

	default:
		break;
	}

	return DIR_MULT * ret;
}

int mc_interface_get_tachometer_abs_value(bool reset) {
	int ret = 0;

	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		ret = mcpwm_get_tachometer_abs_value(reset);
		break;

	case MOTOR_TYPE_FOC:
		ret = mcpwm_foc_get_tachometer_abs_value(reset);
		break;

	default:
		break;
	}

	return ret;
}

float mc_interface_get_last_inj_adc_isr_duration(void) {
	float ret = 0.0;

	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		ret = mcpwm_get_last_inj_adc_isr_duration();
		break;

	case MOTOR_TYPE_FOC:
		ret = mcpwm_foc_get_last_adc_isr_duration();
		break;

	case MOTOR_TYPE_GPD:
		ret = gpdrive_get_last_adc_isr_duration();
		break;

	default:
		break;
	}

	return ret;
}

float mc_interface_read_reset_avg_motor_current(void) {
	if (motor_now()->m_conf.motor_type == MOTOR_TYPE_GPD) {
		return gpdrive_get_current_filtered();
	}

	float res = motor_now()->m_motor_current_sum / motor_now()->m_motor_current_iterations;
	motor_now()->m_motor_current_sum = 0.0;
	motor_now()->m_motor_current_iterations = 0.0;
	return res;
}

float mc_interface_read_reset_avg_input_current(void) {
	if (motor_now()->m_conf.motor_type == MOTOR_TYPE_GPD) {
		return gpdrive_get_current_filtered() * gpdrive_get_modulation();
	}

	float res = motor_now()->m_input_current_sum / motor_now()->m_input_current_iterations;
	motor_now()->m_input_current_sum = 0.0;
	motor_now()->m_input_current_iterations = 0.0;
	return res;
}

/**
 * Read and reset the average direct axis motor current. (FOC only)
 *
 * @return
 * The average D axis current.
 */
float mc_interface_read_reset_avg_id(void) {
	float res = motor_now()->m_motor_id_sum / motor_now()->m_motor_id_iterations;
	motor_now()->m_motor_id_sum = 0.0;
	motor_now()->m_motor_id_iterations = 0.0;
	return res;
}

/**
 * Read and reset the average quadrature axis motor current. (FOC only)
 *
 * @return
 * The average Q axis current.
 */
float mc_interface_read_reset_avg_iq(void) {
	float res = motor_now()->m_motor_iq_sum / motor_now()->m_motor_iq_iterations;
	motor_now()->m_motor_iq_sum = 0.0;
	motor_now()->m_motor_iq_iterations = 0.0;
	return DIR_MULT * res;
}

/**
 * Read and reset the average direct axis motor voltage. (FOC only)
 *
 * @return
 * The average D axis voltage.
 */
float mc_interface_read_reset_avg_vd(void) {
	float res = motor_now()->m_motor_vd_sum / motor_now()->m_motor_vd_iterations;
	motor_now()->m_motor_vd_sum = 0.0;
	motor_now()->m_motor_vd_iterations = 0.0;
	return res;
}

/**
 * Read and reset the average quadrature axis motor voltage. (FOC only)
 *
 * @return
 * The average Q axis voltage.
 */
float mc_interface_read_reset_avg_vq(void) {
	float res = motor_now()->m_motor_vq_sum / motor_now()->m_motor_vq_iterations;
	motor_now()->m_motor_vq_sum = 0.0;
	motor_now()->m_motor_vq_iterations = 0.0;
	return DIR_MULT * res;
}

float mc_interface_get_pid_pos_set(void) {
	return motor_now()->m_position_set;
}

float mc_interface_get_pid_pos_now(void) {
	float ret = 0.0;

	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		ret = encoder_read_deg();
		break;

	case MOTOR_TYPE_FOC:
		ret = mcpwm_foc_get_pid_pos_now();
		break;

	default:
		break;
	}

	ret *= DIR_MULT;
	utils_norm_angle(&ret);

	return ret;
}

float mc_interface_get_last_sample_adc_isr_duration(void) {
	return m_last_adc_duration_sample;
}

void mc_interface_sample_print_data(debug_sampling_mode mode, uint16_t len, uint8_t decimation,
									void(*reply_func)(unsigned char* data, unsigned int len)) {
	if (len > ADC_SAMPLE_MAX_LEN) {
		len = ADC_SAMPLE_MAX_LEN;
	}

	if (mode == DEBUG_SAMPLING_SEND_LAST_SAMPLES) {
		chEvtSignal(sample_send_tp, (eventmask_t) 1);
	} else {
		m_sample_trigger = -1;
		m_sample_now = 0;
		m_sample_len = len;
		m_sample_int = decimation;
		m_sample_mode = mode;
		send_func_sample = reply_func;
#ifdef HW_HAS_DUAL_MOTORS
		m_sample_is_second_motor = motor_now() == &m_motor_2;
#endif
	}
}

/**
 * Get filtered MOSFET temperature. The temperature is pre-calculated, so this
 * functions is fast.
 *
 * @return
 * The filtered MOSFET temperature.
 */
float mc_interface_temp_fet_filtered(void) {
	return motor_now()->m_temp_fet;
}

/**
 * Get filtered motor temperature. The temperature is pre-calculated, so this
 * functions is fast.
 *
 * @return
 * The filtered motor temperature.
 */
float mc_interface_temp_motor_filtered(void) {
	return motor_now()->m_temp_motor;
}

/**
 * Get the battery level, based on battery settings in configuration. Notice that
 * this function is based on remaining watt hours, and not amp hours.
 *
 * @param wh_left
 * Pointer to where to store the remaining watt hours, can be null.
 *
 * @return
 * Battery level, range 0 to 1
 */
float mc_interface_get_battery_level(float *wh_left, float cell_volt, bool reset) {
	const volatile mc_configuration *conf = mc_interface_get_configuration();
	const float v_in = GET_INPUT_VOLTAGE();
	float battery_avg_voltage = 0.0;
	float battery_avg_voltage_left = 0.0;
	float ah_left = 0;
	float ah_tot = conf->si_battery_ah;

	switch (conf->si_battery_type) {
	case BATTERY_TYPE_LIION_3_0__4_2:
		battery_avg_voltage = ((3.2 + 4.2) / 2.0) * (float)(conf->si_battery_cells);
		battery_avg_voltage_left = ((3.2 * (float)(conf->si_battery_cells) + v_in) / 2.0);
		float batt_left = utils_map(v_in / (float)(conf->si_battery_cells),
									3.2, 4.2, 0.0, 1.0);
		batt_left = utils_batt_liion_norm_v_to_capacity(batt_left);
		ah_tot *= 0.85; // 0.85 because the battery is not fully depleted at 3.2V / cell
		ah_left = batt_left * ah_tot;
		break;

	case BATTERY_TYPE_LIIRON_2_6__3_6:
		battery_avg_voltage = ((2.8 + 3.6) / 2.0) * (float)(conf->si_battery_cells);
		battery_avg_voltage_left = ((2.8 * (float)(conf->si_battery_cells) + v_in) / 2.0);
		ah_left = utils_map(v_in / (float)(conf->si_battery_cells),
				2.6, 3.6, 0.0, conf->si_battery_ah);
		break;

	case BATTERY_TYPE_LEAD_ACID:
		// TODO: This does not really work for lead-acid batteries
		battery_avg_voltage = ((2.1 + 2.36) / 2.0) * (float)(conf->si_battery_cells);
		battery_avg_voltage_left = ((2.1 * (float)(conf->si_battery_cells) + v_in) / 2.0);
		ah_left = utils_map(v_in / (float)(conf->si_battery_cells),
				2.1, 2.36, 0.0, conf->si_battery_ah);
		break;

	case BATTERY_TYPE_LIION_VTC6:
	case BATTERY_TYPE_LIION_PANASONIC_PF: {
		float volt_vtc6[] = { 2.693, 3.035, 3.181, 3.282, 3.366, 3.444, 3.510, 3.557, 3.629, 3.676, 3.719, 3.764, 3.806, 3.844, 3.882, 3.938, 3.995, 4.047, 4.076, 4.091, 4.116, 4.193 };
		float soc_vtc6[] = { -4.0, 1.0, 5.9, 10.9, 15.8, 20.8, 25.7, 30.7, 35.6, 40.6, 45.5, 50.5, 55.4, 60.4, 65.3, 70.3, 75.2, 80.2, 85.1, 90.1, 95.0, 100.0 };

		float volt_pf[] = { 3.12, 3.20, 3.23, 3.26, 3.34, 3.41, 3.46, 3.50, 3.53, 3.55, 3.62, 3.66, 3.71, 3.76, 3.81, 3.86, 3.90, 4.01, 4.03, 4.05, 4.08, 4.10 };
		float soc_pf[] = { -8.89, 0.00, 3.33, 6.67, 15.56, 23.33, 28.89, 33.33, 36.67, 38.89, 46.67, 51.11, 56.67, 62.22, 67.78, 73.33, 77.78, 90.00, 92.22, 94.44, 97.78, 100.00 };

		float* volt_ptr = volt_vtc6;
		float* soc_ptr = soc_vtc6;
		if (conf->si_battery_type == BATTERY_TYPE_LIION_PANASONIC_PF) {
			volt_ptr = volt_pf;
			soc_ptr = soc_pf;
		}
	
		static float soc;
		int i_volt = 0;

		if (cell_volt > 0.0) {
			float single_cell_volt = cell_volt;
			utils_truncate_number(&single_cell_volt, volt_ptr[0] + 0.01, volt_ptr[21]);

			int start_i = single_cell_volt >= volt_ptr[10] ? 10 : 0;
			for (int i = start_i; i <= 21; i++) {
				if (single_cell_volt <= volt_ptr[i]) {
					i_volt = i;
					break;
				}
			}
			float soc_now = utils_map(single_cell_volt, volt_ptr[i_volt - 1], volt_ptr[i_volt], soc_ptr[i_volt - 1], soc_ptr[i_volt]);

			if (!soc || reset) {
				soc = soc_now;
			} else if ((soc_now - soc) > 0.1) {
				soc += 0.1;
			} else if ((soc_now - soc) < -0.1) {
				soc -= 0.1;
			}
		}
		return soc / 10.0;
	} break;

	default:
		break;
	}

	const float wh_batt_tot = ah_tot * battery_avg_voltage;
	const float wh_batt_left = ah_left * battery_avg_voltage_left;

	if (wh_left) {
		*wh_left = wh_batt_left;
	}

	return wh_batt_left / wh_batt_tot;
}

/**
 * Get the speed based on wheel diameter, gearing and motor pole settings.
 *
 * @return
 * Speed, in m/s
 */
float mc_interface_get_speed(void) {
	const volatile mc_configuration *conf = mc_interface_get_configuration();
	const float rpm = mc_interface_get_rpm() / (conf->si_motor_poles / 2.0);
	return (rpm / 60.0) * conf->si_wheel_diameter * M_PI / conf->si_gear_ratio;
}

/**
 * Get the distance traveled based on wheel diameter, gearing and motor pole settings.
 *
 * @return
 * Distance traveled since boot, in meters
 */
float mc_interface_get_distance(void) {
	const volatile mc_configuration *conf = mc_interface_get_configuration();
	const float tacho_scale = (conf->si_wheel_diameter * M_PI) / (3.0 * conf->si_motor_poles * conf->si_gear_ratio);
	return mc_interface_get_tachometer_value(false) * tacho_scale;
}

/**
 * Get the absolute distance traveled based on wheel diameter, gearing and motor pole settings.
 *
 * @return
 * Absolute distance traveled since boot, in meters
 */
float mc_interface_get_distance_abs(void) {
	const volatile mc_configuration *conf = mc_interface_get_configuration();
	const float tacho_scale = (conf->si_wheel_diameter * M_PI) / (3.0 * conf->si_motor_poles * conf->si_gear_ratio);
	return mc_interface_get_tachometer_abs_value(false) * tacho_scale;
}

setup_values mc_interface_get_setup_values(void) {
	setup_values val = {0, 0, 0, 0, 0, 0, 0};
	val.num_vescs = 1;

	val.ah_tot += mc_interface_get_amp_hours(false);
	val.ah_charge_tot += mc_interface_get_amp_hours_charged(false);
	val.wh_tot += mc_interface_get_watt_hours(false);
	val.wh_charge_tot += mc_interface_get_watt_hours_charged(false);
	val.current_tot += mc_interface_get_tot_current_filtered();
	val.current_in_tot += mc_interface_get_tot_current_in_filtered();

	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		can_status_msg *msg = comm_can_get_status_msg_index(i);
		if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < 0.1) {
			val.current_tot += msg->current;
			val.num_vescs++;
		}

		can_status_msg_2 *msg2 = comm_can_get_status_msg_2_index(i);
		if (msg2->id >= 0 && UTILS_AGE_S(msg2->rx_time) < 0.1) {
			val.ah_tot += msg2->amp_hours;
			val.ah_charge_tot += msg2->amp_hours_charged;
		}

		can_status_msg_3 *msg3 = comm_can_get_status_msg_3_index(i);
		if (msg3->id >= 0 && UTILS_AGE_S(msg3->rx_time) < 0.1) {
			val.wh_tot += msg3->watt_hours;
			val.wh_charge_tot += msg3->watt_hours_charged;
		}

		can_status_msg_4 *msg4 = comm_can_get_status_msg_4_index(i);
		if (msg4->id >= 0 && UTILS_AGE_S(msg4->rx_time) < 0.1) {
			val.current_in_tot += msg4->current_in;
		}
	}

	return val;
}

// MC implementation functions

/**
 * A helper function that should be called before sending commands to control
 * the motor. If the state is detecting, the detection will be stopped.
 *
 * @return
 * The amount if milliseconds left until user commands are allowed again.
 *
 */
int mc_interface_try_input(void) {
	// TODO: Remove this later
	if (mc_interface_get_state() == MC_STATE_DETECTING) {
		mcpwm_stop_pwm();
		motor_now()->m_ignore_iterations = MCPWM_DETECT_STOP_TIME;
	}

	int retval = motor_now()->m_ignore_iterations;

	if (!motor_now()->m_ignore_iterations && motor_now()->m_lock_enabled) {
		if (!motor_now()->m_lock_override_once) {
			retval = 1;
		} else {
			motor_now()->m_lock_override_once = false;
		}
	}

	switch (motor_now()->m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		if (!mcpwm_init_done()) {
			retval = 1;
		}
		break;

	case MOTOR_TYPE_FOC:
		if (!mcpwm_foc_init_done()) {
			retval = 1;
		}
		break;

	default:
		break;
	}

	return retval;
}

void mc_interface_fault_stop(mc_fault_code fault, bool is_second_motor, bool is_isr) {
	m_fault_stop_fault = fault;
	m_fault_stop_is_second_motor = is_second_motor;

	if (is_isr) {
		chSysLockFromISR();
		chEvtSignalI(fault_stop_tp, (eventmask_t) 1);
		chSysUnlockFromISR();
	} else {
		chEvtSignal(fault_stop_tp, (eventmask_t) 1);
	}
}

void mc_interface_mc_timer_isr(bool is_second_motor) {
	ledpwm_update_pwm();

#ifdef HW_HAS_DUAL_MOTORS
	volatile motor_if_state_t *motor = is_second_motor ? &m_motor_2 : &m_motor_1;
#else
	volatile motor_if_state_t *motor = &m_motor_1;
	(void)is_second_motor;
#endif

	volatile mc_configuration *conf_now = &motor->m_conf;
	const float input_voltage = GET_INPUT_VOLTAGE();

	// Check for faults that should stop the motor
	static int wrong_voltage_iterations = 0;
	if (input_voltage < conf_now->l_min_vin ||
			input_voltage > conf_now->l_max_vin) {
		wrong_voltage_iterations++;

		if ((wrong_voltage_iterations >= 8)) {
			mc_interface_fault_stop(input_voltage < conf_now->l_min_vin ?
					FAULT_CODE_UNDER_VOLTAGE : FAULT_CODE_OVER_VOLTAGE, is_second_motor, true);
		}
	} else {
		wrong_voltage_iterations = 0;
	}

	if (input_voltage > mc_peak.voltagemax)
	{
		mc_peak.voltagemax = input_voltage;
	}
	if (input_voltage < mc_peak.voltagemin)
	{
		mc_peak.voltagemin = input_voltage;
	}

	const float current_inx = mc_interface_get_tot_current_in_filtered();
	mc_peak.m_input_current_sumx += current_inx;
	mc_peak.bccounter++;

	if (mc_peak.bccounter > 2000)
	{
		mc_peak.bccounter = 0;
		float bc = mc_peak.m_input_current_sumx / 2000.0;
		mc_peak.m_input_current_sumx = 0.0;

		if (bc > mc_peak.batterycurrentmax)
		{
			mc_peak.batterycurrentmax = bc;
		}

		if (bc < mc_peak.batterycurrentmin)
		{
			mc_peak.batterycurrentmin = bc;
		}

		float power = bc * input_voltage;
		if (power < mc_peak.powermin)
		{
			mc_peak.powermin = power;
		}
		if (power > mc_peak.powermax)
		{
			mc_peak.powermax = power;
		}

		const float motorcurrent = mc_interface_get_tot_current();

		if (motorcurrent > mc_peak.motorcurrentmax)
		{
			mc_peak.motorcurrentmax = motorcurrent;
		}
		if (motorcurrent < mc_peak.motorcurrentmin)
		{
			mc_peak.motorcurrentmin = motorcurrent;
		}
	}

	const float motorcurrentfiltered = mc_interface_get_tot_current_filtered();
	if (motorcurrentfiltered > mc_peak.motorcurrentmaxfiltered)
	{
		mc_peak.motorcurrentmaxfiltered = motorcurrentfiltered;
	}
	if (motorcurrentfiltered < mc_peak.motorcurrentminfiltered)
	{
		mc_peak.motorcurrentminfiltered = motorcurrentfiltered;
	}

	if (conf_now->motor_type == MOTOR_TYPE_FOC) {
		const float abs_motor = mcpwm_foc_get_abs_motor_current();
		if (abs_motor > mc_peak.abspeak)
		{
			mc_peak.abspeak = abs_motor;
		}

		const float abs_motor_filtered = mcpwm_foc_get_abs_motor_current_filtered();
		if (abs_motor_filtered > mc_peak.abs_peak_filtered)
		{
			mc_peak.abs_peak_filtered = abs_motor_filtered;
		}
	}

	const float rpm = mc_interface_get_rpm();
	if (rpm > mc_peak.rpmmax)
	{
		mc_peak.rpmmax = rpm;
	}
	if (rpm < mc_peak.rpmmin)
	{
		mc_peak.rpmmin = rpm;
	}

	const float fettemp = mc_interface_temp_fet_filtered();
	if (fettemp > mc_peak.fettempmax)
	{
		mc_peak.fettempmax = fettemp;
	} else if (fettemp < mc_peak.fettempmin) {
		mc_peak.fettempmin = fettemp;
	}


	float motortemp = mc_interface_temp_motor_filtered();
	if (motortemp > mc_peak.motortempmax)
	{
		mc_peak.motortempmax = motortemp;
	} else if (motortemp < mc_peak.motortempmin) {
		mc_peak.motortempmin = motortemp;
	}

	float motorpeak = mc_interface_get_tot_current();
	mc_peak.motorpeak = motorpeak;

	if (motorpeak > mc_peak.motorpeakmax)
	{
		mc_peak.motorpeakmax = motorpeak;
	}
	if(motorpeak < mc_peak.motorpeakmin){
	  mc_peak.motorpeakmin = motorpeak;
	}

	if (mc_interface_get_tot_current_in() > mc_peak.batterypeak)
	{
		mc_peak.batterypeak = mc_interface_get_tot_current_in();
	}

	float speed = mc_interface_get_speed() * 3.6;
	if (speed > mc_peak.speedmax)
	{
		mc_peak.speedmax = speed;
	}
	if(speed < mc_peak.speedmin){
	  mc_peak.speedmin = speed;
	}
	mc_peak.reset_done = false;

	float throttle_now = (float)ADC_Value[ADC_IND_EXT];
	throttle_now /= 4095;
	throttle_now *= V_REG;

	static float throttle_v_before = 0.0;
	static int throttle_cnt = 0;

	if (!throttle_v_before) {
		throttle_v_before = throttle_now;
	}

	if (throttle_cnt >= throttle_skip_num) {

		float diff = throttle_now - throttle_v_before;

		if (diff > max_throttle_rise) {
			max_throttle_rise = diff;
		}
		throttle_v_before = throttle_now;
		throttle_cnt = 0;
	}
	throttle_cnt++;

	// Fetch these values in a config-specific way to avoid some overhead of the general
	// functions. That will make this interrupt run a bit faster.
	mc_state state;
	float current;
	float current_filtered;
	float current_in_filtered;
	float abs_current;
	float abs_current_filtered;
	if (conf_now->motor_type == MOTOR_TYPE_FOC) {
		state = mcpwm_foc_get_state_motor(is_second_motor);
		current = mcpwm_foc_get_tot_current_motor(is_second_motor);
		current_filtered = mcpwm_foc_get_tot_current_filtered_motor(is_second_motor);
		current_in_filtered = mcpwm_foc_get_tot_current_in_filtered_motor(is_second_motor);
		abs_current = mcpwm_foc_get_abs_motor_current_motor(is_second_motor);
		abs_current_filtered = mcpwm_foc_get_abs_motor_current_filtered_motor(is_second_motor);
	} else {
		state = mcpwm_get_state();
		current = mcpwm_get_tot_current();
		current_filtered = mcpwm_get_tot_current_filtered();
		current_in_filtered = mcpwm_get_tot_current_in_filtered();
		abs_current = mcpwm_get_tot_current();
		abs_current_filtered = current_filtered;
	}

	if (state == MC_STATE_RUNNING) {
		motor->m_cycles_running++;
	} else {
		motor->m_cycles_running = 0;
	}

	if (pwn_done_func) {
		pwn_done_func();
	}

	motor->m_motor_current_sum += current_filtered;
	motor->m_input_current_sum += current_in_filtered;
	motor->m_motor_current_iterations++;
	motor->m_input_current_iterations++;

	motor->m_motor_id_sum += mcpwm_foc_get_id();
	motor->m_motor_iq_sum += mcpwm_foc_get_iq();
	motor->m_motor_id_iterations++;
	motor->m_motor_iq_iterations++;

	motor->m_motor_vd_sum += mcpwm_foc_get_vd();
	motor->m_motor_vq_sum += mcpwm_foc_get_vq();
	motor->m_motor_vd_iterations++;
	motor->m_motor_vq_iterations++;

	// Current fault code
	if (conf_now->l_slow_abs_current) {
		if (fabsf(abs_current_filtered) > conf_now->l_abs_current_max) {
			mc_interface_fault_stop(FAULT_CODE_ABS_OVER_CURRENT, is_second_motor, true);
		}
	} else {
		if (fabsf(abs_current) > conf_now->l_abs_current_max) {
			mc_interface_fault_stop(FAULT_CODE_ABS_OVER_CURRENT, is_second_motor, true);
		}
	}

	// DRV fault code
#ifdef HW_HAS_DUAL_PARALLEL
	if (IS_DRV_FAULT() || IS_DRV_FAULT_2()) {
		is_second_motor = IS_DRV_FAULT_2();
#else
	if (is_second_motor ? IS_DRV_FAULT_2() : IS_DRV_FAULT()) {
#endif
		mc_interface_fault_stop(FAULT_CODE_DRV, is_second_motor, true);
	}

#ifdef HW_USE_BRK
	// BRK fault code
	if (TIM_GetFlagStatus(TIM1, TIM_FLAG_Break) != RESET) {
		mc_interface_fault_stop(FAULT_CODE_BRK, is_second_motor, true);
		// latch the BRK/FAULT pin to low until next MCU reset
		palSetPadMode(BRK_GPIO, BRK_PIN, PAL_MODE_OUTPUT_PUSHPULL);
		palClearPad(BRK_GPIO, BRK_PIN);
	}
#endif

#ifdef HW_HAS_GATE_DRIVER_SUPPLY_MONITOR
	if(motor->m_gate_driver_voltage > HW_GATE_DRIVER_SUPPLY_MAX_VOLTAGE && mcpwm_foc_is_dccal_done()) {
		mc_interface_fault_stop(FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE, is_second_motor, true);
	}

	if(motor->m_gate_driver_voltage < HW_GATE_DRIVER_SUPPLY_MIN_VOLTAGE && mcpwm_foc_is_dccal_done()) {
		mc_interface_fault_stop(FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE, is_second_motor, true);
	}
#endif

	float f_samp = motor->m_f_samp_now;

	// Watt and ah counters
	if (fabsf(current_filtered) > 1.0) {
		// Some extra filtering
		static float curr_diff_sum = 0.0;
		static float curr_diff_samples = 0;

		curr_diff_sum += current_in_filtered / f_samp;
		curr_diff_samples += 1.0 / f_samp;

		if (curr_diff_samples >= 0.01) {
			if (curr_diff_sum > 0.0) {
				motor->m_amp_seconds += curr_diff_sum;
				motor->m_watt_seconds += curr_diff_sum * input_voltage;
			} else {
				motor->m_amp_seconds_charged -= curr_diff_sum;
				motor->m_watt_seconds_charged -= curr_diff_sum * input_voltage;
			}

			curr_diff_samples = 0.0;
			curr_diff_sum = 0.0;
		}
	}

	bool sample = false;
	debug_sampling_mode sample_mode =
			m_sample_is_second_motor == is_second_motor ?
					m_sample_mode : DEBUG_SAMPLING_OFF;

	switch (sample_mode) {
	case DEBUG_SAMPLING_NOW:
		if (m_sample_now == m_sample_len) {
			m_sample_mode = DEBUG_SAMPLING_OFF;
			m_sample_mode_last = DEBUG_SAMPLING_NOW;
			chSysLockFromISR();
			chEvtSignalI(sample_send_tp, (eventmask_t) 1);
			chSysUnlockFromISR();
		} else {
			sample = true;
		}
		break;

	case DEBUG_SAMPLING_START:
		if (state == MC_STATE_RUNNING || m_sample_now > 0) {
			sample = true;
		}

		if (m_sample_now == m_sample_len) {
			m_sample_mode_last = m_sample_mode;
			m_sample_mode = DEBUG_SAMPLING_OFF;
			chSysLockFromISR();
			chEvtSignalI(sample_send_tp, (eventmask_t) 1);
			chSysUnlockFromISR();
		}
		break;

	case DEBUG_SAMPLING_TRIGGER_START:
	case DEBUG_SAMPLING_TRIGGER_START_NOSEND: {
		sample = true;

		int sample_last = -1;
		if (m_sample_trigger >= 0) {
			sample_last = m_sample_trigger - m_sample_len;
			if (sample_last < 0) {
				sample_last += ADC_SAMPLE_MAX_LEN;
			}
		}

		if (m_sample_now == sample_last) {
			m_sample_mode_last = m_sample_mode;
			sample = false;

			if (m_sample_mode == DEBUG_SAMPLING_TRIGGER_START) {
				chSysLockFromISR();
				chEvtSignalI(sample_send_tp, (eventmask_t) 1);
				chSysUnlockFromISR();
			}

			m_sample_mode = DEBUG_SAMPLING_OFF;
		}

		if (state == MC_STATE_RUNNING && m_sample_trigger < 0) {
			m_sample_trigger = m_sample_now;
		}
	} break;

	case DEBUG_SAMPLING_TRIGGER_FAULT:
	case DEBUG_SAMPLING_TRIGGER_FAULT_NOSEND: {
		sample = true;

		int sample_last = -1;
		if (m_sample_trigger >= 0) {
			sample_last = m_sample_trigger - m_sample_len;
			if (sample_last < 0) {
				sample_last += ADC_SAMPLE_MAX_LEN;
			}
		}

		if (m_sample_now == sample_last) {
			m_sample_mode_last = m_sample_mode;
			sample = false;

			if (m_sample_mode == DEBUG_SAMPLING_TRIGGER_FAULT) {
				chSysLockFromISR();
				chEvtSignalI(sample_send_tp, (eventmask_t) 1);
				chSysUnlockFromISR();
			}

			m_sample_mode = DEBUG_SAMPLING_OFF;
		}

		if (motor->m_fault_now != FAULT_CODE_NONE && m_sample_trigger < 0) {
			m_sample_trigger = m_sample_now;
		}
	} break;

	default:
		break;
	}

	if (sample) {
		static int a = 0;
		a++;

		if (a >= m_sample_int) {
			a = 0;

			if (m_sample_now >= ADC_SAMPLE_MAX_LEN) {
				m_sample_now = 0;
			}

			int16_t zero;
			if (conf_now->motor_type == MOTOR_TYPE_FOC) {
				if (is_second_motor) {
					zero = (ADC_V_L4 + ADC_V_L5 + ADC_V_L6) / 3;
				} else {
					zero = (ADC_V_L1 + ADC_V_L2 + ADC_V_L3) / 3;
				}
				m_phase_samples[m_sample_now] = (uint8_t)(mcpwm_foc_get_phase() / 360.0 * 250.0);
//				m_phase_samples[m_sample_now] = (uint8_t)(mcpwm_foc_get_phase_observer() / 360.0 * 250.0);
//				float ang = utils_angle_difference(mcpwm_foc_get_phase_observer(), mcpwm_foc_get_phase_encoder()) + 180.0;
//				m_phase_samples[m_sample_now] = (uint8_t)(ang / 360.0 * 250.0);
			} else {
				zero = mcpwm_vzero;
				m_phase_samples[m_sample_now] = 0;
			}

			if (state == MC_STATE_DETECTING) {
				m_curr0_samples[m_sample_now] = (int16_t)mcpwm_detect_currents[mcpwm_get_comm_step() - 1];
				m_curr1_samples[m_sample_now] = (int16_t)mcpwm_detect_currents_diff[mcpwm_get_comm_step() - 1];

				m_ph1_samples[m_sample_now] = (int16_t)mcpwm_detect_voltages[0];
				m_ph2_samples[m_sample_now] = (int16_t)mcpwm_detect_voltages[1];
				m_ph3_samples[m_sample_now] = (int16_t)mcpwm_detect_voltages[2];
			} else {
				if (is_second_motor) {
					m_curr0_samples[m_sample_now] = ADC_curr_norm_value[3];
					m_curr1_samples[m_sample_now] = ADC_curr_norm_value[4];

					m_ph1_samples[m_sample_now] = ADC_V_L4 - zero;
					m_ph2_samples[m_sample_now] = ADC_V_L5 - zero;
					m_ph3_samples[m_sample_now] = ADC_V_L6 - zero;
				} else {
					m_curr0_samples[m_sample_now] = ADC_curr_norm_value[0];
					m_curr1_samples[m_sample_now] = ADC_curr_norm_value[1];

					m_ph1_samples[m_sample_now] = ADC_V_L1 - zero;
					m_ph2_samples[m_sample_now] = ADC_V_L2 - zero;
					m_ph3_samples[m_sample_now] = ADC_V_L3 - zero;
				}
			}

			m_vzero_samples[m_sample_now] = zero;
			m_curr_fir_samples[m_sample_now] = (int16_t)(current * (8.0 / FAC_CURRENT));
			m_f_sw_samples[m_sample_now] = (int16_t)(f_samp / 10.0);
			m_status_samples[m_sample_now] = mcpwm_get_comm_step() | (mcpwm_read_hall_phase() << 3);

			m_sample_now++;

			m_last_adc_duration_sample = mc_interface_get_last_inj_adc_isr_duration();
		}
	}
}

void mc_interface_adc_inj_int_handler(void) {
	switch (m_motor_1.m_conf.motor_type) {
	case MOTOR_TYPE_BLDC:
	case MOTOR_TYPE_DC:
		mcpwm_adc_inj_int_handler();
		break;

	case MOTOR_TYPE_FOC:
		break;

	default:
		break;
	}
}

/**
 * Update the override limits for a configuration based on MOSFET temperature etc.
 *
 * @param conf
 * The configaration to update.
 */
static void update_override_limits(volatile motor_if_state_t *motor, volatile mc_configuration *conf) {
	bool is_motor_1 = motor == &m_motor_1;

	const float v_in = GET_INPUT_VOLTAGE();
	float rpm_now = 0.0;

	if (motor->m_conf.motor_type == MOTOR_TYPE_FOC) {
		// Low latency is important for avoiding oscillations
		rpm_now = DIR_MULT * mcpwm_foc_get_rpm_fast();
	} else {
		rpm_now = mc_interface_get_rpm();
	}

	const float duty_now_abs = fabsf(mc_interface_get_duty_cycle_now());

	UTILS_LP_FAST(motor->m_temp_fet, NTC_TEMP(is_motor_1 ? ADC_IND_TEMP_MOS : ADC_IND_TEMP_MOS_M2), 0.1);
	if (conf->l_temp_fet_start == 0.0 && conf->l_temp_fet_end == 0.0) {
		motor->m_temp_fet = -100.0;
	}
	float temp_motor = 0.0;

	switch(conf->m_motor_temp_sens_type) {
	case TEMP_SENSOR_NTC_10K_25C:
		temp_motor = is_motor_1 ? NTC_TEMP_MOTOR(conf->m_ntc_motor_beta) : NTC_TEMP_MOTOR_2(conf->m_ntc_motor_beta);
		break;

	case TEMP_SENSOR_NTC_100K_25C:
		temp_motor = is_motor_1 ? NTC100K_TEMP_MOTOR(conf->m_ntc_motor_beta) : NTC100K_TEMP_MOTOR_2(conf->m_ntc_motor_beta);
		break;

	case TEMP_SENSOR_PTC_1K_100C:
		temp_motor = is_motor_1 ? PTC_TEMP_MOTOR(1000.0, conf->m_ptc_motor_coeff, 100) : PTC_TEMP_MOTOR_2(1000.0, conf->m_ptc_motor_coeff, 100);
		break;
		
	case TEMP_SENSOR_KTY83_122: {
		// KTY83_122 datasheet used to approximate resistance at given temperature to cubic polynom
		// https://docs.google.com/spreadsheets/d/1iJA66biczfaXRNClSsrVF9RJuSAKoDG-bnRZFMOcuwU/edit?usp=sharing
		// Thanks to: https://vasilisks.wordpress.com/2017/12/14/getting-temperature-from-ntc-kty83-kty84-on-mcu/#more-645
		// You can change pull up resistor and update NTC_RES_MOTOR for your hardware without changing polynom
		float res = NTC_RES_MOTOR(ADC_Value[is_motor_1 ? ADC_IND_TEMP_MOTOR : ADC_IND_TEMP_MOTOR_2]);
		float pow2 = res * res;
		temp_motor = 0.0000000102114874947423 * pow2 * res - 0.000069967997703501 * pow2 +
				0.243402040973194 * res - 160.145048329356;
	} break;

	case TEMP_SENSOR_KTY84_130: {
		float res = NTC_RES_MOTOR(ADC_Value[is_motor_1 ? ADC_IND_TEMP_MOTOR : ADC_IND_TEMP_MOTOR_2]);
		float pow2 = res * res;
		temp_motor = 0.00000001753671940525 * pow2 * res - 0.00010947809727469 * pow2 +
				0.335831536214307 * res - 143.317802704923;
	} break;
	}

	// If the reading is messed up (by e.g. reading 0 on the ADC and dividing by 0) we avoid putting an
	// invalid value in the filter, as it will never recover. It is probably safest to keep running the
	// motor even if the temperature reading fails. A config option to reduce power on invalid temperature
	// readings might be useful.
	if (UTILS_IS_NAN(temp_motor) || UTILS_IS_INF(temp_motor) || temp_motor > 600.0 || temp_motor < -200.0) {
		temp_motor = -100.0;
	}

	UTILS_LP_FAST(motor->m_temp_motor, temp_motor, MOTOR_TEMP_LPF);

#ifdef HW_HAS_GATE_DRIVER_SUPPLY_MONITOR
	UTILS_LP_FAST(motor->m_gate_driver_voltage, GET_GATE_DRIVER_SUPPLY_VOLTAGE(), 0.01);
#endif

	const float l_current_min_tmp = conf->l_current_min * conf->l_current_min_scale;
	const float l_current_max_tmp = conf->l_current_max * conf->l_current_max_scale;

	// Temperature MOSFET
	mc_interface_clear_warning(WARNING_CODE_3);
	float lo_min_mos = l_current_min_tmp;
	float lo_max_mos = l_current_max_tmp;
	if (motor->m_temp_fet < conf->l_temp_fet_start) {
		// Keep values
	} else if (motor->m_temp_fet > conf->l_temp_fet_end) {
		lo_min_mos = 0.0;
		lo_max_mos = 0.0;
		mc_interface_fault_stop(FAULT_CODE_OVER_TEMP_FET, !is_motor_1, false);
	} else {
		float maxc = fabsf(l_current_max_tmp);
		if (fabsf(l_current_min_tmp) > maxc) {
			maxc = fabsf(l_current_min_tmp);
		}

		maxc = utils_map(motor->m_temp_fet, conf->l_temp_fet_start, conf->l_temp_fet_end, maxc, 0.0);

		if (fabsf(l_current_min_tmp) > maxc) {
			lo_min_mos = SIGN(l_current_min_tmp) * maxc;
		}

		if (fabsf(l_current_max_tmp) > maxc) {
			lo_max_mos = SIGN(l_current_max_tmp) * maxc;
		}
		mc_interface_set_warning(WARNING_CODE_3);
	}

	// Temperature MOTOR
	mc_interface_clear_warning(WARNING_CODE_8);
	mc_interface_clear_warning(WARNING_CODE_4);
	float lo_min_mot = l_current_min_tmp;
	float lo_max_mot = l_current_max_tmp;
	if (!conf->m_temp_sens_ignored && temp_motor < -90.0) {
		utils_truncate_number_abs(&lo_max_mot, 200.0);
		mc_interface_set_warning(WARNING_CODE_8);
	} else if (motor->m_temp_motor < conf->l_temp_motor_start) {
		// Keep values
	} else if (motor->m_temp_motor > conf->l_temp_motor_end) {
		lo_min_mot = 0.0;
		lo_max_mot = 0.0;
		mc_interface_fault_stop(FAULT_CODE_OVER_TEMP_MOTOR, !is_motor_1, false);
	} else {
		float maxc = fabsf(l_current_max_tmp);
		if (fabsf(l_current_min_tmp) > maxc) {
			maxc = fabsf(l_current_min_tmp);
		}

		maxc = utils_map(motor->m_temp_motor, conf->l_temp_motor_start, conf->l_temp_motor_end, maxc, 0.0);

		if (fabsf(l_current_min_tmp) > maxc) {
			lo_min_mot = SIGN(l_current_min_tmp) * maxc;
		}

		if (fabsf(l_current_max_tmp) > maxc) {
			lo_max_mot = SIGN(l_current_max_tmp) * maxc;
		}
		mc_interface_set_warning(WARNING_CODE_4);
	}

	// Decreased temperatures during acceleration
	// in order to still have braking torque available
	const float temp_fet_accel_start = utils_map(conf->l_temp_accel_dec, 0.0, 1.0, conf->l_temp_fet_start, 25.0);
	const float temp_fet_accel_end = utils_map(conf->l_temp_accel_dec, 0.0, 1.0, conf->l_temp_fet_end, 25.0);
	const float temp_motor_accel_start = utils_map(conf->l_temp_accel_dec, 0.0, 1.0, conf->l_temp_motor_start, 25.0);
	const float temp_motor_accel_end = utils_map(conf->l_temp_accel_dec, 0.0, 1.0, conf->l_temp_motor_end, 25.0);

	float lo_fet_temp_accel = 0.0;
	if (motor->m_temp_fet < temp_fet_accel_start) {
		lo_fet_temp_accel = l_current_max_tmp;
	} else if (motor->m_temp_fet > temp_fet_accel_end) {
		lo_fet_temp_accel = 0.0;
	} else {
		lo_fet_temp_accel = utils_map(motor->m_temp_fet, temp_fet_accel_start,
				temp_fet_accel_end, l_current_max_tmp, 0.0);
	}

	float lo_motor_temp_accel = 0.0;
	if (motor->m_temp_motor < temp_motor_accel_start) {
		lo_motor_temp_accel = l_current_max_tmp;
	} else if (motor->m_temp_motor > temp_motor_accel_end) {
		lo_motor_temp_accel = 0.0;
	} else {
		lo_motor_temp_accel = utils_map(motor->m_temp_motor, temp_motor_accel_start,
				temp_motor_accel_end, l_current_max_tmp, 0.0);
	}

	// RPM max
	mc_interface_clear_warning(WARNING_CODE_19);
	mc_interface_clear_warning(WARNING_CODE_5);
	float lo_max_rpm = 0.0;
	const float rpm_pos_cut_start = conf->l_max_erpm * conf->l_erpm_start;
	const float rpm_pos_cut_end = conf->l_max_erpm;
	if (rpm_now < rpm_pos_cut_start) {
		lo_max_rpm = l_current_max_tmp;
	} else if (rpm_now > rpm_pos_cut_end) {
		lo_max_rpm = 0.0;
		mc_interface_set_warning(WARNING_CODE_19);
	} else {
		lo_max_rpm = utils_map(rpm_now, rpm_pos_cut_start, rpm_pos_cut_end, l_current_max_tmp, 0.0);
		mc_interface_set_warning(WARNING_CODE_5);
	}

	// RPM min
	mc_interface_clear_warning(WARNING_CODE_20);
	mc_interface_clear_warning(WARNING_CODE_6);
	float lo_min_rpm = 0.0;
	const float rpm_neg_cut_start = conf->l_min_erpm * conf->l_erpm_start;
	const float rpm_neg_cut_end = conf->l_min_erpm;
	if (rpm_now > rpm_neg_cut_start) {
		lo_min_rpm = l_current_max_tmp;
	} else if (rpm_now < rpm_neg_cut_end) {
		lo_min_rpm = 0.0;
		mc_interface_set_warning(WARNING_CODE_20);
	} else {
		lo_min_rpm = utils_map(rpm_now, rpm_neg_cut_start, rpm_neg_cut_end, l_current_max_tmp, 0.0);
		mc_interface_set_warning(WARNING_CODE_6);
	}

	// Duty max
	mc_interface_clear_warning(WARNING_CODE_7);
	float lo_max_duty = 0.0;
	if (duty_now_abs < conf->l_duty_start) {
		lo_max_duty = l_current_max_tmp;
	} else {
		lo_max_duty = utils_map(duty_now_abs, conf->l_duty_start, conf->l_max_duty, l_current_max_tmp, 0.0);
		mc_interface_set_warning(WARNING_CODE_7);
	}

	mc_interface_clear_warning(WARNING_CODE_35);
	float lo_max_nrf_adc = l_current_max_tmp;
#ifdef HW_TC500
	if (!conf->m_temp_sens_ignored && app_suron_get_nrf_adc() > 50) {
		mc_interface_set_warning(WARNING_CODE_35);
		utils_truncate_number_abs(&lo_max_nrf_adc, 200.0);
	}
#endif

	float lo_max = utils_min_abs(lo_max_mos, lo_max_mot);
	float lo_min = utils_min_abs(lo_min_mos, lo_min_mot);

	lo_max = utils_min_abs(lo_max, lo_max_rpm);
	lo_max = utils_min_abs(lo_max, lo_min_rpm);
	lo_max = utils_min_abs(lo_max, lo_fet_temp_accel);
	lo_max = utils_min_abs(lo_max, lo_motor_temp_accel);
	lo_max = utils_min_abs(lo_max, lo_max_duty);
	lo_max = utils_min_abs(lo_max, lo_max_nrf_adc);

	if (lo_max < conf->cc_min_current) {
		lo_max = conf->cc_min_current;
	}

	if (lo_min > -conf->cc_min_current) {
		lo_min = -conf->cc_min_current;
	}

	conf->lo_current_max = lo_max;
	conf->lo_current_min = lo_min;

	// Battery cutoff
	mc_interface_clear_warning(WARNING_CODE_17);
	mc_interface_clear_warning(WARNING_CODE_11);
	float lo_in_max_batt = 0.0;
	static float v_in_cutoff_filtered;
	UTILS_LP_FAST(v_in_cutoff_filtered, v_in, 0.5);
	if (v_in_cutoff_filtered > conf->l_battery_cut_start) {
		lo_in_max_batt = conf->l_in_current_max;
	} else if (v_in_cutoff_filtered < conf->l_battery_cut_end) {
		lo_in_max_batt = 0.0;
		mc_interface_set_warning(WARNING_CODE_17);
	} else {
		lo_in_max_batt = utils_map(v_in_cutoff_filtered, conf->l_battery_cut_start,
				conf->l_battery_cut_end, conf->l_in_current_max, 0.0);
		mc_interface_set_warning(WARNING_CODE_11);
	}

	// Battery cutoff regen
	mc_interface_clear_warning(WARNING_CODE_18);
	mc_interface_clear_warning(WARNING_CODE_12);
	float lo_in_min_batt_regen = 0.0;
	bool regen_adc_enabled = (app_get_configuration()->app_adc_conf.ctrl_type == ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER ||
								app_get_configuration()->app_adc_conf.ctrl_type == ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON);
	if (v_in < conf->l_battery_cut_regen_start) {
		lo_in_min_batt_regen = conf->l_in_current_min;
	} else if (v_in > conf->l_battery_cut_regen_end) {
		lo_in_min_batt_regen = 0.0;
		if (regen_adc_enabled) {
			mc_interface_set_warning(WARNING_CODE_18);
		}
	} else {
		lo_in_min_batt_regen = utils_map(v_in, conf->l_battery_cut_regen_start,
			conf->l_battery_cut_regen_end, conf->l_in_current_min, 0.0);
		if (regen_adc_enabled) {
			mc_interface_set_warning(WARNING_CODE_12);
		}
	}

	// Wattage limits
	const float lo_in_max_watt = conf->l_watt_max / v_in;
	const float lo_in_min_watt = conf->l_watt_min / v_in;

	float lo_in_max = utils_min_abs(lo_in_max_watt, lo_in_max_batt);
	float lo_in_min = utils_min_abs(lo_in_min_watt, lo_in_min_batt_regen);

	mc_interface_clear_warning(WARNING_CODE_13);
	mc_interface_clear_warning(WARNING_CODE_14);
	if (lo_in_max == lo_in_max_watt && (mc_interface_get_tot_current_in_filtered() > lo_in_max_watt)) {
		mc_interface_set_warning(WARNING_CODE_13);
	}
	if (lo_in_min == lo_in_min_watt && (mc_interface_get_tot_current_in_filtered() < lo_in_min_watt)) {
		mc_interface_set_warning(WARNING_CODE_14);
	}

	// BMS limits
	bms_update_limits(&lo_in_min,  &lo_in_max, conf->l_in_current_min, conf->l_in_current_max);

	conf->lo_in_current_max = utils_min_abs(conf->l_in_current_max, lo_in_max);
	conf->lo_in_current_min = utils_min_abs(conf->l_in_current_min, lo_in_min);

	// Maximum current right now
//	float duty_abs = fabsf(mc_interface_get_duty_cycle_now());
//
//	// TODO: This is not an elegant solution.
//	if (m_conf.motor_type == MOTOR_TYPE_FOC) {
//		duty_abs *= SQRT3_BY_2;
//	}
//
//	if (duty_abs > 0.001) {
//		conf->lo_current_motor_max_now = utils_min_abs(conf->lo_current_max, conf->lo_in_current_max / duty_abs);
//		conf->lo_current_motor_min_now = utils_min_abs(conf->lo_current_min, conf->lo_in_current_min / duty_abs);
//	} else {
//		conf->lo_current_motor_max_now = conf->lo_current_max;
//		conf->lo_current_motor_min_now = conf->lo_current_min;
//	}

	// Note: The above code should work, but many people have reported issues with it. Leaving it
	// disabled for now until I have done more investigation.
	conf->lo_current_motor_max_now = conf->lo_current_max;
	conf->lo_current_motor_min_now = conf->lo_current_min;

	if (mcpwm_foc_get_control_mode() == CONTROL_MODE_CURRENT && (mc_interface_get_duty_cycle_now() < 0.0)) {
		conf->lo_current_min = -conf->lo_current_max;

		conf->lo_in_current_min = -conf->lo_in_current_max;
	}
}

static volatile motor_if_state_t *motor_now(void) {
#ifdef HW_HAS_DUAL_MOTORS
	return mc_interface_motor_now() == 1 ? &m_motor_1 : &m_motor_2;
#else
	return &m_motor_1;
#endif
}

static void run_timer_tasks(volatile motor_if_state_t *motor) {
	bool is_motor_1 = motor == &m_motor_1;
	mc_interface_select_motor_thread(is_motor_1 ? 1 : 2);

	motor->m_f_samp_now = mc_interface_get_sampling_frequency_now();

	// Update backup data (for motor 1 only)
	if (is_motor_1) {
		uint32_t odometer = (uint32_t)mc_interface_get_distance_abs();
		if (m_motor_1.m_odometer_last > odometer) { //if tacho was reset
			g_backup.odometer += odometer;
		} else {
			g_backup.odometer += odometer - m_motor_1.m_odometer_last;
		}
		m_motor_1.m_odometer_last = odometer;
	}

	static volatile systime_t working_hours_cnt_last = 0;
	if (chVTTimeElapsedSinceX(working_hours_cnt_last) > S2ST(1)) {
		fram_data.working_seconds++;
		working_hours_cnt_last = chVTGetSystemTimeX();
	}

	// Decrease fault iterations
	if (motor->m_ignore_iterations > 0) {
		motor->m_ignore_iterations--;
	} else {
		if (!(is_motor_1 ? IS_DRV_FAULT() : IS_DRV_FAULT_2())) {
			motor->m_fault_now = FAULT_CODE_NONE;
		}
	}

	update_override_limits(motor, &motor->m_conf);

#define peak_max(a, b)   (((a) > (b)) ? (a) : (b))
#define peak_min(a, b)   (((a) < (b) || !(b)) ? (a) : (b))

	if (!mc_peak.reset_done && !(PWR->CSR & PWR_CSR_PVDO) && (chVTGetSystemTimeX() >= S2ST(1))) {
		bms_peak* bms_pk = bms_get_peak(false);

		mc_peak_history_tmp.voltagemax = peak_max((uint16_t)(mc_peak.voltagemax * 1e2), mc_peak_history_tmp.voltagemax);
		mc_peak_history_tmp.voltagemin = peak_min((uint16_t)(mc_peak.voltagemin * 1e2), mc_peak_history_tmp.voltagemin);
		mc_peak_history_tmp.powermax = peak_max((uint16_t)(mc_peak.powermax), mc_peak_history_tmp.powermax);
		mc_peak_history_tmp.powermin = peak_min((int32_t)(mc_peak.powermin), mc_peak_history_tmp.powermin);
		mc_peak_history_tmp.motorcurrentmax = peak_max((uint16_t)(mc_peak.motorcurrentmax * 1e1), mc_peak_history_tmp.motorcurrentmax);
		mc_peak_history_tmp.motorcurrentmin = peak_min((int16_t)(mc_peak.motorcurrentmin * 1e1), mc_peak_history_tmp.motorcurrentmin);
		mc_peak_history_tmp.motorcurrentmaxfiltered = peak_max((uint16_t)(mc_peak.motorcurrentmaxfiltered * 1e1), mc_peak_history_tmp.motorcurrentmaxfiltered);
		mc_peak_history_tmp.motorcurrentminfiltered = peak_min((int16_t)(mc_peak.motorcurrentminfiltered * 1e1), mc_peak_history_tmp.motorcurrentminfiltered);
		mc_peak_history_tmp.batterycurrentmax = peak_max((uint16_t)(mc_peak.batterycurrentmax * 1e1), mc_peak_history_tmp.batterycurrentmax);
		mc_peak_history_tmp.batterycurrentmin = peak_min((int16_t)(mc_peak.batterycurrentmin * 1e1), mc_peak_history_tmp.batterycurrentmin);
		mc_peak_history_tmp.abspeak = peak_max((uint16_t)(mc_peak.abspeak * 1e1), mc_peak_history_tmp.abspeak);
		mc_peak_history_tmp.abs_peak_filtered = peak_max((uint16_t)(mc_peak.abs_peak_filtered * 1e1), mc_peak_history_tmp.abs_peak_filtered);

		mc_peak_history_tmp.motorpeakmax = peak_max((uint16_t)(mc_peak.motorpeakmax * 1e1), mc_peak_history_tmp.motorpeakmax);
		mc_peak_history_tmp.motorpeakmin = peak_min((int16_t)(mc_peak.motorpeakmin * 1e1), mc_peak_history_tmp.motorpeakmin);
		mc_peak_history_tmp.batterypeak = peak_max((uint16_t)(mc_peak.batterypeak * 1e1), mc_peak_history_tmp.batterypeak);

		mc_peak_history_tmp.motortempmax = peak_max((int16_t)(mc_peak.motortempmax * 1e2), mc_peak_history_tmp.motortempmax);
		mc_peak_history_tmp.motortempmin = peak_min((int16_t)(mc_peak.motortempmin * 1e2), mc_peak_history_tmp.motortempmin);
		mc_peak_history_tmp.fettempmax = peak_max((int16_t)(mc_peak.fettempmax * 1e2), mc_peak_history_tmp.fettempmax);
		mc_peak_history_tmp.fettempmin = peak_min((int16_t)(mc_peak.fettempmin * 1e2), mc_peak_history_tmp.fettempmin);
		mc_peak_history_tmp.rpmmax = peak_max((uint16_t)(mc_peak.rpmmax), mc_peak_history_tmp.rpmmax);
		mc_peak_history_tmp.rpmmin = peak_min((int32_t)(mc_peak.rpmmin), mc_peak_history_tmp.rpmmin);
		mc_peak_history_tmp.speedmax = peak_max((uint16_t)(mc_peak.speedmax * 1e2), mc_peak_history_tmp.speedmax);
		mc_peak_history_tmp.speedmin = peak_min((int16_t)(mc_peak.speedmin * 1e2), mc_peak_history_tmp.speedmin);

		mc_peak_history_tmp.bms_max_cell_temp = peak_max(bms_pk->max_cell_temp, mc_peak_history_tmp.bms_max_cell_temp);
		mc_peak_history_tmp.bms_min_cell_temp = peak_min(bms_pk->min_cell_temp, mc_peak_history_tmp.bms_min_cell_temp);
		mc_peak_history_tmp.bms_max_cell_volt = peak_max(bms_pk->max_cell_volt, mc_peak_history_tmp.bms_max_cell_volt);
		mc_peak_history_tmp.bms_min_cell_volt = peak_min(bms_pk->min_cell_volt, mc_peak_history_tmp.bms_min_cell_volt);
		mc_peak_history_tmp.bms_max_dis_fet_temp = peak_max(bms_pk->max_dis_fet_temp, mc_peak_history_tmp.bms_max_dis_fet_temp);

		mc_peak_history_tmp.bms_max_current_in = peak_max((int16_t)(bms_pk->max_current_in / 100), mc_peak_history_tmp.bms_max_current_in);
		mc_peak_history_tmp.bms_min_current_in = peak_min((int16_t)(bms_pk->min_current_in / 100), mc_peak_history_tmp.bms_min_current_in);

		mc_peak_history_tmp.odometer = mc_interface_get_odometer();
		mc_peak_history_tmp.cnt = peak_histoy_bkp_sram_cnt;

		memcpy((void*)&mc_peak_history[peak_histoy_bkp_sram_num], (void*)&mc_peak_history_tmp, sizeof(mc_peak_history_t));
	}

	// Update auxiliary output
	switch (motor->m_conf.m_out_aux_mode) {
	case OUT_AUX_MODE_OFF:
		AUX_OFF();
		break;

	case OUT_AUX_MODE_ON_AFTER_2S:
		if (chVTGetSystemTimeX() >= MS2ST(2000)) {
			AUX_ON();
		}
		break;

	case OUT_AUX_MODE_ON_AFTER_5S:
		if (chVTGetSystemTimeX() >= MS2ST(5000)) {
			AUX_ON();
		}
		break;

	case OUT_AUX_MODE_ON_AFTER_10S:
		if (chVTGetSystemTimeX() >= MS2ST(10000)) {
			AUX_ON();
		}
		break;

	default:
		break;
	}


	// Trigger encoder error rate fault, using 5% errors as threshold.
	// Relevant only in FOC mode with encoder enabled
	if(motor->m_conf.motor_type == MOTOR_TYPE_FOC &&
			motor->m_conf.foc_sensor_mode == FOC_SENSOR_MODE_ENCODER &&
			mcpwm_foc_is_using_encoder() &&
			encoder_spi_get_error_rate() > 0.05) {
		mc_interface_fault_stop(FAULT_CODE_ENCODER_SPI, !is_motor_1, false);
	}

	if(motor->m_conf.motor_type == MOTOR_TYPE_FOC &&
			motor->m_conf.foc_sensor_mode == FOC_SENSOR_MODE_ENCODER &&
			mcpwm_foc_is_using_encoder() &&
			encoder_get_no_magnet_error_rate() > 0.05) {
		mc_interface_fault_stop(FAULT_CODE_ENCODER_NO_MAGNET, !is_motor_1, false);
	}

	if(motor->m_conf.motor_type == MOTOR_TYPE_FOC &&
			motor->m_conf.foc_sensor_mode == FOC_SENSOR_MODE_ENCODER &&
			motor->m_conf.m_sensor_port_mode == SENSOR_PORT_MODE_SINCOS) {

		if (encoder_sincos_get_signal_below_min_error_rate() > 0.05)
			mc_interface_fault_stop(FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE, !is_motor_1, false);
		if (encoder_sincos_get_signal_above_max_error_rate() > 0.05)
			mc_interface_fault_stop(FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE, !is_motor_1, false);
	}

	bool dccal_done = mc_interface_dccal_done();

	if(motor->m_conf.motor_type == MOTOR_TYPE_FOC &&
			motor->m_conf.foc_sensor_mode == FOC_SENSOR_MODE_ENCODER &&
			motor->m_conf.m_sensor_port_mode == SENSOR_PORT_MODE_AD2S1205) {
		if (encoder_resolver_loss_of_tracking_error_rate() > 0.05)
			mc_interface_fault_stop(FAULT_CODE_RESOLVER_LOT, !is_motor_1, false);
		if (encoder_resolver_degradation_of_signal_error_rate() > 0.05)
			mc_interface_fault_stop(FAULT_CODE_RESOLVER_DOS, !is_motor_1, false);
		if (encoder_resolver_loss_of_signal_error_rate() > 0.04)
			mc_interface_fault_stop(FAULT_CODE_RESOLVER_LOS, !is_motor_1, false);
	}
	// TODO: Implement for BLDC and GPDRIVE
	if(motor->m_conf.motor_type == MOTOR_TYPE_FOC && dccal_done) {
		int curr0_offset;
		int curr1_offset;
		int curr2_offset;

#ifdef HW_HAS_DUAL_MOTORS
		mcpwm_foc_get_current_offsets(&curr0_offset, &curr1_offset, &curr2_offset, motor == &m_motor_2);
#else
		mcpwm_foc_get_current_offsets(&curr0_offset, &curr1_offset, &curr2_offset, false);
#endif

#ifdef HW_HAS_DUAL_PARALLEL
#define MIDDLE_ADC 4096
#else
#define MIDDLE_ADC 2048
#endif

		if (abs(curr0_offset - MIDDLE_ADC) > HW_MAX_CURRENT_OFFSET) {
			mc_interface_fault_stop(FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1, !is_motor_1, false);
		}
		if (abs(curr1_offset - MIDDLE_ADC) > HW_MAX_CURRENT_OFFSET) {
			mc_interface_fault_stop(FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2, !is_motor_1, false);
		}
#ifdef HW_HAS_3_SHUNTS
		if (abs(curr2_offset - MIDDLE_ADC) > HW_MAX_CURRENT_OFFSET) {
			mc_interface_fault_stop(FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3, !is_motor_1, false);
		}
#endif
		float phase0_volt_startup;
		float phase1_volt_startup;
		float phase2_volt_startup;
		float input_volt_startup;

		mcpwm_foc_get_phase_volt_startup(&phase0_volt_startup, &phase1_volt_startup, &phase2_volt_startup, &input_volt_startup);

		if (phase0_volt_startup < 0.1 || (phase0_volt_startup / input_volt_startup) > 0.9) {
			mc_interface_fault_stop(FAULT_CODE_PHASE_1_FET, !is_motor_1, false);
		}
		if (phase1_volt_startup < 0.1 || (phase1_volt_startup / input_volt_startup) > 0.9) {
			mc_interface_fault_stop(FAULT_CODE_PHASE_2_FET, !is_motor_1, false);
		}
		if (phase2_volt_startup < 0.1 || (phase2_volt_startup / input_volt_startup) > 0.9) {
			mc_interface_fault_stop(FAULT_CODE_PHASE_3_FET, !is_motor_1, false);
		}
	}

	// Monitor currents balance. The sum of the 3 currents should be zero
#ifdef HW_HAS_3_SHUNTS
	if (!motor->m_conf.foc_sample_high_current && dccal_done) { // This won't work when high current sampling is used
		motor->m_motor_current_unbalance = mc_interface_get_abs_motor_current_unbalance();

		if (motor->m_motor_current_unbalance > MCCONF_MAX_CURRENT_UNBALANCE) {
			UTILS_LP_FAST(motor->m_motor_current_unbalance_error_rate, 1.0, (1 / 1000.0));
		} else {
			UTILS_LP_FAST(motor->m_motor_current_unbalance_error_rate, 0.0, (1 / 1000.0));
		}

		if (motor->m_motor_current_unbalance_error_rate > MCCONF_MAX_CURRENT_UNBALANCE_RATE) {
			mc_interface_fault_stop(FAULT_CODE_UNBALANCED_CURRENTS, !is_motor_1, false);
		}
	}
#endif
}

static THD_FUNCTION(timer_thread, arg) {
	(void)arg;

	chRegSetThreadName("mcif timer");

	for(;;) {
		run_timer_tasks(&m_motor_1);
#ifdef HW_HAS_DUAL_MOTORS
		run_timer_tasks(&m_motor_2);
#endif

		chThdSleepMilliseconds(1);
	}
}

static THD_FUNCTION(sample_send_thread, arg) {
	(void)arg;

	chRegSetThreadName("SampleSender");
	sample_send_tp = chThdGetSelfX();

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		int len = 0;
		int offset = 0;

		switch (m_sample_mode_last) {
		case DEBUG_SAMPLING_NOW:
		case DEBUG_SAMPLING_START:
			len = m_sample_len;
			break;

		case DEBUG_SAMPLING_TRIGGER_START:
		case DEBUG_SAMPLING_TRIGGER_FAULT:
		case DEBUG_SAMPLING_TRIGGER_START_NOSEND:
		case DEBUG_SAMPLING_TRIGGER_FAULT_NOSEND:
			len = ADC_SAMPLE_MAX_LEN;
			offset = m_sample_trigger - m_sample_len;
			break;

		default:
			break;
		}

		for (int i = 0;i < len;i++) {
			uint8_t buffer[40];
			int32_t index = 0;
			int ind_samp = i + offset;

			while (ind_samp >= ADC_SAMPLE_MAX_LEN) {
				ind_samp -= ADC_SAMPLE_MAX_LEN;
			}

			while (ind_samp < 0) {
				ind_samp += ADC_SAMPLE_MAX_LEN;
			}

			buffer[index++] = COMM_SAMPLE_PRINT;
			buffer_append_float32_auto(buffer, (float)m_curr0_samples[ind_samp] * FAC_CURRENT, &index);
			buffer_append_float32_auto(buffer, (float)m_curr1_samples[ind_samp] * FAC_CURRENT, &index);
			buffer_append_float32_auto(buffer, ((float)m_ph1_samples[ind_samp] / 4096.0 * V_REG) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR, &index);
			buffer_append_float32_auto(buffer, ((float)m_ph2_samples[ind_samp] / 4096.0 * V_REG) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR, &index);
			buffer_append_float32_auto(buffer, ((float)m_ph3_samples[ind_samp] / 4096.0 * V_REG) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR, &index);
			buffer_append_float32_auto(buffer, ((float)m_vzero_samples[ind_samp] / 4096.0 * V_REG) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_INPUT_FACTOR, &index);
			buffer_append_float32_auto(buffer, (float)m_curr_fir_samples[ind_samp] / (8.0 / FAC_CURRENT), &index);
			buffer_append_float32_auto(buffer, (float)m_f_sw_samples[ind_samp] * 10.0, &index);
			buffer[index++] = m_status_samples[ind_samp];
			buffer[index++] = m_phase_samples[ind_samp];

			send_func_sample(buffer, index);
		}
	}
}

static THD_FUNCTION(fault_stop_thread, arg) {
	(void)arg;

	chRegSetThreadName("Fault Stop");
	fault_stop_tp = chThdGetSelfX();

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);
#ifdef HW_HAS_DUAL_MOTORS
		volatile motor_if_state_t *motor = m_fault_stop_is_second_motor ? &m_motor_2 : &m_motor_1;
#else
		volatile motor_if_state_t *motor = &m_motor_1;
#endif

		mc_interface_select_motor_thread(m_fault_stop_is_second_motor ? 2 : 1);

		if (motor->m_fault_now == m_fault_stop_fault) {
			motor->m_ignore_iterations = motor->m_conf.m_fault_stop_time_ms;
			continue;
		}

		// Some hardwares always have a DRV-fault at boot. Therefore we do not log it in the
		// beginning to avoid confusing the user. After dccal all faults should be gone if
		// everything is ok.
		bool is_log_ok = (mc_interface_dccal_done() || motor->m_fault_now != FAULT_CODE_DRV);

		if (is_log_ok && motor->m_fault_now == FAULT_CODE_NONE) {
			// Sent to terminal fault logger so that all faults and their conditions
			// can be printed for debugging.
			utils_sys_lock_cnt();
			volatile int val_samp = TIM8->CCR1;
			volatile int current_samp = TIM1->CCR4;
			volatile int tim_top = TIM1->ARR;
			utils_sys_unlock_cnt();

			fault_data fdata;
			fdata.motor = m_fault_stop_is_second_motor ? 2 : 1;
			fdata.fault = m_fault_stop_fault;
			fdata.current = mc_interface_get_tot_current();
			fdata.current_filtered = mc_interface_get_tot_current_filtered();
			fdata.voltage = GET_INPUT_VOLTAGE();
			fdata.gate_driver_voltage = motor->m_gate_driver_voltage;
			fdata.duty = mc_interface_get_duty_cycle_now();
			fdata.rpm = mc_interface_get_rpm();
			fdata.odometer = mc_interface_get_odometer();
			fdata.cycles_running = motor->m_cycles_running;
			fdata.tim_val_samp = val_samp;
			fdata.tim_current_samp = current_samp;
			fdata.tim_top = tim_top;
			fdata.comm_step = mcpwm_get_comm_step();
			fdata.temperature = NTC_TEMP(ADC_IND_TEMP_MOS);
			fdata.current_in = mc_interface_get_tot_current_in_filtered();
			fdata.motor_temperature = mc_interface_temp_motor_filtered();
#ifdef HW_HAS_DRV8301
			if (m_fault_stop_fault == FAULT_CODE_DRV) {
				fdata.drv8301_faults = drv8301_read_faults();
			}
#elif defined(HW_HAS_DRV8320S)
			if (m_fault_stop_fault == FAULT_CODE_DRV) {
				fdata.drv8301_faults = drv8320s_read_faults();
			}
#elif defined(HW_HAS_DRV8323S)
			if (m_fault_stop_fault == FAULT_CODE_DRV) {
				fdata.drv8301_faults = drv8323s_read_faults();
			}
#endif
			if (!(PWR->CSR & PWR_CSR_PVDO)) //if MCU VDD is below 2.9V
			{
				terminal_add_fault_data(&fdata);

				fram_data.error_cnt[fdata.fault]++;
				fram_data.last_fault_data.fault = fdata.fault;
				fram_data.last_fault_data.current = fdata.current;
				fram_data.last_fault_data.current_filtered = fdata.current_filtered;
				fram_data.last_fault_data.voltage = fdata.voltage;
				fram_data.last_fault_data.gate_driver_voltage = fdata.gate_driver_voltage;
				fram_data.last_fault_data.duty = fdata.duty;
				fram_data.last_fault_data.rpm = fdata.rpm;
				fram_data.last_fault_data.odometer = fdata.odometer;
				fram_data.last_fault_data.temperature = fdata.temperature;
				fram_data.last_fault_data.current_in = fdata.current_in;
				fram_data.last_fault_data.motor_temperature = fdata.motor_temperature;
			}
		}

		motor->m_ignore_iterations = motor->m_conf.m_fault_stop_time_ms;

		switch (motor->m_conf.motor_type) {
		case MOTOR_TYPE_BLDC:
		case MOTOR_TYPE_DC:
			mcpwm_stop_pwm();
			break;

		case MOTOR_TYPE_FOC:
			mcpwm_foc_stop_pwm(m_fault_stop_is_second_motor);
			break;

		case MOTOR_TYPE_GPD:
			gpdrive_set_mode(GPD_OUTPUT_MODE_NONE);
			break;

		default:
			break;
		}

		motor->m_fault_now = m_fault_stop_fault;

#ifdef HW_TC500
		app_suron_signal_fault();
#endif
	}
}

/**
 * Get mc_configuration CRC (motor 1 or 2)
 *
 * @param conf
 * Pointer to mc_configuration or NULL for current config
 *
 * @param is_motor_2
 * true if motor2, false if motor1
 * 
 * @return
 * CRC16 (with crc field in struct temporarily set to zero).
 */
unsigned mc_interface_calc_crc(mc_configuration* conf_in, bool is_motor_2) {
	volatile mc_configuration* conf = conf_in;

	if(conf == NULL) {
		if(is_motor_2) {
#ifdef HW_HAS_DUAL_MOTORS
			conf = &(m_motor_2.m_conf);
#else
			return 0; //shouldn't be here
#endif
		} else {
			conf = &(m_motor_1.m_conf);
		}
	}

	unsigned crc_old = conf->crc;
	conf->crc = 0;
	unsigned crc_new = crc16((uint8_t*)conf, sizeof(mc_configuration));
	conf->crc = crc_old;
	return crc_new;
}

/**
 * Set odometer value in meters.
 *
 * @param new_odometer_meters
 * new odometer value in meters
 */
void mc_interface_set_odometer(uint32_t new_odometer_meters) {
	g_backup.odometer = new_odometer_meters;
}

/**
 * Return current odometer value in meters.
 *
 * @return
 * Odometer value in meters, including current trip
 */
uint32_t mc_interface_get_odometer(void) {
	return g_backup.odometer;
}

/**
 * Ignore motor control commands for this amount of time on both motors.
 */
void mc_interface_ignore_input_both(int time_ms) {
	if (time_ms > m_motor_1.m_ignore_iterations) {
		m_motor_1.m_ignore_iterations = time_ms;
	}

#ifdef HW_HAS_DUAL_MOTORS
	if (time_ms > m_motor_2.m_ignore_iterations) {
		m_motor_2.m_ignore_iterations = time_ms;
	}
#endif
}

uint8_t mc_interface_fault_to_number(mc_fault_code fault) {
	switch (fault) {
		case FAULT_CODE_NONE: return 0; break;
		case FAULT_CODE_ABS_OVER_CURRENT: return 2; break;
		case FAULT_CODE_OVER_TEMP_FET: return 4; break;
		case FAULT_CODE_OVER_TEMP_MOTOR: return 6; break;
		case FAULT_CODE_OVER_VOLTAGE: return 7; break;
		case FAULT_CODE_UNDER_VOLTAGE: return 9; break;
		case FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE: return 10; break;
		case FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE: return 12; break;
		case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1: return 14; break;
		case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2: return 15; break;
		case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3: return 17; break;
		case FAULT_CODE_UNBALANCED_CURRENTS: return 19; break;
		case FAULT_CODE_PHASE_1_FET: return 20; break;
		case FAULT_CODE_PHASE_2_FET: return 22; break;
		case FAULT_CODE_PHASE_3_FET: return 24; break;
		case FAULT_CODE_FLASH_CORRUPTION_APP_CFG: return 25; break;
		case FAULT_CODE_FLASH_CORRUPTION_MC_CFG: return 27; break;
		case FAULT_CODE_FLASH_CORRUPTION: return 28; break;
		case FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET: return 30; break;
		case FAULT_CODE_MCU_UNDER_VOLTAGE: return 31; break;
		case FAULT_CODE_ENCODER_SPI: return 33; break;
		case FAULT_CODE_BRK: return 35; break;
		case FAULT_CODE_RESOLVER_LOT: return 36; break;
		case FAULT_CODE_RESOLVER_DOS: return 38; break;
		case FAULT_CODE_RESOLVER_LOS: return 40; break;
		case FAULT_CODE_ENCODER_NO_MAGNET: return 41; break;
		case FAULT_CODE_DRV: return 42; break;
		case FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE: return 44; break;
		case FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE: return 46; break;
		case FAULT_CODE_TORP_1: return 49; break;
		case FAULT_CODE_TORP_3: return 51; break;
		case FAULT_CODE_TORP_2: return 53; break;
		default: return 0; break;
	}
}

float mc_interface_get_throttle_rise(void) {
	return max_throttle_rise;
}

void mc_interface_set_warning(mc_warning_code war) {
	if (!(((motor_now()->m_warning_old >> war) & 1ULL)) && !(((motor_now()->m_warning >> war) & 1ULL))) {
		fram_data.warning_cnt[war]++;
	}
	motor_now()->m_warning |= 1ULL << war;
}

void mc_interface_clear_warning(mc_warning_code war) {
	motor_now()->m_warning_old |= ((motor_now()->m_warning >> war) & 1ULL) << war;

	motor_now()->m_warning &= ~(1ULL << war);
}

void mc_interface_reset_peak(void) {
	mc_peak.motorpeakmax = 0.0;
	mc_peak.batterypeak = 0.0;

	mc_peak.voltagemax = 0.0;
	mc_peak.voltagemin = GET_INPUT_VOLTAGE();
	mc_peak.motorcurrentmax = 0.0;
	mc_peak.motorcurrentmaxfiltered = 0.0;
	mc_peak.batterycurrentmax = 0.0;
	mc_peak.batterycurrentmin = 0.0;
	mc_peak.abspeak = 0.0;
	mc_peak.abs_peak_filtered = 0.0;
	mc_peak.motortempmax = -100.0;
	mc_peak.fettempmax = -100.0;
	mc_peak.rpmmax = 0.0;
	mc_peak.powermax = 0.0;
	mc_peak.powermin = 0.0;
	mc_peak.speedmax = 0.0;

	mc_peak.fettempmin = 10000.0;
	mc_peak.motortempmin = 10000.0;
	mc_peak.motorcurrentmin = 0.0;
	mc_peak.motorcurrentminfiltered = 0.0;
	mc_peak.speedmin = 10000.0;
	mc_peak.rpmmin = 10000.0;
	mc_peak.motorpeakmin = 10000.0;

	mc_peak.reset_done = true;
}

void mc_interface_print_peak(void) {
	commands_printf("---------------------PEAK----------------------");
	commands_printf("Max Voltage: %.2f V\n", (double)mc_peak.voltagemax);
	commands_printf("Min Voltage: %.2f V\n", (double)mc_peak.voltagemin);

	commands_printf("Power Max: %.2f W\n", (double)mc_peak.powermax);
	commands_printf("Power Regen: %.2f W\n", (double)mc_peak.powermin);

	commands_printf("Motor Current Max: %.2f A\n", (double)mc_peak.motorcurrentmax);
	commands_printf("Motor Current Min: %.2f A\n", (double)mc_peak.motorcurrentmin);

	commands_printf("Motor CurrentFiltered Max: %.2f A\n", (double)mc_peak.motorcurrentmaxfiltered);
	commands_printf("Motor CurrentFiltered Min: %.2f A\n", (double)mc_peak.motorcurrentminfiltered);

	commands_printf("Battery Current Max: %.2f A\n", (double)mc_peak.batterycurrentmax);
	commands_printf("Battery Current Regen: %.2f A\n", (double)mc_peak.batterycurrentmin);

	commands_printf("ABS Current Max: %.2f A\n", (double)mc_peak.abspeak);
	commands_printf("ABS Current Filtered Max: %.2f A\n", (double)mc_peak.abs_peak_filtered);

	commands_printf("Motor Peak Max: %.2f A\n", (double)mc_peak.motorpeakmax);
	commands_printf("Motor Peak Min: %.2f A\n", (double)mc_peak.motorpeakmin);

	commands_printf("BatteryPeak: %.2f A\n", (double)mc_peak.batterypeak);

	commands_printf("Motor Temp Max: %.2f C\n", (double)mc_peak.motortempmax);
	commands_printf("Motor Temp Min: %.2f C\n", (double)mc_peak.motortempmin);

	commands_printf("Fet Temp Max: %.2f C\n", (double)mc_peak.fettempmax);
	commands_printf("Fet Temp Min: %.2f C\n", (double)mc_peak.fettempmin);

	commands_printf("RPM  Max: %.2f ERPM\n", (double)mc_peak.rpmmax);
	commands_printf("RPM  Min: %.2f ERPM\n", (double)mc_peak.rpmmin);

	commands_printf("Speed Max: %.2f km/h\n", (double)mc_peak.speedmax);
	commands_printf("Speed Min: %.2f km/h\n", (double)mc_peak.speedmin);

	volatile bms_peak* bms_pk = bms_get_peak(false);
	commands_printf("BMS max cell temp: %d C\n", bms_pk->max_cell_temp);
	commands_printf("BMS min cell temp: %d C\n", bms_pk->min_cell_temp);

	commands_printf("BMS max dis. fet temp: %d C\n", bms_pk->max_dis_fet_temp);

	commands_printf("BMS max cell V: %.2f\n", (double)((float)bms_pk->max_cell_volt / 1e3));
	commands_printf("BMS min cell V: %.2f\n", (double)((float)bms_pk->min_cell_volt / 1e3));

	commands_printf("BMS max current in A: %.2f\n", (double)((float)bms_pk->max_current_in / 1e3));
	commands_printf("BMS min current in A: %.2f\n", (double)((float)bms_pk->min_current_in / 1e3));
}

static THD_FUNCTION(fram_thread, arg) {
	(void)arg;

	chRegSetThreadName("FRAM update");

	chThdSleepMilliseconds(100);
	hw_start_i2c();
	chThdSleepMilliseconds(10);

	i2cAcquireBus(&HW_I2C_DEV);

	uint8_t fram_fail_num = 0;

	fram_data_temp.fram_start_address = 0;
	i2caddr_t i2c_addr = utils_MB85RC16_fram_I2CAddressAdapt(&fram_data_temp.fram_start_address);

	msg_t ret = i2cMasterTransmitTimeout(&HW_I2C_DEV, i2c_addr, (uint8_t*)&fram_data_temp.fram_start_address + 1, 1, (uint8_t*)&fram_data_temp.crc, sizeof(fram_data_t) - 2, MS2ST(10)); 

	uint32_t crc = (uint32_t)crc16((uint8_t*)&fram_data_temp.struct_version, sizeof(fram_data_temp) - 6); 

	bool fram_has_valid_data = true;
	if (crc != fram_data_temp.crc || ret != MSG_OK) {
		fram_data_temp.fram_start_address = 0x400; 
		i2c_addr = utils_MB85RC16_fram_I2CAddressAdapt(&fram_data_temp.fram_start_address);

		msg_t ret_bkp = i2cMasterTransmitTimeout(&HW_I2C_DEV, i2c_addr, (uint8_t*)&fram_data_temp.fram_start_address + 1, 1, (uint8_t*)&fram_data_temp.crc, sizeof(fram_data_t) - 2, MS2ST(10));

		crc = (uint32_t)crc16((uint8_t*)&fram_data_temp.struct_version, sizeof(fram_data_t) - 6);

		if (crc != fram_data_temp.crc || ret_bkp != MSG_OK) {
			fram_has_valid_data = false;

			memset((uint8_t*)&fram_data.struct_version, 0, sizeof(fram_data_t) - 6);
			if (ret != MSG_OK && ret_bkp != MSG_OK) {
				fram_comm_ok = false;
				return;
			}
		}
	}
	if (fram_has_valid_data) {
		memcpy((void*)&fram_data, (void*)&fram_data_temp, sizeof(fram_data_t));
		mc_interface_set_odometer(fram_data.odometer);
		motor_now()->m_warning = 0;
		motor_now()->m_warning_old = 0;

#ifdef HW_TC500
		app_suron_set_mod_state(fram_data.last_mode);
#endif
	} 

	fram_data.startup_num++;

	for (;;) {
		chThdSleepMilliseconds(100);

		if (fram_fail_num > 5) {
			fram_fail_num = 0;

			i2cReleaseBus(&HW_I2C_DEV);
			hw_stop_i2c();
			chThdSleepMilliseconds(100);
			hw_start_i2c();
			chThdSleepMilliseconds(500);
			i2cAcquireBus(&HW_I2C_DEV);
		}

		fram_data.odometer = mc_interface_get_odometer();
#ifdef HW_TC500
		fram_data.last_mode = app_suron_get_modbutton_state();
#endif

		if (!mc_peak.reset_done && mc_interface_dccal_done()) {
			bms_peak* bms_pk = bms_get_peak(false);
			fram_data.peak_all_time.voltagemax = peak_max((uint16_t)(mc_peak.voltagemax * 1e2), fram_data.peak_all_time.voltagemax);
			fram_data.peak_all_time.voltagemin = peak_min((uint16_t)(mc_peak.voltagemin * 1e2), fram_data.peak_all_time.voltagemin);
			fram_data.peak_all_time.powermax = peak_max((uint16_t)(mc_peak.powermax), fram_data.peak_all_time.powermax);
			fram_data.peak_all_time.powermin = peak_min((int32_t)(mc_peak.powermin), fram_data.peak_all_time.powermin);
			fram_data.peak_all_time.motorcurrentmax = peak_max((uint16_t)(mc_peak.motorcurrentmax * 1e1), fram_data.peak_all_time.motorcurrentmax);
			fram_data.peak_all_time.motorcurrentmin = peak_min((int16_t)(mc_peak.motorcurrentmin * 1e1), fram_data.peak_all_time.motorcurrentmin);
			fram_data.peak_all_time.motorcurrentmaxfiltered = peak_max((uint16_t)(mc_peak.motorcurrentmaxfiltered * 1e1), fram_data.peak_all_time.motorcurrentmaxfiltered);
			fram_data.peak_all_time.motorcurrentminfiltered = peak_min((int16_t)(mc_peak.motorcurrentminfiltered * 1e1), fram_data.peak_all_time.motorcurrentminfiltered);
			fram_data.peak_all_time.batterycurrentmax = peak_max((uint16_t)(mc_peak.batterycurrentmax * 1e1), fram_data.peak_all_time.batterycurrentmax);
			fram_data.peak_all_time.batterycurrentmin = peak_min((int16_t)(mc_peak.batterycurrentmin * 1e1), fram_data.peak_all_time.batterycurrentmin);
			fram_data.peak_all_time.abspeak = peak_max((uint16_t)(mc_peak.abspeak * 1e1), fram_data.peak_all_time.abspeak);
			fram_data.peak_all_time.abs_peak_filtered = peak_max((uint16_t)(mc_peak.abs_peak_filtered * 1e1), fram_data.peak_all_time.abs_peak_filtered);

			fram_data.peak_all_time.motorpeakmax = peak_max((uint16_t)(mc_peak.motorpeakmax * 1e1), fram_data.peak_all_time.motorpeakmax);
			fram_data.peak_all_time.motorpeakmin = peak_min((int16_t)(mc_peak.motorpeakmin * 1e1), fram_data.peak_all_time.motorpeakmin);
			fram_data.peak_all_time.batterypeak = peak_max((uint16_t)(mc_peak.batterypeak * 1e1), fram_data.peak_all_time.batterypeak);

			fram_data.peak_all_time.motortempmax = peak_max((int16_t)(mc_peak.motortempmax * 1e2), fram_data.peak_all_time.motortempmax);
			fram_data.peak_all_time.motortempmin = peak_min((int16_t)(mc_peak.motortempmin * 1e2), fram_data.peak_all_time.motortempmin);
			fram_data.peak_all_time.fettempmax = peak_max((int16_t)(mc_peak.fettempmax * 1e2), fram_data.peak_all_time.fettempmax);
			fram_data.peak_all_time.fettempmin = peak_min((int16_t)(mc_peak.fettempmin * 1e2), fram_data.peak_all_time.fettempmin);
			fram_data.peak_all_time.rpmmax = peak_max((uint16_t)(mc_peak.rpmmax), fram_data.peak_all_time.rpmmax);
			fram_data.peak_all_time.rpmmin = peak_min((int32_t)(mc_peak.rpmmin), fram_data.peak_all_time.rpmmin);
			fram_data.peak_all_time.speedmax = peak_max((uint16_t)(mc_peak.speedmax * 1e2), fram_data.peak_all_time.speedmax);
			fram_data.peak_all_time.speedmin = peak_min((int16_t)(mc_peak.speedmin * 1e2), fram_data.peak_all_time.speedmin);

			if (bms_pk->max_cell_volt) {
				fram_data.peak_all_time.bms_max_cell_temp = peak_max(bms_pk->max_cell_temp, fram_data.peak_all_time.bms_max_cell_temp);
				fram_data.peak_all_time.bms_min_cell_temp = peak_min(bms_pk->min_cell_temp, fram_data.peak_all_time.bms_min_cell_temp);
				fram_data.peak_all_time.bms_max_cell_volt = peak_max(bms_pk->max_cell_volt, fram_data.peak_all_time.bms_max_cell_volt);
				fram_data.peak_all_time.bms_min_cell_volt = peak_min(bms_pk->min_cell_volt, fram_data.peak_all_time.bms_min_cell_volt);
				fram_data.peak_all_time.bms_max_dis_fet_temp = peak_max(bms_pk->max_dis_fet_temp, fram_data.peak_all_time.bms_max_dis_fet_temp);

				fram_data.peak_all_time.bms_max_current_in = peak_max((int16_t)(bms_pk->max_current_in / 100), fram_data.peak_all_time.bms_max_current_in);
				fram_data.peak_all_time.bms_min_current_in = peak_min((int16_t)(bms_pk->min_current_in / 100), fram_data.peak_all_time.bms_min_current_in);
			}
		}

		memcpy((void*)&fram_data_temp, (void*)&fram_data, sizeof(fram_data_t)); 

		fram_data_temp.fram_start_address = 0;
		fram_data_temp.struct_len = sizeof(fram_data_temp) - 6; 
		fram_data_temp.crc = (uint32_t)crc16((uint8_t*)&fram_data_temp.struct_version, sizeof(fram_data_temp) - 6);
		i2c_addr = utils_MB85RC16_fram_I2CAddressAdapt(&fram_data_temp.fram_start_address);

		if ((HW_I2C_DEV.i2c->SR2 & I2C_SR2_BUSY) || (HW_I2C_DEV.i2c->CR1 & I2C_CR1_START)) {
			fram_fail_num++;
			continue;
		}
		ret = i2cMasterTransmitTimeout(&HW_I2C_DEV, i2c_addr, (uint8_t*)&fram_data_temp.fram_start_address + 1, sizeof(fram_data_temp) - 1, NULL, 0, MS2ST(10));
		if (ret != MSG_OK) {
			fram_fail_num++;
			continue;
		}
		
		chThdSleepMilliseconds(50);

		fram_data_temp.fram_start_address = 0x400;
		i2c_addr = utils_MB85RC16_fram_I2CAddressAdapt(&fram_data_temp.fram_start_address);

		if ((HW_I2C_DEV.i2c->SR2 & I2C_SR2_BUSY) || (HW_I2C_DEV.i2c->CR1 & I2C_CR1_START)) {
			fram_fail_num++;
			continue;
		}
		ret = i2cMasterTransmitTimeout(&HW_I2C_DEV, i2c_addr, (uint8_t*)&fram_data_temp.fram_start_address + 1, sizeof(fram_data_temp) - 1, NULL, 0, MS2ST(10));
		if (ret != MSG_OK) {
			fram_fail_num++;
		}
	}
}

void mc_interface_reset_fram(uint16_t reset_type) {
	if (reset_type == 0x2000) {
		memset((uint8_t*)&fram_data.struct_version, 0, sizeof(fram_data_t) - 6);
		mc_interface_set_odometer(0);
	}
}

bool mc_interface_get_fram_state(void) {
	return fram_comm_ok;
}
