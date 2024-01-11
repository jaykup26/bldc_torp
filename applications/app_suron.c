#include "app.h"
#ifdef HW_TC500
#include "ch.h"
#include "hal.h"

// Some useful includes
#include "mc_interface.h"
#include "utils.h"
#include "encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"
#include "timer.h"
#include "mempools.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

#define FILTER_SAMPLES			15
#define SURON_TIM_CLOCK			100000.0 // Hz

// Threads
static THD_FUNCTION(suron_thread, arg);
static THD_WORKING_AREA(surn_thread_wa, 256);

// Private variables
static volatile bool is_running = false;
static volatile suron_conf config;
static volatile bool force_reapply_mcconf = false;
static volatile bool fault_occurred = false;
static volatile bool warning_code = false;
static volatile bool kill_switch_pressed = false;
static volatile suron_mod_button_state_t external_app_mod_state = SURON_MOD_BUTTON_STATE_SPORT;
static volatile suron_mod_button_state_t mod_state;
static volatile uint16_t nrf_adc_val = 0;

void app_suron_configure(suron_conf* conf) {
	config = *conf;
}

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_suron_start(void) {
	if (!is_running) {
		chThdCreateStatic(surn_thread_wa, sizeof(surn_thread_wa),
			NORMALPRIO, suron_thread, NULL);
	}
	app_suron_reapply_mcconf();
}

void app_suron_apply_conf(suron_mod_button_state_t mode) {
	mc_configuration* mcconf = mempools_alloc_mcconf();
	*mcconf = *mc_interface_get_configuration();
	app_configuration* appconf = mempools_alloc_appconf();
	*appconf = *app_get_configuration();

	if (mode == SURON_MOD_BUTTON_STATE_ECO) {
		mcconf->l_current_max = config.eco_l_current_max;
		mcconf->l_in_current_max = config.eco_l_in_current_max;
		mcconf->l_watt_max = config.eco_l_watt_max;
		mcconf->l_max_erpm = config.eco_l_max_erpm;

		appconf->app_adc_conf.ctrl_type = config.eco_adc_ctrl_type;
		appconf->app_adc_conf.brake_current_throttle = config.eco_adc_brake_current_throttle;
		appconf->app_adc_conf.brake_current_throttle2 = config.eco_adc_brake_current_throttle2;
		appconf->app_adc_conf.brake_current_lever = config.eco_adc_brake_current_lever;
		appconf->app_adc_conf.tc_max_diff = config.eco_adc_tc_max_diff;
		appconf->app_adc_conf.throttle_exp = config.eco_adc_throttle_exp;
		appconf->app_adc_conf.ramp_time_pos = config.eco_adc_ramp_time_pos;

	} else if (mode == SURON_MOD_BUTTON_STATE_SPORT) {
		conf_general_read_mc_configuration(mcconf, false);
		//conf_general_read_app_configuration(appconf);

	} else if (mode == SURON_MOD_BUTTON_STATE_ROAD_LEGAL) {
		conf_general_read_mc_configuration(mcconf, false);

		mcconf->l_current_max = 300.0;
		mcconf->l_watt_max = 3500.0;

		float rpm = 12.5 * mcconf->si_gear_ratio * 60.0 / (mcconf->si_wheel_diameter * M_PI);
		mcconf->l_max_erpm = rpm * (mcconf->si_motor_poles / 2.0);
	}

	commands_apply_mcconf_hw_limits(mcconf);
	mc_interface_set_configuration(mcconf);

	app_adc_configure(&appconf->app_adc_conf, false);

	mempools_free_mcconf(mcconf);
	mempools_free_appconf(appconf);
}

void app_suron_reapply_mcconf(void) {
	force_reapply_mcconf = true;
}

void app_suron_signal_fault(void) {
	fault_occurred = true;
}

suron_mod_button_state_t app_suron_get_modbutton_state(void) {
	return mod_state;
}

void app_suron_set_mod_state(suron_mod_button_state_t mode) {
	external_app_mod_state = mode;
}

bool app_suron_get_warning(void) {
	return warning_code;
}

void app_suron_set_nrf_adc(uint16_t adc_val) {
	nrf_adc_val = adc_val;
}

uint16_t app_suron_get_nrf_adc(void) {
	return nrf_adc_val;
}

void app_suron_timer_init(void) {
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = (uint16_t)(SURON_TIM_CLOCK / 10.0) - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)((SYSTEM_CORE_CLOCK / 2) / SURON_TIM_CLOCK) - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseStructure.TIM_Period / 2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM14, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM14, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM14, ENABLE);
	//TIM_Cmd(TIM14, ENABLE);
}

static THD_FUNCTION(suron_thread, arg) {
	(void)arg;

	is_running = true;

	chRegSetThreadName("SuRon Thread");

	mc_fault_code active_fault = FAULT_CODE_NONE;
	mc_fault_code startup_fault_ignore = mc_interface_get_fault_old();
	uint32_t last_fault_timer = 0;
	uint32_t last_change_timer = 0;
	bool blink_fault = false;
	uint16_t hz = 0;
	uint16_t hz_old = 0;

	app_suron_timer_init();

	uint8_t hw_major, hw_minor, type;
	get_hw_info(&hw_major, &hw_minor, &type);
	if (type == 2 && hw_major == 1 && hw_minor >= 4) {
		uint8_t buffer[2];
		buffer[0] = COMM_EXT_NRF_GET_ADC_VAL;
		buffer[1] = 1;
		app_uartcomm_send_packet(buffer, 2);
	}
	
	for (;;) {

		if (!active_fault && (fault_occurred || mc_interface_get_fault())) {
			if (mc_interface_get_fault()) {
				active_fault = mc_interface_get_fault();
			} else {
				active_fault = mc_interface_get_fault_old();
			}
			fault_occurred = false;
			last_fault_timer = timer_time_now();

			if (startup_fault_ignore == active_fault) {
				active_fault = FAULT_CODE_NONE;
				startup_fault_ignore = FAULT_CODE_NONE;
				last_fault_timer = 0;
			}
		}

		static uint8_t state_samples = 0;
		static uint8_t mod_button_state_sum = 0;
		static uint8_t crash_sensor_state_sum = 0;
		static uint8_t kickstand_sensor_state_sum = 0;

		if (state_samples < FILTER_SAMPLES) {
			mod_button_state_sum += palReadPad(HW_SURON_MOD_PORT, HW_SURON_MOD_PIN);
			crash_sensor_state_sum += palReadPad(HW_SURON_CRASH_PORT, HW_SURON_CRASH_PIN);
			kickstand_sensor_state_sum += palReadPad(HW_SURON_NOGARA_PORT, HW_SURON_NOGARA_PIN);
			state_samples++;
		}

		if (state_samples == FILTER_SAMPLES) {
			float mod_button_state_filtered = (float)mod_button_state_sum / (float)FILTER_SAMPLES;
			float crash_sensor_state_filtered = (float)crash_sensor_state_sum / (float)FILTER_SAMPLES;
			float kickstand_sensor_state_filtered = (float)kickstand_sensor_state_sum / (float)FILTER_SAMPLES;

			suron_mod_button_state_t mod_state_now;

			if (config.mod_button == SURON_MOD_BUTTON_TYPE_BUTTON && (config.kill_switch_type == SURON_KILL_SWITCH_TYPE_MOD_BUTTON
									|| config.kill_switch_type == SURON_KILL_SWITCH_TYPE_MOD_BUTTON_BRAKE_LEVER) && mod_button_state_filtered < 0.2) {
				kill_switch_pressed = true;
			}

			if (config.mod_button == SURON_MOD_BUTTON_TYPE_BUTTON) {
				mod_state_now = mod_state;
				if (mod_button_state_filtered > 0.8) {
					mod_state_now = SURON_MOD_BUTTON_STATE_SPORT;
				}
				if (mod_button_state_filtered < 0.2) {
					mod_state_now = SURON_MOD_BUTTON_STATE_ECO;
				}
				if (external_app_mod_state == SURON_MOD_BUTTON_STATE_ROAD_LEGAL) {
					mod_state_now = external_app_mod_state;
				}
			} else if (config.mod_button == SURON_MOD_BUTTON_TYPE_EXTERNAL_APP) {
				mod_state_now = external_app_mod_state;
			} else  {
				mod_state_now = SURON_MOD_BUTTON_STATE_SPORT;
			}

			if ((mod_state != mod_state_now) || (mod_state != SURON_MOD_BUTTON_STATE_SPORT && force_reapply_mcconf)) {
				mod_state = mod_state_now;
				if (external_app_mod_state != SURON_MOD_BUTTON_STATE_ROAD_LEGAL) {
					external_app_mod_state = mod_state_now;
				}
				if (!kill_switch_pressed) {
					app_suron_apply_conf(mod_state);
				}
			}
			force_reapply_mcconf = false;

			warning_code = false;
			if (config.crash_sensor && crash_sensor_state_filtered > 0.8) {
			mc_interface_clear_warning(WARNING_CODE_1);
			mc_interface_clear_warning(WARNING_CODE_0);
			mc_interface_clear_warning(WARNING_CODE_2);
				warning_code = true;
				mc_interface_set_warning(WARNING_CODE_1);
			} 
			if (config.kickstand && kickstand_sensor_state_filtered > 0.8) {
				warning_code = true;
				mc_interface_set_warning(WARNING_CODE_0);
			} 
			if (kill_switch_pressed) {
				warning_code = true;
				mc_interface_set_warning(WARNING_CODE_2);
			}

			state_samples = 0;
			mod_button_state_sum = 0;
			crash_sensor_state_sum = 0;
			kickstand_sensor_state_sum = 0;
		}
		if (!active_fault) {
			float speed = fabs(mc_interface_get_speed());

			hz = (uint16_t)(speed * 3.6 * 5.15 * 10.0);
		}

		if (active_fault && timer_seconds_elapsed_since(last_fault_timer) < 16.2) {
			if (timer_seconds_elapsed_since(last_fault_timer) < 1.0) {
				hz = 0;
			}
			else if (timer_seconds_elapsed_since(last_change_timer) > 3.0) {
				last_change_timer = timer_time_now();
				if (!blink_fault) {
					uint8_t fault_num = mc_interface_fault_to_number(active_fault);
					if (fault_num < 49) {
						hz = (uint16_t)((float)fault_num * 5.20 * 10.0);
					} else if (fault_num < 73) {
						hz = (uint16_t)((float)fault_num * 5.15 * 10.0);
					} else {
						hz = (uint16_t)((float)fault_num * 5.12 * 10.0);
					}
					blink_fault = true;
				} else {
					hz = 0;
					blink_fault = false;
				}
			}
		} else if (active_fault) {
			active_fault = FAULT_CODE_NONE;
			last_fault_timer = 0;
			last_change_timer = 0;
			blink_fault = false;
		}

		if (hz > 0 && hz < 10000) {
			if (hz != hz_old || active_fault) {

				uint32_t period = (uint16_t)(SURON_TIM_CLOCK / ((float)hz / 10.0)) - 1;

				TIM14->ARR = period;
				TIM14->CCR1 = period / 2;
				if (!hz_old) {
					TIM_Cmd(TIM14, ENABLE);
				}
			}
		}
		else {
			if (hz_old) {
				TIM_Cmd(TIM14, DISABLE);
			}
		}
		hz_old = hz;
		
		chThdSleepMilliseconds(5);
	}
}
#endif