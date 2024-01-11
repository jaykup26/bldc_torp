#include "app.h"
#ifdef HW_TC500
#include "ch.h"
#include "hal.h"
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
#include "datatypes.h"
#include "bms.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

#define HW_UART2_PACKET_MAX_PL_LEN	128
#define HW_UART2_BUFFER_LEN			(HW_UART2_PACKET_MAX_PL_LEN + 5)

// Threads
static THD_FUNCTION(surron_bms_thread, arg);
static THD_WORKING_AREA(surron_bms_thread_wa, 512);

static THD_FUNCTION(surron_bms_packet_process_thread, arg);
static THD_WORKING_AREA(surron_bms_packet_process_thread_wa, 2048);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile bool uart_is_running = false;
static volatile bool restarted_now = false;

static volatile bool has_surron_battery = false;
static volatile bool has_surron_old_display = false;
static volatile systime_t voltage_update_time = 0;
static volatile systime_t soc_update_time = 0;
static volatile systime_t war_err_update_time = 0;
static volatile systime_t voltage_req_update_time = 0;
static volatile systime_t soc_req_update_time = 0;

static volatile COMM_SURRON_BMS_PACKET_ID reqested_packet_id = 0;
static volatile COMM_SURRON_BMS_PACKET_ID received_packet_id = 0;
static volatile uint16_t bms_rx_timeout;
static volatile uint32_t crc_fail_num = 0;

static volatile float calc_soc;
static volatile float calc_soc_min_cell_volt;
static volatile systime_t update_time_cell_v = 0;
static volatile systime_t temp_update_time = 0;
static volatile systime_t other_update_time = 0;

typedef struct {
	volatile unsigned short rx_timeout;
	unsigned int rx_read_ptr;
	unsigned int rx_write_ptr;
	int bytes_left;
	unsigned char rx_buffer[HW_UART2_BUFFER_LEN];
	unsigned char tx_buffer[HW_UART2_BUFFER_LEN];
} PACKET_STATE_t;

typedef enum {
	BMS_PACKET_TYPE_SEND_TO_BMS = 0,
	BMS_PACKET_TYPE_RESP_FROM_BMS,
	BMS_PACKET_TYPE_REQ_FROM_OLD_DISPLAY,
	BMS_PACKET_TYPE_RESP_TO_OLD_DISPLAY,
	BMS_PACKET_TYPE_SEND_TO_DISPLAY,
	BMS_PACKET_TYPE_REQ_FROM_EXTERNAL_APP
} COMM_SURRON_BMS_PACKET_TYPE;

typedef struct {
	void(*send_func)(unsigned char* data, unsigned int len);
	uint8_t tx_buffer[40];
	uint8_t tx_len;
	bool has_app_request; 
	bool has_request;	
	systime_t address_req_update_time;
} READ_ADDRESS_STATE_t;

static SerialConfig uart_cfg5 = {
		9600,
		0,
		USART_CR2_LINEN,
		0
};

static PACKET_STATE_t m_handler_states[1]; 
static READ_ADDRESS_STATE_t bms_read_address_states[1]; 

static int app_surron_bms_try_decode_packet(unsigned char* buffer, unsigned int in_len, int* bytes_left);

void app_surron_bms_send_command(COMM_SURRON_BMS_PACKET_ID packet_id);
void app_surron_bms_process_byte(uint8_t rx_data);
void app_surron_bms_send_to_display(void);
void app_surron_bms_send_to_display_letter(void);

void app_surron_bms_start(void) {
	stop_now = false;
	if (!is_running) {
		chThdCreateStatic(surron_bms_thread_wa, sizeof(surron_bms_thread_wa),
			NORMALPRIO, surron_bms_thread, NULL);

		chThdCreateStatic(surron_bms_packet_process_thread_wa, sizeof(surron_bms_packet_process_thread_wa),
			NORMALPRIO, surron_bms_packet_process_thread, NULL);

		sdStart(&HW_UART2_DEV, &uart_cfg5);
		palSetPadMode(HW_UART2_TX_PORT, HW_UART2_TX_PIN, PAL_MODE_ALTERNATE(HW_UART2_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);
		palSetPadMode(HW_UART2_RX_PORT, HW_UART2_RX_PIN, PAL_MODE_ALTERNATE(HW_UART2_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);
		palSetPadMode(HW_UART2_TXEN_PORT, HW_UART2_TXEN_PIN, PAL_MODE_OUTPUT_PUSHPULL);
		palClearPad(HW_UART2_TXEN_PORT, HW_UART2_TXEN_PIN);

		uart_is_running = true;
	} else {
		restarted_now = true;
	}
}

void app_surron_bms_stop(void) {
	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

void send_packet(unsigned char* data, unsigned int len) {
	if (uart_is_running) {
		palSetPad(HW_UART2_TXEN_PORT, HW_UART2_TXEN_PIN);
		sdWrite(&HW_UART2_DEV, data, len);
	}
}

void app_surron_bms_packet_send_packet(unsigned char* data, unsigned int len, COMM_SURRON_BMS_PACKET_TYPE packet_type) {
	if (len == 0 || len > HW_UART2_PACKET_MAX_PL_LEN) {
		return;
	}
	bms_rx_timeout = 100; 

	int b_ind = 0;

	PACKET_STATE_t* handler = &m_handler_states[0];

	switch (packet_type) {
		case BMS_PACKET_TYPE_RESP_TO_OLD_DISPLAY: {
			handler->tx_buffer[b_ind++] = 0x47;
			handler->tx_buffer[b_ind++] = 0x16;
			handler->tx_buffer[b_ind++] = 0x01;
			handler->tx_buffer[b_ind++] = data[0];
			handler->tx_buffer[b_ind++] = len - 1;
		} break;
		case BMS_PACKET_TYPE_SEND_TO_DISPLAY: {
			handler->tx_buffer[b_ind++] = 0x57;
			handler->tx_buffer[b_ind++] = 0x83;
			handler->tx_buffer[b_ind++] = 0x01;
			handler->tx_buffer[b_ind++] = data[0];
			handler->tx_buffer[b_ind++] = len; 
		} break;
		case BMS_PACKET_TYPE_SEND_TO_BMS: {
			handler->tx_buffer[b_ind++] = 0x46;
			handler->tx_buffer[b_ind++] = 0x16;
			handler->tx_buffer[b_ind++] = 0x01;
			handler->tx_buffer[b_ind++] = data[0];
		} break;
		case BMS_PACKET_TYPE_REQ_FROM_EXTERNAL_APP: {
			handler->tx_buffer[b_ind++] = 0x46;
			handler->tx_buffer[b_ind++] = 0x16;
			handler->tx_buffer[b_ind++] = data[0];
		} break;
		default:
			break;
	}

	memcpy(handler->tx_buffer + b_ind, data + 1, len - 1);
	b_ind += len - 1;

	uint8_t chk8 = 0; 

	for (int i = 0; i < b_ind; i++) {
		chk8 += handler->tx_buffer[i];
	}
	chk8 %= 256;

	handler->tx_buffer[b_ind++] = chk8;

	send_packet(handler->tx_buffer, b_ind);
}

static THD_FUNCTION(surron_bms_thread, arg) {
	(void)arg;

	is_running = true;
	bool sent_fixed_data_req = false;
	bool timer_comm_send_startup = false;

	bool soc_req_rtc = false;
	bool war_err_req_cap = false;

	chRegSetThreadName("SurRon BMS");

	const volatile mc_configuration* mc_conf = mc_interface_get_configuration();
	volatile bms_values* bms_val = bms_get_values();
	volatile bms_info* bms_inf = bms_get_info();

	for (;;) {
		if (stop_now) {
			is_running = false;
			return;
		}

		static volatile systime_t soc_update_last = 0;
		if (chVTTimeElapsedSinceX(soc_update_last) > S2ST(1)) {
			float lowest_cell;

			if (update_time_cell_v && UTILS_AGE_S(update_time_cell_v) < 2.0) {
				lowest_cell = calc_soc_min_cell_volt;
			} else {
				lowest_cell = GET_INPUT_VOLTAGE() / (float)(mc_conf->si_battery_cells);
			}
			calc_soc = mc_interface_get_battery_level(0, lowest_cell, restarted_now) * 10.0;
			restarted_now = false;
			soc_update_last = chVTGetSystemTimeX();
		}

		if (!has_surron_battery && !has_surron_old_display) { 
			app_surron_bms_send_to_display();
			chThdSleepMilliseconds(100);
			app_surron_bms_send_command(COMM_SURRON_BMS_GET_VOLTAGE);
			chThdSleepSeconds(1);
			continue;
		}

		bool from_voltage_req = has_surron_old_display && voltage_update_time && ST2MS(chVTTimeElapsedSinceX(voltage_update_time)) > 2;
		bool from_soc_req = has_surron_old_display && soc_update_time && ST2MS(chVTTimeElapsedSinceX(soc_update_time)) > 2;
		bool from_war_err_req = has_surron_old_display && war_err_update_time && ST2MS(chVTTimeElapsedSinceX(war_err_update_time)) > 2;

		if (has_surron_battery && !sent_fixed_data_req && (!has_surron_old_display || from_voltage_req)) {
			if (!reqested_packet_id && received_packet_id < COMM_SURRON_BMS_GET_SOH) {  
				chThdSleepMilliseconds(30);
				app_surron_bms_send_command(COMM_SURRON_BMS_GET_SOH);
			}
			if (!reqested_packet_id && received_packet_id < COMM_SURRON_BMS_GET_FULL_CAP) {
				chThdSleepMilliseconds(30);
				app_surron_bms_send_command(COMM_SURRON_BMS_GET_FULL_CAP);
			}
			if (!reqested_packet_id && received_packet_id < COMM_SURRON_BMS_GET_CYCLES) {
				chThdSleepMilliseconds(30);
				app_surron_bms_send_command(COMM_SURRON_BMS_GET_CYCLES);
			}
			if (!reqested_packet_id && received_packet_id < COMM_SURRON_BMS_GET_SOFTWARE_VER) {
				chThdSleepMilliseconds(30);
				app_surron_bms_send_command(COMM_SURRON_BMS_GET_SOFTWARE_VER);
			}
			if (!reqested_packet_id && received_packet_id < COMM_SURRON_BMS_GET_MANU_DATE) {
				chThdSleepMilliseconds(30);
				app_surron_bms_send_command(COMM_SURRON_BMS_GET_MANU_DATE);
			}
			if (!reqested_packet_id && received_packet_id < COMM_SURRON_BMS_GET_SN) {
				chThdSleepMilliseconds(30);
				app_surron_bms_send_command(COMM_SURRON_BMS_GET_SN);
			}
			if (!reqested_packet_id && received_packet_id < COMM_SURRON_BMS_GET_MIN_MAX) {
				chThdSleepMilliseconds(30);
				app_surron_bms_send_command(COMM_SURRON_BMS_GET_MIN_MAX);
			}
			if (!reqested_packet_id && received_packet_id == COMM_SURRON_BMS_GET_MIN_MAX) {
				received_packet_id = 0; 
				sent_fixed_data_req = true;
				timer_comm_send_startup = true;
				voltage_update_time = 0; 
			}
		}
		if (has_surron_battery && sent_fixed_data_req) {
			if (!bms_read_address_states->has_app_request && (!bms_read_address_states->address_req_update_time || ST2S(chVTTimeElapsedSinceX(bms_read_address_states->address_req_update_time)) > 5)) {
				if (!reqested_packet_id && !has_surron_old_display) {
					bool do_other_update = timer_comm_send_startup || (other_update_time && ST2S(chVTTimeElapsedSinceX(other_update_time)) > 60); 
					bool do_temp_update = timer_comm_send_startup || (temp_update_time && ST2S(chVTTimeElapsedSinceX(temp_update_time)) > 30);
					if (crc_fail_num < 500) {
						if (!reqested_packet_id && received_packet_id < COMM_SURRON_BMS_GET_TEMP) {
							chThdSleepMilliseconds(30);
							app_surron_bms_send_to_display(); 
							if (bms_inf->battery_type != 1) { 
								chThdSleepMilliseconds(25);
								app_surron_bms_send_to_display_letter();
								chThdSleepMilliseconds(25);
								app_surron_bms_send_to_display_letter();
							}
							if (do_temp_update) {
								chThdSleepMilliseconds(40);
								app_surron_bms_send_command(COMM_SURRON_BMS_GET_TEMP);
								temp_update_time = 0;
							}
						}
						if (!reqested_packet_id && received_packet_id < COMM_SURRON_BMS_GET_CURRENT) {
							bms_inf->battery_type == 1 ? chThdSleepMilliseconds(40) : chThdSleepMilliseconds(10); 
							app_surron_bms_send_command(COMM_SURRON_BMS_GET_CURRENT);
						}
						if (!reqested_packet_id && received_packet_id < COMM_SURRON_BMS_GET_SOC) {
							bms_inf->battery_type == 1 ? chThdSleepMilliseconds(40) : chThdSleepMilliseconds(10);
							app_surron_bms_send_command(COMM_SURRON_BMS_GET_SOC);
						}
						if (!reqested_packet_id && received_packet_id < COMM_SURRON_BMS_GET_REMAIN_CAP && do_other_update) {
							bms_inf->battery_type == 1 ? chThdSleepMilliseconds(40) : chThdSleepMilliseconds(10);
							app_surron_bms_send_command(COMM_SURRON_BMS_GET_REMAIN_CAP);
						}
						if (!reqested_packet_id && received_packet_id < COMM_SURRON_BMS_GET_WAR_ERR) {
							bms_inf->battery_type == 1 ? chThdSleepMilliseconds(40) : chThdSleepMilliseconds(10);
							app_surron_bms_send_command(COMM_SURRON_BMS_GET_WAR_ERR);
						}
						if (!reqested_packet_id && received_packet_id < COMM_SURRON_BMS_GET_RTC && do_other_update) {
							if (bms_inf->battery_type == 1) { chThdSleepMilliseconds(40); other_update_time = 0; } else { chThdSleepMilliseconds(10); };
							app_surron_bms_send_command(COMM_SURRON_BMS_GET_RTC);
						}
						if (!reqested_packet_id && received_packet_id < COMM_SURRON_BMS_GET_CELL_VOLTAGE) {
							bms_inf->battery_type == 1 ? chThdSleepMilliseconds(40) : chThdSleepMilliseconds(10);
							app_surron_bms_send_command(COMM_SURRON_BMS_GET_CELL_VOLTAGE);
						}
						if (bms_inf->battery_type != 1 && !reqested_packet_id && received_packet_id < COMM_SURRON_BMS_GET_ERROR_COUNTER && do_other_update) { 
							chThdSleepMilliseconds(10);
							app_surron_bms_send_command(COMM_SURRON_BMS_GET_ERROR_COUNTER);
							other_update_time = 0;
						}
						if (!reqested_packet_id && (received_packet_id == COMM_SURRON_BMS_GET_CELL_VOLTAGE || received_packet_id == COMM_SURRON_BMS_GET_ERROR_COUNTER)) {
							received_packet_id = 0;
							timer_comm_send_startup = false;
							if (!temp_update_time) {
								temp_update_time = chVTGetSystemTimeX();
							}
							if (!other_update_time) {
								other_update_time = chVTGetSystemTimeX();
							}
						}
					} else { 
						if (!reqested_packet_id && received_packet_id < COMM_SURRON_BMS_GET_TEMP) {
							chThdSleepMilliseconds(30);
							app_surron_bms_send_to_display();
							if (bms_inf->battery_type != 1) {
									chThdSleepMilliseconds(25);
									app_surron_bms_send_to_display_letter();
									chThdSleepMilliseconds(25);
									app_surron_bms_send_to_display_letter();
							}
							if (do_temp_update) {
								chThdSleepMilliseconds(40);
								app_surron_bms_send_command(COMM_SURRON_BMS_GET_TEMP);
								temp_update_time = 0;
							}
						}
						if (bms_inf->battery_type == 1 && !reqested_packet_id && received_packet_id < COMM_SURRON_BMS_GET_CURRENT) {
							chThdSleepMilliseconds(40);
							app_surron_bms_send_command(COMM_SURRON_BMS_GET_CURRENT);
						}
						if (!reqested_packet_id && received_packet_id < COMM_SURRON_BMS_GET_SOC) {
							chThdSleepMilliseconds(40);
							app_surron_bms_send_command(COMM_SURRON_BMS_GET_SOC);
						}
						if (!reqested_packet_id && received_packet_id < COMM_SURRON_BMS_GET_WAR_ERR) {
							chThdSleepMilliseconds(40);
							app_surron_bms_send_command(COMM_SURRON_BMS_GET_WAR_ERR);
						}
						if (!reqested_packet_id && received_packet_id >= COMM_SURRON_BMS_GET_WAR_ERR) {
							received_packet_id = 0;
							timer_comm_send_startup = false;
							other_update_time = 0;
							if (!temp_update_time) {
								temp_update_time = chVTGetSystemTimeX();
							}
						}
					}
				}

				if (has_surron_old_display) {
					if (from_voltage_req) {
						if (!reqested_packet_id && received_packet_id < COMM_SURRON_BMS_GET_CELL_VOLTAGE) {
							app_surron_bms_send_command(COMM_SURRON_BMS_GET_CELL_VOLTAGE);
							voltage_update_time = 0;
						}
					}

					if (from_soc_req) {
						if (!soc_req_rtc && !reqested_packet_id && received_packet_id < COMM_SURRON_BMS_GET_TEMP) {
							app_surron_bms_send_command(COMM_SURRON_BMS_GET_TEMP);
						}

						if (!soc_req_rtc && !reqested_packet_id && received_packet_id < COMM_SURRON_BMS_GET_CURRENT) {
							app_surron_bms_send_command(COMM_SURRON_BMS_GET_CURRENT);
							soc_update_time = 0;
							soc_req_rtc = true;
						}
						
						if (soc_req_rtc && !reqested_packet_id && received_packet_id < COMM_SURRON_BMS_GET_RTC) {
							app_surron_bms_send_command(COMM_SURRON_BMS_GET_RTC);
							soc_update_time = 0;
							soc_req_rtc = false;
						}
					}

					if (from_war_err_req) {
						if (!war_err_req_cap && !reqested_packet_id && received_packet_id < COMM_SURRON_BMS_GET_ERROR_COUNTER) {
							app_surron_bms_send_command(COMM_SURRON_BMS_GET_ERROR_COUNTER);
							war_err_update_time = 0;
							war_err_req_cap = true;
						}
						
						if (war_err_req_cap && !reqested_packet_id && received_packet_id < COMM_SURRON_BMS_GET_REMAIN_CAP) {
							app_surron_bms_send_command(COMM_SURRON_BMS_GET_REMAIN_CAP);
							war_err_update_time = 0;
							war_err_req_cap = false;
						}
					}
				}
			}
			else if (bms_read_address_states->has_app_request && !reqested_packet_id && (!has_surron_old_display || from_voltage_req || from_soc_req)) {
				bms_read_address_states->has_request = true;
				app_surron_bms_packet_send_packet(bms_read_address_states->tx_buffer, bms_read_address_states->tx_len, BMS_PACKET_TYPE_REQ_FROM_EXTERNAL_APP);
				bms_read_address_states->has_app_request = false;

				voltage_update_time = 0;
				soc_update_time = 0;
			}
		}

		
		if (has_surron_old_display && !has_surron_battery && voltage_req_update_time && ST2MS(chVTTimeElapsedSinceX(voltage_req_update_time)) > 4) {
			int32_t ind = 0;
			uint8_t packet[5];

			uint32_t volt = (uint32_t)(GET_INPUT_VOLTAGE() * 1000.0);
			packet[ind++] = COMM_SURRON_BMS_GET_VOLTAGE;
			packet[ind++] = volt;
			packet[ind++] = volt >> 8;
			packet[ind++] = volt >> 16;
			packet[ind++] = volt >> 24;
			app_surron_bms_packet_send_packet(packet, ind, BMS_PACKET_TYPE_RESP_TO_OLD_DISPLAY);
			voltage_req_update_time = 0;

		}
		if (has_surron_old_display && !has_surron_battery && soc_req_update_time && ST2MS(chVTTimeElapsedSinceX(soc_req_update_time)) > 4) {
			int32_t ind = 0;
			uint8_t packet[5];
			
			uint16_t soc = (uint16_t)calc_soc;
			packet[ind++] = COMM_SURRON_BMS_GET_SOC;
			packet[ind++] = soc;
			packet[ind++] = soc >> 8;
			packet[ind++] = 0x00;
			packet[ind++] = 0x00;
			app_surron_bms_packet_send_packet(packet, ind, BMS_PACKET_TYPE_RESP_TO_OLD_DISPLAY);
			soc_req_update_time = 0;
		}

		
		if (m_handler_states[0].rx_timeout) {
			m_handler_states[0].rx_timeout--;
		}
		else {
			m_handler_states[0].rx_read_ptr = 0;
			m_handler_states[0].rx_write_ptr = 0;
			m_handler_states[0].bytes_left = 0;
		}

		
		if (bms_rx_timeout) {
			bms_rx_timeout--;
		}
		else if (reqested_packet_id || bms_read_address_states->has_request) { 
			if (!has_surron_old_display && !bms_read_address_states->has_request && reqested_packet_id) {
				received_packet_id = reqested_packet_id; 
			} else {
				received_packet_id = 0;
			}
			reqested_packet_id = 0;
			bms_read_address_states->has_request = false;
			if (has_surron_battery) {
				bms_val->timeout_num++;
			}
		}

		bms_val->crc_fail_num = (uint16_t)crc_fail_num;

		chThdSleepMilliseconds(2);
	}
}

static THD_FUNCTION(surron_bms_packet_process_thread, arg) {
	(void)arg;

	chRegSetThreadName("SurRon BMS rx");

	event_listener_t el5;
	chEvtRegisterMaskWithFlags(&HW_UART2_DEV.event, &el5, EVENT_MASK(0), CHN_INPUT_AVAILABLE | CHN_TRANSMISSION_END);

	for (;;) {
		chEvtWaitAnyTimeout(ALL_EVENTS, TIME_INFINITE);

		eventflags_t el5_flags = el5.el_flags;
		if (el5_flags & CHN_TRANSMISSION_END) { 
			palClearPad(HW_UART2_TXEN_PORT, HW_UART2_TXEN_PIN);
			chEvtGetAndClearFlags(&el5);
		}

		if (!(el5_flags & CHN_INPUT_AVAILABLE)) {
			continue;
		}

		bool rx = true;
		while (rx && uart_is_running) {
			rx = false;

			msg_t res = sdGetTimeout(&HW_UART2_DEV, TIME_IMMEDIATE);
			if (res != MSG_TIMEOUT) {
				app_surron_bms_process_byte(res);
				rx = true;
			}
		}
	}
}

uint16_t app_surron_bms_buffer_get_uint16(const uint8_t* buffer, int32_t* index) {
	uint16_t res =	((uint16_t)buffer[*index + 1]) << 8 |
					((uint16_t)buffer[*index]);
	*index += 2;
	return res;
}

int16_t app_surron_bms_buffer_get_int16(const uint8_t* buffer, int32_t* index) {
	int16_t res =	((uint16_t)buffer[*index + 1]) << 8 |
					((uint16_t)buffer[*index]);
	*index += 2;
	return res;
}

int32_t app_surron_bms_buffer_get_int32(const uint8_t* buffer, int32_t* index) {
	int32_t res =	((uint32_t)buffer[*index + 3]) << 24 |
					((uint32_t)buffer[*index + 2]) << 16 |
					((uint32_t)buffer[*index + 1]) << 8 |
					((uint32_t)buffer[*index]);
	*index += 4;
	return res;
}

float app_surron_bms_buffer_get_float16(const uint8_t* buffer, float scale, int32_t* index) {
	return (float)app_surron_bms_buffer_get_int16(buffer, index) / scale;
}

float app_surron_bms_buffer_get_float32(const uint8_t* buffer, float scale, int32_t* index) {
	return (float)app_surron_bms_buffer_get_int32(buffer, index) / scale;
}

void app_surron_bms_commands_process_packet(unsigned char* data, unsigned int len, COMM_SURRON_BMS_PACKET_TYPE packet_type,
	void(*reply_func)(unsigned char* data, unsigned int len)) {

	if (!len) {
		return;
	}
	
	COMM_SURRON_BMS_PACKET_ID packet_id;

	packet_id = data[0];
	bool for_ext_app_resp = bms_read_address_states->has_request; 
	bms_read_address_states->has_request = false;

	if (packet_type == BMS_PACKET_TYPE_RESP_FROM_BMS && !for_ext_app_resp) {
		data += 2;
		len -= 2;

		bms_values* bms_val = bms_get_values();
		bms_info* bms_inf = bms_get_info();
		bms_peak* bms_pk = bms_get_peak(false);

		if (reqested_packet_id == packet_id) {
			reqested_packet_id = 0;
			received_packet_id = packet_id;
		}
		else {
			received_packet_id = 0;
			reqested_packet_id = 0;
			voltage_update_time = 0;
			soc_update_time = 0;
			war_err_update_time = 0;
		}

		has_surron_battery = true;

		bms_val->update_time = chVTGetSystemTimeX();

		switch (packet_id) {
		case COMM_SURRON_BMS_GET_TEMP: {
			int32_t ind = 0;
			int8_t cell_temp_1 = data[ind++];
			int8_t cell_temp_2 = data[ind++];
			int8_t cell_temp_3 = data[ind++];
			int8_t ic_temp = data[ind++];
			int8_t dis_fet_temp = data[ind++];
			int8_t max_cell_temp;
			int8_t min_cell_temp;

			bms_val->temps_adc[0] = (float)cell_temp_1;
			bms_val->temps_adc[1] = (float)cell_temp_2;
			bms_val->temps_adc[2] = (float)cell_temp_3;
			bms_val->temp_ic = (float)ic_temp;
			bms_val->temps_adc[3] = (float)dis_fet_temp;
			bms_val->temps_adc[4] = (float)(int8_t)data[ind++];
			bms_val->temps_adc[5] = (float)(int8_t)data[ind++];

			bms_val->temp_adc_num = 6;

			
			if (cell_temp_2 >= cell_temp_3) {
				max_cell_temp = cell_temp_2;
			} else {
				max_cell_temp = cell_temp_3;
			}

			if (cell_temp_2 <= cell_temp_3) {
				min_cell_temp = cell_temp_2;
			} else {
				min_cell_temp = cell_temp_3;
			}

			if (max_cell_temp > bms_pk->max_cell_temp) {
				bms_pk->max_cell_temp = max_cell_temp;
			}

			if (dis_fet_temp > bms_pk->max_dis_fet_temp) {
				bms_pk->max_dis_fet_temp = dis_fet_temp;
			}

			if (min_cell_temp < bms_pk->min_cell_temp || !bms_pk->min_cell_temp) {
				bms_pk->min_cell_temp = min_cell_temp;
			}

		} break;

		case COMM_SURRON_BMS_GET_VOLTAGE: {
			int32_t ind = 0;

			bms_val->v_tot = app_surron_bms_buffer_get_float32(data, 1e3, &ind);
			
			voltage_update_time = chVTGetSystemTime();
		} break;

		case COMM_SURRON_BMS_GET_CURRENT: {
			int32_t ind = 0;

			int32_t i_in = app_surron_bms_buffer_get_int32(data, &ind);
			bms_val->i_in = (float)i_in / 1e3;
			i_in *= -1; 
			
			if (i_in > 0 && (i_in > bms_pk->max_current_in)) {
				bms_pk->max_current_in = i_in;
			}
			if (i_in < bms_pk->min_current_in || !bms_pk->min_current_in) {
				bms_pk->min_current_in = i_in;
			}
		} break;

		case COMM_SURRON_BMS_GET_SOC: {
			int32_t ind = 0;

			bms_val->soc = app_surron_bms_buffer_get_float16(data, 1e2, &ind);

			soc_update_time = chVTGetSystemTime();
		} break;

		case COMM_SURRON_BMS_GET_SOH: {
			int32_t ind = 0;

			bms_val->soh = app_surron_bms_buffer_get_float16(data, 1e2, &ind);
		} break;

		case COMM_SURRON_BMS_GET_REMAIN_CAP: {
			int32_t ind = 0;

			bms_inf->remain_cap = app_surron_bms_buffer_get_uint16(data, &ind);
		} break;

		case COMM_SURRON_BMS_GET_FULL_CAP: {
			int32_t ind = 0;

			bms_inf->full_cap = app_surron_bms_buffer_get_uint16(data, &ind);
		} break;

		case COMM_SURRON_BMS_GET_WAR_ERR: {
			int32_t ind = 2;

			bool was_primary_overcurrent_error = (bms_val->live_error >> 11) & 1;
			bool was_secondary_overcurrent_error = (bms_val->live_error >> 12) & 1;

			bms_val->live_error = (uint32_t)app_surron_bms_buffer_get_int32(data, &ind);
			bms_val->live_warning = (uint32_t)app_surron_bms_buffer_get_int32(data, &ind);

			if (!was_primary_overcurrent_error && ((bms_val->live_error >> 11) & 1)) {
				fram_data.error_cnt[40]++; 
			}
			if (!was_secondary_overcurrent_error && ((bms_val->live_error >> 12) & 1)) {
				fram_data.error_cnt[41]++;
			}

			ind = 12;
			uint16_t bal_state_bits = app_surron_bms_buffer_get_uint16(data, &ind);

			for (size_t i = 0; i < 16; i++)
			{
				bms_val->bal_state[i] = ((bal_state_bits >> i) & 1);
			}
			bms_val->cell_num = 16;

			war_err_update_time = chVTGetSystemTime();
		} break;

		case COMM_SURRON_BMS_GET_CYCLES: {
			int32_t ind = 0;

			bms_inf->cycle_count = app_surron_bms_buffer_get_uint16(data, &ind);
		} break;

		case COMM_SURRON_BMS_GET_SOFTWARE_VER: {
			int32_t ind = 0;

			bms_inf->software[1] = data[ind++];
			bms_inf->software[0] = data[ind++];
			bms_inf->hardware[1] = data[ind++];
			bms_inf->hardware[0] = data[ind++];

			if (bms_inf->software[1] == 100 && bms_inf->hardware[1] == 100) {  
				bms_inf->battery_type = 1;
			}
		} break;

		case COMM_SURRON_BMS_GET_MANU_DATE: {
			int32_t ind = 0;

			bms_inf->manu_date[0] = data[ind++];
			bms_inf->manu_date[1] = data[ind++];
			bms_inf->manu_date[2] = data[ind++];
		} break;

		case COMM_SURRON_BMS_GET_RTC: {
			int32_t ind = 0;

			memcpy(bms_inf->rtc, data + ind, 6);
		} break;

		case COMM_SURRON_BMS_GET_SN: {
			int32_t ind = 0;

			memcpy(bms_inf->sn, data + ind, 32);
		} break;

		case COMM_SURRON_BMS_GET_CELL_VOLTAGE: {
			int32_t ind = 0;

			calc_soc_min_cell_volt = 5.0;
			for (size_t i = 0; i < 16; i++)
			{
				uint16_t v_cell_i = app_surron_bms_buffer_get_uint16(data, &ind);
				bms_val->v_cell[i] = (float)v_cell_i / 1000.0;

				if (v_cell_i > bms_pk->max_cell_volt) {
					bms_pk->max_cell_volt = v_cell_i;
				}
				if (v_cell_i < bms_pk->min_cell_volt || !bms_pk->min_cell_volt) {
					bms_pk->min_cell_volt = v_cell_i;
				}

				if (bms_val->v_cell[i] < calc_soc_min_cell_volt) {
					calc_soc_min_cell_volt = bms_val->v_cell[i];
				}
			}
			bms_val->cell_num = 16;
			update_time_cell_v = chVTGetSystemTimeX();
		} break;

		case COMM_SURRON_BMS_GET_MIN_MAX: {
			int32_t ind = 0;

			bms_inf->max_dis_current = app_surron_bms_buffer_get_int32(data, &ind);
			bms_inf->max_ch_current = app_surron_bms_buffer_get_int32(data, &ind);
			bms_inf->max_cell_volt = app_surron_bms_buffer_get_uint16(data, &ind);
			bms_inf->min_cell_volt = app_surron_bms_buffer_get_uint16(data, &ind);
			bms_inf->max_temp = data[ind++];
			bms_inf->min_temp = data[ind++];
		} break;

		case COMM_SURRON_BMS_GET_ERROR_COUNTER: {
			int32_t ind = 0;

			memcpy(bms_inf->error_counter, data + ind, 64);
		} break;

		default:
			break;
		}
	} else if (packet_type == BMS_PACKET_TYPE_REQ_FROM_OLD_DISPLAY) {
		switch (packet_id) {
		case COMM_SURRON_BMS_GET_VOLTAGE: { 

			has_surron_old_display = true;

			voltage_req_update_time = chVTGetSystemTime();
		} break;

		case COMM_SURRON_BMS_GET_SOC: {
			soc_req_update_time = chVTGetSystemTime();
		} break;

		case COMM_SURRON_BMS_GET_WAR_ERR: {
		} break;

		default:
			break;
		}
	} else if (packet_type == BMS_PACKET_TYPE_REQ_FROM_EXTERNAL_APP) {
		if (!bms_read_address_states->has_app_request) {
			memcpy(bms_read_address_states->tx_buffer, data, len);
			bms_read_address_states->tx_len = len;
			bms_read_address_states->send_func = reply_func;
			bms_read_address_states->has_app_request = true;

			bms_read_address_states->address_req_update_time = chVTGetSystemTime();
		}
	} else if (packet_type == BMS_PACKET_TYPE_RESP_FROM_BMS && for_ext_app_resp) {
		if (bms_read_address_states->send_func) {
			int32_t ind = 0;
			uint8_t send_buffer[40];

			send_buffer[ind++] = COMM_BMS_GET_READ_ADDRESS;
			memcpy(send_buffer + ind, data - 1, len + 1); 
			ind += len + 1;
			bms_read_address_states->send_func(send_buffer, ind);
		}
	}
}

void app_surron_bms_process_byte(uint8_t rx_data) {
	PACKET_STATE_t* handler = &m_handler_states[0];

	handler->rx_timeout = 1000;

	unsigned int data_len = handler->rx_write_ptr - handler->rx_read_ptr;

	if (data_len >= HW_UART2_BUFFER_LEN) {
		handler->rx_write_ptr = 0;
		handler->rx_read_ptr = 0;
		handler->bytes_left = 0;
		handler->rx_buffer[handler->rx_write_ptr++] = rx_data;
		return;
	}

	if (handler->rx_write_ptr >= HW_UART2_BUFFER_LEN) {
		memmove(handler->rx_buffer,
			handler->rx_buffer + handler->rx_read_ptr,
			data_len);

		handler->rx_read_ptr = 0;
		handler->rx_write_ptr = data_len;
	}

	handler->rx_buffer[handler->rx_write_ptr++] = rx_data;
	data_len++;

	if (handler->bytes_left > 1) {
		handler->bytes_left--;
		return;
	}

	for (;;) {
		int res = app_surron_bms_try_decode_packet(handler->rx_buffer + handler->rx_read_ptr,
			data_len, &handler->bytes_left);

		
		if (res == -2) {
			break;
		}

		if (res > 0) {
			data_len -= res;
			handler->rx_read_ptr += res;
		}
		else if (res == -1) {
			
			handler->rx_read_ptr++;
			data_len--;
		}
	}

	if (data_len == 0) {
		handler->rx_read_ptr = 0;
		handler->rx_write_ptr = 0;
	}
}

static int app_surron_bms_try_decode_packet(unsigned char* buffer, unsigned int in_len, int* bytes_left) {
	*bytes_left = 0;

	if (in_len == 0) {
		*bytes_left = 1;
		return -2;
	}

	bool header0 = buffer[0] == 0x47;
	bool header0_req = buffer[0] == 0x46;

	if (!header0 && !header0_req) {
		return -1;
	}

	if (in_len == 1) {
		*bytes_left = 1;
		return -2;
	}
	bool header1 = buffer[1] == 0x16;
	if (!header1) {
		return -1;
	}

	unsigned int data_start = 5;

	if (in_len < data_start) {
		*bytes_left = data_start - in_len;
		return -2;
	}

	unsigned int len = 0;

	len = (unsigned int)buffer[4];

	if (len > HW_UART2_PACKET_MAX_PL_LEN) {
		return -1;
	}

	if (header0_req) {
		len = 0;
	}

	if (in_len < (len + data_start + 1)) {
		*bytes_left = (len + data_start + 1) - in_len;
		return -2;
	}

	uint8_t chk8_calc = 0;

	for (uint8_t i = 0; i < in_len - 1; i++) {
		chk8_calc += buffer[i];
	}
	chk8_calc %= 256;

	uint8_t chk8_rx = buffer[in_len - 1];

	if (chk8_calc == chk8_rx) {
		if (header0) {
			app_surron_bms_commands_process_packet(buffer + 3, len + 2, BMS_PACKET_TYPE_RESP_FROM_BMS, 0);
		}
		if (header0_req) {
			app_surron_bms_commands_process_packet(buffer + 3, (unsigned int)buffer[4], BMS_PACKET_TYPE_REQ_FROM_OLD_DISPLAY, 0);
		}

		return in_len;
	}
	else {
		crc_fail_num++;
		return -1;
	}
}

void app_surron_bms_send_command(COMM_SURRON_BMS_PACKET_ID packet_id) {
	switch (packet_id) {
		case COMM_SURRON_BMS_GET_TEMP: {
			uint8_t packet[2] = { packet_id, 0x8 };
			app_surron_bms_packet_send_packet(packet, 2, BMS_PACKET_TYPE_SEND_TO_BMS);
			reqested_packet_id = packet_id;
		} break;

		case COMM_SURRON_BMS_GET_VOLTAGE: {
			uint8_t packet[2] = { packet_id, 0x4 };
			app_surron_bms_packet_send_packet(packet, 2, BMS_PACKET_TYPE_SEND_TO_BMS);
			reqested_packet_id = packet_id;
		} break;

		case COMM_SURRON_BMS_GET_CURRENT: {
			uint8_t packet[2] = { packet_id, 0x4 };
			app_surron_bms_packet_send_packet(packet, 2, BMS_PACKET_TYPE_SEND_TO_BMS);
			reqested_packet_id = packet_id;
		} break;

		case COMM_SURRON_BMS_GET_SOC: {
			uint8_t packet[2] = { packet_id, 0x4 };
			app_surron_bms_packet_send_packet(packet, 2, BMS_PACKET_TYPE_SEND_TO_BMS);
			reqested_packet_id = packet_id;
		} break;

		case COMM_SURRON_BMS_GET_SOH: {
			uint8_t packet[2] = { packet_id, 0x04 };
			app_surron_bms_packet_send_packet(packet, 2, BMS_PACKET_TYPE_SEND_TO_BMS);
			reqested_packet_id = packet_id;
		} break;

		case COMM_SURRON_BMS_GET_REMAIN_CAP: {
			uint8_t packet[2] = { packet_id, 0x4 };
			app_surron_bms_packet_send_packet(packet, 2, BMS_PACKET_TYPE_SEND_TO_BMS);
			reqested_packet_id = packet_id;
		} break;

		case COMM_SURRON_BMS_GET_FULL_CAP: {
			uint8_t packet[2] = { packet_id, 0x04 };
			app_surron_bms_packet_send_packet(packet, 2, BMS_PACKET_TYPE_SEND_TO_BMS);
			reqested_packet_id = packet_id;
		} break;

		case COMM_SURRON_BMS_GET_WAR_ERR: {
			uint8_t packet[2] = { packet_id, 0x10 };
			app_surron_bms_packet_send_packet(packet, 2, BMS_PACKET_TYPE_SEND_TO_BMS);
			reqested_packet_id = packet_id;
		} break;

		case COMM_SURRON_BMS_GET_CYCLES: {
			uint8_t packet[2] = { packet_id, 0x04 };
			app_surron_bms_packet_send_packet(packet, 2, BMS_PACKET_TYPE_SEND_TO_BMS);
			reqested_packet_id = packet_id;
		} break;

		case COMM_SURRON_BMS_GET_SOFTWARE_VER: {
			uint8_t packet[2] = { packet_id, 0x08 };
			app_surron_bms_packet_send_packet(packet, 2, BMS_PACKET_TYPE_SEND_TO_BMS);
			reqested_packet_id = packet_id;
		} break;

		case COMM_SURRON_BMS_GET_MANU_DATE: {
			uint8_t packet[2] = { packet_id, 0x04 };
			app_surron_bms_packet_send_packet(packet, 2, BMS_PACKET_TYPE_SEND_TO_BMS);
			reqested_packet_id = packet_id;
		} break;

		case COMM_SURRON_BMS_GET_RTC: {
			uint8_t packet[2] = { packet_id, 0x6 };
			app_surron_bms_packet_send_packet(packet, 2, BMS_PACKET_TYPE_SEND_TO_BMS);
			reqested_packet_id = packet_id;
		} break;

		case COMM_SURRON_BMS_GET_SN: {
			uint8_t packet[2] = { packet_id, 0x20 };
			app_surron_bms_packet_send_packet(packet, 2, BMS_PACKET_TYPE_SEND_TO_BMS);
			reqested_packet_id = packet_id;
		} break;

		case COMM_SURRON_BMS_GET_CELL_VOLTAGE: {
			uint8_t packet[2] = { packet_id, 0x20 };
			app_surron_bms_packet_send_packet(packet, 2, BMS_PACKET_TYPE_SEND_TO_BMS);
			reqested_packet_id = packet_id;
		} break;

		case COMM_SURRON_BMS_GET_MIN_MAX: {
			uint8_t packet[2] = { packet_id, 0x0E };
			app_surron_bms_packet_send_packet(packet, 2, BMS_PACKET_TYPE_SEND_TO_BMS);
			reqested_packet_id = packet_id;
		} break;

		case COMM_SURRON_BMS_GET_ERROR_COUNTER: {
			uint8_t packet[2] = { packet_id, 0x40 };
			app_surron_bms_packet_send_packet(packet, 2, BMS_PACKET_TYPE_SEND_TO_BMS);
			reqested_packet_id = packet_id;
		} break;

		default:
		break;
	}
}

void app_surron_bms_send_to_display() {
	const volatile suron_conf* sur_conf = &app_get_configuration()->app_suron_conf;
	const volatile bms_values* bms_val = bms_get_values();

	int32_t ind = 0;
	uint8_t packet[12];
	memset(packet, 0, sizeof(packet));

	uint32_t volt = (uint32_t)(GET_INPUT_VOLTAGE() * 1000.0);
	uint8_t soc;

	if (sur_conf->app_battery_type == SURON_BATTERY_TYPE_STOCK && bms_val->update_time && UTILS_AGE_S(bms_val->update_time) < 2.0) {
		soc = (uint8_t)(bms_val->soc * 100.0);
	} else {
		soc = (uint8_t)calc_soc;
	}

	packet[ind++] = 0x48;
	packet[ind++] = soc;
	packet[ind++] = volt;
	packet[ind++] = volt >> 8;
	packet[ind++] = volt >> 16;
	packet[ind++] = volt >> 24;
	ind += 6; 

	app_surron_bms_packet_send_packet(packet, ind, BMS_PACKET_TYPE_SEND_TO_DISPLAY);
}

void app_surron_bms_send_to_display_letter() { 
	int32_t ind = 0;
	uint8_t packet[2];

	packet[ind++] = 0x4B;
	packet[ind++] = 0x00;

	app_surron_bms_packet_send_packet(packet, ind, BMS_PACKET_TYPE_SEND_TO_DISPLAY);
}
#endif