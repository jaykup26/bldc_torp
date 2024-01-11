/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

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

#ifndef HW_TC500_H_
#define HW_TC500_H_

#define HW_NAME					"TC500-1.0"
#define HW_TC500
#define TORP_RELEASE

// HW properties
#define HW_HAS_3_SHUNTS

#define MOTOR_TEMP_LPF 						0.005

//Override default APP confg
#define APPCONF_APP_TO_USE					APP_ADC_UART
#define APPCONF_CAN_MODE					CAN_MODE_TORP

#define APPCONF_ADC_CTRL_TYPE						ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER
#define APPCONF_ADC_BRAKE_CURRENT_THROTTLE			0.0
#define APPCONF_ADC_BRAKE_CURRENT_LEVER				0.3
#define APPCONF_ADC_BRAKE_CURRENT_THROTTLE2			0.0
#define APPCONF_ADC_THROTTLE_PROTECTION_VOLT_RISE	0.6
#define APPCONF_ADC_TPC		26.59
#define APPCONF_ADC_MULTI_ESC						false

#define APPCONF_ADC_HYST					0.03
#define APPCONF_ADC_VOLTAGE_START			0.57
#define APPCONF_ADC_VOLTAGE_END				2.79
#define APPCONF_ADC_VOLTAGE_CENTER			1.68
#define APPCONF_ADC_VOLTAGE2_START			0.0
#define APPCONF_ADC_VOLTAGE2_END			0.0

#define APPCONF_IMU_TYPE					IMU_TYPE_OFF
#define APPCONF_IMU_SAMPLE_RATE_HZ          300.0
#define APPCONF_IMU_MAHONY_KP               0.0
#define APPCONF_IMU_MAHONY_KP               0.0

#define APPCONF_SURON_KICKSTAND				1
#define APPCONF_SURON_CRASH_SENSOR			1
#define APPCONF_SURON_MOD_BUTTON			SURON_MOD_BUTTON_TYPE_BUTTON
#define APPCONF_SURON_BRAKE					1
#define APPCONF_SURON_APP_KILL_SWITCH_TYPE	SURON_KILL_SWITCH_TYPE_NONE
#define APPCONF_SURON_ECO_L_CURRENT_MAX		150.0
#define APPCONF_SURON_ECO_L_IN_CURRENT_MAX	60.0
#define APPCONF_SURON_ECO_L_WATT_MAX		1500000.0
#define APPCONF_SURON_ECO_L_RPM_MAX			15128.0
#define APPCONF_SURON_ECO_ADC_CTRL_TYPE		ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER
#define APPCONF_SURON_ECO_ADC_BRAKE_CURRENT_THROTTLE	0.0
#define APPCONF_SURON_ECO_ADC_BRAKE_CURRENT_LEVER		0.3
#define APPCONF_SURON_ECO_ADC_BRAKE_CURRENT_THROTTLE2	0.0

//Override default MC confg
#define MCCONF_DEFAULT_MOTOR_TYPE			MOTOR_TYPE_FOC
#define MCCONF_M_INVERT_DIRECTION			true
#define MCCONF_L_CURRENT_MAX				250.0
#define MCCONF_L_CURRENT_MIN				-200.0
#define MCCONF_L_IN_CURRENT_MAX				90.0
#define MCCONF_L_IN_CURRENT_MIN				-60.0
#define MCCONF_L_MAX_ABS_CURRENT			700.0

#define MCCONF_L_BATTERY_CUT_START			50.5
#define MCCONF_L_BATTERY_CUT_END			48.0
#define MCCONF_L_BATTERY_CUT_REGEN_START	65.86
#define MCCONF_L_BATTERY_CUT_REGEN_END		67.2

#define MCCONF_L_LIM_TEMP_ACCEL_DEC			0.0
#define MCCONF_L_LIM_TEMP_FET_START			75.0
#define MCCONF_L_LIM_TEMP_FET_END			90.0
#define MCCONF_L_LIM_TEMP_FET_START_HW_12	65.0
#define MCCONF_L_LIM_TEMP_FET_END_HW_12		75.0
#define MCCONF_L_LIM_TEMP_MOTOR_START		99.6
#define MCCONF_L_LIM_TEMP_MOTOR_END			120.0

#define MCCONF_BMS_TYPE						BMS_TYPE_SURRON
#define MCCONF_BMS_T_LIMIT_START			56.0
#define MCCONF_BMS_T_LIMIT_END				66.0
#define MCCONF_BMS_T_DIS_LIMIT_START		70.0
#define MCCONF_BMS_T_DIS_LIMIT_END			80.0

#define MCCONF_L_MIN_VOLTAGE				43.0
#define MCCONF_L_MAX_VOLTAGE				90.0
#define MCCONF_L_MAX_DUTY					0.99
#define MCCONF_M_MOTOR_TEMP_SENS_TYPE		TEMP_SENSOR_KTY84_130

#define MCCONF_FOC_SENSOR_MODE				FOC_SENSOR_MODE_HALL
#define MCCONF_FOC_MOTOR_R					0.0037
#define MCCONF_FOC_MOTOR_L					0.00001651
#define MCCONF_FOC_MOTOR_FLUX_LINKAGE		0.013
#define MCCONF_FOC_CURRENT_KP				0.0700
#define MCCONF_FOC_CURRENT_KI				3.35
#define MCCONF_FOC_CURRENT_KP_START			0.02
#define MCCONF_FOC_CURRENT_KP_POWER_END		16000.0
#define MCCONF_FOC_OBSERVER_GAIN			4.00e6

#define MCCONF_FOC_SAT_COMP					0.3
#define MCCONF_FOC_SAT_COMP_START_ERPM		3500.0
#define MCCONF_FOC_TEMP_COMP				true
#define MCCONF_FOC_TEMP_COMP_BASE_TEMP		25.0

#define MCCONF_FOC_SL_ERPM					3500.0
#define MCCONF_FOC_HALL_TAB_0				255
#define MCCONF_FOC_HALL_TAB_1				49
#define MCCONF_FOC_HALL_TAB_2				117
#define MCCONF_FOC_HALL_TAB_3				83
#define MCCONF_FOC_HALL_TAB_4				184
#define MCCONF_FOC_HALL_TAB_5				17
#define MCCONF_FOC_HALL_TAB_6				150
#define MCCONF_FOC_HALL_TAB_7				255

#define MCCONF_FOC_F_SW						30000.0

#define MCCONF_SI_MOTOR_POLES				10
#define MCCONF_SI_GEAR_RATIO				7.6
#define MCCONF_SI_WHEEL_DIAMETER			0.63

#define MCCONF_SI_BATTERY_TYPE				BATTERY_TYPE_LIION_PANASONIC_PF
#define MCCONF_SI_BATTERY_CELLS				16

// Macros
#define DCCAL_ON()
#define DCCAL_OFF()
#define LED_RED_ON()			palSetPad(GPIOB, 1)
#define LED_RED_OFF()			palClearPad(GPIOB, 1)

#define HW_BEFORE_FW_UPDATE()		palSetPad(HW_SURON_UVLO_PORT, HW_SURON_UVLO_PIN); \
									palSetPadMode(HW_SURON_UVLO_PORT, HW_SURON_UVLO_PIN, PAL_MODE_OUTPUT_PUSHPULL);

/*
 * ADC Vector
 *
 * 0:	IN0		SENS1
 * 1:	IN1		SENS2
 * 2:	IN2		SENS3
 * 3:	IN10	CURR1
 * 4:	IN11	CURR2
 * 5:	IN12	CURR3
 * 6:	IN5		ADC_EXT1
 * 7:	IN6		ADC_EXT2
 * 8:	IN3		TEMP_PCB
 * 9:	IN14	TEMP_MOTOR
 * 10:	IN15	ADC_EXT3, Shutdown on MK3
 * 11:	IN13	AN_IN
 * 12:	Vrefint
 * 13:	IN4		THROTTLE_CURR
 * 14:	IN1		SENS2
 */

#define HW_ADC_CHANNELS			15
#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			5

// ADC Indexes
#define ADC_IND_SENS1			2
#define ADC_IND_SENS2			1
#define ADC_IND_SENS3			0
#define ADC_IND_CURR1			3
#define ADC_IND_CURR2			4
#define ADC_IND_CURR3			5
#define ADC_IND_VIN_SENS		11
#define ADC_IND_EXT				6
#define ADC_IND_EXT2			7
#define ADC_IND_TEMP_MOS2       10
//#define ADC_IND_TEMP_MOS		8
#define ADC_IND_TEMP_MOS		ADC_IND_TEMP_MOS2
#define ADC_IND_TEMP_MOTOR		9
#define ADC_IND_VREFINT			12
#define ADC_IND_VOUT_GATE_DRV	8
#define ADC_IND_EXT_CURR		13

// ADC macros and settings

// Component parameters (can be overridden)
#ifndef V_REG
#define V_REG					3.3
#endif
#ifndef VIN_R1
#define VIN_R1					100000.0
#endif
#ifndef VIN_R2
#define VIN_R2					3600.0
#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		20.0
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES		0.00010
#endif

// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

// 12V supply voltage
#define GET_GATE_DRIVER_SUPPLY_VOLTAGE()	((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VOUT_GATE_DRV] * ((10000.0 + 1800.0) / 1800.0))

// NTC Termistors
#define NTC_RES(adc_val)		((4095.0 * 10000.0) / adc_val - 10000.0)
#define NTC_TEMP(adc_ind)		(1.0 / ((logf(NTC_RES((4095 - ADC_Value[adc_ind])) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)

#define NTC_RES_MOTOR(adc_val)	((10000.0 / ((4095.0 / (float)adc_val) - 1.0)) - 1800.0) // Motor temp sensor on low side
#define NTC_TEMP_MOTOR(beta)	(1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)

// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)

// Double samples in beginning and end for positive current measurement.
// Useful when the shunt sense traces have noise that causes offset.
#ifndef CURR1_DOUBLE_SAMPLE
#define CURR1_DOUBLE_SAMPLE		0
#endif
#ifndef CURR2_DOUBLE_SAMPLE
#define CURR2_DOUBLE_SAMPLE		0
#endif
#ifndef CURR3_DOUBLE_SAMPLE
#define CURR3_DOUBLE_SAMPLE		0
#endif

// COMM-port ADC GPIOs
#define HW_ADC_EXT_GPIO			GPIOA
#define HW_ADC_EXT_PIN			5
#define HW_ADC_EXT2_GPIO		GPIOA
#define HW_ADC_EXT2_PIN			6

// UART Peripheral
#define HW_UART_DEV				SD3
#define HW_UART_GPIO_AF			GPIO_AF_USART3
#define HW_UART_TX_PORT			GPIOB
#define HW_UART_TX_PIN			10
#define HW_UART_RX_PORT			GPIOB
#define HW_UART_RX_PIN			11

#define HW_UART2_DEV			SD5
#define HW_UART2_GPIO_AF		GPIO_AF_UART5
#define HW_UART2_TX_PORT		GPIOC
#define HW_UART2_TX_PIN			12
#define HW_UART2_RX_PORT		GPIOD
#define HW_UART2_RX_PIN			2
#define HW_UART2_TXEN_PORT		GPIOA
#define HW_UART2_TXEN_PIN		15

//Torp PINS
#define HW_SURON_BRAKE_PORT		GPIOB
#define HW_SURON_BRAKE_PIN      5

#define HW_SURON_CRASH_PORT		GPIOB
#define HW_SURON_CRASH_PIN      0

#define HW_SURON_MOD_PORT		GPIOC
#define HW_SURON_MOD_PIN		9

#define HW_SURON_SPEED_PORT		GPIOA
#define HW_SURON_SPEED_PIN      7

#define HW_SURON_NOGARA_PORT    GPIOC
#define HW_SURON_NOGARA_PIN     11

#define HW_SURON_UVLO_PORT		GPIOB
#define HW_SURON_UVLO_PIN		3

#define HW_SURON_KEY_PORT		GPIOA
#define HW_SURON_KEY_PIN		13

// ICU Peripheral for servo decoding
#define HW_USE_SERVO_TIM4
#define HW_ICU_TIMER			TIM4
#define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ICU_DEV				ICUD4
#define HW_ICU_CHANNEL			ICU_CHANNEL_1
#define HW_ICU_GPIO_AF			GPIO_AF_TIM4
#define HW_ICU_GPIO				GPIOD
#define HW_ICU_PIN				2

// I2C Peripheral
#define HW_I2C_DEV				I2CD1
#define HW_I2C_GPIO_AF			GPIO_AF_I2C1
#define HW_I2C_SCL_PORT			GPIOB
#define HW_I2C_SCL_PIN			6
#define HW_I2C_SDA_PORT			GPIOB
#define HW_I2C_SDA_PIN			7

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		8
#define HW_ENC_TIM				TIM3
#define HW_ENC_TIM_AF			GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource8
#define HW_ENC_EXTI_CH			EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC		EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM3_IRQHandler

#if !defined(HW60_IS_MK3) && !defined(HW60_IS_MK4)
// NRF pins
#define NRF_PORT_CSN			GPIOB
#define NRF_PIN_CSN				12
#define NRF_PORT_SCK			GPIOB
#define NRF_PIN_SCK				4
#define NRF_PORT_MOSI			GPIOB
#define NRF_PIN_MOSI			3
#define NRF_PORT_MISO			GPIOD
#define NRF_PIN_MISO			2
#endif

// SPI pins
#define HW_SPI_DEV				SPID1
#define HW_SPI_GPIO_AF			GPIO_AF_SPI1
#define HW_SPI_PORT_NSS			GPIOA
#define HW_SPI_PIN_NSS			4
#define HW_SPI_PORT_SCK			GPIOA
#define HW_SPI_PIN_SCK			5
#define HW_SPI_PORT_MOSI		GPIOA
#define HW_SPI_PIN_MOSI			7
#define HW_SPI_PORT_MISO		GPIOA
#define HW_SPI_PIN_MISO			6

// NRF SWD
#define NRF5x_SWDIO_GPIO		GPIOC
#define NRF5x_SWDIO_PIN			10
#define NRF5x_SWCLK_GPIO		GPIOC
#define NRF5x_SWCLK_PIN			13


// Measurement macros
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// Setting limits
#define HW_LIM_CURRENT			-500.0, 800.0
#define HW_LIM_CURRENT_IN		-500.0, 500.0
#define HW_LIM_CURRENT_ABS		0.0, 850.0
#define HW_LIM_VIN				1.0, 99.0
#define HW_LIM_ERPM				-200e3, 200e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.99
#define HW_LIM_TEMP_FET			-40.0, 130.0

#define HW_GATE_DRIVER_SUPPLY_MIN_VOLTAGE	10.0
#define HW_GATE_DRIVER_SUPPLY_MAX_VOLTAGE	14.0

char* get_hw_name(void);
void get_hw_info(uint8_t* major, uint8_t* minor, uint8_t* type);

#endif /* HW_TC500_H_ */
