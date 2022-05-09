/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_comp.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_dac.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_opamp.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "old_types.h"
#include "stdbool.h"

//#include "DQ_PhaseLockedLoop.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
typedef enum ShowData_e
{
    CONTROL_MODE,
    CONTROL_CALIB,
    DCDC_DC,
    DCDC_FRQ,
    DCDC_PERIOD,
    DCAC_DT,
    DCAC_PULSE,
    DCAC_STEPS,
    CONTROL_PARAM_1,
    CONTROL_PARAM_2,
    CONTROL_PARAM_3,
    CONTROL_PARAM_4,
    CONTROL_PARAM_5,
    CONTROL_PARAM_6,
    CONTROL_PARAM_7,
    CONTROL_PARAM_8,
    CONTROL_PARAM_9,
    CONTROL_PARAM_10,
    FAULT_TYPE,
    ABOUT
}ShowData_t;
/* USER CODE END EM */

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI4_SCK_GRID_Pin LL_GPIO_PIN_2
#define SPI4_SCK_GRID_GPIO_Port GPIOE
#define IN_TOP_LF_MOS_GPIO_Pin LL_GPIO_PIN_3
#define IN_TOP_LF_MOS_GPIO_GPIO_Port GPIOE
#define PGOOD_5V_Pin LL_GPIO_PIN_4
#define PGOOD_5V_GPIO_Port GPIOE
#define SPI4_MISO_GRID_Pin LL_GPIO_PIN_5
#define SPI4_MISO_GRID_GPIO_Port GPIOE
#define SPI4_MOSI_GRID_Pin LL_GPIO_PIN_6
#define SPI4_MOSI_GRID_GPIO_Port GPIOE
#define NTC_MOS_ADC2_IN7_COMP3_Pin LL_GPIO_PIN_1
#define NTC_MOS_ADC2_IN7_COMP3_GPIO_Port GPIOC
#define NTC_SHUNT_ADC2_IN9_Pin LL_GPIO_PIN_3
#define NTC_SHUNT_ADC2_IN9_GPIO_Port GPIOC
#define I_PV_ADC1_IN1_Pin LL_GPIO_PIN_0
#define I_PV_ADC1_IN1_GPIO_Port GPIOA
#define I_PV_OPAMP_VINP_Pin LL_GPIO_PIN_1
#define I_PV_OPAMP_VINP_GPIO_Port GPIOA
#define I_PV_OPAMP1_OUT_Pin LL_GPIO_PIN_2
#define I_PV_OPAMP1_OUT_GPIO_Port GPIOA
#define I_PV_OPAMP_VINM0_Pin LL_GPIO_PIN_3
#define I_PV_OPAMP_VINM0_GPIO_Port GPIOA
#define I_LLC_OPAMP2_OUT_Pin LL_GPIO_PIN_6
#define I_LLC_OPAMP2_OUT_GPIO_Port GPIOA
#define I_PV_COMP2_Pin LL_GPIO_PIN_7
#define I_PV_COMP2_GPIO_Port GPIOA
#define I_LLC_ADC2_IN5_Pin LL_GPIO_PIN_4
#define I_LLC_ADC2_IN5_GPIO_Port GPIOC
#define I_PV_OPAMP_VINM1_Pin LL_GPIO_PIN_5
#define I_PV_OPAMP_VINM1_GPIO_Port GPIOC
#define I_LLC_OPAMP2_COMP4_Pin LL_GPIO_PIN_0
#define I_LLC_OPAMP2_COMP4_GPIO_Port GPIOB
#define I_LLC_COMP1_Pin LL_GPIO_PIN_1
#define I_LLC_COMP1_GPIO_Port GPIOB
#define STATUS2_LED_Pin LL_GPIO_PIN_11
#define STATUS2_LED_GPIO_Port GPIOF
#define STATUS1_LED_Pin LL_GPIO_PIN_13
#define STATUS1_LED_GPIO_Port GPIOF
#define VMON_V12_Pin LL_GPIO_PIN_7
#define VMON_V12_GPIO_Port GPIOE
#define NTC5_Pin LL_GPIO_PIN_8
#define NTC5_GPIO_Port GPIOE
#define NTC4_Pin LL_GPIO_PIN_10
#define NTC4_GPIO_Port GPIOE
#define NTC3_Pin LL_GPIO_PIN_11
#define NTC3_GPIO_Port GPIOE
#define OC_OUT_DCAC_Pin LL_GPIO_PIN_15
#define OC_OUT_DCAC_GPIO_Port GPIOE
#define VPV_OPAMP4_VINM_Pin LL_GPIO_PIN_10
#define VPV_OPAMP4_VINM_GPIO_Port GPIOB
#define VPV_ADC1_IN14_Pin LL_GPIO_PIN_11
#define VPV_ADC1_IN14_GPIO_Port GPIOB
#define VPV_OPAMP4_OUT_Pin LL_GPIO_PIN_12
#define VPV_OPAMP4_OUT_GPIO_Port GPIOB
#define VPV_OPAMP4_INP_Pin LL_GPIO_PIN_13
#define VPV_OPAMP4_INP_GPIO_Port GPIOB
#define IN_TOP_LF_MOS__Pin LL_GPIO_PIN_14
#define IN_TOP_LF_MOS__GPIO_Port GPIOB
#define IN_BOTT_LF_MOS__Pin LL_GPIO_PIN_15
#define IN_BOTT_LF_MOS__GPIO_Port GPIOB
#define ESP_USART3_TX_Pin LL_GPIO_PIN_8
#define ESP_USART3_TX_GPIO_Port GPIOD
#define ESP_USART3_RX_Pin LL_GPIO_PIN_9
#define ESP_USART3_RX_GPIO_Port GPIOD
#define ESP_EN_Pin LL_GPIO_PIN_10
#define ESP_EN_GPIO_Port GPIOD
#define RELAY_SENSE_COMP6_Pin LL_GPIO_PIN_11
#define RELAY_SENSE_COMP6_GPIO_Port GPIOD
#define RELAY_SENSE_OPAMP5_Pin LL_GPIO_PIN_12
#define RELAY_SENSE_OPAMP5_GPIO_Port GPIOD
#define HVDC_FUSE_MON_Pin LL_GPIO_PIN_13
#define HVDC_FUSE_MON_GPIO_Port GPIOD
#define OC_OUT_GRID_Pin LL_GPIO_PIN_14
#define OC_OUT_GRID_GPIO_Port GPIOD
#define AC_FUSE_MON_Pin LL_GPIO_PIN_15
#define AC_FUSE_MON_GPIO_Port GPIOD
#define IN_TOP_HF_MOS__Pin LL_GPIO_PIN_6
#define IN_TOP_HF_MOS__GPIO_Port GPIOC
#define IN_BOTT_HF_MOS__Pin LL_GPIO_PIN_7
#define IN_BOTT_HF_MOS__GPIO_Port GPIOC
#define IN_BOTT_LF_MOS_GPIO_Pin LL_GPIO_PIN_1
#define IN_BOTT_LF_MOS_GPIO_GPIO_Port GPIOG
#define SD_CS_Pin LL_GPIO_PIN_8
#define SD_CS_GPIO_Port GPIOC
#define TRIG_ADC_GRID_Pin LL_GPIO_PIN_9
#define TRIG_ADC_GRID_GPIO_Port GPIOC
#define UV_TOP_MOS_Pin LL_GPIO_PIN_15
#define UV_TOP_MOS_GPIO_Port GPIOA
#define SPI3_MISO_DCAC_Pin LL_GPIO_PIN_11
#define SPI3_MISO_DCAC_GPIO_Port GPIOC
#define SPI3_MOSI_DCAC_Pin LL_GPIO_PIN_12
#define SPI3_MOSI_DCAC_GPIO_Port GPIOC
#define SPI3_SCK_DCAC_Pin LL_GPIO_PIN_9
#define SPI3_SCK_DCAC_GPIO_Port GPIOG
#define EN_isoDCDC_Pin LL_GPIO_PIN_2
#define EN_isoDCDC_GPIO_Port GPIOD
#define RELAY_GRID_LVL2_Pin LL_GPIO_PIN_3
#define RELAY_GRID_LVL2_GPIO_Port GPIOD
#define RELAY_GRID_Pin LL_GPIO_PIN_4
#define RELAY_GRID_GPIO_Port GPIOD
#define PGOOD_12V_Pin LL_GPIO_PIN_7
#define PGOOD_12V_GPIO_Port GPIOD
#define PWM_isoDCDC_Pin LL_GPIO_PIN_9
#define PWM_isoDCDC_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define CORE_CLOCK 170000000
#define CLOCK_DURATION_MICROS ((1/CORE_CLOCK)*1000000) // or 0.00588235
#define HRTIM_INPUT_CLOCK CORE_CLOCK

#define TIM8_INPUT_CLOCK CORE_CLOCK


#define ADC1_TRIG_DUTY_CYCLE 0.15
#define ADC5_TRIG_DUTY_CYCLE 0.15
#define ADC23_TRIG_DUTY_CYCLE   0.89//0.015//0.09 środek działa f=41550 //0.43//0.53 //0.825 @ 50khz, 220 lf deadtime działa w open loop, 0.19 -> closed loop z VAC dac, 0.735 stable on SPI4 interrupt, 0.97 @41khz and spi4int closed loop
#define ADC1_DATA_SIZE 2
#define ADC2_DATA_SIZE 1
#define ADC3_DATA_SIZE 1
#define ADC5_DATA_SIZE 1
#define ADC4_DATA_SIZE 5
#define SPI4_DATA_SIZE 3

#define TIMA_PWM_FREQ ( MASTER_PWM_FREQ)
#define TIMA_DUTY_CYCLE 0.50

#define TIMB_PWM_FREQ ( MASTER_PWM_FREQ)
#define TIMB_DUTY_CYCLE 0.50

// org inverter freq 17578

#define TIMF_PWM_FREQ   41504//45674//34000//41504//20753//41505//45674//41550//51200//70312//69600//72000//82400//83000//70312//83000//82804//41402

#define ADC_SAMPLING_AVERAGES 2
//60khz działa


#define TIMF_DUTY_CYCLE 0.50
#define TIMD_DUTY_CYCLE 0.50
#define TIMF_DEADTIME 300  //HF MOSFETS DEADTIME 200 = 300 nS
#define TIMD_DEADTIME 300  // 350 //HF MOSFETS DEADTIME 200 = 300 nS
#define TIM20_PERIOD 22767 //LF MOSFETS TIMER
#define TIM20_DEADTIME 135//235 LF MOSFETS DEADTIME 135 = 400 nS // 235 = 4 uS

#define MASTER_PWM_FREQ 280000
#define MASTER_DUTY_CYCLE 0.5

#define ISO_DCDC_PWM_FREQ MASTER_PWM_FREQ/2
#define ISO_DCDC_DUTY 0.05

#define HRTIMER_LLC_PRESCALER 32
#define HRTIMER_DCAC_PRESCALER 8

#define LLC_DEADTIME 180 //256

#define TIMA_PERIOD ((uint16_t)(((uint64_t)HRTIM_INPUT_CLOCK * HRTIMER_LLC_PRESCALER) / TIMA_PWM_FREQ))

/* Formula below works down to 70.3kHz (with presc ratio = 1) */
#define TIMB_PERIOD ((uint16_t)(((uint64_t)HRTIM_INPUT_CLOCK * HRTIMER_LLC_PRESCALER) / TIMB_PWM_FREQ))

#define TIMF_PERIOD (32767)//(29776/2)//32767//(29776)//((uint16_t)(((uint64_t)HRTIM_INPUT_CLOCK * HRTIMER_DCAC_PRESCALER) / (TIMF_PWM_FREQ*2))) //16364// Zmiana x2 bo up-down mode

#define TIMD_PERIOD TIMF_PERIOD

#define TIME_PERIOD TIMF_PERIOD //((uint16_t)(((uint64_t)HRTIM_INPUT_CLOCK * HRTIMER_DCAC_PRESCALER) / TIME_PWM_FREQ))

/* Formula below works down to 35.1kHz (with presc ratio = 1) */
#define MASTER_PERIOD ((uint16_t)(((uint64_t)HRTIM_INPUT_CLOCK * HRTIMER_LLC_PRESCALER) / MASTER_PWM_FREQ))

#define ISO_DCDC_PERIOD ((uint16_t)((uint64_t)TIM8_INPUT_CLOCK / ISO_DCDC_PWM_FREQ-1))

#define PHASE_SHIFT_INIT ((uint16_t)MASTER_PERIOD / 2) // 180 DEG = PERIOD / 2, - 0 DEG = PERIOD

#define DUTY_CYCLE_INIT  ((uint16_t)MASTER_PERIOD / 2)

#define VREF 2609

#define R25 50
#define R30 3480
#define R8 0.003
#define PV_CURRENT_OPAMP_GAIN ((R30/R25)*R8)

#define R10 750000
#define R11 750000
#define R33 750000
#define R12 10000
#define AMC1311_FULL_VOLTAGE 2
#define TRUE 1
#define FALSE 0

#define SD_SPI_HANDLE hspi1

#define REL_DEL_ON_TIME_uS 3720 //3720 us ON delay of ALQ312+2N7002+Shottky+24V zener at 12.5V supply
#define REL_DEL_OFF_TIME_uS 2200 //2200 us OFF delay of ALQ312+2N7002+Shottky+24V zener at 12.5V supply

#define REL_OFF_TICK            (u16)((SAMPLINGFREQ*REL_DEL_OFF_TIME_uS)/1000000)
#define REL_ON_TICK             (u16)((SAMPLINGFREQ*REL_DEL_ON_TIME_uS)/1000000)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
