/*
 * DCDC_Converter.h
 *
 *  Created on: Aug 6, 2020
 *      Author: user01
 */

#ifndef INC_DCDC_CONVERTER_H_
#define INC_DCDC_CONVERTER_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "400WControl.h"
/* Exported types ------------------------------------------------------------*/
typedef struct DCDC_TypeDef_s
{
  u32 Counter;
  u32 PhaseShift;
  u32 DeadTime;
  u32 DutyCycle;
  u32 frequency;
}DCDC_TypeDef_t, *PDCDC_TypeDef_t;

typedef enum DCDC_Commands_e
{
  DCDC_ConverterStart = 0,
  DCDC_ConverterStop
}DCDC_Commands_t;

typedef enum DCDC_Status_e
{
  DCDC_Stopped = 0,
  DCDC_Running,
  DCDC_Indefined
}DCDC_Status_t;

typedef enum DCDC_Error_e
{
  DCDC_ERROR_NONE               = 0x00,
  DCDC_ERROR_INVALID_PARAMETER,
  DCDC_ERROR_ON_INIT,
  DCDC_ERROR_INVALID_COMMAND,
  DCDC_ERROR_ON_SEND_COMMAND,
  DCDC_ERROR_ON_GET_STATUS,
  DCDC_ERROR_ON_GET_CONFIGURATION,
  DCDC_ERROR_ON_SET_DEADTIME,
  DCDC_ERROR_ON_SET_PHASESHIFT,
  DCDC_ERROR_ON_SET_DUTYCYCLE,
  DCDC_ERROR_ON_SET_FREQUENCY
}DCDC_Error_t;



/* Formula below works down to 35.1kHz (with presc ratio = 1) */
//#define MASTER_PERIOD ((uint16_t)(((uint64_t)HRTIM_INPUT_CLOCK * HRTIMER_LLC_PRESCALER) / MASTER_PWM_FREQ))

//#define ISO_DCDC_PERIOD ((uint16_t)((uint64_t)TIM8_INPUT_CLOCK / ISO_DCDC_PWM_FREQ-1))
/* Exported constants --------------------------------------------------------*/
#define DCDC_COUNTER          MASTER_PERIOD //2048 //2048 //1440 50kHz
//#define DCDC_DEADTIME       190
#define DCDC_DUTYCYCLE        (DCDC_COUNTER*0.5)//512
#define DCDC_FREQUENCY        MASTER_PWM_FREQ//1024//1024 //720 50kHz

/* Exported macro ------------------------------------------------------------*/
//#define DCDC_IncreasePhaseShift(step) DCDC_SetPhaseShift(DCDC_GetPhaseShift() + (step))
//#define DCDC_DecreasePhaseShift(step) DCDC_SetPhaseShift(DCDC_GetPhaseShift() - (step))
#define DCDC_IncreaseDeadTime(step) DCDC_SetDeadTime(DCDC_GetDeadTime() + (step))
#define DCDC_DecreaseDeadTime(step) DCDC_SetDeadTime(DCDC_GetDeadTime() - (step))

#define DCDC_DecreaseDutyCycle(step) DCDC_SetDutyCycle(DCDC_GetDutyCycle() + (step))
#define DCDC_IncreaseDutyCycle(step) DCDC_SetDutyCycle(DCDC_GetDutyCycle() - (step))

#define DCDC_DecreaseFrequency(step) DCDC_SetFrequency(DCDC_GetFrequency() + (step))
#define DCDC_IncreaseFrequency(step) DCDC_SetFrequency(DCDC_GetFrequency() - (step))

/* Exported functions ------------------------------------------------------- */
DCDC_Error_t DCDC_Init(PDCDC_TypeDef_t pDCDCInit);
DCDC_Error_t DCDC_SendCommand(DCDC_Commands_t cmd);
DCDC_Status_t DCDC_GetStatus();
DCDC_Error_t DCDC_GetConfiguration(PDCDC_TypeDef_t pDCDCInit);
DCDC_Error_t DCDC_SetPhaseShift(u32 dwPhaseShift);
DCDC_Error_t DCDC_SetDeadTime(u32 dwDeadTime);
DCDC_Error_t DCDC_SetDeadTime(u32 dwDeadTime);
DCDC_Error_t DCDC_SetFrequency(u32 LLC_frequency);
u32 DCDC_GetDutyCycle();
u32 DCDC_GetFrequency();
u32 DCDC_GetPeriod();
u32 DCDC_GetPhaseShift();
void HRTIM_TIMD_DMA_START();
void HRTIM_TIMD_DMA_STOP();
void HRTIM_TIMF_DMA_START();
void DCDC_RefreshDisplay(DCDC_Error_t err, u32 steps, u32 changes);
void DCDC_TIMx_Configuration(void);

void PID_Init(PI_ControllerTYPEDEF *Direct_PID, PI_ControllerTYPEDEF *Quadrature_PID,PI_ControllerTYPEDEF *Reactive_PID,PI_ControllerTYPEDEF *Active_PID,PI_ControllerTYPEDEF *Bus_DC_PID, PI_ControllerTYPEDEF *PLL_PID,PI_ControllerTYPEDEF *MPPT_PID);
void PID_Init_Integral_Part(void);

#endif /* INC_DCDC_CONVERTER_H_ */
