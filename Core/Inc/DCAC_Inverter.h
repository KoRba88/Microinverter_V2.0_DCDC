/*
 * DCAC_Inverter.h
 *
 *  Created on: Aug 6, 2020
 *      Author: user01
 */
#include "main.h"
//#include "DQ_PhaseLockedLoop.h"

#ifndef INC_DCAC_INVERTER_H_
#define INC_DCAC_INVERTER_H_



/* Exported types ------------------------------------------------------------*/
typedef struct DCAC_TypeDef_s
{
  u32 Counter;
  u32 DeadTime;
}DCAC_TypeDef_t, *PDCAC_TypeDef_t;

typedef enum DCAC_Commands_e
{
  DCAC_Start = 0,
  DCAC_Stop
}DCAC_Commands_t;

typedef enum DCAC_Status_e
{
  DCAC_Stopped = 0,
  DCAC_Running,
  DCAC_Indefined
}DCAC_Status_t;

typedef enum DCAC_Error_e
{
  DCAC_ERROR_NONE               = 0x00,
  DCAC_ERROR_INVALID_PARAMETER,
  DCAC_ERROR_ON_INIT,
  DCAC_ERROR_INVALID_COMMAND,
  DCAC_ERROR_ON_SEND_COMMAND,
  DCAC_ERROR_ON_GET_STATUS,
  DCAC_ERROR_ON_GET_CONFIGURATION,
  DCAC_ERROR_ON_SET_DEADTIME,
}DCAC_Error_t;

/* Exported constants --------------------------------------------------------*/
#define DCAC_COUNTER        (TIMF_PERIOD)  //Zmiana x2 bo up-down mode//20600 //SAMPLINGFREQ//35156//65534//131036//4096//32767//
#define DCAC_DEADTIME       TIMF_DEADTIME  //210//2.92us

/* Exported macro ------------------------------------------------------------*/
#define DCAC_IncreaseDeadTime(step) DCAC_SetDeadTime(DCAC_GetDeadTime() + (step))
#define DCAC_DecreaseDeadTime(step) DCAC_SetDeadTime(DCAC_GetDeadTime() - (step))


/* Exported functions ------------------------------------------------------- */
DCAC_Error_t DCAC_Init(PDCAC_TypeDef_t pDCACInit);
DCAC_Error_t DCAC_SendCommand(DCAC_Commands_t cmd);
DCAC_Error_t DCAC_SetPulse(u32 PulseCurrent, u32 PulseVoltage);
DCAC_Status_t DCAC_GetStatus();
DCAC_Error_t DCAC_GetConfiguration(PDCAC_TypeDef_t pDCACInit);
DCAC_Error_t DCAC_SetDeadTime(u32 dwDeadTime);
u32 DCAC_GetDeadTime();
void DCAC_RefreshDisplay(DCAC_Error_t err, u32 steps, u32 PulseCurrent, u32 changes);

#endif /* INC_DCAC_INVERTER_H_ */
