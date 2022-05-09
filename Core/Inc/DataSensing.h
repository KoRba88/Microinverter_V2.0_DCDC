/*
 * DataSensing.h
 *
 *  Created on: Aug 6, 2020
 *      Author: user01
 */
#include "main.h"
#ifndef INC_DATASENSING_H_
#define INC_DATASENSING_H_

/* Exported types ------------------------------------------------------------*/
typedef void( *PFN_ON_ACQUISTION )( void );

typedef struct DS_TypeDef_s
{
    u32 Counter;
    volatile uint16_t *DataRegister;
    u32 RegisterSize;
    PFN_ON_ACQUISTION OnAcquisition;
}DS_TypeDef_t, *PDS_TypeDef_t;


typedef struct DS_Dataregisters_s
{



}DS_Dataregisters_t, *PDS_Dataregisters_t;

typedef enum DS_Commands_e
{
  DS_Start = 0,
  DS_Stop
}DS_Commands_t;

typedef struct DS_Data_s
{
    s16 AC_LineCurrent;
    s16 AC_LineVoltage;
    s16 DC_BusVoltage;
    s16 DC_PanelCurrent;
    s16 DC_PanelVoltage;
    s16 NTC5;
    s16 NTC1;
    s16 NTC4;
    s16 NTC3;
    s16 NTC2;
    s16 Reserved;

}DS_Data_t, *PDS_Data_t;


typedef struct DS_Data_su16
{
    u16 AC_LineCurrent;
    u16 AC_LineVoltage;
    u16 DC_BusVoltage;
    u16 DC_PanelCurrent;
    u16 DC_PanelVoltage;
    u16 NTC5;
    u16 NTC1;
    u16 NTC4;
    u16 NTC3;
    u16 NTC2;
    u16 Reserved;
}DS_Data_tu16, *PDS_Data_tu16;

typedef struct DS_Data_s_sum
{
    u32 AC_LineCurrent;
    u32 AC_LineVoltage;
    u32 DC_BusVoltage;
    u32 DC_PanelCurrent;
    u32 DC_PanelVoltage;
    u32 NTC5;
    u32 NTC1;
    u32 NTC4;
    u32 NTC3;
    u32 NTC2;
    u32 Reserved;
}DS_Data_t_sum, *PDS_Data_t_sum;

typedef enum DS_Status_e
{
  DS_Stopped = 0,
  DS_Running,
  DS_Indefined
}DS_Status_t;

typedef enum DS_Error_e
{
  DS_ERROR_NONE               = 0x00,
  DS_ERROR_INVALID_PARAMETER,
  DS_ERROR_ON_INIT,
  DS_ERROR_INVALID_COMMAND,
  DS_ERROR_ON_SEND_COMMAND,
  DS_ERROR_ON_GET_STATUS,
  DS_ERROR_ON_GET_CONFIGURATION,
}DS_Error_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
DS_Error_t DS_Init(PDS_TypeDef_t pDSInit);
DS_Error_t DS_SendCommand(DS_Commands_t cmd);
DS_Status_t DS_GetStatus();
PDS_Data_t DS_GetData();
DS_Error_t DS_GetConfiguration(PDS_TypeDef_t pDSInit);
void DS_RefreshDisplay(DS_Error_t err);
void DS_SetAcquistionEvent(PFN_ON_ACQUISTION pfFn);

#endif /* INC_DATASENSING_H_ */
