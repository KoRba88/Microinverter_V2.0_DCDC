/*
 * 400WControl.h
 *
 *  Created on: Aug 6, 2020
 *      Author: user01
 */
#include "main.h"
#ifndef INC_400WCONTROL_H_
#define INC_400WCONTROL_H_

/* Exported types ------------------------------------------------------------*/
typedef struct
{
  s16 qI_Quadrature;
  s16 qI_Direct;
} Curr_Components;

typedef struct
{
  s16 qV_Quadrature;
  s16 qV_Direct;
} Volt_Components;

typedef struct
{
  s16 P_Active;
  s16 Q_Reactive;
}
Power_Components;

typedef struct
{
  s16 qValpha;
  s16 qVbeta;
}
Volt_AlphaBeta_Components;


typedef struct
{
  s16 Reference;
  s16 Kp_Gain;
  s16 Ki_Gain;

  s16 Lower_Limit_Output;     //Lower Limit for Output limitation
  s16 Upper_Limit_Output;     //Lower Limit for Output limitation
  bool Max_PID_Output;
  bool Min_PID_Output;

} PI_ControllerTYPEDEF;

typedef enum
{
    STOP,
    STOPPING,
    LOM,
    BUS_FAULT,
    PV_VOLTAGE_DVDT,
    DIAGNOSTIC_AC_LINE,
    FREQ_OUT_OF_RANGE,
    FREQ_INSIDE_RANGE,
    GRID_VOLTAGE_OUT_OF_RANGE,
    GRID_VOLTAGE_INSIDE_RANGE,
    VDCIN_INSIDE_RANGE,
    VDCIN_OUT_OF_RANGE,
    DIAGNOSTIC_DC_LINE,
    OUT_CURRENT_LIMIT,
    BUSPRECHARGE,
    PV_VOLTAGE_MIN,
    BUS_OVERVOLTAGE,
    BUS_UNDERVOLTAGE,
    START,
    RUN,
    GRID_PRE_INSERTION,
    GRID_INSERTION,
    STOP_WITH_DELAY,
    WAIT_BUS_NORMAL_RANGE,
} SystStatus_t;


typedef struct Param_s
{
  s16 *pIntegral;		//(1)
  s16 *pProportional;		//(1)
} Param_t, *PParam_t;

typedef struct
{
  u16 *xz1;		//(1)
  u16 *xz2;		//(1)
  u16 *xz3;		//(1)
} Param_t_zero;

typedef struct ControlParam_s
{
    Param_t PLL;			//(1)
//    Param_t Mppt;			//(1)
    Param_t DCBUS;			//(1)
    Param_t Id;				//(1)
    Param_t Iq;				//(1)
    Param_t Q;				//(1)
    Param_t_zero k1k2;		        //(1)

}ControlParam_t, * PControlParam_t;

typedef struct Power_s
{
  s16 *pPin;             //(1)
  s16 *pPout_act;        //(1)
  s16 *pOutFreq;         //(1)
  s16 *pQ;               //(1)
  s16 *pPowerThreshold;  //(1)
} Power_t, * PPower_t;

typedef struct VoltageCurrent_s
{
  s16 *pVin_DC;        //(1)
  s16 *pIin_DC;        //(1)
  s16 *pVout_AC;       //(1)
  s16 *pIout_AC;       //(1)
  s16 *pVDC_Bus;       //(1)
  s16 *pVDC_BusRef;    //(1)
} VoltageCurrent_t, *PVoltageCurrent_t;

typedef struct Status_s
{
  s16 *pFault;
  s16 *pState;
} Status_t, *PStatus_t;
typedef enum ControlParamName_e
{
  PLL_KI,
  PLL_KP,
//  MPPT_KI,
//  MPPT_KP,
  DCBUS_KI,
  DCBUS_KP,
  ID_KI,
  ID_KP,
  IQ_KI,
  IQ_KP,
  Q_KI,
  Q_KP
}ControlParamName_t;

typedef enum
{
  InitCalib,
  OpenLoop,
  ClosedLoop
} ControlMode_t;

typedef struct Photov_s
{
  Power_t Power;
  VoltageCurrent_t VoltageCurrent;
  Status_t Status;
}Photov_t, *PPhotov_t;

/* Exported constants --------------------------------------------------------*/
#define DATA_SENSING_SIZE 3

#define CONTROL_ERROR_NONE    0
#define CONTROL_ERROR_RUNNING 1

//#define FAULT_NONE                     0x00000000
//#define FAULT_SHORT_CIRCUIT            0x00000001
//#define FAULT_SHORT_OVERVOLTAGE        0x00000002
//#define FAULT_SHORT_OVERCURRENT        0x00000004
//#define FAULT_SHORT_GRIDOUTAGE         0x00000008

#define FAULT_NONE                     0x00000000
//#define INPUT_OVERCURRENT              0x00000001
//#define BUS_OVERVOLTAGE                0x00000002
//#define BUS_UNDERVOLTAGE               0x00000004
//#define OUTPUT_OVERCURRENT             0x00000008
//#define GRIDOUTAGE                     0x00000016


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void InitControl(ControlMode_t mode);
SystStatus_t GetStatusControl(void);
void StartControl(void);
void StopControl(void);
s16 GetAmplitude(void);
void SetAmplitude(s16 amp);
u8 SetControlMode(ControlMode_t mode);
u8 CalibrationControl(void);

s16 GetControlParam(ControlParamName_t ParamName);
void SetControlParam(ControlParamName_t ParamName, s16 value);

void GetControlParametersAddress(PControlParam_t pControl);
void GetDataParametersAddress(PPhotov_t pData);
PPhotov_t GetDataLinkStruct();

void RefreshDisplayControl(u32 err, u32 display);
void RefreshDisplayControlParam(u32 err, u32 display);

void __attribute__( ( optimize( "O0" ) ) ) delay_cycles( uint32_t cyc );

#endif /* INC_400WCONTROL_H_ */
