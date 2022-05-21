/*
 * 400WControl.c
 *
 *  Created on: Aug 6, 2020
 *      Author: user01
 */
//test
#include "DCAC_Inverter.h"
#include "DCDC_Converter.h"
#include "DataSensing.h"
#include "400WControl.h"
#include "hw_config.h"
#include "DQ_PhaseLockedLoop.h"
#include "PI_Regulator.h"
#include "PLL_Regulator.h"
//#include "lcd.h"
//#include "color_lcd.h"
#include "stdio.h"
#include "DAC_Debug.h"
#include "Solar_Mul_Div.h"
#include "Solar_MPPT.h"
#include "DQ_PhaseLockedLoop.h"
#include "main.h"
#include "app_fatfs.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//#define DC_BUS_THRESHOLD           11700      //  depending on hardware(DC BUS Sensor)
#define GRID_FREQ 50
#define pi 3.14159

#define _MAX_VOLTAGE_SCALING   0x7FFF
#define _MAX_VOLTAGE_REF       886      //850    new value scaled to 1120 380V = 28088 code ->443 at bus uint16_t
#define _MAX_VOLTAGE_REF_PV    56      //850    new value scaled to 1120
#define _VOLTAGE_REF(x)        ((s16)((_MAX_VOLTAGE_SCALING/_MAX_VOLTAGE_REF)*(x)))
#define _PV_VOLTAGE_REF(x)     ((s16)((_MAX_VOLTAGE_SCALING/_MAX_VOLTAGE_REF_PV)*(x)))

#define _WAIT_SECONDS(x)       ((u32)SAMPLINGFREQ*(x)) //SAMPLINGFREQ

#define GRID_VOLTAGE_TRESHOLD             60
#define GRID_WAIT               ((u32)150000)   //8.5s
#define PRECHARGE_WAIT               ((u32)150000)   //8.5s
#define GRID_SENS               ((u32)10000)   //8.5s
#define MAX_AMPLITUDE                S16_MAX
#define MAX_DATA_INIT_COUNT            (SAMPLINGFREQ * 6)    // 5 sec

#define DC_BUS_WAIT             _WAIT_SECONDS(5)
#define PV_MIN_WAIT             _WAIT_SECONDS(1)
#define RELAY_DELAY_TIME         (SAMPLINGFREQ / 100) //10msec
#define DC_PANEL_VOLTAGE_MIN    _PV_VOLTAGE_REF(20)
#define DC_PANEL_VOLTAGE_MAX    _PV_VOLTAGE_REF(50)
#define ONE_TICK_uS            ((1/SAMPLINGFREQ)*1000000)

#define REL_OFF_WAIT           ((_WAIT_SECONDS(0.000001)*REL_DEL_OFF_TIME_uS))



#define DC_BUS_VOLTAGE_MIN      ((s16)(BUS_Voltage_PID.Reference * 0.98))  //-5%  //-2%
#define DC_BUS_VOLTAGE_MAX      ((s16)(BUS_Voltage_PID.Reference * 1.02))  //+5%  //+2%

/* Private macro -------------------------------------------------------------*/
#define U16_TO_DOUBLE_3_3(x)   (((double)((u16)x)*3.3)/U16_MAX)
#define LSELLINE(line, dis)    (((((line) - Line3) / 24 == dis / 2)) ? 6 : 0)
#define MAX_POWER_THRESHOLD 32768 //(3000 * 3450) << 15

#define FREQUENCY_IN_HERTZ  50

#define _MAX_OMEGA_SCALING 0x7FFF
#define _MAX_OMEGA_VALUE   5278

#define OMEGA_IN_S16    ((s16)((_MAX_OMEGA_SCALING/_MAX_OMEGA_VALUE)*FREQUENCY_IN_HERTZ*6.283185))
#define MAX_OMEGA   ((s16)(OMEGA_IN_S16 * 1.006))
#define MIN_OMEGA   ((s16)(OMEGA_IN_S16 * 0.994))
#define CHECK_OMEGA(x) ((x) < MAX_OMEGA && (x) > MAX_OMEGA)

#define TIME_OUT_STOPPING  166000   //166000 about 10s

#define HALF_DUT_DCAC (TIMF_PERIOD * 0.5)

#define delta_theta (SAMPLINGFREQ / (2*pi) / GRID_FREQ)

/* Private variables ---------------------------------------------------------*/
//static u16 ii=0;

//Power_Components buffer[2048]={0};
//Curr_Components buffer2[2048]={0};
//static u16 oii=0;

extern volatile u32 TIMD_DMA_Buffer[1];

extern SPI_HandleTypeDef hspi4;
extern UART_HandleTypeDef huart3;

uint16_t buffer_set = 0;
uint16_t buffer[6][4095]={0};
//uint16_t buffer2[4095]={0};
//uint16_t ii=0;
uint8_t update_i=0;
s16 PV_Voltage=0;
s16 PV_Voltage_prev=0;
s16 PV_Current=0;

u16 PV_Voltage_startup=1200;

u8 attention=0;
//extern u8 flag;
u16 enable_inverter=0;

extern s16 Theta;

extern u16 Theta_time;

extern const s16 Sin_Cos_Table[];
extern s16 qVbeta;
extern s16 qValpha;
s16 Grid_Volt_sum=0;
s16 Grid_Volt_temp=0;
s16 Grid_Volt_avg=0;
s16 Grid_Voltage=0;
s16 Grid_Voltage_prec=0;
s16 Grid_Voltage_filtered=0;

s16 Voltage_Inverter=0;
s16 qIalpha_Inverter=0;
s16 qIalpha_Inverter_filtered=0;

s16 qIalpha_Inverter_max=0;
s16 qIalpha_Inverter_min=0;
u16 comp=0;

s16 qIbeta_Inverter=0;
s16 Bus_Voltage=0;
s16 Bus_Voltage_sum=0;
s16 Bus_Voltage_min=0;
s16 Bus_Voltage_max=0;
s16 Vref=0;
s16 PowerThreshold = MAX_POWER_THRESHOLD;

s16 LOCAL_AVG_BUS_DC=0;
extern s16 AVG_Current_DC;
extern ShowData_t StateShow;

s16 Output_qVd_Grid=0;
s16 DCDC_PhaseShiftValue = 0;
extern s16 Output_qId_Inverter;
extern s16 Output_qIq_Inverter;
extern s16 Theta;
extern u16 Theta_Grid;

SystStatus_t State_Control;
SystStatus_t Diagnostic_Control;

extern volatile uint16_t to_grid_State_Control[SPI4_DATA_SIZE];
uint16_t *ptr_State_Control;
uint16_t *ptr_org_State_Control;

extern volatile uint32_t Pulse1_IT;
extern volatile uint32_t Pulse2_IT;

uint32_t *ptr_Pulse1_IT;
bool *ptr_Pulse2_IT;

uint16_t OC_PROT_ON = 0;

bool Fault_FLAG=FALSE;

bool BusOverVoltage = FALSE;
bool GridOutage = FALSE;

u32 Wait_GRID_Insertion=0;
u32 Wait_PreCharge=0;
u16 Wait_MPPT=0;
u32 Wait_startup=0;
u32 Wait_BUS_DC_Reading=0;
u32 WAIT_PV_IN_STABLE=0;
u32 WAIT_RELAY_STABLE=0;
u32 StoppingCount;

extern Volt_Components Grid_Volt_q_d;  // Vq & Vd, voltages on a reference
                                       // frame synchronous with the Grid_Voltage*/
extern Curr_Components   Inverter_q_d;

Power_Components  Actual_QD_Power;
Volt_AlphaBeta_Components Control_Volt_AlphaBeta;
Volt_AlphaBeta_Components Control_Volt_Beta_AntiTrasf;

PI_ControllerTYPEDEF MPPT_PID;
PI_ControllerTYPEDEF PLL_PID;
PI_ControllerTYPEDEF DQ_PLL_PID;
PI_ControllerTYPEDEF Quadrature_Current_PID;
PI_ControllerTYPEDEF Direct_Current_PID;
PI_ControllerTYPEDEF Reactive_Power_PID;
PI_ControllerTYPEDEF Active_Power_PID;
PI_ControllerTYPEDEF BUS_Voltage_PID;

static volatile ControlParam_t CtrlParam;
static volatile Photov_t Data;


static bool olstart=TRUE;

s16 Amplitude = 24000;
s16 Amplitude2 = 24582;

bool calib=FALSE;
bool RELAY_ON=FALSE;

//u16 z1=16384;//15712;//15744;
//Ac Current offset
u16 z1=766;//15712;//15744;  //DC current compensation (Iac)
u16 z2=49152;//48752;

u8 aqusitionDMA1_1 = 0;
u8 aqusitionDMA1_5 = 0;

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void ExecControl(void);
void ExecControlOpenLoop(void);
void ExecControlInitParamOffSet(void);
void SetBusDCReference(s16 DCVoltage);
void CalcAndSetACComponents(SystStatus_t state);

void CallRefreshDisplay();

extern void MPPT_func();
extern void delay_us(uint32_t delay_us);
/*Acquisition Buffer*/
//volatile DS_Data_t DataSensing;
volatile DS_Data_tu16 DataSensingIO;
//volatile DS_Data_t DataSensingOffSet;
volatile DS_Data_tu16 DataSensingOffSet;
volatile DS_Data_t_sum DataSensing_sum;

volatile u32 DataInitCount = 0;

static ControlMode_t ControlMode;

static s16 State = 0;
static s16 Fault = FAULT_NONE;

//static s16 qValpha_shifted=0;
static u16 qValpha_shifted_pos=0;
static u32 Pulse1=0;
static u32 Pulse2=0;
//static u16 testing=0;

//static float Pulse1f;
//static float Pulse2f;

bool polarity=FALSE;
bool LF_MOS_SET=TRUE;
bool FIRST_CYCLE=TRUE;
bool RESET_TIMER_D=TRUE;
bool LF_UPDATE=FALSE;
//u16 CCR_Val=563;//DUTY CYCLE 50% modified 24/02
u16 CCR_Val=520;//DUTY CYCLE 50%
u32 ActualFreq_Val = MAX_FREQ;
u8 mppt_req=0;
u16 count_mppt=0;
u16 waiting_time=0;
bool MPPT_EN=FALSE;
bool PLL_par_red_kp=FALSE;
bool PLL_par_red_ki=FALSE;

//Param_t_zero k1k2val;
ControlParam_t k1k2val;


// anti islanding variables
float Kappa=0.00005;
s16 ThetaPert=0;
bool ZCFlag=TRUE;
u8 IndSin=0;
u8 IndCos=0;
s16 SinThetaPert;
s16 CosThetaPert;

s32 Iq_tmp_1=0;
s32 Iq_tmp_2=0;
s16 Iq_1=0;
s16 Iq_2=0;
s32 Id_tmp_1=0;
s32 Id_tmp_2=0;
s16 Id_1=0;
s16 Id_2=0;

u8 MPPT_num=0;
bool Vacprot=FALSE;

bool PLL_reducing=FALSE;

extern SystStatus_t Freq_Control;
extern SystStatus_t GDVoltage;

s16 Grid_Voltage_min=0;
s16 Grid_Voltage_max=0;

u16 bus_pre_charge=0;
u16 timeperiod_positive=0;
u16 timeperiod_negative=0;

extern u32 DC_comp_Power;
extern u32 DC_comp_Power_prev;

#define MODINDEX   (TIMF_PERIOD-300) //18525//18500//33700 //2000   //indice di mod max = 95%
#define MODINDEX50 1000

extern s16 Cos_Theta;
extern s16 Sin_Theta;
u16 Index_Sin_max,Index_Sin_min=0;
u16 waiting_output_current_limit=0;

#define new_mul_q15_q15_q31(Op1, Op2) (((s32)((s32)Op1*(s32)Op2))>=0x40000000 ? 0x7fffffff :(((s32)((s32)Op1*(s32)Op2))<<1))

//testing data sensing values

extern float PV_VOLTAGE;
extern float PV_CURRENT;
extern float HV_DCDC_SE;
extern float HV_DCDC_SE_OFFSET;

extern float V_AC_GRID_SE;
extern float V_AC_GRID_SE_OFFSET;
extern float I_AC_GRID_SE;
extern float I_AC_GRID_SE_OFFSET;

extern float HV_DCDC_VOLT;
extern float AC_GRID_VOLT;
extern float TEMP_INT_uC;
extern float NTC1_V;


extern float PV_POWER;

//u16 PhaseShift = PHASE_SHIFT_INIT;
//u16 PhaseShift = 1;

uint16_t HVDC_BUS_BUFFER[8000];
uint16_t GRID_VAC_BUFFER[8000];
uint16_t GRID_IAC_BUFFER[8000];
int16_t P_ACTIVE_BUFFER[8000];
int16_t P_REACTIVE_BUFFER[8000];

extern s16 AVG_Alpha_Current;
extern s16 AVG_Beta_Current;

void __attribute__( ( optimize( "O0" ) ) )
delay_cycles( uint32_t cyc ) {
  for ( uint32_t d_i = 0; d_i < cyc; ++d_i ) { asm( "NOP" ); }
}

int _write(int file, char *ptr, int len)
{
  /* Implement your write code here, this is used by puts and printf for example */
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
}

//some variables for FatFs
//  FATFS FatFs; 	//Fatfs handle
//  FIL fil; 		//File handle
 // FRESULT fres; //Result after operations

FATFS FatFs; 	//Fatfs handle
FIL fil; 		//File handle
FRESULT fres; //Result after operations

/*******************************************************************************
* Function Name  : SetBusDCReference
* Description    : Set The bus DC Reference
* Input          : DC PAnel voltage
* Return         : None
*******************************************************************************/
void SetBusDCReference(s16 DCVoltage)
{
  if (DCVoltage <= _VOLTAGE_REF(200))
    BUS_Voltage_PID.Reference = _VOLTAGE_REF(380);

  if (DCVoltage > _VOLTAGE_REF(200) && DCVoltage <= _VOLTAGE_REF(300))
    BUS_Voltage_PID.Reference = _VOLTAGE_REF(380);

  if (DCVoltage > _VOLTAGE_REF(300))
    BUS_Voltage_PID.Reference = _VOLTAGE_REF(380);
}

/*******************************************************************************
* Function Name  : InitControl
* Description    : Init the parameters for the control algoritm
* Input          : mode the control mode (open/closed/init)
* Return         : None
*******************************************************************************/
void InitControl(ControlMode_t mode)
{
    //GPIO_InitTypeDef GPIO_InitStructure;
    DCDC_TypeDef_t InitStructure;
    DCAC_TypeDef_t DCAC_InitStructure;
    DS_TypeDef_t DS_InitStructure;

    /*DCDC Converter configuration */
    InitStructure.Counter = DCDC_COUNTER;
    InitStructure.DutyCycle = DCDC_DUTYCYCLE;
    InitStructure.frequency = DCDC_FREQUENCY;
    DCDC_Init(&InitStructure);

    /*DCAC Inverter init configuration*/
    DCAC_InitStructure.Counter = DCAC_COUNTER;
    DCAC_InitStructure.DeadTime = DCAC_DEADTIME;
    DCAC_Init(&DCAC_InitStructure);



    ControlMode = mode;
    /*DataSensing Init configuration*/
    DS_InitStructure.Counter = DCAC_COUNTER; //--the same counter of the DCAC
    DS_InitStructure.DataRegister =  (volatile uint16_t *)&DataSensingIO;
    DS_InitStructure.RegisterSize = DATA_SENSING_SIZE;

    //if(State_Control==STOP)
    //{
        if (mode == ClosedLoop)
          DS_InitStructure.OnAcquisition = ExecControl;
        else
          DS_InitStructure.OnAcquisition = ExecControlOpenLoop;



      DS_Init(&DS_InitStructure);
    //}
    /*Calibration of data sensing ***AFTER DS_Init*** */
      //ptr_State_Control = &State_Control;
      //extern volatile uint16_t to_grid_State_Control[SPI4_DATA_SIZE];
      ptr_State_Control = &to_grid_State_Control[0];
      ptr_org_State_Control = &State_Control;

      ptr_Pulse1_IT = &Pulse1_IT;
      ptr_Pulse2_IT = &Pulse2_IT;

  	  State_Control = STOP;
  	  calib = FALSE;

      Freq_Control = FREQ_OUT_OF_RANGE;
      GDVoltage = GRID_VOLTAGE_OUT_OF_RANGE;
      Diagnostic_Control = STOP;
      Fault = 0;
      CalibrationControl();



    *ptr_State_Control=State_Control;
    *(ptr_State_Control+1)=State_Control;
    *(ptr_State_Control+2)=State_Control;
    /* Configure RELE   : PA1-----------------------------------*/
    /* Relay in main
     *
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
     */
    PID_Init_Integral_Part();
    PID_Init(&Direct_Current_PID, &Quadrature_Current_PID, &Reactive_Power_PID,&Active_Power_PID,&BUS_Voltage_PID,&DQ_PLL_PID, &MPPT_PID);

    //SetBusDCReference(DataSensingIO.DC_BusVoltage);

 // PID_Init(PI_ControllerTYPEDEF *Direct_PID, PI_ControllerTYPEDEF *Quadrature_PID,PI_ControllerTYPEDEF *Reactive_PID,PI_ControllerTYPEDEF *Active_PID,PI_ControllerTYPEDEF *Bus_DC_PID, PI_ControllerTYPEDEF *PLL_PID,PI_ControllerTYPEDEF *MPPT_PID)

    //Get Control Parameters address
    GetControlParametersAddress((PControlParam_t)&CtrlParam);
    //Get Data Parameters address
    GetDataParametersAddress((PPhotov_t)&Data);

    StartControl();
    //OFF RELE' GRID
   // GPIO_ResetBits(GPIOA, GPIO_Pin_1);
}

/*******************************************************************************
* Function Name  : GetControlParam
* Description    : Get the value of a control parameter
* Input          : Parameter enumeration to get
* Return         : value of control parmaeter requested
*******************************************************************************/
s16 GetControlParam(ControlParamName_t ParamName)
{
  return **((((s16 **)(&CtrlParam)) + (u8)ParamName));
}

/*******************************************************************************
* Function Name  : SetControlParam
* Description    : Set the value of a control parameter
* Input          : Parameter enumeration to set
*                : value new value to set
* Return         : none
*******************************************************************************/
void SetControlParam(ControlParamName_t ParamName, s16 value)
{
   **((((s16 **)(&CtrlParam)) + (u8)ParamName)) = value;
}

/*******************************************************************************
* Function Name  : GetDataLinkStruct
* Description    : Get the pointer to data link structure
* Input          : none
* Return         : The pointer to data link;
*******************************************************************************/
PPhotov_t GetDataLinkStruct()
{
  return (PPhotov_t)&Data;
}


/*******************************************************************************
* Function Name  : SetControlMode
* Description    : Change the control mode
*                  before DS_Init..
* Input          : none
* Return         : None
*******************************************************************************/
u8 SetControlMode(ControlMode_t mode)
{
    u8 nRet = CONTROL_ERROR_RUNNING;
    if (GetStatusControl() == STOP)
    {
      nRet = CONTROL_ERROR_NONE;
      switch (mode)
      {
        case InitCalib:
          {
            //ControlMode = mode;
            DS_SetAcquistionEvent(ExecControlInitParamOffSet);
            break;
          }
        case ClosedLoop:
          {
            ControlMode = mode;
            DS_SetAcquistionEvent(ExecControl);
            break;
          }
        case OpenLoop:
        default:
          {
            ControlMode = mode;
            DS_SetAcquistionEvent(ExecControlOpenLoop);
            break;
          }
      }
    }
    return nRet;
}

/*******************************************************************************
* Function Name  : CalibrationControl
* Description    : Perform data sensing calibration this function MUST be called
*                  before DS_Init..
* Input          : none
* Return         : None
*******************************************************************************/
u8 CalibrationControl()
{
  u8 nRet = CONTROL_ERROR_RUNNING;
   if (GetStatusControl() == STOP)
    {
      /*Init constant off set for the */
      DataSensingOffSet.AC_LineCurrent  = 0;
      DataSensingOffSet.AC_LineVoltage  = 0;
      DataSensingOffSet.DC_PanelCurrent = 0;
      DataSensingOffSet.DC_PanelVoltage = 0;
      DataSensingOffSet.DC_BusVoltage   = 0;

      DataSensing_sum.AC_LineCurrent  = 0;
      DataSensing_sum.AC_LineVoltage  = 0;
      DataSensing_sum.DC_PanelCurrent = 0;
      DataSensing_sum.DC_PanelVoltage = 0;
      DataSensing_sum.DC_BusVoltage   = 0;

      SetControlMode(InitCalib);

      DataInitCount = 0;
      DS_SendCommand(DS_Start);
      while (DataInitCount < MAX_DATA_INIT_COUNT); //wait untill init complete
      //HAL_SPI_DMAStop(&hspi4);
      DS_SendCommand(DS_Stop);
      DataInitCount = 0;
      if(calib==TRUE)
       {


    	TIM20->CCMR1 &= ~(TIM_CCMR1_OC2M_0); //TOP LF MOSFET LOW / BOTTOM LF MOSFET HIGH
    	HRTIM1_COMMON->ODISR |=HRTIM_ODISR_TA1ODIS; //TIMA OUTPUT 1 DISABLE
    	HRTIM1_COMMON->ODISR |=HRTIM_ODISR_TA2ODIS; //TIMA OUTPUT 2 DISABLE
    	HRTIM1_COMMON->ODISR |=HRTIM_ODISR_TB1ODIS; //TIMB OUTPUT 1 DISABLE
    	HRTIM1_COMMON->ODISR |=HRTIM_ODISR_TB2ODIS; //TIMB OUTPUT 2 DISABLE
  		//HRTIM1_COMMON->ODISR |=HRTIM_ODISR_TE1ODIS; //TIME OUTPUT 1 DISABLE
  		HRTIM1_COMMON->ODISR |=HRTIM_ODISR_TE2ODIS; //TIME OUTPUT 2 DISABLE
  		HRTIM1_COMMON->ODISR |=HRTIM_ODISR_TF1ODIS; //TIMF OUTPUT 1 DISABLE
  		HRTIM1_COMMON->ODISR |=HRTIM_ODISR_TF2ODIS; //TIMF OUTPUT 2 DISABLE

  		HRTIM1_COMMON->ODISR |=HRTIM_ODISR_TD1ODIS; //TIMD OUTPUT 1 DISABLE
  		HRTIM1_COMMON->ODISR |=HRTIM_ODISR_TD2ODIS; //TIMD OUTPUT 2 DISABLE

  		//HRTIM1_COMMON->OENR |=HRTIM_OENR_TE2OEN; //TIME OUTPUT 2 ENABLE // GRID-DCAC ADC TRIG
        LL_GPIO_SetOutputPin(RELAY_GRID_GPIO_Port, RELAY_GRID_Pin); //RELAY ON
        DataInitCount = 0;
        DS_SendCommand(DS_Start);
        while (DataInitCount < MAX_DATA_INIT_COUNT); //wait untill init complete
        DS_SendCommand(DS_Stop);
        DataInitCount = 0;

       }
      LL_GPIO_ResetOutputPin(RELAY_GRID_GPIO_Port, RELAY_GRID_Pin);
      SetControlMode(ControlMode);
      //SetControlMode(ClosedLoop);
      calib=TRUE;
      nRet = CONTROL_ERROR_NONE;
    }
   return nRet;
}

/*******************************************************************************
* Function Name  : GetStatusControl
* Description    : Get The status
* Input          : none
* Return         : algothm status
*******************************************************************************/
SystStatus_t GetStatusControl()
{
    return State_Control;
}

/*******************************************************************************
* Function Name  : GetDataParametersAddress
* Description    : Get The pointers to the Data parameters used in the algoritm
* Input          : Data parameters pointers structures
* Return         : none
*******************************************************************************/
void GetDataParametersAddress(PPhotov_t pData)
{

  //To review
  pData->Power.pPin = NULL;
  pData->Power.pPout_act = &Actual_QD_Power.P_Active;
  pData->Power.pOutFreq = &Output_qVd_Grid;
  pData->Power.pQ = &Actual_QD_Power.Q_Reactive;
  pData->Power.pPowerThreshold =  &PowerThreshold;

  pData->VoltageCurrent.pVin_DC  = (s16 *)&DataSensingOffSet.DC_PanelVoltage;
  pData->VoltageCurrent.pIin_DC  = (s16 *)&DataSensingOffSet.DC_PanelCurrent;
  pData->VoltageCurrent.pVout_AC = (s16 *)&DataSensingOffSet.AC_LineVoltage;
  pData->VoltageCurrent.pIout_AC = (s16 *)&DataSensingOffSet.AC_LineCurrent;
  pData->VoltageCurrent.pVDC_Bus = (s16 *)&DataSensingOffSet.DC_BusVoltage;
  pData->VoltageCurrent.pVDC_BusRef = &BUS_Voltage_PID.Reference;

  pData->Status.pFault = &Fault;
  pData->Status.pState = &State;
}

/*******************************************************************************
* Function Name  : GetControlParametersAddress
* Description    : Get The pointers to the control parameters used in the algoritm
* Input          : Control parameters pointers structures
* Return         : none
*******************************************************************************/
void GetControlParametersAddress(PControlParam_t pControl)
{
    //MPPT
//    pControl->Mppt.pIntegral = &(MPPT_PID.Ki_Gain);
//    pControl->Mppt.pProportional = &(MPPT_PID.Kp_Gain);

    //PLL
    pControl->PLL.pIntegral = &(DQ_PLL_PID.Ki_Gain);
    pControl->PLL.pProportional = &(DQ_PLL_PID.Kp_Gain);

    //DCBUS
    pControl->DCBUS.pIntegral = &(BUS_Voltage_PID.Ki_Gain);
    pControl->DCBUS.pProportional = &(BUS_Voltage_PID.Kp_Gain);

    //id
    pControl->Id.pIntegral = &(Direct_Current_PID.Ki_Gain);
    pControl->Id.pProportional = &(Direct_Current_PID.Kp_Gain);

    //iq
    pControl->Iq.pIntegral = &(Quadrature_Current_PID.Ki_Gain);
    pControl->Iq.pProportional = &(Quadrature_Current_PID.Kp_Gain);

    //Reactive = Q
 //   pControl->Q.pIntegral = &(Reactive_Power_PID.Ki_Gain);
 //   pControl->Q.pProportional = &(Reactive_Power_PID.Kp_Gain);

    //Reactive = Q
    pControl->k1k2.xz1 = &z1;
    pControl->k1k2.xz2 = &z2;
}

/*******************************************************************************
* Function Name  : StartControl
* Description    : Start all logical peripherals to
* Input          : none
* Return         : None
*******************************************************************************/
bool PRECHARGE_OK=false;
void StartControl()
{


    //PID_Init_Integral_Part();
    //PID_Init(&Direct_Current_PID, &Quadrature_Current_PID, &Reactive_Power_PID,&Active_Power_PID,&BUS_Voltage_PID,&DQ_PLL_PID, &MPPT_PID);

    BusOverVoltage = FALSE;
    GridOutage = FALSE;
    Fault = FAULT_NONE;
    StoppingCount = TIME_OUT_STOPPING;

    //disable the inverter modulation before the burst mode
    //DCAC_SetPulse(2047,2047);
    //DCAC_SetPulse(DCAC_COUNTER,DCAC_COUNTER);

    //TIM20->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_0); //TOP LF MOSFET HIGH / BOTTOM LF MOSFET LOW
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].SETx1R &= ~ (1 << 19);
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].SETx1R |= (1 << 0);

    //Freq_Control = FREQ_OUT_OF_RANGE;
    //GDVoltage = GRID_VOLTAGE_OUT_OF_RANGE;

    State_Control = DIAGNOSTIC_AC_LINE;

    //SET RELE' GRID
    //GPIO_ResetBits(GPIOA, GPIO_Pin_1);
    DCAC_SendCommand(DCAC_Start);
    DS_SendCommand(DS_Start);


    //DCDC_SendCommand(DCDC_ConverterStart);
    DCDC_SetPhaseShift(PHASE_SHIFT_INIT);
    DCDC_SetFrequency(MASTER_PWM_FREQ); // SET FREQUENCY AND PERIOD OF DCDC
    if (ControlMode == OpenLoop)
    {

        DCDC_SendCommand(DCDC_ConverterStart);

        while(DCDC_GetPhaseShift() < (DCDC_GetPeriod()))
        {
      	  DCDC_SetPhaseShift(DCDC_GetPhaseShift()+1);
      	  delay_cycles(850);
        }


            while(DCDC_GetFrequency() > 240000)
            {
          	  ActualFreq_Val=ActualFreq_Val - 1;
          	  DCDC_SetFrequency((u32)ActualFreq_Val);
          	  delay_cycles(170);
            }

        PRECHARGE_OK=true;
    }




    // DAC_Start(); //commentato per testare l open loop
//   flag=1;
}

/*******************************************************************************
* Function Name  : StopControl
* Description    : Start all logical peripherals to
* Input          : none
* Return         : None
*******************************************************************************/
void StopControl()
{
   // DAC_Stop();

    State_Control=STOPPING;
}

/*******************************************************************************
* Function Name  : RefreshDisplayControlParam
* Description    : display the information for the controls parameters Ki, Kp
* Input          : err error to display, display the option to modify
* Return         : None
*******************************************************************************/
void RefreshDisplayControlParam(u32 err, u32 display)
{
	/*
    char strLineMessage[64];
    u8 sSel;
    u16 Param1;

    Param1 = DataSensingOffSet.AC_LineCurrent;

    LCD_DisplayStringLine(Line0, " Control Parameters ");

    LCD_ClearLine(Line1);

    LCD_DisplayStringLine(Line2, "          Ki     Kp ");

    sSel = 6;
    if (display & 0x01)
      sSel = 13;

//    sprintf(strLineMessage, "Mppt  %6.d %6.d", *CtrlParam.Mppt.pIntegral, *CtrlParam.Mppt.pProportional);
//    LCD_ClearLine(Line3);
//    LCD_DisplayStringLineEx(Line3, sSel, LSELLINE(Line3, display), (u8 *)strLineMessage);

    sprintf(strLineMessage, "PLL  %6.d %6.d", *CtrlParam.PLL.pIntegral, *CtrlParam.PLL.pProportional);
    LCD_ClearLine(Line3);
    LCD_DisplayStringLineEx(Line3, sSel, LSELLINE(Line3, display), (u8 *)strLineMessage);

    sprintf(strLineMessage, "DCBUS %6.d %6.d", *CtrlParam.DCBUS.pIntegral, *CtrlParam.DCBUS.pProportional);
    LCD_ClearLine(Line4);
    LCD_DisplayStringLineEx(Line4, sSel, LSELLINE(Line4, display), (u8 *)strLineMessage);

    sprintf(strLineMessage, "Id    %6.d %6.d", *CtrlParam.Id.pIntegral, *CtrlParam.Id.pProportional);
    LCD_ClearLine(Line5);
    LCD_DisplayStringLineEx(Line5, sSel, LSELLINE(Line5, display), (u8 *)strLineMessage);

    sprintf(strLineMessage, "Iq    %6.d %6.d", *CtrlParam.Iq.pIntegral, *CtrlParam.Iq.pProportional);
    LCD_ClearLine(Line6);
    LCD_DisplayStringLineEx(Line6, sSel, LSELLINE(Line6, display), (u8 *)strLineMessage);

//    sprintf(strLineMessage, "Q     %6.d %6.d", *CtrlParam.Q.pIntegral, *CtrlParam.Q.pProportional);
//    LCD_ClearLine(Line7);
//    LCD_DisplayStringLineEx(Line7, sSel, LSELLINE(Line7, display), (u8 *)strLineMessage);

//    sprintf(strLineMessage, "Z1,Z2  %5.d %5.d", *CtrlParam.k1k2.xz1, *CtrlParam.k1k2.xz2 );
//    LCD_ClearLine(Line7);
//    sprintf(strLineMessage, "ACCoffset %6.d", *CtrlParam.k1k2.xz1);
    sprintf(strLineMessage, "AcC,A %5.d %5.d",*CtrlParam.k1k2.xz1,*CtrlParam.k1k2.xz1);
    LCD_ClearLine(Line7);
    LCD_DisplayStringLineEx(Line7, sSel, LSELLINE(Line7, display), (u8 *)strLineMessage);

    LCD_ClearLine(Line8);
    */
}

/*******************************************************************************
* Function Name  : ExecControlInitParamOffSet
* Description    : Execute control to initialaze OFFSET Parameters
* Input          : none
* Return         : None
*******************************************************************************/
void ExecControlInitParamOffSet()
{

	DataSensing_sum.AC_LineCurrent = ((DataSensing_sum.AC_LineCurrent)+(u32)(DataSensingIO.AC_LineCurrent));

	DataSensing_sum.AC_LineVoltage = ((DataSensing_sum.AC_LineVoltage)+(u32)(DataSensingIO.AC_LineVoltage));

    //avg = ((avg * (n-1)) + currentReading + (n/2)) / n;

   // DataSensing_sum.AC_LineVoltage = ((DataSensing_sum.AC_LineVoltage * (MAX_DATA_INIT_COUNT-1)) + (DataSensingIO.AC_LineVoltage) + (MAX_DATA_INIT_COUNT/2)) / MAX_DATA_INIT_COUNT;

	DataSensing_sum.DC_PanelCurrent = ((DataSensing_sum.DC_PanelCurrent)+(u32)(DataSensingIO.DC_PanelCurrent));
	DataSensing_sum.DC_BusVoltage = ((DataSensing_sum.DC_BusVoltage)+(u32)(DataSensingIO.DC_BusVoltage));
	DataSensing_sum.DC_PanelVoltage = ((DataSensing_sum.DC_PanelVoltage)+(u32)(DataSensingIO.DC_PanelVoltage));


    if (DataInitCount < (MAX_DATA_INIT_COUNT-1))
      DataInitCount++;

    else if(DataInitCount==(MAX_DATA_INIT_COUNT-1))
    {

      ((vu16*)&DataSensingOffSet)[0] = (u16)(DataSensing_sum.AC_LineCurrent / (DataInitCount+1));
      ((vu16*)&DataSensingOffSet)[1] = (u16)(DataSensing_sum.AC_LineVoltage / (DataInitCount+1));
      ((vu16*)&DataSensingOffSet)[2] = (u16)(DataSensing_sum.DC_BusVoltage / (DataInitCount+1));
      ((vu16*)&DataSensingOffSet)[3] = (u16)(DataSensing_sum.DC_PanelCurrent / (DataInitCount+1));
      ((vu16*)&DataSensingOffSet)[4] = (u16)(DataSensing_sum.DC_PanelVoltage / (DataInitCount+1));

      DataInitCount++;
    }


}


/*******************************************************************************
* Function Name  : ExecControlOpenLoop
* Description    : Execute control algoritm in OpenLoop mode
* Input          : none
* Return         : None
*******************************************************************************/
/*
void ExecControlOpenLoop()
 {
  s32 SinWave_q31;
  s16 SineWave, Sin_Theta;
  u16 Index_Sin;

  //HRTIM1_COMMON->OENR |=HRTIM_OENR_TF1OEN; //TIMF OUTPUT 1 ENABLE
  //HRTIM1_COMMON->OENR |=HRTIM_OENR_TF2OEN; //TIMF OUTPUT 2 ENABLE
  for (int i = 0; i < 5; i++)
      ((vu16*)&DataSensingIO)[i] -= ((vu16*)&DataSensingOffSet)[i];

    State_Control=START;

    //Theta +=300;
    //Theta +=158;
    //Theta +=65; 50hz - 50khz
    Theta +=110; //50hz - 50khz
    Index_Sin = ((u16)(Theta>>7) & (u16)(0x01FF));

    Sin_Theta = Sin_Cos_Table[Index_Sin];
    new_mul_q15_q15_q31(Amplitude,Sin_Theta);
    //mul_q15_q15_q31(Amplitude2,Sin_Theta,&SinWave_q31);
    SineWave = (s16)(new_mul_q15_q15_q31(Amplitude,Sin_Theta)/65530);
    //SineWave = (s16)(SinWave_q31/78634);


    LL_GPIO_SetOutputPin(RELAY_GRID_GPIO_Port, RELAY_GRID_Pin);

    if(new_mul_q15_q15_q31(Amplitude,Sin_Theta)<=0)
    {
    //GPIO_WriteBit(GPIOB, GPIO_Pin_8,Bit_SET);
    	TIM20->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_0); //TOP LF MOSFET HIGH / BOTTOM LF MOSFET LOW
    	//delay_us(1);
      polarity=FALSE;
  //  MPPT_func();

    }
    if(new_mul_q15_q15_q31(Amplitude,Sin_Theta)>0)
    {
      olstart=FALSE;
      polarity=TRUE;
      TIM20->CCMR1 &= ~(TIM_CCMR1_OC2M_0); //TOP LF MOSFET LOW / BOTTOM LF MOSFET HIGH

   // MPPT_func();
    }

    if(polarity==FALSE)
    {
    //qValpha_shifted_pos = (u16)(SineWave+0x8000)>>4;
      qValpha_shifted_pos = (((u16)(SineWave+0x8000)));
      Pulse1=(u16)(qValpha_shifted_pos);
      //Pulse1 = ((u16)(new_mul_q15_q15_q31((s16)(qValpha_shifted_pos ), MODINDEX) >> 16));

    }
    else if (polarity==TRUE)
    {
   // qValpha_shifted_pos = (u16)(5+(s16)(SineWave>>4));
      qValpha_shifted_pos = (u16)(300+(s16)(SineWave));
      Pulse1=(u16)(qValpha_shifted_pos);
      //Pulse1 = (((u16)(new_mul_q15_q15_q31(qValpha_shifted_pos, MODINDEX) >> 16)+56));
    //GPIO_WriteBit(GPIOB, GPIO_Pin_8,Bit_RESET);
      //TIM20->CCMR1 &= ~(TIM_CCMR1_OC2M_0); //TOP LF MOSFET LOW / BOTTOM LF MOSFET HIGH
    }


    DCAC_SetPulse((u16)(Pulse1), (u16)(Pulse1));

    //DCAC_SetPulse((u16)(0), (u16)(0));
}
*/
uint16_t ii = 0;
uint16_t HVDC_BUS_VAL = 0;
uint16_t GRID_VAC_VAL = 0;
uint16_t GRID_IAC_VAL = 0;
/*
int _write(int file, char *ptr, int len)
{
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}
*/
void ExecControlOpenLoop()
 {
  s32 SinWave_q31;
  s16 SineWave, Sin_Theta;
  u16 Index_Sin;

  //HVDC_BUS_VAL = DataSensingIO.DC_BusVoltage;
  //GRID_VAC_VAL = DataSensingIO.AC_LineVoltage;
  //GRID_IAC_VAL = DataSensingIO.AC_LineCurrent;

  //uint8_t data[] = "HELLO WORLD \r\n";

    DMA1_Channel5->CCR &= ~( DMA_CCR_EN ); //CHANNEL ENABLE
    DMA1_Channel3->CCR &= ~( DMA_CCR_EN ); //CHANNEL ENABLE
    DMA1_Channel4->CCR &= ~( DMA_CCR_EN ); //CHANNEL ENABLE
    DMA1_Channel6->CCR &= ~( DMA_CCR_EN ); //CHANNEL ENABLE
    State_Control=START;

    //Theta += 131;//50khz 50hz
    //Theta += 158;
    Theta += 154;
    Index_Sin = (((u16)(Theta>>7)) & (u16)(0x01FF));

    Sin_Theta = Sin_Cos_Table[Index_Sin];
    mul_q15_q15_q31(Amplitude,Sin_Theta,&SinWave_q31);
    SineWave = (s16)(SinWave_q31/65536);
    /*PORT A.1 drives the RELE' */
    //GPIO_SetBits(GPIOA, GPIO_Pin_1);

    if (DCAC_GetStatus() == DCAC_Running)
    {
    	RESET_TIMER_D = TRUE;

		if(RELAY_ON == FALSE)
		{
		LL_GPIO_SetOutputPin(RELAY_GRID_GPIO_Port, RELAY_GRID_Pin);
		delay_us(20000);
		RELAY_ON = TRUE;
		}

		//LL_GPIO_TogglePin(SD_CS_GPIO_Port, SD_CS_Pin);

		if((SinWave_q31)<0)
		{


		  polarity=FALSE;

		  qValpha_shifted_pos = ((u16)(SineWave+(TIMF_PERIOD)-327));
		  Pulse1=(u16)(qValpha_shifted_pos);
		  DCAC_SetPulse((u16)(Pulse1), (u16)(Pulse1));



		}
		if((SinWave_q31)>0)
		{

		  olstart=FALSE;
		  polarity=TRUE;

		  qValpha_shifted_pos = (u16)(327+(s16)(SineWave));
		  Pulse1=(u16)(qValpha_shifted_pos);

		  DCAC_SetPulse((u16)(Pulse1), (u16)(Pulse1));



		}

		if((SinWave_q31)<00000000)

		{

			  if((LF_MOS_SET == TRUE) || FIRST_CYCLE == TRUE)
			  {
				  LL_GPIO_ResetOutputPin(SD_CS_GPIO_Port, SD_CS_Pin);
				  RESET_TIMER_D = FALSE;
				  LF_MOS_SET = FALSE;
				  FIRST_CYCLE = FALSE;


				  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_F].CMP2xR = (uint32_t)(60);

				  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].SETx1R |= 1 << 3;

				  //HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = (uint32_t)(Pulse1-330);

				  HRTIM1_COMMON->CR2 |= (0x50 << 0); //UPDATE TIM F AND D SIMULT
				  LF_UPDATE = TRUE;


			  }
			  else
			  {
				  if (LF_UPDATE == TRUE)
				  {

					  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].SETx1R &= ~ (1 << 3);

					  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].RSTx1R |= (1 << 0);


					  LF_UPDATE = FALSE;
					  HRTIM1_COMMON->CR2 |= (0x50 << 0); //UPDATE TIM F AND D SIMULT
				  }

			  }

		}

		if((SinWave_q31)>00000000)

		{
			 if((LF_MOS_SET == FALSE) || (FIRST_CYCLE == TRUE))
					  {

						  RESET_TIMER_D = FALSE;
						  //LF_MOS_SET = TRUE;
						  FIRST_CYCLE = FALSE;

						  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_F].CMP2xR = (uint32_t)(16);

						  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].SETx1R |= 1 << 3;

						  //HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = (uint32_t)(Pulse1+300);

						  HRTIM1_COMMON->CR2 |= (0x50 << 0); //UPDATE TIM F AND D SIMULT

						  LF_UPDATE = TRUE;


					  }
					  else
					  {

						  if (LF_UPDATE == TRUE)
						  {
							  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].SETx1R &= ~ (1 << 3);
							  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].SETx1R |= (1 << 0);
							  LF_UPDATE = FALSE;

							  HRTIM1_COMMON->CR2 |= (0x50 << 0); //UPDATE TIM F AND D SIMULT
						  }

					  }
					  LF_MOS_SET = TRUE;

		}

    }
}

/*******************************************************************************
* Function Name  : GetAmplitude
* Description    : Get the amplitude for open loop control mode
* Input          : none
* Return         : amplitude value
*******************************************************************************/
s16 GetAmplitude()
{
  return Amplitude;
}

/*******************************************************************************
* Function Name  : SetAmplitude
* Description    : Set the amplitude for open loop control mode
* Input          : the amplitude to set
* Return         : none
*******************************************************************************/
void SetAmplitude(s16 amp)
{

  if (amp >= 0 && amp < MAX_AMPLITUDE)
    Amplitude = amp;
}

/*******************************************************************************
* Function Name  : ExecControl
* Description    : Execute control algoritm
* Input          : none
* Return         : None
*******************************************************************************/
void ExecControl()
{
	//LL_GPIO_SetOutputPin(SD_CS_GPIO_Port, SD_CS_Pin);
    //DS_SendCommand(DS_Stop);
    //DCAC_SendCommand(DCAC_Stop);
	 // State_Control = STOP;
	  //calib = FALSE;

    //Freq_Control = FREQ_OUT_OF_RANGE;
    //GDVoltage = GRID_VOLTAGE_OUT_OF_RANGE;
    //Diagnostic_Control = STOP;
    //Fault = 0;
    //CalibrationControl();

	//GPIOG->BSRR = (1<<9);
  u32 freq_actual = (u32) DCDC_GetFrequency();
  freq_actual = freq_actual + 1;
  freq_actual = freq_actual - 1;
  //GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
  static s16 BusFiltered = 0;
  static s16 PVvoltageFiltered = 0;
  static s16 PVcurrentFiltered = 0;

  u16 Param1;


  PV_Voltage=(u16)(DataSensingIO.DC_PanelVoltage);
  PV_Current=((u16)(DataSensingIO.DC_PanelCurrent))<<1;
/*
  ((vu16*)&DataSensingIO)[2] >>=1;  //DC BUS to consider only the positive values


  ((vu16*)&DataSensingIO)[4] >>=1;  //DC Panel Current consider only the positve values

  Param1 = ((vu16*)&DataSensingOffSet)[3];
  if (((vu16*)&DataSensingIO)[3] > Param1)
      ((vu16*)&DataSensingIO)[3] -= Param1;
  else
    ((vu16*)&DataSensingIO)[3] = 0;
  ((vu16*)&DataSensingIO)[3] = ((vu16*)&DataSensingIO)[3] >> 1;  //DC Panel Voltage consider only the positve values
*/
  Param1 = ((vu16*)&DataSensingOffSet)[1];
  ((vu16*)&DataSensingIO)[1] -= Param1;     //AC Line Voltage Data Sensing compensation
  Param1 = ((vu16*)&DataSensingOffSet)[0];
  ((vu16*)&DataSensingIO)[0] -= Param1;     //AC Line Current Data Sensing compensation






  if(PV_Voltage<=1200 && State_Control==GRID_INSERTION) //22V min
  {
      //GPIO_ResetBits(GPIOA, GPIO_Pin_1);
	  //LL_GPIO_ResetOutputPin(RELAY_GRID_GPIO_Port, RELAY_GRID_Pin);
      State_Control = STOP_WITH_DELAY;
      Diagnostic_Control=PV_VOLTAGE_MIN;
  }


  Bus_Voltage = (s16) (DataSensingIO.DC_BusVoltage) >>1; // >> 3 było
  //Grid_Voltage = -(s16)((DataSensingIO.AC_LineVoltage)<<2); //OVERSAMPLING X2
  Grid_Voltage = -(s16)((DataSensingIO.AC_LineVoltage)); //OVERSAMPLING X8

  //VERIFICARE CHE I SEGNI DELLA CORRENTE SIANO CONCORDI IN USCITA DALL'INVERTER E IN USCITA DAL LEM. SE DISCORDI AGIRE VIA FW(CON UN -) O HW CON AVVOLGIMENTO IN SENSO OPPOSTO
  //qIalpha_Inverter=-(s16)(DataSensingIO.AC_LineCurrent)<<2; //OVERSAMPLING X2 //by G.S. 19/03/12
  qIalpha_Inverter=-(s16)(DataSensingIO.AC_LineCurrent); //OVERSAMPLING X8 //by G.S. 19/03/12 //
  //qIalpha_Inverter=-(s16)(((DataSensingIO.AC_LineCurrent)) - (*CtrlParam.k1k2.xz1));  //dc current compensation
  //u16 param_current = 0;
  //param_current = *CtrlParam.k1k2.xz1;
 //Output current sensing max - protection  (2A) - 15000 ok 2A
  if((Inverter_q_d.qI_Quadrature > 15000) && State_Control==GRID_INSERTION && MPPT_EN==TRUE) //2 A max output peak current
  {
     // 0.5 sec before the protection trips
     if(waiting_output_current_limit>=12500)
      {
      waiting_output_current_limit=0;
      Fault = OUT_CURRENT_LIMIT;
      //GPIO_ResetBits(GPIOA, GPIO_Pin_1);
      //LL_GPIO_ResetOutputPin(RELAY_GRID_GPIO_Port, RELAY_GRID_Pin);
      State_Control = STOP_WITH_DELAY;
      Diagnostic_Control=OUT_CURRENT_LIMIT;
      }
      else waiting_output_current_limit=waiting_output_current_limit+3;
   }
   else
   {
     if((waiting_output_current_limit-1)<=1) waiting_output_current_limit=0;
      else waiting_output_current_limit--;
   }
  /*************************GRID RELATED**********************************/



  Grid_Volt_q_d= DQ_PLL_Grid(Grid_Voltage);
  //Grid_Volt_q_d.qV_Quadrature = SimpleMovingAvarage_quadrature(Grid_Volt_q_d.qV_Quadrature);
  //Grid_Volt_q_d.qV_Direct = SimpleMovingAvarage_direct(Grid_Volt_q_d.qV_Direct);
  if(State_Control==GRID_INSERTION && DQ_PLL_PID.Reference < 50 && (Theta_time==((Theta_Grid))))
  {
	  //DQ_PLL_PID.Reference = (DQ_PLL_PID.Reference) + 1;
  }
  Output_qVd_Grid=(s16)(PLL_PID_Regulator(&DQ_PLL_PID,Grid_Volt_q_d));
  Calc_Theta_Grid(Output_qVd_Grid);
/*
  if(oii<2048)
  {
  buffer[oii]= Calc_Theta_Grid;
  oii++;
  }
  */
  /************************* MPPT ROUTINE ******************************/
if(State_Control==START && MPPT_EN==FALSE && PLL_par_red_kp==FALSE && PLL_reducing!=TRUE)
{
   if(waiting_time<=1000) //2.5 sec
   {
     waiting_time++;
   }
   else
   {
     PLL_par_red_kp=TRUE;
     PLL_par_red_ki=TRUE;
     waiting_time=0;
   }
}

if(State_Control==GRID_INSERTION && MPPT_EN==TRUE)
{
  if((Theta + 0x8000) <= 1000 && (Theta + 0x8000) >= 0)
  {
    PVvoltageFiltered = (s16)(((s32)(((s32)PVvoltageFiltered<<3) - (s32)PVvoltageFiltered) +  (s32)(PV_Voltage))>>3);
    PVcurrentFiltered = (s16)(((s32)(((s32)PVcurrentFiltered<<3) - (s32)PVcurrentFiltered) +  (s32)(PV_Current))>>3);
    mppt_req=1;
    count_mppt++;
  }
  else
  {
    if(mppt_req==1 && count_mppt>=150)  //600usec per ciclo di count_mppt con 10000 is 6sec
    {
     PV_Voltage=(s16)PVvoltageFiltered;
     PV_Current=(s16)PVcurrentFiltered;
     //MPPT_func();
     mppt_req=0;
     count_mppt=0;

     MPPT_num++;
     if(MPPT_num==5) Vacprot=TRUE;
    }
  }
}
  /////********************Only for Theta Estimation*******************************************////

  qIalpha_Inverter+=0;//500;
  qIbeta_Inverter= (s16)(Generate_90Degrees_Delay(qIalpha_Inverter));

  //AlphaBeta_Filtering(qIalpha_Inverter,qIbeta_Inverter);

  //AVG_Current_DC=(s16)Current_DC_Filtering(PV_Current); // NIC NIE ROBI

  Inverter_q_d=DQ_Filtering(DQ_Current_Inverter(qIalpha_Inverter,qIbeta_Inverter)); //test filtra

  //Inverter_q_d=DQ_Current_Inverter(qIalpha_Inverter,qIbeta_Inverter); //test bez filtra

  //Inverter_q_d.qI_Quadrature = SimpleMovingAvarage_quadrature(Inverter_q_d.qI_Quadrature);
  //Inverter_q_d.qI_Direct = SimpleMovingAvarage_direct(Inverter_q_d.qI_Direct);

  //Inverter_q_d=DQ_Filtering(DQ_Current_Inverter(AVG_Alpha_Current,AVG_Beta_Current)); //test filtra

  //Inverter_q_d=DQ_Current_Inverter(AVG_Alpha_Current,AVG_Beta_Current); //test filtra

  //Inverter_q_d=DQ_Current_Inverter(qIalpha_Inverter,qIbeta_Inverter); //test bez filtra

  Actual_QD_Power =  DQ_Power_Estimation(Inverter_q_d);

  if ((ii<2048) && (State_Control == GRID_INSERTION))
  {

 	buffer[0][ii]=Actual_QD_Power.P_Active;
 	buffer[1][ii]=Actual_QD_Power.Q_Reactive;
 	//buffer[2][ii]=Inverter_q_d.qI_Direct;
 	buffer[3][ii]=Inverter_q_d.qI_Quadrature;
 	//buffer[4][ii]=Grid_Volt_q_d.qV_Direct;
 	buffer[5][ii]=Grid_Volt_q_d.qV_Quadrature;
 	ii++;
  }

  /**********************************************************************/

  PV_Voltage_prev=PV_Voltage;

  switch (State_Control)
  {

 case DIAGNOSTIC_AC_LINE:
    if(Freq_Control == FREQ_INSIDE_RANGE && GDVoltage == GRID_VOLTAGE_INSIDE_RANGE)
    {
      State_Control=DIAGNOSTIC_DC_LINE;
     // CallRefreshDisplay();
    }
    CalcAndSetACComponents(State_Control);

    // RESET DCAC DRIVERS

    //LL_GPIO_ResetOutputPin(RST_TOP_MOS_GPIO_Port, RST_TOP_MOS_Pin);
    //LL_GPIO_ResetOutputPin(RST_BOTT_MOS_GPIO_Port, RST_BOTT_MOS_Pin);
    //LL_GPIO_ResetOutputPin(RST_TOP_IGBT_GPIO_Port, RST_TOP_IGBT_Pin);
    //LL_GPIO_ResetOutputPin(RST_BOTT_IGBT_GPIO_Port, RST_BOTT_IGBT_Pin);

   break;

  case DIAGNOSTIC_DC_LINE:
    Wait_BUS_DC_Reading++;
    if (Wait_BUS_DC_Reading >= DC_BUS_WAIT)
    {
     if(PV_Voltage >= 1300 && PV_Voltage < 4096) //20V min - 50V max
      {
      Wait_BUS_DC_Reading =0;
      Wait_GRID_Insertion =0;
		  if(Bus_Voltage < (DataSensingOffSet.DC_BusVoltage)*2) //Wait after shutdown for Bus Voltage down
		  {
			  State_Control=BUSPRECHARGE;
			  DCDC_SendCommand(DCDC_ConverterStart);

	          while(DCDC_GetPhaseShift() < (DCDC_GetPeriod()))
	          {
	        	  DCDC_SetPhaseShift(DCDC_GetPhaseShift()+1);
	        	  delay_cycles(1700);
	          }

	          while(DCDC_GetFrequency() > 140000)
	          {
	          	  ActualFreq_Val=ActualFreq_Val - 1;
	          	  DCDC_SetFrequency((u32)ActualFreq_Val);
	          	  delay_cycles(170);
	          }
		  }

      //CallRefreshDisplay();
      }
      else
      {
        Wait_BUS_DC_Reading =0;
        Freq_Control= FREQ_OUT_OF_RANGE;
        GDVoltage = GRID_VOLTAGE_OUT_OF_RANGE;
        State_Control=DIAGNOSTIC_AC_LINE;
       // CallRefreshDisplay();
      }
    }
    CalcAndSetACComponents(State_Control);
    break;

  case BUSPRECHARGE:

     if(Bus_Voltage >= DC_BUS_VOLTAGE_MAX) //Vbus reg value at 400V
      {
        DCDC_SendCommand(DCDC_ConverterStop);
        bus_pre_charge=0;
        PRECHARGE_OK = TRUE;
        State_Control =START;

      //CallRefreshDisplay();
      }
     else
      {

       //DCDC_SendCommand(DCDC_ConverterStop);

       if(PV_Voltage < PV_Voltage_startup) //18V min input voltage
        {
          //TIM2->CCR1 = 1024;
          //TIM3->CCR3 = 1024;
    	  //DCDC_SetFrequency(MAX_FREQ);
          bus_pre_charge++;
        }
        else
        { //Vin goes @18V input
          //TIM2->CCR1 = 512;
          //TIM3->CCR3 = 512;
          //COMP1 ->CSR &= ~COMP_CSR_EN;
          //DCDC_SetFrequency(MAX_FREQ);

          DCDC_SendCommand(DCDC_ConverterStart);


/*
          while(DCDC_GetFrequency() > 140000)
          {
			  DCDC_SetFrequency(DCDC_GetFrequency()-1);
			  delay_cycles(17000);
          }
*/
        }
      }
    CalcAndSetACComponents(State_Control);

    if(bus_pre_charge>=_WAIT_SECONDS(3)  && Bus_Voltage<DC_BUS_VOLTAGE_MIN)
      {
        PV_Voltage_startup=1111;

      }
   break;

  case START:


	 //RELESE RESET IN DCAC DRIVERS

	 //LL_GPIO_SetOutputPin(RST_TOP_MOS_GPIO_Port, RST_TOP_MOS_Pin);
	 //LL_GPIO_SetOutputPin(RST_BOTT_MOS_GPIO_Port, RST_BOTT_MOS_Pin);
	 //LL_GPIO_SetOutputPin(RST_TOP_IGBT_GPIO_Port, RST_TOP_IGBT_Pin);
	 //LL_GPIO_SetOutputPin(RST_BOTT_IGBT_GPIO_Port, RST_BOTT_IGBT_Pin);


     Wait_GRID_Insertion++;
     /*  anti-island control before insert in grid  */
     if ((Bus_Voltage >= DC_BUS_VOLTAGE_MAX))
      {
        DCDC_SendCommand(DCDC_ConverterStop);
      }
    if ((Bus_Voltage < DC_BUS_VOLTAGE_MIN))
      {
        DCDC_SendCommand(DCDC_ConverterStart);
      }
      /*Burst Mode Stop*/
    	// Relay Zero Voltage Switch ON
      if ((Wait_GRID_Insertion >= GRID_WAIT) && (State_Control==START) && (Theta_time==((Theta_Grid/4)-REL_ON_TICK) || Theta_time==((Theta_Grid*3/4)-REL_ON_TICK)))
    //if ((Wait_GRID_Insertion >= GRID_WAIT) && (State_Control==START) && (Theta + 0x8000)<=500)
      {

        DCDC_SendCommand(DCDC_ConverterStart);

        /*PORT A.1 drives the RELE' */
        //GPIO_SetBits(GPIOA, GPIO_Pin_1);

        COMP7 ->CSR &= ~COMP_CSR_EN;//Glitch test
        TIM1->AF1 &= ~TIM1_AF1_BKINE; //DISABLE

        LL_GPIO_SetOutputPin(RELAY_GRID_GPIO_Port, RELAY_GRID_Pin); //RELAY ON
        Wait_GRID_Insertion=0;



        State_Control =GRID_INSERTION;

      }

     CalcAndSetACComponents(State_Control);
    break;

  case STOP_WITH_DELAY:

   if(WAIT_RELAY_STABLE>=REL_OFF_WAIT)
    {
	  LL_GPIO_ResetOutputPin(RELAY_GRID_GPIO_Port, RELAY_GRID_Pin);
      WAIT_RELAY_STABLE=0;
      State_Control = BUS_FAULT;
    }
    else
    {  WAIT_RELAY_STABLE++; }

    CalcAndSetACComponents(State_Control);
   break;

  case GRID_INSERTION:
    /* anti-island control RUNNING  */
    /*Bus Over voltage*/


    *ptr_State_Control=State_Control;
    *(ptr_State_Control+1)=State_Control;
    *(ptr_State_Control+2)=State_Control;
    Wait_MPPT++;
    if(Wait_MPPT==55000)
     { MPPT_EN=TRUE; }

    BusFiltered = (s16)(((s32)(((s32)BusFiltered<<6) - (s32)BusFiltered) +  (s32)(Bus_Voltage))>>6);

    if (BusFiltered > 16000) //3804 nowe nastawy 440V
        {
         Fault = BUS_OVERVOLTAGE;
         //LL_GPIO_ResetOutputPin(RELAY_GRID_GPIO_Port, RELAY_GRID_Pin);
         State_Control = STOP_WITH_DELAY;
         Diagnostic_Control=BUS_OVERVOLTAGE;
        }
     if (BusFiltered < 5500 && MPPT_EN==TRUE) //nowe nastawy 340V
        {
         Fault = BUS_UNDERVOLTAGE;
         //LL_GPIO_ResetOutputPin(RELAY_GRID_GPIO_Port, RELAY_GRID_Pin);
         State_Control = STOP_WITH_DELAY;
         Diagnostic_Control=BUS_UNDERVOLTAGE;
        }

    //COMP1 ->CSR |= COMP_CSR_EN;

    CalcAndSetACComponents(State_Control);

    /*

	  if(OC_PROT_ON < REL_ON_TICK+27000) //  stabilize relay
		  {
			  OC_PROT_ON++;
		  }
	  if(OC_PROT_ON == REL_ON_TICK+27000)
		  {

		      COMP7 ->CSR |= COMP_CSR_EN;//Glitch test

		      //TIM1->AF1 |= TIM1_AF1_BKINE; //ENABLE

			  OC_PROT_ON = REL_ON_TICK+1+27000;
		  }
	  */

    if ( MPPT_EN==TRUE)

    {

		 // if (ii<8000)
		 // {
			  //P_ACTIVE_BUFFER[ii] = Inverter_q_d.qI_Direct;
			  //P_REACTIVE_BUFFER[ii] = Inverter_q_d.qI_Quadrature;
			  //GRID_IAC_BUFFER[ii] = DataSensingIO.AC_LineCurrent;
			  //ii++;
		 // }
/*
		  else if (ii==8000)
		  {
////////////////////////////////////////////////////////////////////////////////////////////////////////////
			  fres = f_open(&fil, "P_Act.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
			  if(fres == FR_OK) {
				//myprintf("I was able to open 'write.txt' for writing\r\n");
			  } else {
				while(1);
			  }


			  for(int u=0;u<8000;u++)
			  {
				  UINT bytesWrote;
				  char strNumber[6];
				  sprintf(strNumber,"%u\n", P_ACTIVE_BUFFER[u]);
				  fres = f_write(&fil, strNumber, sizeof(strNumber), &bytesWrote);
				  if(fres == FR_OK) {
					//myprintf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
				  } else {
					  while(1);
				  }
			  }


			   f_close(&fil);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

				  fres = f_open(&fil, "Q_React.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
				  if(fres == FR_OK) {
					//myprintf("I was able to open 'write.txt' for writing\r\n");
				  } else {
					while(1);
				  }


				  for(int u=0;u<8000;u++)
				  {
					  UINT bytesWrote;
					  char strNumber[6];
					  sprintf(strNumber,"%u\n", P_REACTIVE_BUFFER[u]);
					  fres = f_write(&fil, strNumber, sizeof(strNumber), &bytesWrote);
					  if(fres == FR_OK) {
						//myprintf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
					  } else {
						  while(1);
					  }
				  }


				   f_close(&fil);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
					  fres = f_open(&fil, "IACBUF.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
					  if(fres == FR_OK) {
						//myprintf("I was able to open 'write.txt' for writing\r\n");
					  } else {
						while(1);
					  }


					  for(int u=0;u<8000;u++)
					  {
						  UINT bytesWrote;
						  char strNumber[6];
						  sprintf(strNumber,"%u\n", GRID_IAC_BUFFER[u]);
						  fres = f_write(&fil, strNumber, sizeof(strNumber), &bytesWrote);
						  if(fres == FR_OK) {
							//myprintf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
						  } else {
							  while(1);
						  }
					  }


					   f_close(&fil);
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////


			   fres = f_mount(NULL, "/", 1);

		  */  //ii++;
		 //}
    }
    //buffer_set++;
	 // }



    break;

  case STOP:

      DCAC_SendCommand(DCAC_Stop);
      DS_SendCommand(DS_Stop);
      MPPT_EN=FALSE;


      if(Diagnostic_Control==PV_VOLTAGE_MIN)
      { State_Control= PV_VOLTAGE_MIN; }
      if(Diagnostic_Control==BUS_OVERVOLTAGE)
      { State_Control= BUS_OVERVOLTAGE; }
      if(Diagnostic_Control==BUS_UNDERVOLTAGE)
      { State_Control= BUS_UNDERVOLTAGE; }
      if(Diagnostic_Control==GRID_VOLTAGE_OUT_OF_RANGE)
      { State_Control= GRID_VOLTAGE_OUT_OF_RANGE; }
      if(Diagnostic_Control==FREQ_OUT_OF_RANGE)
      { State_Control= FREQ_OUT_OF_RANGE; }
      if(Diagnostic_Control==OUT_CURRENT_LIMIT)
      { State_Control= OUT_CURRENT_LIMIT; }
      if(Diagnostic_Control==PV_VOLTAGE_DVDT)
      { State_Control= PV_VOLTAGE_DVDT; }
//      if(Diagnostic_Control==LOM)
//      { State_Control= LOM; }

      //LL_ADC_Disable(ADC1);
      //LL_ADC_Disable(ADC2);

      //DMA1_Channel5->CCR &= ~DMA_CCR_TCIE; //ENABLE TRANSFER COMPLETE INTERRUPT DIS
    //  CallRefreshDisplay();
      //InitControl(ClosedLoop);
      //__INLINE void NVIC_SystemReset(void);

	    StartControl();
	    //SetBusDCReference(DataSensingIO.DC_BusVoltage);
    break;

  //case BUS_UNDERVOLTAGE:
//	  State_Control = DIAGNOSTIC_AC_LINE;
 // break;

  case STOPPING:

    DCDC_SendCommand(DCDC_ConverterStop);
    //DCAC_SetPulse(2047,2047);
   // DCAC_SetPulse(DCAC_COUNTER,DCAC_COUNTER);
    /*PORT A.1 drives the RELE' Open */
    LL_GPIO_ResetOutputPin(RELAY_GRID_GPIO_Port, RELAY_GRID_Pin);  //mod Rele'

    State_Control = STOP;

    //CalcAndSetACComponents(State_Control);
    break;
  case BUS_FAULT:

    //stop the DCDC
    DCDC_SendCommand(DCDC_ConverterStop);

    //TIM20->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_0); //TOP LF MOSFET HIGH / BOTTOM LF MOSFET LOW

    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].SETx1R |= (1 << 0);

    LL_GPIO_ResetOutputPin(RELAY_GRID_GPIO_Port, RELAY_GRID_Pin);  //mod Rele'
    State_Control = STOPPING;
  //CalcAndSetACComponents(State_Control);

    break;

  default:
//       if(State_Control==LOM)
//       {}
//       else
//       {
//       GPIO_ResetBits(GPIOA, GPIO_Pin_1);
//       State_Control = STOP_WITH_DELAY;
//       Diagnostic_Control = LOM;
//       }
    break;
  }

  //GPIOG->BRR = (1<<9);
} // void

void CalcAndSetACComponents(SystStatus_t state)
{

    Quadrature_Current_PID.Reference = 26000;//(PID_Bus_Voltage(&BUS_Voltage_PID,Bus_Voltage));

    Direct_Current_PID.Reference = (PID_Reactive_Power(&Reactive_Power_PID, Actual_QD_Power.Q_Reactive));

    Output_qId_Inverter = (s16)(PID_DirectCurrent(&Direct_Current_PID, ((Inverter_q_d.qI_Direct))));

    Output_qIq_Inverter = ((s16)(PID_QuadratureCurrent(&Quadrature_Current_PID, ((Inverter_q_d.qI_Quadrature)))));

    CrossDecoupling_Control();

    RevPark_Circle_Limitation();

    Control_Volt_AlphaBeta= Rev_Park(Output_qIq_Inverter,Output_qId_Inverter);

    //if(State_Control!=DIAGNOSTIC_DC_LINE && State_Control!=DIAGNOSTIC_AC_LINE && State_Control!=BUSPRECHARGE)
    if(State_Control == GRID_INSERTION)

    {
		if(Control_Volt_AlphaBeta.qValpha <= 0) //50103 303
		{
			  polarity=TRUE;
			  LL_GPIO_SetOutputPin(SD_CS_GPIO_Port, SD_CS_Pin);
			  Pulse1 = (((u16)(new_mul_q15_q15_q31(-Control_Volt_AlphaBeta.qValpha, MODINDEX) >> 16)));
			  DCAC_SetPulse(Pulse1,Pulse1);

		      if(Pulse1<=500)
		      {
				  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].DTxR = 0x640464;
				  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].DTxR = 0x640464;
				  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_F].DTxR = 0x640464;
		      }

			  if((LF_MOS_SET == FALSE) || (FIRST_CYCLE == TRUE))
			  {
				  RESET_TIMER_D = FALSE;
				  //LF_MOS_SET = TRUE;
				  FIRST_CYCLE = FALSE;
				  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_F].CMP2xR = (uint32_t)(16);

				  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].SETx1R |= 1 << 19;


				  HRTIM1_COMMON->CR2 |= (0x50 << 0); //UPDATE TIM F AND D SIMULT

				  LF_UPDATE = TRUE;

			  }
			  else
			  {

				  if (LF_UPDATE == TRUE)
				  {
					  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].SETx1R &= ~ (1 << 19);
					  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].SETx1R |= (1 << 0);


					  LF_UPDATE = FALSE;
					  HRTIM1_COMMON->CR2 |= (0x50 << 0); //UPDATE TIM F AND D SIMULT
				  }

			  }
			  LF_MOS_SET = TRUE;


		 }
       else if(Control_Volt_AlphaBeta.qValpha > 0)
        {

    	      polarity=FALSE;
    	      LL_GPIO_ResetOutputPin(SD_CS_GPIO_Port, SD_CS_Pin);
			  Pulse2 = ((u16)(new_mul_q15_q15_q31((s16)(0x8000 - Control_Volt_AlphaBeta.qValpha ), MODINDEX) >> 16));
			  DCAC_SetPulse(Pulse2,Pulse2);

		      if(Pulse2>=24500 && Pulse2<26000)
		      {
				  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].DTxR = 0x640464;
				  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].DTxR = 0x640464;
				  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_F].DTxR = 0x640464;
		      }

    		  if((LF_MOS_SET == TRUE) || FIRST_CYCLE == TRUE)
    		  {
    			  RESET_TIMER_D = FALSE;
    			  LF_MOS_SET = FALSE;
    			  FIRST_CYCLE = FALSE;

    			  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_F].CMP2xR = (uint32_t)(60);

    			  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].SETx1R |= 1 << 19;

    			  HRTIM1_COMMON->CR2 |= (0x50 << 0); //UPDATE TIM F AND D SIMULT
    			  LF_UPDATE = TRUE;
    		  }
    		  else
    		  {
    			  if (LF_UPDATE == TRUE)
    			  {

    				  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].SETx1R &= ~ (1 << 19);

    				  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].RSTx1R |= (1 << 0);

    				  LF_UPDATE = FALSE;



    				  HRTIM1_COMMON->CR2 |= (0x50 << 0); //UPDATE TIM F AND D SIMULT
    			  }

    		  }

       }

		//if(polarity == FALSE)
		//{
			//GPIOG->BRR = (1<<9);
		//}
		//else if (polarity == TRUE)
		//{
			//GPIOG->BSRR = (1<<9);
		//}
    }

// ********************* commentata per debug open loop *********************
  //  DAC_SetChannel(DAC_CH_1,(u16) Theta +0x8000);
// *************************************************************************
}

void HAL_HRTIM_Fault1Callback(HRTIM_HandleTypeDef * hhrtim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hhrtim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_HRTIM_Fault1Callback could be implemented in the user file
   */
}

