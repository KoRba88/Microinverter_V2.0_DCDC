/*
 * DCAC_Inverter.c
 *
 *  Created on: Aug 6, 2020
 *      Author: user01
 */
/* Includes ------------------------------------------------------------------*/
#include "DCAC_Inverter.h"
//#include "lcd.h"
//#include "color_lcd.h"
#include "stdio.h"
#include "400WControl.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//#define CLOCK_DURATION_MICROS    0.01388888

//Timer 8 up-down counter
#define TIM_PERIOD         DCAC_COUNTER //((LocalDevice.Init.Counter) >> 1)

#define TIM_DEAD_TIME       LocalDevice.Init.DeadTime
#define TIM8_DEAD_TIME      DCAC_DeadTimeToReg(TIM_DEAD_TIME)

#define TIM_MIN_PULSE      ((0) + (TIM_DEAD_TIME << 1))
#define TIM_MAX_PULSE      (TIM_PERIOD - (TIM_DEAD_TIME << 1))
#define TIM_DEFAULT_PULSE  (TIM_PERIOD >> 1)

#define DCAC_MAX_PULSE TIM_MAX_PULSE
#define DCAC_MIN_PULSE TIM_MIN_PULSE

#define TIM_MIN_DEAD_TIME   (1)
#define TIM_MAX_DEAD_TIME   TIM_MAX_PULSE - TIM_MIN_DEAD_TIME


/* Private macro -------------------------------------------------------------*/
#define LSELLEN_DCAC(x, y) (((x)==(y))? 20 : 0)

/* Private variables ---------------------------------------------------------*/
static struct DCAC_Device_s
{
    DCAC_TypeDef_t Init;
    DCAC_Status_t State;
} LocalDevice;

//static TIM_OCInitTypeDef  TIM_OCInitStructure;
//static TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
//static GPIO_InitTypeDef GPIO_InitStructure;


/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void DCAC_TIMx_Configuration(void);
static void DCAC_GPIO_Configuration(void);
static u32 DCAC_DeadTimeToReg(u32 deadTime);
u32 DCAC_RegToDeadTime(u32 deadTime);

extern volatile u32 TIMF_DMA_Buffer[1];
/*******************************************************************************
* Function Name  : DCAC_Init
* Description    : Configures Peripherals used by the DCAC Converter
* Input          : PDCAC_TypeDef_t pointer to a structure
* Return         : errors on initialization
*******************************************************************************/
DCAC_Error_t DCAC_Init(PDCAC_TypeDef_t pDCACInit)
{
    DCAC_Error_t nRet;

    nRet = DCAC_ERROR_INVALID_PARAMETER;

    if (pDCACInit)
    {
        nRet = DCAC_ERROR_ON_INIT;
        if (LocalDevice.State != DCAC_Running)
        {
            LocalDevice.Init.Counter  = pDCACInit->Counter;
            LocalDevice.Init.DeadTime = pDCACInit->DeadTime;
            LocalDevice.State = DCAC_Stopped;

            /*Init peripherals */
            DCAC_GPIO_Configuration();
            DCAC_TIMx_Configuration();

            DCAC_SetDeadTime(LocalDevice.Init.DeadTime);

            nRet = DCAC_ERROR_NONE;
        }
    }

    return nRet;
}
/*******************************************************************************
* Function Name  : DCAC_Init
* Description    : Configures Peripherals used by the DCAC Converter
* Input          : PDCAC_TypeDef_t pointer to a structure
* Return         : errors on initialization
*******************************************************************************/
DCAC_Error_t DCAC_SendCommand(DCAC_Commands_t cmd)
{
    DCAC_Error_t nRet;

    nRet = DCAC_ERROR_INVALID_COMMAND;

    if (cmd == DCAC_Start)
    {
       nRet = DCAC_ERROR_ON_SEND_COMMAND;
       if (DCAC_GetStatus() != DCAC_Running)
       {
           LocalDevice.State = DCAC_Running;

           //TIM20->BDTR |= TIM_BDTR_MOE; //ENABLE ALL OUTPUTS - LF 50 Hz MOS


           HRTIM1_COMMON->OENR |=HRTIM_OENR_TD2OEN; //TIMD OUTPUT 2 ENABLE
           HRTIM1_COMMON->OENR |=HRTIM_OENR_TD1OEN; //TIMD OUTPUT 1 ENABLE

           HRTIM1_COMMON->OENR |=HRTIM_OENR_TF2OEN; //TIMF OUTPUT 2 ENABLE
           HRTIM1_COMMON->OENR |=HRTIM_OENR_TF1OEN; //TIMF OUTPUT 1 ENABLE


          nRet = DCAC_ERROR_NONE;
       }
    }
    else
    if (cmd == DCAC_Stop)
    {
       nRet = DCAC_ERROR_ON_SEND_COMMAND;
       if (DCAC_GetStatus() != DCAC_Stopped)
       {
           LocalDevice.State = DCAC_Stopped;

           //TIM20->BDTR &= ~(TIM_BDTR_MOE); //DISABLE ALL OUTPUTS

           HRTIM1_COMMON->ODISR |=HRTIM_ODISR_TD1ODIS; //TIMD OUTPUT 1 DISABLE
           HRTIM1_COMMON->ODISR |=HRTIM_ODISR_TD2ODIS; //TIMD OUTPUT 2 DISABLE

           HRTIM1_COMMON->ODISR |=HRTIM_ODISR_TF1ODIS; //TIMF OUTPUT 1 DISABLE
           HRTIM1_COMMON->ODISR |=HRTIM_ODISR_TF2ODIS; //TIMF OUTPUT 2 DISABLE



          nRet = DCAC_ERROR_NONE;
       }
    }

   return nRet;
}

/*******************************************************************************
* Function Name  : DCAC_SetPulse
* Description    : Set the pulse of the pwm for  the voltage and current
* Input          : The oc for voltage and the oc for current
* Return         : Error code
*******************************************************************************/
uint16_t Pulse_Buffer[1000] = {0};
uint16_t iii = 0;
DCAC_Error_t DCAC_SetPulse(u32 PulseChannel1, u32 PulseChannel2)
{
    DCAC_Error_t nRet;
//    nRet = DCAC_ERROR_INVALID_PARAMETER;

//    if (PulseCurrent < DCAC_MAX_PULSE && PulseCurrent > DCAC_MIN_PULSE &&
//        PulseVoltage < DCAC_MAX_PULSE && PulseVoltage > DCAC_MIN_PULSE )
//    {
      if(PulseChannel1>=DCAC_MAX_PULSE) PulseChannel1=DCAC_MAX_PULSE; //-110
      if(PulseChannel1<=DCAC_MIN_PULSE) PulseChannel1=DCAC_MIN_PULSE;
     //  if(PulseChannel1>=TIMF_PERIOD*0.97) PulseChannel1=TIMF_PERIOD*0.97;

      // if(PulseChannel1<=100) PulseChannel1=100;
//
      //if(PulseChannel1<=DCAC_MIN_PULSE) PulseChannel1=DCAC_MIN_PULSE;


       if(iii <= 1000)
       {
    	   Pulse_Buffer[iii] = PulseChannel1;
    	   iii++;
       }




//    if(PulseChannel2<=204) PulseChannel2=204;

//    if(PulseChannel1>=2047) PulseChannel1=2047;
//    if(PulseChannel2>=2047) PulseChannel2=2047;
//
    //if(PulseChannel1<=1) PulseChannel1=1;
//    if(PulseChannel2<=1) PulseChannel2=1;

     // TO BE DONE !!!!!!!!!!!!!!!

      HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_F].CMP1xR = (uint32_t)(PulseChannel1); //FROM CMP4

      HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = (uint32_t)(PulseChannel1);



        nRet = DCAC_ERROR_NONE;
//    }

    return nRet;

}

/*******************************************************************************
* Function Name  : DCAC_GetStatus
* Description    : Get DCAC status
* Input          : NONE
* Return         : Actual State of the device
*******************************************************************************/
DCAC_Status_t DCAC_GetStatus()
{
    return LocalDevice.State;
}

/*******************************************************************************
* Function Name  : DCAC_GetConfiguration
* Description    : Get DCAC Configuration
* Input          : Pointer to a allocated structure where will be copied the
*                  configuration values
* Return         : 0 no error, non 0 if error
*******************************************************************************/
DCAC_Error_t DCAC_GetConfiguration(PDCAC_TypeDef_t pDCACInit)
{
    DCAC_Error_t nRet;
    nRet = DCAC_ERROR_INVALID_PARAMETER;

    if (pDCACInit)
    {
        pDCACInit->Counter = LocalDevice.Init.Counter;
        pDCACInit->DeadTime = LocalDevice.Init.DeadTime;
        nRet = DCAC_ERROR_NONE;
    }

    return nRet;
}


/*******************************************************************************
* Function Name  : DCAC_GetDeadTime
* Description    : Get the DeadTime for the DCAC converter
* Input          : NONE
* Return         : Actual deadtime
*******************************************************************************/
u32 DCAC_GetDeadTime()
{
    return LocalDevice.Init.DeadTime;
}

/*******************************************************************************
* Function Name  : DCAC_SetDeadTime
* Description    : Set the DeadTime for the DCAC converter
* Input          : dwDeadTime the new deadtime
* Return         : Error on function 0 no error.
*******************************************************************************/
DCAC_Error_t DCAC_SetDeadTime(u32 dwDeadTime)
{
    DCAC_Error_t nRet;
    nRet = DCAC_ERROR_ON_SET_DEADTIME;

    if (LocalDevice.State == DCAC_Stopped)
    {
        nRet = DCAC_ERROR_INVALID_PARAMETER;
        if(dwDeadTime < TIM_MAX_DEAD_TIME && dwDeadTime > TIM_MIN_DEAD_TIME )
        {
            LocalDevice.Init.DeadTime = dwDeadTime;
//            TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Disable;
//            TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Disable;
//            TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
//            TIM_BDTRInitStructure.TIM_DeadTime = DCAC_DeadTimeToReg(LocalDevice.Init.DeadTime);
//            TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
//            TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
//            TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
//            TIM_BDTRConfig(TIM8, &TIM_BDTRInitStructure);

            nRet = DCAC_ERROR_NONE;
        }
    }

     return nRet;

}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports DCAC timers outputs.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DCAC_GPIO_Configuration(void)
{
    /* Configure TIM8CH2N:PB0-----------------------------------*/
    /* Configure TIM8CH3N:PB1-----------------------------------*/
	/*
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  //  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0|GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
*/
    /* Configure TIM8CH2:PC7-----------------------------------*/
    /* Configure TIM8CH3:PC8-----------------------------------*/
	/*
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7|GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
*/
}

/*******************************************************************************
* Function Name  : TIMx_Configuration
* Description    : Timers configuration for DCAC converter
* Input          : NONE
* Return         : None
*******************************************************************************/
void DCAC_TIMx_Configuration(void)
{
/*
 Others configuration for tim8 on DataSensing
*/
	/*
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_Pulse = TIM_DEFAULT_PULSE;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OC2Init(TIM8, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_Pulse = TIM_DEFAULT_PULSE;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OC3Init(TIM8, &TIM_OCInitStructure);
*/

	  TIM20->CCER |= TIM_CCER_CC2E; ////ENABLE CHANNEL 2
	  TIM20->CCER |= TIM_CCER_CC2NE; ////ENABLE CHANNEL 2N
	  TIM20->CR1  |= (TIM_CR1_ARPE | TIM_CR1_CEN); // ENABLE AUTORELOAD PRELOAD AND TIMER COUNTER


}


/*
DTG[7:5]=0xx => DT=DTG[7:0]x tdtg with tdtg=tDTS.
DTG[7:5]=10x => DT=(64+DTG[5:0])xtdtg with Tdtg=2xtDTS.
DTG[7:5]=110 => DT=(32+DTG[4:0])xtdtg with Tdtg=8xtDTS.
DTG[7:5]=111 => DT=(32+DTG[4:0])xtdtg with Tdtg=16xtDTS.
*/
/*******************************************************************************
* Function Name  : DCAC_DeadTimeToReg
* Description    : Convert DeadTime in clocks to register value
* Input          : deadTime numberof clocks
* Return         : Register value for timer
*******************************************************************************/
static u32 DCAC_DeadTimeToReg(u32 deadTime)
{
    u32 nRet;

    nRet = 0;
    switch(deadTime & 0x0380)
    {
    case 0x0000: //no Prescaler
        nRet = deadTime & 0x7F;
        break;
    case 0x0080:
        nRet = ((((deadTime)>>1)-64) & 0x3F) | 0x80;
        break;
    case 0x0100:
    case 0x0180:
        nRet = ((((deadTime)>>3)-32) & 0x1F) | 0xC0;
        break;
    case 0x0200:
    case 0x0280:
    case 0x0300:
    case 0x0380:
        nRet = ((((deadTime)>>4)-32) & 0x1F) | 0xE0;
        break;
    default:
        nRet = 0x00;
    }
//    if ( deadTime != DCAC_RegToDeadTime(nRet))
//        nRet = 0x00;

    return nRet;
}
/*
DTG[7:5]=0xx => DT=DTG[7:0]x tdtg with tdtg=tDTS.
DTG[7:5]=10x => DT=(64+DTG[5:0])xtdtg with Tdtg=2xtDTS.
DTG[7:5]=110 => DT=(32+DTG[4:0])xtdtg with Tdtg=8xtDTS.
DTG[7:5]=111 => DT=(32+DTG[4:0])xtdtg with Tdtg=16xtDTS.
*/
/*******************************************************************************
* Function Name  : DCAC_RegToDeadTime
* Description    : Convert register value to DeadTime in clocks
* Input          : Register value for timer
* Return         : deadTime numberof clocks
*******************************************************************************/
u32 DCAC_RegToDeadTime(u32 regDead)
{
    u32 nRet;

    nRet = 0;
    switch(regDead & 0xE0)
    {
    case 0x00: //no Prescaler
    case 0x20: //no Prescaler
    case 0x40: //no Prescaler
    case 0x60: //no Prescaler
        nRet = regDead & 0x7F;                     /*0 - 127 Step(1)*/
        break;
    case 0x80:
    case 0xA0:
        nRet = ((regDead & 0x3F) + 0x40) * 0x02;   /*128 - 254 Step(2)*/
        break;
    case 0xC0:
        nRet = ((regDead & 0x1F) + 0x20) * 0x08;   /*256 - 504 Step(8)*/
        break;
    case 0xE0:
        nRet = ((regDead & 0x1F) + 0x20) * 0x10;   /*256 - 504 Step(16)*/
        break;
    default:
        nRet = 0x00;
    }

    return nRet;
}


/*******************************************************************************
* Function Name  : DCAC_RefreshDisplay
* Description    : Display same info from the object in display
* Input          : Error and step of phase shift
* Return         : None
*******************************************************************************/
/*
void DCAC_RefreshDisplay(DCAC_Error_t err, u32 steps, u32 PulseCurrent, u32 Changes)
{
    char strLineMessage[64];

    LCD_DisplayStringLine(Line0, "   DCAC Inverter    ");

    if (DCAC_GetStatus() == DCAC_Running)
        sprintf(strLineMessage, "ON  %d (%.2f kHz)", DCAC_COUNTER, 1/(((double)(DCAC_COUNTER) * CLOCK_DURATION_MICROS))*1000);
    else
        sprintf(strLineMessage, "OFF %d (%.2f kHz)", DCAC_COUNTER, 1/(((double)(DCAC_COUNTER) * CLOCK_DURATION_MICROS))*1000);
    LCD_ClearLine(Line1);
    LCD_DisplayStringLine(Line1, (u8 *)strLineMessage);

    sprintf(strLineMessage, "DeadTime %d(%.2fus)", DCAC_GetDeadTime(), ((double)(DCAC_GetDeadTime()) * CLOCK_DURATION_MICROS));
    LCD_ClearLine(Line2);
    LCD_DisplayStringLineEx(Line2, 0, LSELLEN_DCAC(Changes, 0), (u8 *)strLineMessage);

    sprintf(strLineMessage, "Amp %d (%.2fv)", GetAmplitude(), ((double)(GetAmplitude() *3.3) / S16_MAX ) );
    LCD_ClearLine(Line3);
    LCD_DisplayStringLineEx(Line3, 0, LSELLEN_DCAC(Changes, 1), (u8 *)strLineMessage);


    sprintf(strLineMessage, "Steps %d(%.3fus)", steps, ((double)(steps) * CLOCK_DURATION_MICROS));
    LCD_ClearLine(Line4);
    LCD_DisplayStringLineEx(Line4, 0, LSELLEN_DCAC(Changes, 2), (u8 *)strLineMessage);

    LCD_ClearLine(Line5);
    LCD_ClearLine(Line6);
    LCD_ClearLine(Line7);

    sprintf(strLineMessage, "Error 0x%.8X    ", err);
    LCD_ClearLine(Line8);
    LCD_DisplayStringLine(Line8, (u8 *)strLineMessage);

    LCD_ClearLine(Line9);
}
*/
