/*
 * DCDC_Converter.c
 *
 *  Created on: Aug 6, 2020
 *      Author: user01
 */

/* Includes ------------------------------------------------------------------*/
#include "DCDC_Converter.h"
#include "Solar_MPPT.h"
//#include "lcd.h"
//#include "color_lcd.h"
#include "stdio.h"
#include "main.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//#define CLOCK_DURATION_MICROS    0.01388888

//#define TIM_PERIOD          LocalDevice.Init.Counter
//#define TIM1_MAX_PULSE      (TIM1_PERIOD >> 1)
//Timer 2 up-down counter for deadTime manager
//#define TIM2_PERIOD         (TIM1_PERIOD >> 1)
//#define TIM2_MAX_PULSE      (TIM1_MAX_PULSE >> 1)

//#define TIM2_PERIOD         (TIM_PERIOD >> 1) // DIVIDED BY 2 CENTER ALIGNED MODE
//#define TIM2_INIT_PULSE     (TIM2_PERIOD >> 1)// DIVIDED BY 2 50% TIM2 PERIOD

//#define TIM3_PERIOD         (TIM_PERIOD >> 1) // DIVIDED BY 2 CENTER ALIGNED MODE
//#define TIM3_INIT_PULSE      (TIM3_PERIOD >> 1) // DIVIDED BY 2 50% TIM2 PERIOD

/* Formula below works down to 70.3kHz (with presc ratio = 1) */
//#define MASTER_PERIOD ((uint16_t)(((uint64_t)HRTIM_INPUT_CLOCK * HRTIMER_LLC_PRESCALER) / MASTER_PWM_FREQ))
//#define TIMA_PERIOD ((uint16_t)(((uint64_t)HRTIM_INPUT_CLOCK * HRTIMER_LLC_PRESCALER) / TIMA_PWM_FREQ))
//#define TIMB_PERIOD ((uint16_t)(((uint64_t)HRTIM_INPUT_CLOCK * HRTIMER_LLC_PRESCALER) / TIMB_PWM_FREQ))
//#define ISO_DCDC_PERIOD ((uint16_t)((uint64_t)TIM8_INPUT_CLOCK/2 / ISO_DCDC_PWM_FREQ-1))

//#define HRTIMER_LLC_PRESCALER 32


//150 = 2.1us = 139 dead time register
//256 = 3.55us = 192 dead time register
//#define TIM_DEAD_TIME       LocalDevice.Init.DeadTime
//#define TIM1_DEAD_TIME      DCDC_DeadTimeToReg(TIM_DEAD_TIME)
//#define TIM2_DEAD_TIME      (TIM_DEAD_TIME >> 1)
//
//#define TIM_MIN_DEAD_TIME   (1)
//#define TIM_MAX_DEAD_TIME   TIM2_MAX_PULSE - TIM_MIN_DEAD_TIME
//
//#define TIM_MIN_SHIFT       (0)
//#define TIM_MAX_SHIFT       (TIM_MIN_SHIFT + TIM1_MAX_PULSE - TIM_DEAD_TIME - 4)

extern HRTIM_HandleTypeDef hhrtim1;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static struct DCDC_Device_s
{
    DCDC_TypeDef_t Init;
    DCDC_Status_t State;
} LocalDevice;

//static TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//static TIM_OCInitTypeDef  TIM_OCInitStructure;
//static TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void DCDC_TIMx_Configuration(void);
static void DCDC_TIMx_Configuration_startup(void);
static void DCDC_GPIO_Configuration(void);
static u32 DCDC_DeadTimeToReg(u32 deadTime);
u32 DCDC_RegToDeadTime(u32 deadTime);
static GPIO_InitTypeDef GPIO_InitStructure;
//static void MX_HRTIM1_Init(void);

u32 DCDC_GetFrequency();
u32 DCDC_GetPhaseShift();
u32 DCDC_GetPeriod();

u32 dutyadj;
u32 frequencyinc;
u32 NEW_DCDC_PERIOD;
u32 NEW_isoDCDC_PERDIOD;

volatile u32 TIMD_DMA_Buffer[1] = {0x0}; //{(uint32_t)TIMF_PERIOD};
volatile u32 TIMF_DMA_Buffer[1] = {0x10};
/*******************************************************************************
* Function Name  : HRTIM_TIMD_DMA
* Description    : Start 50Hz HRTIM from DMA
* Input          :
* Return         :
*******************************************************************************/
void HRTIM_TIMD_DMA_START()
{
	//HAL_HRTIM_SimplePWMStart_DMA(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_OUTPUT_TD1, (uint32_t)&TIMD_DMA_Buffer[0], (uint32_t)&(HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].PERxR), 1);
	HAL_HRTIM_WaveformCountStart_DMA(&hhrtim1, HRTIM_TIMERID_TIMER_D);

}

void HRTIM_TIMD_DMA_STOP()
{
	//HAL_HRTIM_SimplePWMStop(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_OUTPUT_TD1);
}

void HRTIM_TIMF_DMA_START()
{
	//HAL_HRTIM_SimplePWMStart_DMA(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_OUTPUT_TF1, (uint32_t)&TIMF_DMA_Buffer[0], (uint32_t)&(HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR), 1);
	HAL_HRTIM_WaveformCountStart_DMA(&hhrtim1, HRTIM_TIMERID_TIMER_F);
}

/*******************************************************************************
* Function Name  : DCDC_Init
* Description    : Configures Peripherals used by the DCDC Converter
* Input          : PDCDC_TypeDef_t pointer to a structure
* Return         : errors on initialization
*******************************************************************************/
DCDC_Error_t DCDC_Init(PDCDC_TypeDef_t pDCDCInit)
{
    DCDC_Error_t nRet;

    nRet = DCDC_ERROR_INVALID_PARAMETER;

    if (pDCDCInit)
    {
        nRet = DCDC_ERROR_ON_INIT;
        if (LocalDevice.State != DCDC_Running)
        {
            LocalDevice.Init.Counter = pDCDCInit->Counter;
            LocalDevice.Init.DutyCycle = pDCDCInit->DutyCycle;
            LocalDevice.Init.frequency = pDCDCInit->frequency;
            LocalDevice.Init.PhaseShift = pDCDCInit->PhaseShift;
            LocalDevice.State = DCDC_Stopped;

            /*Init peripherals */
            DCDC_GPIO_Configuration();

            DCDC_TIMx_Configuration();

      //      DCDC_TIMx_Configuration_startup();

   //         DCDC_SetPhaseShift(LocalDevice.Init.PhaseShift);

   //         DCDC_SetDeadTime(LocalDevice.Init.DeadTime);

            nRet = DCDC_ERROR_NONE;
        }
    }

    return nRet;
}
/*******************************************************************************
* Function Name  : DCDC_Init
* Description    : Configures Peripherals used by the DCDC Converter
* Input          : PDCDC_TypeDef_t pointer to a structure
* Return         : errors on initialization
*******************************************************************************/
DCDC_Error_t DCDC_SendCommand(DCDC_Commands_t cmd)
{
    DCDC_Error_t nRet;

    nRet = DCDC_ERROR_INVALID_COMMAND;

    if (cmd == DCDC_ConverterStart)
    {
       nRet = DCDC_ERROR_ON_SEND_COMMAND;
       if (DCDC_GetStatus() != DCDC_Running || (HRTIM1_COMMON->ODSR & HRTIM_ODSR_TA1ODS) || (HRTIM1_COMMON->ODSR & HRTIM_ODSR_TB1ODS))
      {
           LocalDevice.State = DCDC_Running;

       	  uint32_t isrflags = HRTIM1->sCommonRegs.ISR;
       	  uint32_t ierits   = HRTIM1->sCommonRegs.IER;
       	  if((uint32_t)(isrflags & HRTIM_FLAG_FLT4) != (uint32_t)RESET)
       	  {
       	    if((uint32_t)(ierits & HRTIM_IT_FLT4) != (uint32_t)RESET)
       	    {
       	    	HRTIM1->sCommonRegs.ICR = HRTIM_ICR_FLT4C;
       	    	//DCDC_TIMx_Configuration(); //added to riconfigure the timer for DCDC
       	    }
       	  }

       	  //TESTY HRTIM TIMD 50Hz

          //  HRTIM1_COMMON->OENR |=HRTIM_OENR_TA1OEN; //TIMA OUTPUT 1 ENABLE
          //  HRTIM1_COMMON->OENR |=HRTIM_OENR_TA2OEN; //TIMA OUTPUT 2 ENABLE
          //  HRTIM1_COMMON->OENR |=HRTIM_OENR_TB1OEN; //TIMB OUTPUT 1 ENABLE
          //  HRTIM1_COMMON->OENR |=HRTIM_OENR_TB2OEN; //TIMB OUTPUT 2 ENABLE




            nRet = DCDC_ERROR_NONE;
       }
    }
    else
    if (cmd == DCDC_ConverterStop)
    {
       nRet = DCDC_ERROR_ON_SEND_COMMAND;
       if (DCDC_GetStatus() != DCDC_Stopped)
       {

         LocalDevice.State = DCDC_Stopped;


         HRTIM1_COMMON->ODISR |=HRTIM_ODISR_TA1ODIS; //TIMA OUTPUT 1 DISABLE
         HRTIM1_COMMON->ODISR |=HRTIM_ODISR_TA2ODIS; //TIMA OUTPUT 2 DISABLE
         HRTIM1_COMMON->ODISR |=HRTIM_ODISR_TB1ODIS; //TIMB OUTPUT 1 DISABLE
         HRTIM1_COMMON->ODISR |=HRTIM_ODISR_TB2ODIS; //TIMB OUTPUT 2 DISABLE

         //HRTIM1->sCommonRegs.ODISR = (HRTIM_ODISR_TA1ODIS + HRTIM_ODISR_TA2ODIS + HRTIM_ODISR_TB1ODIS + HRTIM_ODISR_TB2ODIS);

          //DCDC_TIMx_Configuration(); //added to riconfigure the timer for DCDC
          nRet = DCDC_ERROR_NONE;
       }
    }

   return nRet;
}

/*******************************************************************************
* Function Name  : DCDC_GetStatus
* Description    : Get DCDC status
* Input          : NONE
* Return         : Actual State of the device
*******************************************************************************/
DCDC_Status_t DCDC_GetStatus()
{
    return LocalDevice.State;
}

/*******************************************************************************
* Function Name  : DCDC_GetConfiguration
* Description    : Get DCDC Configuration
* Input          : Pointer to a allocated structure where will be copied the
*                  configuration values
* Return         : 0 no error, non 0 if error
*******************************************************************************/
DCDC_Error_t DCDC_GetConfiguration(PDCDC_TypeDef_t pDCDCInit)
{
    DCDC_Error_t nRet;
    nRet = DCDC_ERROR_INVALID_PARAMETER;

    if (pDCDCInit)
    {
        pDCDCInit->Counter = LocalDevice.Init.Counter;
        pDCDCInit->PhaseShift = LocalDevice.Init.PhaseShift;
        pDCDCInit->DeadTime = LocalDevice.Init.DeadTime;
        pDCDCInit->DutyCycle = LocalDevice.Init.DutyCycle;
        pDCDCInit->frequency = LocalDevice.Init.frequency;
        nRet = DCDC_ERROR_NONE;
    }

    return nRet;
}

/*******************************************************************************
* Function Name  : DCDC_GetPeriod
* Description    : Get DCDC actual period
* Input          : NONE
* Return         : the period
*******************************************************************************/
u32 DCDC_GetPeriod()
{
    return LocalDevice.Init.Counter;
}

/*******************************************************************************
* Function Name  : DCDC_GetDuty Cycle
* Description    : Get DCDC actual duty cycle
* Input          : NONE
* Return         : the duty cycle
*******************************************************************************/
u32 DCDC_GetDutyCycle()
{
    return LocalDevice.Init.DutyCycle;
}

/*******************************************************************************
* Function Name  : DCDC_GetFrequency
* Description    : Get the Frequency for the dcdc converter
* Input          : NONE
* Return         : Actual Frequency
*******************************************************************************/
u32 DCDC_GetFrequency()
{
    return LocalDevice.Init.frequency;
}

/*******************************************************************************
* Function Name  : DCDC_GetPhaseShift
* Description    : Get the Frequency for the dcdc converter
* Input          : NONE
* Return         : Actual Frequency
*******************************************************************************/
u32 DCDC_GetPhaseShift()
{
    return LocalDevice.Init.PhaseShift;
}

/*******************************************************************************
* Function Name  : DCDC_SetPhaseShift
* Description    : Set the phase shift for the dcdc converter
* Input          : dwPhaseShift the new phase shift
* Return         : Error on function 0 no error.
*******************************************************************************/
DCDC_Error_t DCDC_SetPhaseShift(u32 dwPhaseShift)
{
	u32 MAX_PHASE_SHIFT = (LocalDevice.Init.Counter)/2; // 180 DEG
	u32 MIN_PHASE_SHIFT = (LocalDevice.Init.Counter);  // 0 DEG

    DCDC_Error_t nRet;
    nRet = DCDC_ERROR_ON_SET_PHASESHIFT;
    if (dwPhaseShift >= (MAX_PHASE_SHIFT-1) && dwPhaseShift <= MIN_PHASE_SHIFT)
    {
        LocalDevice.Init.PhaseShift = dwPhaseShift;
        //TIM_SetCompare1(TIM1, TIM2_MAX_PULSE + TIM2_DEAD_TIME + LocalDevice.Init.PhaseShift);

        //LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_MASTER, (uint32_t)(LocalDevice.Init.PhaseShift));

        HRTIM1->sMasterRegs.MCMP1R = (uint32_t)(LocalDevice.Init.PhaseShift);
        nRet = DCDC_ERROR_NONE;
    }
    return nRet;
}

/*******************************************************************************
* Function Name  : DCDC_SetDuty Cycle
* Description    : Set the Duty Cycle for the dcdc converter
* Input          : dwDeadTime the new deadtime
* Return         : Error on function 0 no error.
*******************************************************************************/
DCDC_Error_t DCDC_SetDutyCycle(u32 dutycycle)
{
    DCDC_Error_t nRet;
    DCDC_TypeDef_t InitStructure;
    nRet = DCDC_ERROR_ON_SET_DUTYCYCLE;
    dutyadj = dutycycle;

   if((dutyadj) <= DCDC_GetPeriod()*0.1)
    {

       // LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, (uint32_t)((DCDC_GetPeriod()*0.1)));
       // LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_B, (uint32_t)((DCDC_GetPeriod()*0.1)));
        LocalDevice.Init.DutyCycle = (DCDC_GetPeriod()*0.1);
        nRet = DCDC_ERROR_NONE;
    }
  else if((dutyadj) >= DCDC_GetPeriod()*0.9)
    {

       // LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, (uint32_t)((DCDC_GetPeriod()*0.9)));
       // LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_B, (uint32_t)((DCDC_GetPeriod()*0.9)));
        LocalDevice.Init.DutyCycle = (DCDC_GetPeriod()*0.9);
        nRet = DCDC_ERROR_NONE;
    }
   else
    {

       // LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, (uint32_t)(dutyadj));
       // LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_B, (uint32_t)(dutyadj));
        LocalDevice.Init.DutyCycle=dutyadj;
        nRet = DCDC_ERROR_NONE;
    }
     /*DCDC Converter configuration */

    return nRet;
}


/*******************************************************************************
* Function Name  : DCDC_SetFrequency
* Description    : Set the Frequency for the LLC DCDC converter
* Input          : MPPT new Frequency
* Return         : Error on function 0 no error.
*******************************************************************************/
DCDC_Error_t DCDC_SetFrequency(u32 LLC_frequency)
{
    DCDC_Error_t nRet;
    DCDC_TypeDef_t InitStructure;
    nRet = DCDC_ERROR_ON_SET_FREQUENCY;
    frequencyinc = LLC_frequency;

   u32 Phase_Period_Ratio = (DCDC_GetPhaseShift() * 0xFFFF) / DCDC_GetPeriod();

   if((frequencyinc) <= MIN_FREQ)
    {

	    NEW_DCDC_PERIOD = ((uint16_t)(((uint64_t)HRTIM_INPUT_CLOCK * HRTIMER_LLC_PRESCALER) / MIN_FREQ));
	    NEW_isoDCDC_PERDIOD = ((uint16_t)((uint64_t)TIM8_INPUT_CLOCK / (MIN_FREQ)));


	    HRTIM1->sMasterRegs.MPER = (uint32_t)(NEW_DCDC_PERIOD);
	    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].PERxR = (uint32_t)(NEW_DCDC_PERIOD);
	    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].PERxR = (uint32_t)(NEW_DCDC_PERIOD);
	    TIM8->ARR = (uint32_t)NEW_isoDCDC_PERDIOD;
	    TIM8->CCR3 = (uint32_t)NEW_isoDCDC_PERDIOD/2;

	    HRTIM1->sMasterRegs.MCMP2R = (uint32_t)(NEW_DCDC_PERIOD * MASTER_DUTY_CYCLE * (1 + ADC1_TRIG_DUTY_CYCLE));
	    HRTIM1->sMasterRegs.MCMP3R = (uint32_t)(NEW_DCDC_PERIOD * MASTER_DUTY_CYCLE * ADC1_TRIG_DUTY_CYCLE);
	    //LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_MASTER, (uint32_t)NEW_DCDC_PERIOD);
		//LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_A, (uint32_t)(NEW_DCDC_PERIOD));
		//LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_B, (uint32_t)(NEW_DCDC_PERIOD));
		//LL_TIM_SetAutoReload(TIM8, (NEW_isoDCDC_PERDIOD));
		//LL_TIM_OC_SetCompareCH3(TIM8, (NEW_isoDCDC_PERDIOD)*0.5);
		//LL_HRTIM_TIM_SetCompare2(HRTIM1, LL_HRTIM_TIMER_MASTER, (uint32_t)(NEW_DCDC_PERIOD * MASTER_DUTY_CYCLE * (1 + ADC1_TRIG_DUTY_CYCLE)));
		//LL_HRTIM_TIM_SetCompare3(HRTIM1, LL_HRTIM_TIMER_MASTER, (uint32_t)(NEW_DCDC_PERIOD * MASTER_DUTY_CYCLE * ADC1_TRIG_DUTY_CYCLE));



        LocalDevice.Init.frequency = MIN_FREQ;
        LocalDevice.Init.Counter = NEW_DCDC_PERIOD;

        DCDC_SetPhaseShift((Phase_Period_Ratio * DCDC_GetPeriod())/0xFFFF);

        DCDC_SetDutyCycle(NEW_DCDC_PERIOD/2); // FIXED 50% - TODO

        /*  update from shadow register */
        HRTIM1->sCommonRegs.CR2 |= HRTIM_CR2_TASWU
                                 + HRTIM_CR2_TBSWU;

        nRet = DCDC_ERROR_NONE;

    }
  else if((frequencyinc) >= MAX_FREQ)
    {
	    NEW_DCDC_PERIOD = ((uint16_t)(((uint64_t)HRTIM_INPUT_CLOCK * HRTIMER_LLC_PRESCALER) / MAX_FREQ));
	    NEW_isoDCDC_PERDIOD = ((uint16_t)((uint64_t)TIM8_INPUT_CLOCK / (MAX_FREQ)));

		//LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_MASTER, (uint32_t)NEW_DCDC_PERIOD);
		//LL_HRTIM_TIM_SetCompare2(HRTIM1, LL_HRTIM_TIMER_MASTER, (uint32_t)(NEW_DCDC_PERIOD * MASTER_DUTY_CYCLE * (1 + ADC1_TRIG_DUTY_CYCLE)));
		//LL_HRTIM_TIM_SetCompare3(HRTIM1, LL_HRTIM_TIMER_MASTER, (uint32_t)(NEW_DCDC_PERIOD * MASTER_DUTY_CYCLE * ADC1_TRIG_DUTY_CYCLE));
		//LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_A, (uint32_t)(NEW_DCDC_PERIOD));
		//LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_B, (uint32_t)(NEW_DCDC_PERIOD));
		//LL_TIM_SetAutoReload(TIM8, (NEW_isoDCDC_PERDIOD));
		//LL_TIM_OC_SetCompareCH3(TIM8, (NEW_isoDCDC_PERDIOD)*0.5);

	    HRTIM1->sMasterRegs.MPER = (uint32_t)(NEW_DCDC_PERIOD);
	    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].PERxR = (uint32_t)(NEW_DCDC_PERIOD);
	    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].PERxR = (uint32_t)(NEW_DCDC_PERIOD);
	    TIM8->ARR = (uint32_t)NEW_isoDCDC_PERDIOD;
	    TIM8->CCR3 = (uint32_t)NEW_isoDCDC_PERDIOD/2;

	    HRTIM1->sMasterRegs.MCMP2R = (uint32_t)(NEW_DCDC_PERIOD * MASTER_DUTY_CYCLE * (1 + ADC1_TRIG_DUTY_CYCLE));
	    HRTIM1->sMasterRegs.MCMP3R = (uint32_t)(NEW_DCDC_PERIOD * MASTER_DUTY_CYCLE * ADC1_TRIG_DUTY_CYCLE);

        LocalDevice.Init.frequency=MAX_FREQ;
        LocalDevice.Init.Counter = NEW_DCDC_PERIOD;

        DCDC_SetPhaseShift((Phase_Period_Ratio * DCDC_GetPeriod())/0xFFFF);

        DCDC_SetDutyCycle(NEW_DCDC_PERIOD/2); // FIXED 50% - TODO

        /*  update from shadow register */
        HRTIM1->sCommonRegs.CR2 |= HRTIM_CR2_TASWU
                                 + HRTIM_CR2_TBSWU;

        nRet = DCDC_ERROR_NONE;
    }
   else
    {


	    NEW_DCDC_PERIOD = ((uint16_t)(((uint64_t)HRTIM_INPUT_CLOCK * HRTIMER_LLC_PRESCALER) / frequencyinc));
	    NEW_isoDCDC_PERDIOD = ((uint16_t)((uint64_t)TIM8_INPUT_CLOCK / (frequencyinc)));

		//LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_MASTER, (uint32_t)NEW_DCDC_PERIOD);
		//LL_HRTIM_TIM_SetCompare2(HRTIM1, LL_HRTIM_TIMER_MASTER, (uint32_t)(NEW_DCDC_PERIOD * MASTER_DUTY_CYCLE * (1 + ADC1_TRIG_DUTY_CYCLE)));
		//LL_HRTIM_TIM_SetCompare3(HRTIM1, LL_HRTIM_TIMER_MASTER, (uint32_t)(NEW_DCDC_PERIOD * MASTER_DUTY_CYCLE * ADC1_TRIG_DUTY_CYCLE));
		//LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_A, (uint32_t)(NEW_DCDC_PERIOD));
		//LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_B, (uint32_t)(NEW_DCDC_PERIOD));
		//LL_TIM_SetAutoReload(TIM8, (NEW_isoDCDC_PERDIOD));
		//LL_TIM_OC_SetCompareCH3(TIM8, (NEW_isoDCDC_PERDIOD)*0.5);

	    HRTIM1->sMasterRegs.MPER = (uint32_t)(NEW_DCDC_PERIOD);
	    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].PERxR = (uint32_t)(NEW_DCDC_PERIOD);
	    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].PERxR = (uint32_t)(NEW_DCDC_PERIOD);
	    TIM8->ARR = (uint32_t)NEW_isoDCDC_PERDIOD;
	    TIM8->CCR3 = (uint32_t)NEW_isoDCDC_PERDIOD/2;

	    HRTIM1->sMasterRegs.MCMP2R = (uint32_t)(NEW_DCDC_PERIOD * MASTER_DUTY_CYCLE * (1 + ADC1_TRIG_DUTY_CYCLE));
	    HRTIM1->sMasterRegs.MCMP3R = (uint32_t)(NEW_DCDC_PERIOD * MASTER_DUTY_CYCLE * ADC1_TRIG_DUTY_CYCLE);

        LocalDevice.Init.frequency = frequencyinc;
        LocalDevice.Init.Counter = NEW_DCDC_PERIOD;

        DCDC_SetPhaseShift((Phase_Period_Ratio * DCDC_GetPeriod())/0xFFFF);
        DCDC_SetDutyCycle(NEW_DCDC_PERIOD/2); // FIXED 50% - TODO

        /*  update from shadow register */
        HRTIM1->sCommonRegs.CR2 |= HRTIM_CR2_TASWU
                                 + HRTIM_CR2_TBSWU;
        nRet = DCDC_ERROR_NONE;


    }
     /*DCDC Converter configuration */

    return nRet;
}


/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports DCDC timers outputs.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DCDC_GPIO_Configuration(void)
{


}

/*******************************************************************************
* Function Name  : TIMx_Configuration
* Description    : Timers configuration for DCDC converter
* Input          : NONE
* Return         : None
*******************************************************************************/
void DCDC_TIMx_Configuration(void)
{

  /* USER CODE BEGIN HRTIM1_Init 0 */

  /* USER CODE END HRTIM1_Init 0 */

  HRTIM_EventCfgTypeDef pEventCfg = {0};
  HRTIM_FaultBlankingCfgTypeDef pFaultBlkCfg = {0};
  HRTIM_FaultCfgTypeDef pFaultCfg = {0};
  HRTIM_ADCTriggerCfgTypeDef pADCTriggerCfg = {0};
  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  HRTIM_CompareCfgTypeDef pCompareCfg = {0};
  HRTIM_TimerCtlTypeDef pTimerCtl = {0};
  HRTIM_TimerEventFilteringCfgTypeDef pTimerEventFilteringCfg = {0};
  HRTIM_DeadTimeCfgTypeDef pDeadTimeCfg = {0};
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};

  /* USER CODE BEGIN HRTIM1_Init 1 */

  /* USER CODE END HRTIM1_Init 1 */
  hhrtim1.Instance = HRTIM1;
  hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
  hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_MASTER;
  hhrtim1.Init.SyncOutputSource = HRTIM_SYNCOUTPUTSOURCE_TIMA_CMP1;
  hhrtim1.Init.SyncOutputPolarity = HRTIM_SYNCOUTPUTPOLARITY_POSITIVE;
  if (HAL_HRTIM_Init(&hhrtim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_EventPrescalerConfig(&hhrtim1, HRTIM_EVENTPRESCALER_DIV1) != HAL_OK)
  {
    Error_Handler();
  }
  pEventCfg.Source = HRTIM_EEV6SRC_COMP1_OUT;
  pEventCfg.Polarity = HRTIM_EVENTPOLARITY_HIGH;
  pEventCfg.Sensitivity = HRTIM_EVENTSENSITIVITY_LEVEL;
  pEventCfg.Filter = HRTIM_EVENTFILTER_3;
  if (HAL_HRTIM_EventConfig(&hhrtim1, HRTIM_EVENT_6, &pEventCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pEventCfg.Source = HRTIM_EEV7SRC_COMP4_OUT;
  if (HAL_HRTIM_EventConfig(&hhrtim1, HRTIM_EVENT_7, &pEventCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultPrescalerConfig(&hhrtim1, HRTIM_FAULTPRESCALER_DIV1) != HAL_OK)
  {
    Error_Handler();
  }
  pFaultBlkCfg.Threshold = 0;
  pFaultBlkCfg.ResetMode = HRTIM_FAULTCOUNTERRST_UNCONDITIONAL;
  pFaultBlkCfg.BlankingSource = HRTIM_FAULTBLANKINGMODE_RSTALIGNED;
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_2, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_2, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_2, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_2, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_2, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_2, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pFaultCfg.Source = HRTIM_FAULTSOURCE_DIGITALINPUT;
  pFaultCfg.Polarity = HRTIM_FAULTPOLARITY_HIGH;
  pFaultCfg.Filter = HRTIM_FAULTFILTER_15;
  pFaultCfg.Lock = HRTIM_FAULTLOCK_READWRITE;
  if (HAL_HRTIM_FaultConfig(&hhrtim1, HRTIM_FAULT_2, &pFaultCfg) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_HRTIM_FaultModeCtl(&hhrtim1, HRTIM_FAULT_2, HRTIM_FAULTMODECTL_DISABLED);
  pADCTriggerCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_MASTER;
  pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT13_MASTER_CMP2;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, &pADCTriggerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, 0x0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, 0x0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, 0x0) != HAL_OK)
  {
    Error_Handler();
  }
  pADCTriggerCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_F;
  pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT13_TIMERF_CMP3;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_3, &pADCTriggerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_3, 0x0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_3, 0x0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_3, 0x0) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = (uint32_t)MASTER_PERIOD;
  pTimeBaseCfg.RepetitionCounter = 0x00;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL32;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_MASTER_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_MASTER_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.InterleavedMode = HRTIM_INTERLEAVED_MODE_DISABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_DISABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
  pTimerCfg.ReSyncUpdate = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = (uint32_t)(PHASE_SHIFT_INIT);
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = (uint32_t)(MASTER_PERIOD * MASTER_DUTY_CYCLE * (1 + ADC1_TRIG_DUTY_CYCLE));
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = (uint32_t)(MASTER_PERIOD * MASTER_DUTY_CYCLE * ADC1_TRIG_DUTY_CYCLE);
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_3, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = (uint32_t)(MASTER_PERIOD*0.75);
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_4, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Mode = HRTIM_MODE_SINGLESHOT_RETRIGGERABLE;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCtl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UP;
  pTimerCtl.GreaterCMP1 = HRTIM_TIMERGTCMP1_EQUAL;
  pTimerCtl.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_DLYPRT;
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
  pTimerCfg.InterleavedMode = HRTIM_INTERLEAVED_MODE_DUAL;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DELAYEDBOTH_EEV7;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_MASTER;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_PER;
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_DISABLED;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_CMP1;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_SET2;//HRTIM_TIM_DMA_RST1;//HRTIM_TIM_DMA_SET1;
  pTimerCfg.DMASrcAddress = (uint32_t)&TIMD_DMA_Buffer[0];//(uint32_t)&TIMD_DMA_Buffer[0];//(uint32_t)&TIMD_DMA_Buffer[0];
  pTimerCfg.DMADstAddress = (uint32_t)&(HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].RSTx2R);//(uint32_t)&(HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].SETx2R);//(uint32_t)&(hhrtim1.Instance->sCommonRegs.BDMADR); //(uint32_t)&(HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].SETx1R);
  pTimerCfg.DMASize = 1; //ZMIANA Z 0x1
  pTimerCfg.InterleavedMode = HRTIM_INTERLEAVED_MODE_DISABLED;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_D_E_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED; //DODANE
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_TIMER_F; //HRTIM_TIMUPDATETRIGGER_TIMER_F;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_OTHER5_CMP2;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_DMABURST;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }

  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_TIMER_F;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_OTHER5_CMP2;//HRTIM_TIMRESETTRIGGER_OTHER5_CMP2;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;

  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0000;//(uint32_t)&TIMF_DMA_Buffer[0];
  pTimerCfg.DMADstAddress = 0x0000;//(uint32_t)&(HRTIM1_COMMON->CR2);
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_F_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }

  pCompareCfg.CompareValue = 0xFFF7;
  pCompareCfg.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  pCompareCfg.AutoDelayedTimeout = 0x0000;

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_4, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = (uint32_t)(TIMF_PERIOD*TIMF_DUTY_CYCLE);
  pCompareCfg.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  pCompareCfg.AutoDelayedTimeout = 0x0000;

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_COMPAREUNIT_4, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerEventFilteringCfg.Filter = HRTIM_TIMEEVFLT_BLANKINGCMP1;
  pTimerEventFilteringCfg.Latch = HRTIM_TIMEVENTLATCH_DISABLED;
  if (HAL_HRTIM_TimerEventFilteringConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_EVENT_6, &pTimerEventFilteringCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_TimerEventFilteringConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_EVENT_6, &pTimerEventFilteringCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pDeadTimeCfg.Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL8;
  pDeadTimeCfg.RisingValue = (uint32_t)(LLC_DEADTIME);
  pDeadTimeCfg.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
  pDeadTimeCfg.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
  pDeadTimeCfg.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
  pDeadTimeCfg.FallingValue = (uint32_t)(LLC_DEADTIME);
  pDeadTimeCfg.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
  pDeadTimeCfg.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
  pDeadTimeCfg.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pDeadTimeCfg.Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL4;
  pDeadTimeCfg.RisingValue = (uint32_t)(TIMD_DEADTIME);
  pDeadTimeCfg.FallingValue = (uint32_t)(TIMD_DEADTIME);
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pDeadTimeCfg.Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL4;
  pDeadTimeCfg.RisingValue = (uint32_t)(TIMF_DEADTIME);
  pDeadTimeCfg.FallingValue = (uint32_t)(TIMF_DEADTIME);
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pDeadTimeCfg.Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL4;
  pDeadTimeCfg.RisingValue = (uint32_t)(0);
  pDeadTimeCfg.FallingValue = (uint32_t)(0);
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }

  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMCMP1;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_MASTERPER;
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_MASTERCMP1;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMCMP1;//HRTIM_OUTPUTSET_TIMDEV8_TIMFCMP1;//HRTIM_OUTPUTSET_TIMDEV8_TIMFCMP1; //HRTIM_OUTPUTSET_TIMCMP1;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_NONE; //HRTIM_OUTPUTRESET_TIMDEV9_TIMFCMP3;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_OUTPUT_TD1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMCMP1;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_NONE;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_OUTPUT_TF1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_NONE;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_NONE;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP3;//HRTIM_OUTPUTRESET_TIMDEV9_TIMFCMP3; //FROM CMP2
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_OUTPUT_TD2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_NONE;
  pOutputCfg.ResetSource = HRTIM_OUTPUTSET_NONE;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }

  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMPER;//HRTIM_OUTPUTSET_TIMEEV8_TIMFCMP3;
  pOutputCfg.ResetSource = HRTIM_OUTPUTSET_TIMPER;//HRTIM_OUTPUTSET_TIMEEV8_TIMFCMP3;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_NONE;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP3;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_OUTPUT_TF2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = (uint32_t)TIMF_PERIOD;
  pTimeBaseCfg.RepetitionCounter = 1;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL16;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }

  pTimerCtl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UPDOWN;
  pTimerCtl.TrigHalf = HRTIM_TIMERTRIGHALF_DISABLED;
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_RollOverModeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_TIM_FEROM_VALLEY|HRTIM_TIM_BMROM_VALLEY
                              |HRTIM_TIM_ADROM_VALLEY|HRTIM_TIM_OUTROM_VALLEY
                              |HRTIM_TIM_ROM_VALLEY) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = (uint32_t)(TIMD_PERIOD/2);
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = (uint32_t)(TIMD_PERIOD*TIMD_DUTY_CYCLE);

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = (uint32_t)TIMF_PERIOD*1.5;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_3, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = (uint32_t)(TIMF_PERIOD);
  pTimeBaseCfg.RepetitionCounter = 1;
  pTimeBaseCfg.Mode = HRTIM_MODE_SINGLESHOT_RETRIGGERABLE;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_RollOverModeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_TIM_FEROM_BOTH|HRTIM_TIM_BMROM_BOTH
                              |HRTIM_TIM_ADROM_BOTH|HRTIM_TIM_OUTROM_BOTH
                              |HRTIM_TIM_ROM_BOTH) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = (uint32_t)TIMF_PERIOD;
  pTimeBaseCfg.RepetitionCounter = 1;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCtl.GreaterCMP3 = HRTIM_TIMERGTCMP3_EQUAL;
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_RollOverModeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_TIM_FEROM_VALLEY|HRTIM_TIM_BMROM_VALLEY
                              |HRTIM_TIM_ADROM_VALLEY|HRTIM_TIM_OUTROM_VALLEY
                              |HRTIM_TIM_ROM_VALLEY) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = (uint32_t)(TIMF_PERIOD/2);
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = (uint32_t)(TIMF_PERIOD/2);
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = (uint32_t)(TIMF_PERIOD * TIMF_DUTY_CYCLE);
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = (uint32_t)(60);
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = (uint32_t)((0));//(uint32_t)((TIMF_PERIOD * ADC23_TRIG_DUTY_CYCLE));
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_COMPAREUNIT_3, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN HRTIM1_Init 2 */
  HRTIM1_COMMON->CR2 |= (1 << 20);
  /* USER CODE END HRTIM1_Init 2 */
  HAL_HRTIM_MspPostInit(&hhrtim1);

  //HAL_HRTIM_BurstDMAConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_BURSTDMA_SET1R);
  //HAL_HRTIM_BurstDMATransfer(&hhrtim1,HRTIM_TIMERINDEX_TIMER_D,TIMD_DMA_Buffer[0],1);
}


/*******************************************************************************
* Function Name  : TIMx_Configuration
* Description    : Timers configuration for DCDC converter
* Input          : NONE
* Return         : None
*******************************************************************************/
void DCDC_TIMx_Configuration_startup(void)
{

}

/*
DTG[7:5]=0xx => DT=DTG[7:0]x tdtg with tdtg=tDTS.
DTG[7:5]=10x => DT=(64+DTG[5:0])xtdtg with Tdtg=2xtDTS.
DTG[7:5]=110 => DT=(32+DTG[4:0])xtdtg with Tdtg=8xtDTS.
DTG[7:5]=111 => DT=(32+DTG[4:0])xtdtg with Tdtg=16xtDTS.
*/
/*******************************************************************************
* Function Name  : DCDC_DeadTimeToReg
* Description    : Convert DeadTime in clocks to register value
* Input          : deadTime numberof clocks
* Return         : Register value for timer
*******************************************************************************/
static u32 DCDC_DeadTimeToReg(u32 deadTime)
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
//    if ( deadTime != DCDC_RegToDeadTime(nRet))
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
* Function Name  : DCDC_RegToDeadTime
* Description    : Convert register value to DeadTime in clocks
* Input          : Register value for timer
* Return         : deadTime numberof clocks
*******************************************************************************/
u32 DCDC_RegToDeadTime(u32 regDead)
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
* Function Name  : DCDC_RefreshDisplay
* Description    : Display same info from the object in display
* Input          : Error and step of phase shift
* Return         : None
*******************************************************************************/
/*
void DCDC_RefreshDisplay(DCDC_Error_t err, u32 steps, u32 Changes)
{
    char strLineMessage[64];
    static u16 DCDCCOUNTER;
    DCDCCOUNTER = LocalDevice.Init.Counter >>1;
    static u16 TIM2_PULSE;
    TIM2_PULSE = TIM2_INIT_PULSE;

    LCD_DisplayStringLine(Line0, "DCDC Converter menu");

    if (DCDC_GetStatus() == DCDC_Running)
        sprintf(strLineMessage, "STATUS   ON  ");
    else
        sprintf(strLineMessage, "STATUS   OFF ");
    LCD_ClearLine(Line1);
    LCD_DisplayStringLine(Line1, (u8 *)strLineMessage);

 // sprintf(strLineMessage, "DeadTime %d(%.2fus)", DCDC_GetDeadTime(), ((double)(DCDC_GetDeadTime()) * CLOCK_DURATION_MICROS));
    sprintf(strLineMessage, "DC %d (%.2f)", (LocalDevice.Init.frequency-DCDC_GetDutyCycle()),(double)(LocalDevice.Init.frequency - DCDC_GetDutyCycle())/DCDCCOUNTER*100);
    LCD_ClearLine(Line2);

#define LSELLEN_DCDC(x, y) (((x)==(y))? 20 : 0)

    LCD_DisplayStringLineEx(Line2, 0, LSELLEN_DCDC(Changes, 0), (u8 *)strLineMessage);

    sprintf(strLineMessage, "Frequency %.2fkHz", 1/(((double)(DCDC_COUNTER) * CLOCK_DURATION_MICROS))*1000);
    LCD_ClearLine(Line3);
    LCD_DisplayStringLineEx(Line3, 0 ,LSELLEN_DCDC(Changes, 1), (u8 *)strLineMessage);

    sprintf(strLineMessage, "Period (%.2fus)", (double)(DCDC_COUNTER) * CLOCK_DURATION_MICROS);
    LCD_ClearLine(Line4);
    LCD_DisplayStringLineEx(Line4, 0, LSELLEN_DCDC(Changes, 2), (u8 *)strLineMessage);

    LCD_ClearLine(Line5);
    LCD_ClearLine(Line6);
    LCD_ClearLine(Line7);

    sprintf(strLineMessage, "Error 0x%.8X    ", err);
    LCD_ClearLine(Line8);
    LCD_DisplayStringLine(Line8, (u8 *)strLineMessage);

    LCD_ClearLine(Line9);
}

*/
