/*
 * DataSensing.c
 *
 *  Created on: Aug 6, 2020
 *      Author: user01
 */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "DataSensing.h"
#include "stm32g4xx_it.h"
//#include "lcd.h"
//#include "color_lcd.h"
#include "stdio.h"
#include "string.h"
#include "400WControl.h"
#include "DCDC_Converter.h"
#include "DCAC_Inverter.h"

#include "stm32g4xx_hal.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BB(reg) ((uint32_t *)(PERIPH_BB_BASE + ((uint32_t)&(reg) - PERIPH_BASE) * 32U))

#define DMA1_CLEAR_IT (BB(DMA1->IFCR)[1])
#define DMA2_CLEAR_IT (BB(DMA1->IFCR)[5])
#define DMA3_CLEAR_IT (BB(DMA1->IFCR)[9])
#define DMA4_CLEAR_IT (BB(DMA1->IFCR)[13])
#define DMA5_CLEAR_IT (BB(DMA1->IFCR)[17])
#define DMA6_CLEAR_IT (BB(DMA1->IFCR)[21])
#define DMA7_CLEAR_IT (BB(DMA1->IFCR)[25])
#define DMA7_CLEAR_HIT (BB(DMA1->IFCR)[26])
#define DMA7_CLEAR_GIT (BB(DMA1->IFCR)[24])
#define DMA8_CLEAR_IT (BB(DMA1->IFCR)[29])
#define DMA8_CLEAR_HIT (BB(DMA1->IFCR)[30])
#define DMA8_CLEAR_GIT (BB(DMA1->IFCR)[28])

#define ADC_CALIBRATION_TIMEOUT_MS       (   1U)
#define ADC_ENABLE_TIMEOUT_MS            (   1U)
#define ADC_DISABLE_TIMEOUT_MS           (   1U)
#define ADC_STOP_CONVERSION_TIMEOUT_MS   (   1U)
#define ADC_CONVERSION_TIMEOUT_MS        ( 500U)

/* Delay between ADC end of calibration and ADC enable.                     */
/* Delay estimation in CPU cycles: Case of ADC enable done                  */
/* immediately after ADC calibration, ADC clock setting slow                */
/* (LL_ADC_CLOCK_ASYNC_DIV32). Use a higher delay if ratio                  */
/* (CPU clock / ADC clock) is above 32.                                     */
#define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES  (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32)

extern TIM_HandleTypeDef htim1;
extern HRTIM_HandleTypeDef hhrtim1;
extern void delay_us(uint32_t delay_us);


//#define CLOCK_DURATION_MICROS    0.01388888
//#define CLOCK_DURATION_MICROS ((1/CORE_CLOCK)*1000000) // or 0.00588235

//#define ADC1_DR_Address    ((u32)0x4001244C)
//#define DS_AcquisitionEvent ADC1_2_IRQHandler

//#define DS_AcquisitionEvent DMA1_Channel5_IRQHandler //TIMF PER

#define DS_AcquisitionEvent DMA1_Channel2_IRQHandler //SPI

//#define DS_AcquisitionEvent HRTIM1_TIMF_IRQHandler
//static TIM_OCInitTypeDef  TIM_OCInitStructure;
extern SystStatus_t *ptr_State_Control;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static struct DS_Device_s
{
    DS_TypeDef_t Init;
    DS_Status_t State;
} DataSensing;

//static TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
static GPIO_InitTypeDef GPIO_InitStructure;
//static ADC_InitTypeDef ADC_InitStructure;
static DMA_InitTypeDef DMA_InitStructure;

volatile uint16_t to_grid_State_Control[SPI4_DATA_SIZE] = {0};

volatile uint16_t DCAC_DATA[SPI4_DATA_SIZE] = {0};

volatile uint32_t Pulse1_IT = 0;
volatile bool Pulse2_IT = 0;
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void DS_TIMx_Configuration(void);
static void DS_GPIO_Configuration(void);
static void DS_ADCx_Configuration(void);
static void DS_DMA_Configuration(void);
static void DS_ADC_EnableAndCalibrate(ADC_TypeDef* ADCx);
static void MX_TIM1_Init(void);
static void MX_TIM1_Init2(void);
extern void ExecControlOpenLoop(void);

extern u8 aqusitionDMA1_1;
extern u8 aqusitionDMA1_5;

extern SPI_HandleTypeDef hspi4;
/*******************************************************************************
* Function Name  : DS_Init
* Description    : Configures Peripherals used by the DataSensing
* Input          : PDS_TypeDef_t pointer to a structure
* Return         : errors on initialization
*******************************************************************************/
DS_Error_t DS_Init(PDS_TypeDef_t pDSInit)
{
    DS_Error_t nRet;

    nRet = DS_ERROR_INVALID_PARAMETER;

    if (pDSInit)
    {
        nRet = DS_ERROR_ON_INIT;
        if (DataSensing.State != DS_Running)
        {
            DataSensing.Init.Counter       = pDSInit->Counter;
            DataSensing.Init.DataRegister  = pDSInit->DataRegister;
            DataSensing.Init.RegisterSize  = pDSInit->RegisterSize;
            DataSensing.Init.OnAcquisition = pDSInit->OnAcquisition;
            DataSensing.State = DS_Stopped;

            /*Init peripherals */
            DS_GPIO_Configuration();
            DS_TIMx_Configuration();
            DS_DMA_Configuration();
            DS_ADCx_Configuration();

            nRet = DS_ERROR_NONE;
        }
    }

    return nRet;
}

/*******************************************************************************
* Function Name  : DS_SetAcquistionEvent
* Description    : Set The acquisition event handle
* Input          : Pointer to function that hadle the acquisition of DS
* Return         : None
*******************************************************************************/
void DS_SetAcquistionEvent(PFN_ON_ACQUISTION pfFn)
{
  if (pfFn)
  {
    DataSensing.Init.OnAcquisition = pfFn;
  }
}

/*******************************************************************************
* Function Name  : DS_GetConfiguration
* Description    : Get DS Configuration
* Input          : Pointer to a allocated structure where will be copied the
*                  configuration values
* Return         : 0 no error, non 0 if error
*******************************************************************************/
DS_Error_t DS_GetConfiguration(PDS_TypeDef_t pDSInit)
{
    DS_Error_t nRet;
    nRet = DS_ERROR_INVALID_PARAMETER;

    if (pDSInit)
    {
        pDSInit->Counter       = DataSensing.Init.Counter;
        pDSInit->DataRegister  = DataSensing.Init.DataRegister;
        pDSInit->RegisterSize  = DataSensing.Init.RegisterSize;
        pDSInit->OnAcquisition = DataSensing.Init.OnAcquisition;
        nRet = DS_ERROR_NONE;
    }

    return nRet;
}

/*******************************************************************************
* Function Name  : DS_GetData
* Description    : Get the working buffer
* Input          : None
* Return         : Pointer to data baffer
*******************************************************************************/
PDS_Data_t DS_GetData()
{
    return (PDS_Data_t)DataSensing.Init.DataRegister;
}

/*******************************************************************************
* Function Name  : DS_Init
* Description    : Configures Peripherals used by the DS Converter
* Input          : PDS_TypeDef_t pointer to a structure
* Return         : errors on initialization
*******************************************************************************/
DS_Error_t DS_SendCommand(DS_Commands_t cmd)
{
    DS_Error_t nRet;

    nRet = DS_ERROR_INVALID_COMMAND;

    if (cmd == DS_Start)
    {
       nRet = DS_ERROR_ON_SEND_COMMAND;
       if (DS_GetStatus() != DS_Running)
       {
          DataSensing.State = DS_Running;

          //Nie zmienać kolejności  wykonywania instrukcji !!!

          MX_TIM1_Init();

          //HAL_TIM_OnePulse_Start(&htim1, TIM_CHANNEL_3); // (Low default) OC signal to DCAC/OC TWO Level HS drivers
          //HAL_TIMEx_OnePulseN_Start(&htim1, TIM_CHANNEL_3); // (High default) Enable/Disable HS gate drivers to finish TWO Level OFF

          TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_3, TIM_CCx_ENABLE);

          uint32_t tmp;
          tmp = TIM_CCER_CC1NE << (TIM_CHANNEL_3 & 0x1FU); /* 0x1FU = 31 bits max shift */
          /* Reset the CCxNE Bit */
          TIM1->CCER &=  ~tmp;
          /* Set or reset the CCxNE Bit */
          TIM1->CCER |= (uint32_t)(TIM_CCxN_ENABLE << (TIM_CHANNEL_3 & 0x1FU)); /* 0x1FU = 31 bits max shift */
          TIM1->BDTR |= TIM_BDTR_MOE; //ENABLE ALL OUTPUTS

          //Disable Break Inputs due to Glith after PWR On
          TIM1->AF1 &= ~TIM1_AF1_BKINE;
          COMP7 ->CSR &= ~COMP_CSR_EN;

          delay_us(10000);
          // TIMx enable counter
          LL_GPIO_SetOutputPin(EN_isoDCDC_GPIO_Port, EN_isoDCDC_Pin);
          TIM8->CCER |= TIM_CCER_CC3E; ////ENABLE CHANNEL 3
          TIM8->BDTR |= TIM_BDTR_MOE; //ENABLE ALL OUTPUTS
          TIM8->CR1  |= TIM_CR1_ARPE | TIM_CR1_CEN; // ENABLE AUTORELOAD PRELOAD AND TIMER COUNTER
          delay_us(10000);
          for (int new_value=ISO_DCDC_PERIOD*ISO_DCDC_DUTY; new_value<=ISO_DCDC_PERIOD*0.5; new_value++){

        	  TIM8->CCR3 = new_value;
        	  delay_us(300);;
          }
          delay_us(100000);

          HRTIM_TIMF_DMA_START();
          HRTIM_TIMD_DMA_START();



          HRTIM1->sMasterRegs.MCR |= (HRTIM_MCR_MCEN | HRTIM_MCR_TACEN | HRTIM_MCR_TBCEN | HRTIM_MCR_TDCEN | HRTIM_MCR_TECEN | HRTIM_MCR_TFCEN);

          //HRTIM1->sMasterRegs.MCR |=HRTIM_MCR_MCEN; //MASTER TIMER COUNTER ENABLE
          //HRTIM1->sMasterRegs.MCR |=HRTIM_MCR_TACEN; //TIMER A COUNTER ENABLE
          //HRTIM1->sMasterRegs.MCR |=HRTIM_MCR_TBCEN; //TIMER B COUNTER ENABLE

          //HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].SETx1R |= (1 << 0);

          //HRTIM1->sMasterRegs.MCR |=HRTIM_MCR_TFCEN; //TIMER F COUNTER ENABLE
          //HRTIM1->sMasterRegs.MCR |=HRTIM_MCR_TDCEN; //TIMER D COUNTER ENABLE
          //HRTIM1->sMasterRegs.MCR |=HRTIM_MCR_TECEN; //TIMER E COUNTER ENABLE //GRID - DCAC ADC TRIG

          HRTIM1_COMMON->OENR |=HRTIM_OENR_TE2OEN; //TIME OUTPUT 2 ENABLE



          //HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_F].TIMxDIER |= (HRTIM_TIM_IT_REP);
          //HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_F].TIMxDIER |= (HRTIM_TIM_IT_CMP1);
          //->OENR |=HRTIM_OENR_TF2OEN; //TIMF OUTPUT 2 ENABLE
          //HRTIM1_COMMON->OENR |=HRTIM_OENR_TF1OEN; //TIMF OUTPUT 1 ENABLE
          //HRTIM1_COMMON->OENR |=HRTIM_OENR_TF1OEN; //TIMF OUTPUT 1 ENABLE
          //HRTIM1_COMMON->OENR |=HRTIM_OENR_TF2OEN; //TIMF OUTPUT 2 ENABLE
          //TIM20->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_0); //TOP LF MOSFET HIGH / BOTTOM LF MOSFET LOW

          //TIM20->CCMR1 &= ~(TIM_CCMR1_OC2M_0); //TOP LF MOSFET LOW / BOTTOM LF MOSFET HIGH

          //Enable Break Inputs after delay due to Glith after DCAC side PWR On
          delay_us(100000);

          //COMP7 ->CSR |= COMP_CSR_EN; /TODO
          //TIM1->AF1 |= TIM1_AF1_BKINE;

          //HAL_SPI_Receive_DMA(&hspi4, (uint8_t*)DataSensing.Init.DataRegister, SPI4_DATA_SIZE);

          // Enable the SPI4 GRID peripheral.

          SPI4->CR2 &= ~(SPI_RXFIFO_THRESHOLD || SPI_CR2_LDMARX);

           //Enable DMA1 Channel 2 to start receiving data.
          DMA1_Channel2->CCR |= ( DMA_CCR_EN );
          DMA1_Channel3->CCR |= ( DMA_CCR_EN );
          SPI4->CR1 |=  ( SPI_CR1_SPE );

          SPI4->CR2 |= (SPI_CR2_TXDMAEN);
          SPI4->CR2 |= (SPI_CR2_RXDMAEN);

          // Enable the SPI3 DCAC peripheral.

          SPI3->CR2 &= ~(SPI_RXFIFO_THRESHOLD || SPI_CR2_LDMARX);

           //Enable DMA1 Channel 2 to start receiving data.
          DMA1_Channel4->CCR |= ( DMA_CCR_EN );
          DMA1_Channel6->CCR |= ( DMA_CCR_EN );
          SPI3->CR1 |=  ( SPI_CR1_SPE );

          SPI3->CR2 |= (SPI_CR2_TXDMAEN);
          SPI3->CR2 |= (SPI_CR2_RXDMAEN);


          //MX_TIM1_Init2();


		/*
				  uint32_t isrflags = HRTIM1->sCommonRegs.ISR;
				  uint32_t ierits   = HRTIM1->sCommonRegs.IER;
				  if((uint32_t)(isrflags & HRTIM_FLAG_FLT4) != (uint32_t)RESET)
				  {
					if((uint32_t)(ierits & HRTIM_IT_FLT4) != (uint32_t)RESET)
					{
						HRTIM1->sCommonRegs.ICR = HRTIM_ICR_FLT4C;
						DCDC_TIMx_Configuration(); //added to riconfigure the timer for DCDC
					}
				  }
		*/
          nRet = DS_ERROR_NONE;
       }
    }
    else
    if (cmd == DS_Stop)
    {
       nRet = DS_ERROR_ON_SEND_COMMAND;
       if (DS_GetStatus() != DS_Stopped)
       {
            DataSensing.State = DS_Stopped;

//            TIM_CtrlPWMOutputs(TIM8, DISABLE);

            // TIMx disable counter
           // TIM_Cmd(TIM8, DISABLE);

            HAL_SPI_DMAStop(&hspi4);

            HRTIM1->sMasterRegs.MCR &=~HRTIM_MCR_MCEN; //MASTER TIMER COUNTER DISABLE
            HRTIM1->sMasterRegs.MCR &=~HRTIM_MCR_TACEN; //TIMER A COUNTER DISABLE
            HRTIM1->sMasterRegs.MCR &=~HRTIM_MCR_TBCEN; //TIMER B COUNTER DISABLE
            HRTIM1->sMasterRegs.MCR &=~HRTIM_MCR_TFCEN; //TIMER F COUNTER DISABLE
            HRTIM1->sMasterRegs.MCR &=~HRTIM_MCR_TECEN; //TIMER E COUNTER DISABLE
            //HRTIM1->sMasterRegs.MCR &=~HRTIM_MCR_TDCEN; //TIMER D COUNTER DISABLE
            HRTIM1_COMMON->ODISR |=HRTIM_ODISR_TE2ODIS; //TIME OUTPUT 2 DISABLE

            HRTIM_TIMD_DMA_STOP();

            TIM8->CCER &= ~TIM_CCER_CC3NE; //DISABLE CHANNEL 3N
            TIM8->CCER &= ~TIM_CCER_CC3E; ////DISABLE CHANNEL 3
            //TIM8->EGR  |= TIM_EGR_UG; //DISABLE UPDATE
            TIM8->BDTR &= ~TIM_BDTR_MOE; //DISABLE ALL OUTPUTS
            TIM8->CR1  &= ~(TIM_CR1_ARPE | TIM_CR1_CEN); // DISABLE AUTORELOAD PRELOAD AND TIMER COUNTER

            LL_GPIO_ResetOutputPin(EN_isoDCDC_GPIO_Port, EN_isoDCDC_Pin);
            // Disable SPI4 peripheral.
            SPI4->CR2 &= ~(SPI_CR2_TXDMAEN);
            SPI4->CR2 &= ~(SPI_CR2_RXDMAEN);
            SPI4->CR1 &= ~SPI_CR1_SPE;
            // Disable DMA1 Channel 2/3 to stop receiving data.
            DMA1_Channel2->CCR &= ~( DMA_CCR_EN );
            DMA1_Channel3->CCR &= ~( DMA_CCR_EN );

            // Disable SPI3 peripheral.
            SPI3->CR2 &= ~(SPI_CR2_TXDMAEN);
            SPI3->CR2 &= ~(SPI_CR2_RXDMAEN);
            SPI3->CR1 &= ~SPI_CR1_SPE;
            // Disable DMA1 Channel 4/6 to stop receiving data.
            DMA1_Channel4->CCR &= ~( DMA_CCR_EN );
            DMA1_Channel6->CCR &= ~( DMA_CCR_EN );

            HAL_TIMEx_OnePulseN_Stop(&htim1, TIM_CHANNEL_3); // (High default) Enable/Disable HS gate drivers
            HAL_TIM_OnePulse_Stop(&htim1, TIM_CHANNEL_3); // (Low default) OC signal to DCAC

            //Disable Break Inputs due to Glith after PWR On
            TIM1->AF1 &= ~TIM1_AF1_BKINE;
            COMP7 ->CSR &= ~COMP_CSR_EN;

            delay_us(100000);
      	  uint32_t isrflags = HRTIM1->sCommonRegs.ISR;
      	  uint32_t ierits   = HRTIM1->sCommonRegs.IER;
      	  if((uint32_t)(isrflags & HRTIM_FLAG_FLT4) != (uint32_t)RESET)
      	  {
      	    if((uint32_t)(ierits & HRTIM_IT_FLT4) != (uint32_t)RESET)
      	    {
      	    	HRTIM1->sCommonRegs.ICR = HRTIM_ICR_FLT4C;
      	    	DCDC_TIMx_Configuration(); //added to riconfigure the timer for DCDC
      	    }
      	  }

            nRet = DS_ERROR_NONE;
       }
    }

   return nRet;
}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports for data sensing.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DS_GPIO_Configuration(void)
{
    /* Configure PC.0 PC.1 (ADC Channel10, 11) as analog input -------------------------*/
   // GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
   // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
   // GPIO_Init(GPIOC, &GPIO_InitStructure);

    /*For Debug pourpouse*/
    /* Configure Test Pin:PC9-----------------------------------*/
   // GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
   // GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
   // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   // GPIO_Init(GPIOC, &GPIO_InitStructure);

//        /* Configure Test Pin:PC9-----------------------------------*/
//    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
//    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*******************************************************************************
* Function Name  : DCAC_GetStatus
* Description    : Get DCAC status
* Input          : NONE
* Return         : Actual State of the device
*******************************************************************************/
DS_Status_t DS_GetStatus()
{
    return DataSensing.State;
}

/*******************************************************************************
* Function Name  : DS_AcquisitionEvent
* Description    : This function handles acquisition event of the datasensing.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void DS_AcquisitionEvent(void)
{

		 //if(HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_F].TIMxISR = )

	           // LL_GPIO_TogglePin(SD_CS_GPIO_Port, SD_CS_Pin);
			  //register uint32_t iTimer = (uint8_t)(POSITION_VAL(HRTIM_MCR_TFCEN) - HRTIM_MCR_MCEN_Pos);
	            //DCAC_SetPulse((u16)(Pulse1_IT), (u16)(Pulse1_IT));

				if(Pulse2_IT==FALSE)
				{
				  //TIM20->CCMR1 &= ~(TIM_CCMR1_OC2M_0); //TOP LF MOSFET LOW / BOTTOM LF MOSFET HIGH
				}
				else if (Pulse2_IT==TRUE)
				{
				  //TIM20->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_0); //TOP LF MOSFET HIGH / BOTTOM LF MOSFET LOW
				}





				  register __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)((uint32_t)(&HRTIM1->sMasterRegs.MICR) +
						  0x300U));
				  SET_BIT(*pReg, HRTIM_MICR_MREP);
				  //SET_BIT(*pReg, HRTIM_MICR_MCMP1);
				  //HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_F].TIMxICR |=HRTIM_TIMICR_CMP1C;
				   //HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_F].TIMxICR |=HRTIM_TIMICR_REPC;

		  if (DMA1->ISR & DMA_ISR_TCIF2)
		  {

			  //LL_GPIO_TogglePin(SD_CS_GPIO_Port, SD_CS_Pin);
			  //TODO
			  //ExecControlOpenLoop();
			  LL_GPIO_SetOutputPin(SD_CS_GPIO_Port, SD_CS_Pin);
				if (DataSensing.State == DS_Running)
				{
					//If event function is associeted

					if (DataSensing.Init.OnAcquisition)
						DataSensing.Init.OnAcquisition();
				}
			  DMA2_CLEAR_IT = 1;

		  }

		  if (DMA1->ISR & DMA_ISR_TCIF1)
		  {

		  DMA1_CLEAR_IT = 1;

		  }

		  if (DMA1->ISR & DMA_ISR_TCIF5)
		  {


		  DMA5_CLEAR_IT = 1;


		  }



		  if (DMA1->ISR & DMA_ISR_TCIF3)
		  {

			  //TODO
			  DMA3_CLEAR_IT = 1;

		  }

		  if (DMA1->ISR & DMA_ISR_TCIF4)
		  {

			  //TODO
			  DMA4_CLEAR_IT = 1;

		  }

		  if (DMA1->ISR & DMA_ISR_TCIF6)
		  {

			  //TODO
			  DMA6_CLEAR_IT = 1;

		  }

		  if ((DMA1->ISR & DMA_ISR_TCIF7) || (DMA1->ISR & DMA_ISR_HTIF7) || (DMA1->ISR & DMA_ISR_GIF7))
		  {

			  //TODO
			  DMA7_CLEAR_IT = 1;
			  DMA7_CLEAR_GIT = 1;
			  DMA7_CLEAR_HIT = 1;
		  }

		  if ((DMA1->ISR & DMA_ISR_TCIF8) || (DMA1->ISR & DMA_ISR_HTIF8) || (DMA1->ISR & DMA_ISR_GIF8))
		  {

			  //TODO
			  DMA8_CLEAR_IT = 1;
			  DMA8_CLEAR_GIT = 1;
			  DMA8_CLEAR_HIT = 1;
		  }
}

/*******************************************************************************
* Function Name  : DS_TIMx_Configuration
* Description    : Timers configuration for Data Sensing
* Input          : NONE
* Return         : None
*******************************************************************************/
void DS_TIMx_Configuration(void)
{
//  TIM8 Configuration: PWM1 Mode
/*
    TIM_TimeBaseStructure.TIM_Period = ((DataSensing.Init.Counter) >> 1);
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

    TIM_SelectOutputTrigger(TIM8, TIM_TRGOSource_Update);
*/
}

/*******************************************************************************
* Function Name  : DS_DMA_Configuration
* Description    : Analog to digital converter configuration
* Input          : NONE
* Return         : None
*******************************************************************************/
void DS_DMA_Configuration(void)
{

	   // DMA1 CHANNEL 1
	   DMA1_Channel1->CPAR  = (uint32_t)(&(ADC1->DR)); // peripheral (source) address
	   DMA1_Channel1->CMAR = (uint32_t)(DataSensing.Init.DataRegister+3); //MEMORY ADDRESS
	   DMA1_Channel1->CNDTR = ( uint16_t )ADC1_DATA_SIZE; //TRANSFER SIZE
	   //DMA1_Channel1->CCR |= DMA_CCR_TCIE; //ENABLE TRANSFER COMPLETE INTERRUPT
	   DMA1_Channel1->CCR |= ( DMA_CCR_EN ); //CHANNEL ENABLE


	   // DMA1 CHANNEL 5
	   DMA1_Channel5->CPAR  = (uint32_t)(&(ADC2->DR)); // peripheral (source) address
	   DMA1_Channel5->CMAR = (uint32_t)(DataSensing.Init.DataRegister+9); //MEMORY ADDRESS
	   DMA1_Channel5->CNDTR = ( uint16_t )ADC2_DATA_SIZE; //TRANSFER SIZE
	   //DMA1_Channel5->CCR |= DMA_CCR_TCIE; //ENABLE TRANSFER COMPLETE INTERRUPT
	   //DMA1_Channel5->CCR |= ( DMA_CCR_EN ); //CHANNEL ENABLE

	   // DMA1 CHANNEL 2
	   DMA1_Channel2->CPAR  = (uint32_t)(&(SPI4->DR)); // peripheral (source) address
	   DMA1_Channel2->CMAR = (uint32_t)(DataSensing.Init.DataRegister); //MEMORY ADDRESS
	   DMA1_Channel2->CNDTR = ( uint16_t )SPI4_DATA_SIZE; //TRANSFER SIZE
	   DMA1_Channel2->CCR |= DMA_CCR_TCIE; //ENABLE TRANSFER COMPLETE INTERRUPT
	   DMA1_Channel2->CCR |= ( DMA_CCR_EN ); //CHANNEL ENABLE


	   // DMA1 CHANNEL 3 SPI4 TX
	   DMA1_Channel3->CPAR  = (uint32_t)(&(SPI4->DR)); // peripheral (source) address
	   DMA1_Channel3->CMAR = (uint32_t)(to_grid_State_Control); //MEMORY ADDRESS
	   DMA1_Channel3->CNDTR = ( uint16_t )SPI4_DATA_SIZE; //TRANSFER SIZE
	   //DMA1_Channel3->CCR |= DMA_CCR_TCIE; //ENABLE TRANSFER COMPLETE INTERRUPT
	   //DMA1_Channel3->CCR |= ( DMA_CCR_EN ); //CHANNEL ENABLE

	   //HAL_SPI_Receive_DMA(&hspi4, (uint8_t*)DataSensing.Init.DataRegister, SPI4_DATA_SIZE);

	   // DMA1 CHANNEL 4 SPI3_DCAC RX
	   DMA1_Channel4->CPAR  = (uint32_t)(&(SPI3->DR)); // peripheral (source) address
	   DMA1_Channel4->CMAR = (uint32_t)(DCAC_DATA); //MEMORY ADDRESS
	   DMA1_Channel4->CNDTR = ( uint16_t )ADC4_DATA_SIZE; //TRANSFER SIZE
	   //DMA1_Channel3->CCR |= DMA_CCR_TCIE; //ENABLE TRANSFER COMPLETE INTERRUPT
	  // DMA1_Channel4->CCR |= ( DMA_CCR_EN ); //CHANNEL ENABLE

	   // DMA1 CHANNEL 6 SPI3_DCAC TX
	    DMA1_Channel6->CPAR  = (uint32_t)(&(SPI3->DR)); // peripheral (source) address
	    DMA1_Channel6->CMAR = (uint32_t)(to_grid_State_Control); //MEMORY ADDRESS
	    DMA1_Channel6->CNDTR = ( uint16_t )SPI4_DATA_SIZE; //TRANSFER SIZE
	   //DMA1_Channel3->CCR |= DMA_CCR_TCIE; //ENABLE TRANSFER COMPLETE INTERRUPT
	   // DMA1_Channel6->CCR |= ( DMA_CCR_EN ); //CHANNEL ENABLE

	   //DMAMUX1_Channel1->CCR |=DMAMUX_CxCR_SOIE;
	   //DMAMUX1_Channel4->CCR |=DMAMUX_CxCR_SOIE;


	   //DMAMUX1_RequestGenerator0->RGCR |=DMAMUX_RGxCR_OIE;
	   //DMAMUX1_RequestGenerator1->RGCR |=DMAMUX_RGxCR_OIE;
	   //DMAMUX1_RequestGenerator2->RGCR |=DMAMUX_RGxCR_OIE;
	   //DMAMUX1_RequestGenerator3->RGCR |=DMAMUX_RGxCR_OIE;
}

/*******************************************************************************
* Function Name  : DS_ADCx_Configuration
* Description    : Analog to digital converter configuration
* Input          : NONE
* Return         : None
*******************************************************************************/
void DS_ADCx_Configuration(void)
{


  DS_ADC_EnableAndCalibrate(ADC1);
  DS_ADC_EnableAndCalibrate(ADC2);
  //DS_ADC_EnableAndCalibrate(ADC3);
  //DS_ADC_EnableAndCalibrate(ADC4);
  //DS_ADC_EnableAndCalibrate(ADC5);

  //ADC5->CR |=ADC_CR_ADSTART;
  //ADC1->CR |=ADC_CR_ADSTART;

  //ADC2->CR |=ADC_CR_ADSTART;
  //ADC3->CR |=ADC_CR_ADSTART;

  //ADC4->CR |=ADC_CR_ADSTART;
}


/*******************************************************************************
* Function Name  : DCAC_ADC_EnableAndCalibrate
* Description    : Enable and calibrate Analog to digital converter
* Input          : ADCx the adc to calibrate
* Return         : None
*******************************************************************************/
void DS_ADC_EnableAndCalibrate(ADC_TypeDef* ADCx)
{
	  __IO uint32_t wait_loop_index = 0U;
	  #if (USE_TIMEOUT == 1)
	  uint32_t Timeout = 0U; /* Variable used for timeout management */
	  #endif /* USE_TIMEOUT */
	//if(LL_ADC_ClearFlag_ADRDY(ADCx) == )
	  LL_ADC_ClearFlag_ADRDY(ADCx);
	if (LL_ADC_IsEnabled(ADCx) == 0)
	  {
	    /* Disable ADC deep power down (enabled by default after reset state) */
	    LL_ADC_DisableDeepPowerDown(ADCx);

	    /* Enable ADC internal voltage regulator */
	    LL_ADC_EnableInternalRegulator(ADCx);

	    /* Delay for ADC internal voltage regulator stabilization.                */
	    /* Compute number of CPU cycles to wait for, from delay in us.            */
	    /* Note: Variable divided by 2 to compensate partially                    */
	    /*       CPU processing cycles (depends on compilation optimization).     */
	    /* Note: If system core clock frequency is below 200kHz, wait time        */
	    /*       is only a few CPU processing cycles.                             */
	    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
	    while(wait_loop_index != 0)
	    {
	      wait_loop_index--;
	    }

	    /* Run ADC self calibration */
	    LL_ADC_StartCalibration(ADCx, LL_ADC_SINGLE_ENDED);

	    /* Poll for ADC effectively calibrated */
	    #if (USE_TIMEOUT == 1)
	    Timeout = ADC_CALIBRATION_TIMEOUT_MS;
	    #endif /* USE_TIMEOUT */

	    while (LL_ADC_IsCalibrationOnGoing(ADCx) != 0)
	    {
	    #if (USE_TIMEOUT == 1)
	      /* Check Systick counter flag to decrement the time-out value */
	      if (LL_SYSTICK_IsActiveCounterFlag())
	      {
	        if(Timeout-- == 0)
	        {
	        /* Time-out occurred. Set LED to blinking mode */
	        LED_Blinking(LED_BLINK_ERROR);
	        }
	      }
	    #endif /* USE_TIMEOUT */
	    }

	    /* Delay between ADC end of calibration and ADC enable.                   */
	    /* Note: Variable divided by 2 to compensate partially                    */
	    /*       CPU processing cycles (depends on compilation optimization).     */
	    wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
	    while(wait_loop_index != 0)
	    {
	      wait_loop_index--;
	    }

	    /* Enable ADC */
	    LL_ADC_Enable(ADCx);

	    /* Poll for ADC ready to convert */
	    #if (USE_TIMEOUT == 1)
	    Timeout = ADC_ENABLE_TIMEOUT_MS;
	    #endif /* USE_TIMEOUT */

	    while (LL_ADC_IsActiveFlag_ADRDY(ADCx) == 0)
	    {
	    #if (USE_TIMEOUT == 1)
	      /* Check Systick counter flag to decrement the time-out value */
	      if (LL_SYSTICK_IsActiveCounterFlag())
	      {
	        if(Timeout-- == 0)
	        {
	        /* Time-out occurred. Set LED to blinking mode */
	        LED_Blinking(LED_BLINK_ERROR);
	        }
	      }
	    #endif /* USE_TIMEOUT */
	    }

	    /* Note: ADC flag ADRDY is not cleared here to be able to check ADC       */
	    /*       status afterwards.                                               */
	    /*       This flag should be cleared at ADC Deactivation, before a new    */
	    /*       ADC activation, using function "LL_ADC_ClearFlag_ADRDY()".       */

	    ADCx->CR |=ADC_CR_ADSTART;
	  }

}

/*******************************************************************************
* Function Name  : DS_RefreshDisplay
* Description    : Display same info from the object in display
* Input          : Error and step of phase shift
* Return         : None
*******************************************************************************/
/*
void DS_RefreshDisplay(DS_Error_t err)
{
    char strLineMessage[64];
    DS_Data_t buff;

    memcpy(&buff, (u32 *)DataSensing.Init.DataRegister, sizeof(DS_Data_t));

    LCD_DisplayStringLine(Line0, "    Data Sensing    ");

    if (DS_GetStatus() == DS_Running)
        sprintf(strLineMessage, "ON  %d (%.2f kHz)", DataSensing.Init.Counter, 1/(((double)(DataSensing.Init.Counter) * CLOCK_DURATION_MICROS))*1000);
    else
        sprintf(strLineMessage, "OFF %d (%.2f kHz)", DataSensing.Init.Counter, 1/(((double)(DataSensing.Init.Counter) * CLOCK_DURATION_MICROS))*1000);
    LCD_ClearLine(Line1);
    LCD_DisplayStringLine(Line1, (u8 *)strLineMessage);

    sprintf(strLineMessage, "AC %.5d A %.5d V",buff.AC_LineCurrent, buff.AC_LineVoltage );
    LCD_ClearLine(Line2);
    LCD_DisplayStringLine(Line2, (u8 *)strLineMessage);

    sprintf(strLineMessage, "DC %.5d A %.5d V",buff.DC_PanelCurrent, buff.DC_PanelVoltage );
    LCD_ClearLine(Line3);
    LCD_DisplayStringLine(Line3, (u8 *)strLineMessage);

    sprintf(strLineMessage, "DC BUS     %.5d V", buff.DC_BusVoltage );
    LCD_ClearLine(Line4);
    LCD_DisplayStringLine(Line4, (u8 *)strLineMessage);

    LCD_ClearLine(Line5);
    LCD_ClearLine(Line6);
    LCD_ClearLine(Line7);

    sprintf(strLineMessage, "Error 0x%.8X    ", err);
    LCD_ClearLine(Line8);
    LCD_DisplayStringLine(Line8, (u8 *)strLineMessage);

    LCD_ClearLine(Line9);

}
*/
static void MX_TIM1_Init(void)
{

	  /* USER CODE BEGIN TIM1_Init 0 */

	  /* USER CODE END TIM1_Init 0 */

	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};
	  TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
	  TIM_OC_InitTypeDef sConfigOC = {0};
	  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	  /* USER CODE BEGIN TIM1_Init 1 */

	  /* USER CODE END TIM1_Init 1 */
	  htim1.Instance = TIM1;
	  htim1.Init.Prescaler = 0;
	  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim1.Init.Period = 65535;
	  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim1.Init.RepetitionCounter = 0;
	  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  if (HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_BKIN;
	  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
	  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_HIGH;
	  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP7;
	  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
	  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_HIGH;

	  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  //TIM1->AF1 &= ~TIM1_AF1_BKINE; //DISABLE

	  sConfigOC.OCMode = TIM_OCMODE_PWM1;
	  sConfigOC.Pulse = 65535;
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
	  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
	  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
	  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	  {
	    Error_Handler();
	  }



	  HAL_TIMEx_EnableDeadTimePreload(&htim1);
	  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
	  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
	  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	  sBreakDeadTimeConfig.DeadTime = 150; // 150 = 1 us
	  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE; //zmiana
	  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	  sBreakDeadTimeConfig.BreakFilter = 3; // 5 = 160 ns
	  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
	  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	  sBreakDeadTimeConfig.Break2Filter = 0;
	  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
	  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN TIM1_Init 2 */

	  /* USER CODE END TIM1_Init 2 */
	  HAL_TIM_MspPostInit(&htim1);

}

static void MX_TIM1_Init2(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;

  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }


}
