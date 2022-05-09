/*
 * hwconfig.c
 *
 *  Created on: Aug 6, 2020
 *      Author: user01
 */
#include "hw_config.h"
//#include "lcd.h"
//#include "color_lcd.h"
#include "DCDC_Converter.h"
#include "DCAC_Inverter.h"
#include "DataSensing.h"
#include "400WControl.h"
#include "DAC_debug.h"
#include "photovSDK.h"


static vu32 TimingDelay = 0;


/*******************************************************************************
* Function Name  : Delay
* Description    : Inserts a delay time.
* Input          : nCount: specifies the delay time length (time base 1 ms).
* Output         : None
* Return         : None
*******************************************************************************/
void Delay(u32 nCount)
{
  TimingDelay = nCount;

  // SysTick_SetReload(72000);

  ///* Enable the SysTick Interrupt */
  //SysTick_ITConfig(ENABLE);

  /* Enable the SysTick Counter */
  //SysTick_CounterCmd(SysTick_Counter_Enable);

  HAL_Delay(TimingDelay);

  while(TimingDelay != 0)
  {
  }

  /* Disable the SysTick Counter */
  //SysTick_CounterCmd(SysTick_Counter_Disable);

  /* Clear the SysTick Counter */
  //SysTick_CounterCmd(SysTick_Counter_Clear);


}

/*******************************************************************************
* Function Name  : Decrement_TimingDelay
* Description    : Decrements the TimingDelay variable.
* Input          : None
* Output         : TimingDelay
* Return         : None
*******************************************************************************/
void Decrement_TimingDelay(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}
