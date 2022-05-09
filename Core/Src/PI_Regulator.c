/*
 * PI_Regulator.c
 *
 *  Created on: Aug 6, 2020
 *      Author: user01
 */
#include "400WControl.h"
#include "DCDC_Converter.h"

static s32 Integral_Voltage;
static s32 Integral_MPPT;
static s32 Integral_Direct_Current;
static s32 Integral_Quadrature_Current;
static s32 Integral_Reactive_Power;
static s32 Integral_Active_Power;
static s32 Integral_Bus_Voltage;


//******************************************************************************
/**
  * @brief  This function compute the output of a PI regulator sum of its
  *         proportional and integral terms
  * @param  CPI PI regulator object
  * @param  int32_t Present process variable error, intended as the reference
  *         value minus the present process variable value
  * @retval int16_t PI output
  */


//*********************************************************************************************************************
void PID_Init(PI_ControllerTYPEDEF *Direct_PID, PI_ControllerTYPEDEF *Quadrature_PID,PI_ControllerTYPEDEF *Reactive_PID,PI_ControllerTYPEDEF *Active_PID,PI_ControllerTYPEDEF *Bus_DC_PID, PI_ControllerTYPEDEF *PLL_PID,PI_ControllerTYPEDEF *MPPT_PID)

{
  PLL_PID->Reference =0;    //PID_DIRECT_VOLTAGE_REFERENCE;
  PLL_PID->Kp_Gain =2700;//2750;//405; //500//3000//450 //50Hz   //PID_DIRECT_VOLTAGE_KP_DEFAULT;8000
  PLL_PID->Ki_Gain =35;//35;//4; //10//100/20//50Hz;   //PID_DIRECT_VOLTAGE_KI_DEFAULT;500

  PLL_PID->Lower_Limit_Output=-S16_MAX;         //Lower Limit for Output limitation
  PLL_PID->Upper_Limit_Output=S16_MAX;          //Upper Limit for Output limitation
  PLL_PID->Max_PID_Output = FALSE;
  PLL_PID->Min_PID_Output = FALSE;

  //PI Id
  Direct_PID->Reference =0;                       //Output_PID_Reactive_Power; //Brak zmian (load żarówki)
  Direct_PID->Kp_Gain = 200;//1100;//870;//2407;//3000;//9488;//7600;//4253428;//4100;//6500;  // filtro RIPEM 7500              //PID_KP_DEFAULT;
  Direct_PID->Ki_Gain = 2000;//2857;//4489;//425;//56;//200;//10;//20;//                     //PID_KI_DEFAULT;

  Direct_PID->Lower_Limit_Output=  -6900;//-8900;// -6900;       //Lower Limit for Output limitation
  Direct_PID->Upper_Limit_Output=  6900;// 8900;//6900;      //Upper Limit for Output limitation  THD
  Direct_PID->Max_PID_Output =      FALSE;
  Direct_PID->Min_PID_Output =      FALSE;

  //PI Iq
  Quadrature_PID->Reference =0;           //PID_DEFAULT;
  Quadrature_PID->Kp_Gain = 200;//1178;//134;//1158;//3350;//3400;//4100;//6190;//4000;      // filtro RIPEM 12000  power factor
  Quadrature_PID->Ki_Gain = 1500;//3660;//7452;//700;//650;//700;//1;    //20   //PID_KI_DEFAULT;

  Quadrature_PID->Lower_Limit_Output=-32067;//-32030;    // Lower Limit for Output limitation
  Quadrature_PID->Upper_Limit_Output= 32067;//32030;    //  Upper Limit for Output limitation
  Quadrature_PID->Max_PID_Output =     FALSE;
  Quadrature_PID->Min_PID_Output =     FALSE;

  // cambiamo il ref....
  Reactive_PID->Reference =0;//30               //PID REFERENCE DEFAULT
  Reactive_PID->Kp_Gain =1500; // filtro RIPEM 3400 //PID_KP_DEFAULT; //Brak zmian (load żarówki)
  Reactive_PID->Ki_Gain =1500;                 //PID_KI_DEFAULT;      //Brak zmian (load żarówki)

  Reactive_PID->Lower_Limit_Output=-32767;    //Lower Limit for Output limitation
  Reactive_PID->Upper_Limit_Output=32767;     //Upper Limit for Output limitation
  Reactive_PID->Max_PID_Output = FALSE;
  Reactive_PID->Min_PID_Output = FALSE;

  Bus_DC_PID->Reference = 10500;//11000; //7692; //8200;//8500; //13000; //15000  //Brak zmian (load żarówki)
  Bus_DC_PID->Kp_Gain = 8000;//8116;//8100;//8100;//10000;//24000;   //5000      //PID_KP_DEFAULT;
  Bus_DC_PID->Ki_Gain = 200;//600;//200;//1050;//20//400;    //300   //PID_KI_DEFAULT;

  Bus_DC_PID->Lower_Limit_Output= 0;  //S16_MIN;     //Lower Limit for Output limitation
  Bus_DC_PID->Upper_Limit_Output= 32767; //Upper Limit for Output limitation
  Bus_DC_PID->Max_PID_Output = FALSE;
  Bus_DC_PID->Min_PID_Output = FALSE;
}


/*******************************************************************************
* Function Name  : PID_DirectCurrent
* Description    : Compute the PI output for the Direct Current.
* Input          : none

* Output         : s16
* Return         : None
*******************************************************************************/
s16 PID_DirectCurrent(PI_ControllerTYPEDEF *Direct_PID, s16 qIdirect_Input)
{
  s32 wProportional_Term=0, wIntegral_Term=0, wIntegralTerm=0, wOutput_32,wIntegral_sum_temp=0;
  s32 Error;

  Error = (Direct_PID->Reference- qIdirect_Input); // Inverter_Current_Components

  /* Proportional term computation*/
  wProportional_Term = Direct_PID->Kp_Gain * Error;

  /* Integral term computation */
  if (Direct_PID->Ki_Gain == 0)
  {
    wIntegralTerm = 0;
  }
  else
  {
    wIntegral_Term = Direct_PID->Ki_Gain * Error;
    wIntegral_sum_temp = wIntegralTerm + wIntegral_Term;

    if (wIntegral_sum_temp < 0)
    {
      if (wIntegralTerm > 0)
      {
        if (wIntegral_Term > 0)
        {
          wIntegral_sum_temp = -S32_MAX;
        }
      }
    }
    else
    {
      if (wIntegralTerm < 0)
      {
        if (wIntegral_Term < 0)
        {
          wIntegral_sum_temp = S32_MAX;
        }
      }
    }

//    if (wIntegral_sum_temp > wUpperIntegralLimit)
//    {
//      wIntegralTerm = wUpperIntegralLimit;
//    }
//    else if (wIntegral_sum_temp < wLowerIntegralLimit)
//    {
//      wIntegralTerm = wLowerIntegralLimit;
//    }
//    else
//    {
      wIntegralTerm = wIntegral_sum_temp;
//    }
  }

 /* WARNING: the below instruction is not MISRA compliant, user should verify
             that Cortex-M3 assembly instruction ASR (arithmetic shift right)
             is used by the compiler to perform the shifts (instead of LSR
             logical shift right)*/

  wOutput_32 = (wProportional_Term/4096) + (wIntegralTerm/4096);

  if (wOutput_32 > Direct_PID->Upper_Limit_Output)
  {
    wOutput_32 = Direct_PID->Upper_Limit_Output;
  }
  else if (wOutput_32 < Direct_PID->Lower_Limit_Output)
  {
    wOutput_32 = Direct_PID->Lower_Limit_Output;
  }
  else
  {}

  return((s16)(wOutput_32));
}

/*******************************************************************************
* Function Name  : PID_QuadratureRegulator
* Description    : Compute the PI output for the Quadrature Voltage.
* Input          : none

* Output         : s16
* Return         : None
*******************************************************************************/

s16 PID_QuadratureCurrent(PI_ControllerTYPEDEF *Quadrature_PID, s16 qIquadrature_Input)
{
  s32 wProportional_Term=0, wIntegral_Term=0, wIntegralTerm=0, wOutput_32,wIntegral_sum_temp=0;
  s32 Error;

  Error = (Quadrature_PID->Reference- qIquadrature_Input); // Inverter_Current_Components

  /* Proportional term computation*/
  wProportional_Term = Quadrature_PID->Kp_Gain * Error;

  /* Integral term computation */
  if (Quadrature_PID->Ki_Gain == 0)
  {
    wIntegralTerm = 0;
  }
  else
  {
    wIntegral_Term = Quadrature_PID->Ki_Gain * Error;
    wIntegral_sum_temp = wIntegralTerm + wIntegral_Term;

    if (wIntegral_sum_temp < 0)
    {
      if (wIntegralTerm > 0)
      {
        if (wIntegral_Term > 0)
        {
          wIntegral_sum_temp = -S32_MAX;
        }
      }
    }
    else
    {
      if (wIntegralTerm < 0)
      {
        if (wIntegral_Term < 0)
        {
          wIntegral_sum_temp = S32_MAX;
        }
      }
    }

//    if (wIntegral_sum_temp > wUpperIntegralLimit)
//    {
//      wIntegralTerm = wUpperIntegralLimit;
//    }
//    else if (wIntegral_sum_temp < wLowerIntegralLimit)
//    {
//      wIntegralTerm = wLowerIntegralLimit;
//    }
//    else
//    {
      wIntegralTerm = wIntegral_sum_temp;
//    }
  }

 /* WARNING: the below instruction is not MISRA compliant, user should verify
             that Cortex-M3 assembly instruction ASR (arithmetic shift right)
             is used by the compiler to perform the shifts (instead of LSR
             logical shift right)*/

  wOutput_32 = (wProportional_Term/4096) + (wIntegralTerm/4096);

  if (wOutput_32 > Quadrature_PID->Upper_Limit_Output)
  {
    wOutput_32 = Quadrature_PID->Upper_Limit_Output;
  }
  else if (wOutput_32 < Quadrature_PID->Lower_Limit_Output)
  {
    wOutput_32 = Quadrature_PID->Lower_Limit_Output;
  }
  else
  {}

  return((s16)(wOutput_32));
}

/*******************************************************************************
* Function Name  : PID_Reactice_POwer
* Description    : Compute the PI output for the Quadrature Voltage.
* Input          : none

* Output         : s16
* Return         : None
*******************************************************************************/

s16 PID_Reactive_Power(PI_ControllerTYPEDEF *Reactive_PID, s16 qReactive_Input)
{
  s32 wProportional_Term=0, wIntegral_Term=0, wIntegralTerm=0, wOutput_32,wIntegral_sum_temp=0;
  s32 Error;

  Error = (Reactive_PID->Reference- qReactive_Input);

  /* Proportional term computation*/
  wProportional_Term = Reactive_PID->Kp_Gain * Error;

  /* Integral term computation */
  if (Reactive_PID->Ki_Gain == 0)
  {
    wIntegralTerm = 0;
  }
  else
  {
    wIntegral_Term = Reactive_PID->Ki_Gain * Error;
    wIntegral_sum_temp = wIntegralTerm + wIntegral_Term;

    if (wIntegral_sum_temp < 0)
    {
      if (wIntegralTerm > 0)
      {
        if (wIntegral_Term > 0)
        {
          wIntegral_sum_temp = -S32_MAX;
        }
      }
    }
    else
    {
      if (wIntegralTerm < 0)
      {
        if (wIntegral_Term < 0)
        {
          wIntegral_sum_temp = S32_MAX;
        }
      }
    }

//    if (wIntegral_sum_temp > wUpperIntegralLimit)
//    {
//      wIntegralTerm = wUpperIntegralLimit;
//    }
//    else if (wIntegral_sum_temp < wLowerIntegralLimit)
//    {
//      wIntegralTerm = wLowerIntegralLimit;
//    }
//    else
//    {
      wIntegralTerm = wIntegral_sum_temp;
//    }
  }

 /* WARNING: the below instruction is not MISRA compliant, user should verify
             that Cortex-M3 assembly instruction ASR (arithmetic shift right)
             is used by the compiler to perform the shifts (instead of LSR
             logical shift right)*/

  wOutput_32 = (wProportional_Term/4096) + (wIntegralTerm/32768);

  if (wOutput_32 > Reactive_PID->Upper_Limit_Output)
  {
    wOutput_32 = Reactive_PID->Upper_Limit_Output;
  }
  else if (wOutput_32 < Reactive_PID->Lower_Limit_Output)
  {
    wOutput_32 = Reactive_PID->Lower_Limit_Output;
  }
  else
  {}

  return((s16)(wOutput_32));
}

