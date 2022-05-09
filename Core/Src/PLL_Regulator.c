/*
 * PLL_Regulator.c
 *
 *  Created on: Aug 6, 2020
 *      Author: user01
 */
#include "400WControl.h"
#include "DCDC_Converter.h"
#include "PLL_Regulator.h"

static s32 Integral_Voltage;
static s32 Integral_MPPT;
static s32 Integral_Direct_Current;
static s32 Integral_Quadrature_Current;
static s32 Integral_Reactive_Power;
static s32 Integral_Active_Power;
static s32 Integral_Bus_Voltage;


u16 PLL_par_red_kp_n=0;
u16 PLL_par_red_ki_n=0;
extern bool PLL_par_red_kp;
extern bool PLL_par_red_ki;

extern bool PLL_reducing;

//******************************************************************************
/**
  * @brief  This function compute the output of a PI regulator sum of its
  *         proportional and integral terms
  * @param  CPI PI regulator object
  * @param  int32_t Present process variable error, intended as the reference
  *         value minus the present process variable value
  * @retval int16_t PI output
  */

/*******************************************************************************
* Function Name  : PID_Init_Integral_Part
* Description    : INITIALIZATION integral Term contribute
* Input          : PI Structures

* Output         : None
* Return         : None
*******************************************************************************/

//void PID_Init_Integral_Part(PI_ControllerTYPEDEF *Direct_PID, PI_ControllerTYPEDEF *Quadrature_PID,PI_ControllerTYPEDEF *Reactive_PID,PI_ControllerTYPEDEF *Active_PID)
void PID_Init_Integral_Part(void)
{
   Integral_Voltage = 0;  // reset integral value
   Integral_MPPT = 0;
   Integral_Direct_Current=0;
   Integral_Quadrature_Current=0;
   Integral_Reactive_Power=0;
   Integral_Active_Power=0;
   Integral_Bus_Voltage=0;

}

/*******************************************************************************
* Function Name  : PLL_PID_Regulator
* Description    : Compute the PI output for the Direct Voltage regulation loop.
* Input          : none

* Output         : s16
* Return         : None
*******************************************************************************/

s16 PLL_PID_Regulator(PI_ControllerTYPEDEF *PLL_PID, Volt_Components Grid_Voltage_Components_Input)
{

s32 Proportional_Term, Integral_Term, output_s32;
s32 Error;

Error = (PLL_PID->Reference - Grid_Voltage_Components_Input.qV_Direct); // Grid_Voltage_Components_Input

//*****************************************************************************
// *********************** PLL Ki reducing ************************************
if(PLL_par_red_ki==TRUE && PLL_par_red_ki_n==1750)
{
//GPIO_WriteBit(GPIOA, GPIO_Pin_6, !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_6));

  if((s16)(PLL_PID->Ki_Gain - 2)<=1)
   {
     PLL_PID->Ki_Gain=1;
     PLL_par_red_ki=FALSE;
   }
  else
   {
    PLL_PID->Ki_Gain = PLL_PID->Ki_Gain - 2;
   }
    PLL_par_red_ki_n=0;
}
else if(PLL_par_red_ki==TRUE && PLL_par_red_ki_n!=1750)
{
  PLL_par_red_ki_n++;
}

//*********************** PLL Kp reducing ************************************
if(PLL_par_red_kp==TRUE && PLL_par_red_kp_n==1750 && PLL_PID->Ki_Gain==1)
{

  if((s16)(PLL_PID->Kp_Gain - 30)<=300)
   {
     PLL_PID->Kp_Gain=300;
     PLL_par_red_kp=FALSE;
   }
  else
   {
    PLL_PID->Kp_Gain = PLL_PID->Kp_Gain - 30;
   }
    PLL_par_red_kp_n=0;
}
else if(PLL_par_red_kp==TRUE && PLL_par_red_kp_n!=1750 && PLL_PID->Ki_Gain==1)
{
  PLL_par_red_kp_n++;
}

if(PLL_PID->Kp_Gain==300 && PLL_PID->Ki_Gain==1 && PLL_reducing!=TRUE)
  {
    PLL_reducing=TRUE;
    PLL_par_red_kp=FALSE;
    PLL_par_red_ki=FALSE;
  }

 // ****************************************************************************

  // Proportional term computation
  Proportional_Term = PLL_PID->Kp_Gain * Error;

  // Integral term computation
  Integral_Term = PLL_PID->Ki_Gain * Error;


 if ( (Integral_Voltage >= 0) && (Integral_Term >= 0) && (PLL_PID->Max_PID_Output == FALSE) )  // freeze integral term in case of over/underflow
    {
      if ( (s32)(Integral_Voltage + Integral_Term) < 0)
      {
        Integral_Voltage = S32_MAX;
      }
      else
      {
        Integral_Voltage += Integral_Term;
      }
    }
  else if ( (Integral_Voltage <= 0) && (Integral_Term <= 0) && (PLL_PID->Min_PID_Output == FALSE) )
    {
     if ( (s32)(Integral_Voltage + Integral_Term) > 0)
     {
       Integral_Voltage = -S32_MAX;
     }
     else
     {
       Integral_Voltage += Integral_Term;
     }
    }
  else if ( (Integral_Voltage <= 0) && (Integral_Term >= 0) )
    {
     Integral_Voltage += Integral_Term;
    }
  else if ( (Integral_Voltage >= 0) && (Integral_Term <= 0) )
    {
     Integral_Voltage += Integral_Term;
    }


  output_s32 = (Proportional_Term/8192 + Integral_Voltage/32768);


    if (output_s32 >= (s32)(PLL_PID->Upper_Limit_Output))
      {
      PLL_PID->Max_PID_Output = TRUE;
      return(PLL_PID->Upper_Limit_Output);
      }
    else if (output_s32 < (s32)(PLL_PID->Lower_Limit_Output))
      {
      PLL_PID->Min_PID_Output = TRUE;
      return(PLL_PID->Lower_Limit_Output);
      }
    else
      {
      PLL_PID->Min_PID_Output = FALSE;
      PLL_PID->Max_PID_Output = FALSE;


      return((s16)(output_s32));
      }

}

/*******************************************************************************
* Function Name  :
* Description    : Compute the PI output for the BUS_VOLTAGE.
* Input          : none

* Output         : s16
* Return         : None
*******************************************************************************/

s16 PID_Bus_Voltage(PI_ControllerTYPEDEF *Bus_DC_PID, u16 qBus_Voltage_Input)
{

s32 Proportional_Term, Integral_Term, output_s32;

s32 Error;


  // cambiato il segno
   Error = (qBus_Voltage_Input-Bus_DC_PID->Reference);

   Proportional_Term =Bus_DC_PID->Kp_Gain * Error;

   // Integral term computation
   Integral_Term = Bus_DC_PID->Ki_Gain * Error;


 if ( (Integral_Bus_Voltage >= 0) && (Integral_Term >= 0) && (Bus_DC_PID->Max_PID_Output == FALSE) )  // freeze integral term in case of over/underflow
    {
      if ( (s32)(Integral_Bus_Voltage + Integral_Term) < 0)
      {
        Integral_Bus_Voltage = S32_MAX;
      }
      else
      {
        Integral_Bus_Voltage += Integral_Term;
      }
    }
  else if ( (Integral_Bus_Voltage <= 0) && (Integral_Term <= 0) && (Bus_DC_PID->Min_PID_Output == FALSE) )
    {
     if ( (s32)(Integral_Bus_Voltage + Integral_Term) > 0)
     {
       Integral_Bus_Voltage = -S32_MAX;
     }
     else
     {
       Integral_Bus_Voltage += Integral_Term;
     }
    }
  else if ( (Integral_Bus_Voltage <= 0) && (Integral_Term >= 0) )
    {
     Integral_Bus_Voltage += Integral_Term;
    }
  else if ( (Integral_Bus_Voltage >= 0) && (Integral_Term <= 0) )
    {
     Integral_Bus_Voltage += Integral_Term;
    }


  output_s32 = (Proportional_Term/4096 + Integral_Bus_Voltage/(32768));


    if (output_s32 >= (s32)(Bus_DC_PID->Upper_Limit_Output))
      {
      Bus_DC_PID->Max_PID_Output = TRUE;
      return(Bus_DC_PID->Upper_Limit_Output);
      }
    else if (output_s32 < (s32)(Bus_DC_PID->Lower_Limit_Output))
      {
      Bus_DC_PID->Min_PID_Output = TRUE;
      return(Bus_DC_PID->Lower_Limit_Output);
      }
    else
      {
      Bus_DC_PID->Min_PID_Output = FALSE;
      Bus_DC_PID->Max_PID_Output = FALSE;


      return((s16)(output_s32));


      }

}

