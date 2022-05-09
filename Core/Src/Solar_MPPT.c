/*
 * Solar_MPPT.c
 *
 *  Created on: Aug 6, 2020
 *      Author: user01
 */

#include "DataSensing.h"
#include "Solar_MPPT.h"
#define STEPS 1

u8 MPPT_Update=0;
bool MPPTstarted=FALSE;
//MPPT calc parameters
s16 Vpanel=4;
s16 Ipanel=0;
extern s16 PV_Voltage;
extern s16 PV_Current;
static u32 Pow=0;
static u32 Powprev=0;
u32 DC_comp_Power=0;
u32 DC_comp_Power_prev=0;
static u8 flag_mppt=0;
s16 Vpanel_prev=4095; //Set for Voc
extern u16 CCR_Val;
extern u32 ActualFreq_Val;
u16 CCR_Val_step=5; //mod GS 24/02
u32 ActualFreq_val_step=5;
u8 tempo=1;


void Step_modify()
{
  flag_mppt=0;
  if((ActualFreq_val_step-1)<1)  ActualFreq_val_step=1;
  else ActualFreq_val_step=ActualFreq_val_step--;
}


void MPPT_func()
{

  //GPIO_WriteBit(GPIOA, GPIO_Pin_4, !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_4)); //COMMENTED OUT PA4 NOT AVAILABLE ON  SCHEMATICS
  ActualFreq_Val = (u32) DCDC_GetFrequency();




  if(ActualFreq_Val > 141000)
  {
	  ActualFreq_Val=ActualFreq_Val - 100;
	  DCDC_SetFrequency((u32)ActualFreq_Val);
  }


  else
  {
	  if(ActualFreq_Val <= 140000)
	  {
		  DCDC_SetFrequency((u32)140000);
	  }
	  else{
		  ActualFreq_Val=ActualFreq_Val - 1;
		  DCDC_SetFrequency((u32)ActualFreq_Val);
	  }

  }


/*
  Vpanel=PV_Voltage;
  Ipanel=PV_Current;

 if(tempo==1 && Ipanel<350)
 {
   if((ActualFreq_Val + ActualFreq_val_step) >= MAX_FREQ)
     {
	  ActualFreq_Val=MAX_FREQ;
      DCDC_SetFrequency((u32)ActualFreq_Val);
     }
   else
     {
	  ActualFreq_Val=ActualFreq_Val + 10;
	  DCDC_SetFrequency((u32)ActualFreq_Val);
     }
 }
 else { tempo=0; }

  if(Ipanel==0) Ipanel=1;

  Pow = (u32)((u32)Vpanel * (u32)Ipanel);
  if(flag_mppt==1) { Step_modify(); }

   //Pow CHANGHES AT DIFFERENT DUTY CYCLE VALUE (LAST AND PREVIOUS VALUE)
    if((Pow >= Powprev) && (Vpanel <= Vpanel_prev))
     {
	if((ActualFreq_Val + ActualFreq_val_step) >= MAX_FREQ)
         {
		ActualFreq_Val = MAX_FREQ;
		DCDC_SetFrequency((u32)ActualFreq_Val);
         }
	else
         {
		ActualFreq_Val=ActualFreq_Val + ActualFreq_val_step;
           DCDC_SetFrequency((u32)ActualFreq_Val);
    	 }
     }
//   if((Pow > Powprev) && (Vpanel > Vpanel_prev) && ((s16)(Pow-Powprev)>=5000) )  //MODALITA' DCDC
       if((Pow > Powprev) && (Vpanel > Vpanel_prev)) //MODALITA' PANNELLO
	{
	if((ActualFreq_Val - ActualFreq_val_step)<=MIN_FREQ)
	  {
		ActualFreq_Val = MIN_FREQ;
		DCDC_SetFrequency((u32)MIN_FREQ);
	  }
	else
          {
		ActualFreq_Val = ActualFreq_Val - ActualFreq_val_step;
		DCDC_SetFrequency((u32)ActualFreq_Val);
	  }
        }
   //   if((Pow<Powprev) && (Vpanel<Vpanel_prev) && ((s16)(Powprev-Pow)>=5000)) //MODALITA' DCDC
	if((Pow<Powprev) && (Vpanel<Vpanel_prev)) //MODALITA' PANNELLO
	  {

	  if((ActualFreq_Val +ActualFreq_val_step)<=MIN_FREQ)
  	    {
		  ActualFreq_Val = MIN_FREQ;
		  DCDC_SetFrequency((u32)MIN_FREQ);
	    }
	   else
            {
		   ActualFreq_Val = ActualFreq_Val - ActualFreq_val_step;
	     flag_mppt=1; //direction changed flag
	     DCDC_SetFrequency((u32)ActualFreq_Val);
            }
          }

	   if((Pow<=Powprev) && (Vpanel>=Vpanel_prev))
	   {
	     if((ActualFreq_Val + ActualFreq_val_step)>=MAX_FREQ)
	       {
	    	 ActualFreq_Val = MAX_FREQ;
	    	 DCDC_SetFrequency((u32)ActualFreq_Val);
   	       }
	     else
             {
	    	 ActualFreq_Val = ActualFreq_Val + ActualFreq_val_step;
	    	 DCDC_SetFrequency((u32)ActualFreq_Val);
             }
           }

  Powprev = Pow;
  Vpanel_prev= Vpanel;
  Pow=0;
  */
}


