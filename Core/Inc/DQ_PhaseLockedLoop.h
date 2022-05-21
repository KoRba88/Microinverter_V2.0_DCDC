/*
 * DQ_PhaseLockedLoop.h
 *
 *  Created on: Aug 6, 2020
 *      Author: user01
 */

#ifndef INC_DQ_PHASELOCKEDLOOP_H_
#define INC_DQ_PHASELOCKEDLOOP_H_

#include "main.h"

//extern const s16 Sin_Cos_Table[256];

#define STEPS 		        512 //256     /*number of entries of the sine look up table*/
#define OFFSET 		        64	/* Offset for cos(Theta)=sin_cos_Table[index_sin+offset]*/
#define INDUCTANCE_VALUE        380    //19 -> 000uH in q1.15 format q1.15= 1*10-3*2-15/  65     //1117----- 34mH in q1.15= 34*10-3*2-15/
#define SAMPLES                 256

#define FILTERFREQ 1200
#define CUTTOFREQ               65    //in Hz

#define SAMPLINGFREQ         (TIMF_PWM_FREQ / ADC_SAMPLING_AVERAGES)    //in Hz
#define NUMBEROFPOINT     ((0.443*SAMPLINGFREQ)/CUTTOFREQ)

#define  NUMBERS_OF_SAMPLES  (SAMPLINGFREQ/FILTERFREQ)

#define SAMPLING_TIME       (s16)((CORE_CLOCK / SAMPLINGFREQ)-1) //(s16)((4096)-1)

#define GRID_FREQ_MIN (SAMPLINGFREQ/47)
#define GRID_FREQ_MAX (SAMPLINGFREQ/53)

Volt_Components DQ_PLL_Grid(s16);

Curr_Components DQ_Current_Inverter(s16,s16);

Volt_AlphaBeta_Components Rev_Park(s16 ,s16);

Volt_Components PLL_AntiTrasfPark(s16,s16);

Volt_AlphaBeta_Components Rev_Park_newPLL(s16 ,s16);

void RevPark_Circle_Limitation(void);

void CrossDecoupling_Control(void);

Power_Components DQ_Power_Estimation(Curr_Components);

Curr_Components DQ_Filtering(Curr_Components);

s16 BUS_DC_Filtering(s16 Bus_Voltage_Input, s16 samples);

s16 Current_DC_Filtering(s16);

void AlphaBeta_Filtering(s16, s16);

s16 Generate_90Degrees_Delay(s16);
s16 Generate_10Degrees_Delay(s16);

void Check_BUS_DC(s16);

//void  Init_ARRAY(void);


void Calc_Theta_Grid(s16);

s16 SimpleMovingAvarage_quadrature(s16 Volt_Output_nofiltered_quad);
s16 SimpleMovingAvarage_direct(s16 Volt_Output_nofiltered);

#endif /* INC_DQ_PHASELOCKEDLOOP_H_ */
