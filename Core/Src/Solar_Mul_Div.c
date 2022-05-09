/*
 * Solar_Mul_Div.c
 *
 *  Created on: Aug 6, 2020
 *      Author: user01
 */
#include "main.h"

/************* ****Multiplication  ************************************/
/* This routine multiplicates two real 16 bits fractional inputs*/
/**************Out= Op1*Op2  *******************************************/
/***********************************************************************/




 void mul_q15_q15_q31(s16 Op1, s16 Op2, s32 *Out)
{
  s32 temp;

  temp = (s32)Op1*(s32)Op2;

  if (temp>=0x40000000)   /* Overflow (-1*-1) */
  {
    temp=0x7fffffff;
  }
  else
  {
    temp=temp<<1;       /* One bit left shift to remove redondant sign bit */



  }
  *Out=temp;

}

/**********************************************************************/
/**************** END ROUTINE******************************************/
/**********************************************************************/


/************* ****Division  *******************************************/
/***   This routine divides a 32 bits input by a 16 bits input**********/
/***   Dividende is in 1.31 format, divisor in 1.15 and result in 1.15*/
/**************Out= Op1/Op2  *******************************************/
/* Input      :    LeftOpMsb -> Left Operand Msb                      */
/*       	  LeftOpLsb -> Left Operand Lsb		              */
/*  		  RightOp -> Right Operand                            */
/*	    	  Output    -> Output pointer                          */
/***********************************************************************/


void div_q31_q15_q15(s16 LeftOpMsb, s16 LeftOpLsb, s16 RightOp, s16 *Output)
{

 long aux1, aux2;
 long temp1, temp2;
 short nb_bits, i, CS, test;

 aux1 = (s32)LeftOpMsb<<16;  /* creating a long to provide*/
 aux1 += (unsigned)LeftOpLsb; /* a 32 bits dividende */

 if( RightOp==0 )
 {
   if( aux1<0 )
   {
     *Output = 0x8000;
   }
   if( aux1>=0 )
   {
     *Output = 0x7fff;
   }
 }
 else
 {
   aux2 = (s32)RightOp<<16;    /* divisor  */
   temp1 = aux1&0x80000000;
   temp2 = aux2&0x80000000;

   if (temp1==temp2)
   {
     CS=0;
     test=0;
   }
   else
   {
     CS=1;	/* CS=1 for different signs */
     test=1;
   }

   aux1 = aux1<<1 ;
   aux1 += test;

   nb_bits=15;

   for (i=0; i<nb_bits; i++)
   {
     if (CS==1)
     {
       aux1 += aux2;
     }
     else
     {
       aux1 -= aux2;
     }

     temp1 = aux1&0x80000000;
     /*     temp2 = aux2&0x80000000;  */    /* temp2 has not been modified  */
     if ( temp1==temp2 )
     {
       CS=0;
       test=1;	// shift !CS into partial remainder
     }
     else
     {
       CS=1;
       test=0;
     }
     aux1 = (aux1<<1);
     aux1 += test;
   }

   *Output = (s16)aux1;
 }
}
