/*
 * PI_Regulator.h
 *
 *  Created on: Aug 6, 2020
 *      Author: user01
 */

#ifndef INC_PI_REGULATOR_H_
#define INC_PI_REGULATOR_H_

s16 PID_DirectCurrent(PI_ControllerTYPEDEF *Direct_PID, s16 qIdirect_Input);
s16 PID_Reactive_Power(PI_ControllerTYPEDEF *Reactive_PID, s16 qReactive_Input);
s16 PID_QuadratureCurrent(PI_ControllerTYPEDEF *Quadrature_PID, s16 qIquadrature_Input);
#endif /* INC_PI_REGULATOR_H_ */
