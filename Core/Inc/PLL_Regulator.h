/*
 * PLL_Regulator.h
 *
 *  Created on: Aug 6, 2020
 *      Author: user01
 */

#ifndef INC_PLL_REGULATOR_H_
#define INC_PLL_REGULATOR_H_

s16 PLL_PID_Regulator(PI_ControllerTYPEDEF *PLL_PID, Volt_Components Grid_Voltage_Components_Input);
s16 PID_Bus_Voltage(PI_ControllerTYPEDEF *Bus_DC_PID, u16 qBus_Voltage_Input);

#endif /* INC_PLL_REGULATOR_H_ */
