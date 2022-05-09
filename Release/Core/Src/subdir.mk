################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/400WControl.c \
../Core/Src/COM_Serial.c \
../Core/Src/DAC_Debug.c \
../Core/Src/DCAC_Inverter.c \
../Core/Src/DCDC_Converter.c \
../Core/Src/DQ_PhaseLockedLoop.c \
../Core/Src/DataSensing.c \
../Core/Src/FrameProtocol.c \
../Core/Src/PI_Regulator.c \
../Core/Src/PLL_Regulator.c \
../Core/Src/Solar_MPPT.c \
../Core/Src/Solar_Mul_Div.c \
../Core/Src/hw_config.c \
../Core/Src/main.c \
../Core/Src/photovSDK.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c 

OBJS += \
./Core/Src/400WControl.o \
./Core/Src/COM_Serial.o \
./Core/Src/DAC_Debug.o \
./Core/Src/DCAC_Inverter.o \
./Core/Src/DCDC_Converter.o \
./Core/Src/DQ_PhaseLockedLoop.o \
./Core/Src/DataSensing.o \
./Core/Src/FrameProtocol.o \
./Core/Src/PI_Regulator.o \
./Core/Src/PLL_Regulator.o \
./Core/Src/Solar_MPPT.o \
./Core/Src/Solar_Mul_Div.o \
./Core/Src/hw_config.o \
./Core/Src/main.o \
./Core/Src/photovSDK.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o 

C_DEPS += \
./Core/Src/400WControl.d \
./Core/Src/COM_Serial.d \
./Core/Src/DAC_Debug.d \
./Core/Src/DCAC_Inverter.d \
./Core/Src/DCDC_Converter.d \
./Core/Src/DQ_PhaseLockedLoop.d \
./Core/Src/DataSensing.d \
./Core/Src/FrameProtocol.d \
./Core/Src/PI_Regulator.d \
./Core/Src/PLL_Regulator.d \
./Core/Src/Solar_MPPT.d \
./Core/Src/Solar_Mul_Div.d \
./Core/Src/hw_config.d \
./Core/Src/main.d \
./Core/Src/photovSDK.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G474xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/400WControl.d ./Core/Src/400WControl.o ./Core/Src/400WControl.su ./Core/Src/COM_Serial.d ./Core/Src/COM_Serial.o ./Core/Src/COM_Serial.su ./Core/Src/DAC_Debug.d ./Core/Src/DAC_Debug.o ./Core/Src/DAC_Debug.su ./Core/Src/DCAC_Inverter.d ./Core/Src/DCAC_Inverter.o ./Core/Src/DCAC_Inverter.su ./Core/Src/DCDC_Converter.d ./Core/Src/DCDC_Converter.o ./Core/Src/DCDC_Converter.su ./Core/Src/DQ_PhaseLockedLoop.d ./Core/Src/DQ_PhaseLockedLoop.o ./Core/Src/DQ_PhaseLockedLoop.su ./Core/Src/DataSensing.d ./Core/Src/DataSensing.o ./Core/Src/DataSensing.su ./Core/Src/FrameProtocol.d ./Core/Src/FrameProtocol.o ./Core/Src/FrameProtocol.su ./Core/Src/PI_Regulator.d ./Core/Src/PI_Regulator.o ./Core/Src/PI_Regulator.su ./Core/Src/PLL_Regulator.d ./Core/Src/PLL_Regulator.o ./Core/Src/PLL_Regulator.su ./Core/Src/Solar_MPPT.d ./Core/Src/Solar_MPPT.o ./Core/Src/Solar_MPPT.su ./Core/Src/Solar_Mul_Div.d ./Core/Src/Solar_Mul_Div.o ./Core/Src/Solar_Mul_Div.su ./Core/Src/hw_config.d ./Core/Src/hw_config.o ./Core/Src/hw_config.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/photovSDK.d ./Core/Src/photovSDK.o ./Core/Src/photovSDK.su ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su

.PHONY: clean-Core-2f-Src

