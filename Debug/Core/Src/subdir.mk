################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/OV5462.c \
../Core/Src/OV5462_registers.c \
../Core/Src/audio.c \
../Core/Src/button_array.c \
../Core/Src/ext_dac.c \
../Core/Src/fatfs_sd.c \
../Core/Src/haptic_pwm.c \
../Core/Src/headlamp.c \
../Core/Src/imu.c \
../Core/Src/main.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/xbee.c 

OBJS += \
./Core/Src/OV5462.o \
./Core/Src/OV5462_registers.o \
./Core/Src/audio.o \
./Core/Src/button_array.o \
./Core/Src/ext_dac.o \
./Core/Src/fatfs_sd.o \
./Core/Src/haptic_pwm.o \
./Core/Src/headlamp.o \
./Core/Src/imu.o \
./Core/Src/main.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/xbee.o 

C_DEPS += \
./Core/Src/OV5462.d \
./Core/Src/OV5462_registers.d \
./Core/Src/audio.d \
./Core/Src/button_array.d \
./Core/Src/ext_dac.d \
./Core/Src/fatfs_sd.d \
./Core/Src/haptic_pwm.d \
./Core/Src/headlamp.d \
./Core/Src/imu.d \
./Core/Src/main.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/xbee.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/OV5462.cyclo ./Core/Src/OV5462.d ./Core/Src/OV5462.o ./Core/Src/OV5462.su ./Core/Src/OV5462_registers.cyclo ./Core/Src/OV5462_registers.d ./Core/Src/OV5462_registers.o ./Core/Src/OV5462_registers.su ./Core/Src/audio.cyclo ./Core/Src/audio.d ./Core/Src/audio.o ./Core/Src/audio.su ./Core/Src/button_array.cyclo ./Core/Src/button_array.d ./Core/Src/button_array.o ./Core/Src/button_array.su ./Core/Src/ext_dac.cyclo ./Core/Src/ext_dac.d ./Core/Src/ext_dac.o ./Core/Src/ext_dac.su ./Core/Src/fatfs_sd.cyclo ./Core/Src/fatfs_sd.d ./Core/Src/fatfs_sd.o ./Core/Src/fatfs_sd.su ./Core/Src/haptic_pwm.cyclo ./Core/Src/haptic_pwm.d ./Core/Src/haptic_pwm.o ./Core/Src/haptic_pwm.su ./Core/Src/headlamp.cyclo ./Core/Src/headlamp.d ./Core/Src/headlamp.o ./Core/Src/headlamp.su ./Core/Src/imu.cyclo ./Core/Src/imu.d ./Core/Src/imu.o ./Core/Src/imu.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/xbee.cyclo ./Core/Src/xbee.d ./Core/Src/xbee.o ./Core/Src/xbee.su

.PHONY: clean-Core-2f-Src

