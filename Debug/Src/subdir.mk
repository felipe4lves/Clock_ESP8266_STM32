################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/I2C_lcd16x2.c \
../Src/main.c \
../Src/stm32l4xx_hal_msp.c \
../Src/stm32l4xx_it.c \
../Src/system_stm32l4xx.c 

OBJS += \
./Src/I2C_lcd16x2.o \
./Src/main.o \
./Src/stm32l4xx_hal_msp.o \
./Src/stm32l4xx_it.o \
./Src/system_stm32l4xx.o 

C_DEPS += \
./Src/I2C_lcd16x2.d \
./Src/main.d \
./Src/stm32l4xx_hal_msp.d \
./Src/stm32l4xx_it.d \
./Src/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32L432xx -DARM_MATH_CM0 -I"D:/Google Drive/codigos_eclipse/Clock_esp8266_i2c_lcd16x2/Inc" -I"D:/Google Drive/codigos_eclipse/Clock_esp8266_i2c_lcd16x2/Drivers/STM32L4xx_HAL_Driver/Inc" -I"D:/Google Drive/codigos_eclipse/Clock_esp8266_i2c_lcd16x2/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"D:/Google Drive/codigos_eclipse/Clock_esp8266_i2c_lcd16x2/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"D:/Google Drive/codigos_eclipse/Clock_esp8266_i2c_lcd16x2/Drivers/CMSIS/Include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


