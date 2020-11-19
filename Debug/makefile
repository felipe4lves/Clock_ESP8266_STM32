################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
Clock_esp8266_i2c_lcd16x2.hex \

SECONDARY_SIZE += \
Clock_esp8266_i2c_lcd16x2.siz \


# All Target
all: Clock_esp8266_i2c_lcd16x2.elf secondary-outputs

# Tool invocations
Clock_esp8266_i2c_lcd16x2.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross ARM GNU C Linker'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -T "D:/Google Drive/codigos_eclipse/Clock_esp8266_i2c_lcd16x2/STM32L432KCUx_FLASH.ld" -Xlinker --gc-sections -Wl,-Map,"Clock_esp8266_i2c_lcd16x2.map" --specs=nano.specs -u _printf_float -u _scanf_float -specs=nosys.specs -o "Clock_esp8266_i2c_lcd16x2.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

Clock_esp8266_i2c_lcd16x2.hex: Clock_esp8266_i2c_lcd16x2.elf
	@echo 'Invoking: Cross ARM GNU Create Flash Image'
	arm-none-eabi-objcopy -O ihex "Clock_esp8266_i2c_lcd16x2.elf"  "Clock_esp8266_i2c_lcd16x2.hex"
	@echo 'Finished building: $@'
	@echo ' '

Clock_esp8266_i2c_lcd16x2.siz: Clock_esp8266_i2c_lcd16x2.elf
	@echo 'Invoking: Cross ARM GNU Print Size'
	arm-none-eabi-size --format=berkeley "Clock_esp8266_i2c_lcd16x2.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(SECONDARY_FLASH)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_UPPER_DEPS)$(C_DEPS) Clock_esp8266_i2c_lcd16x2.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets
