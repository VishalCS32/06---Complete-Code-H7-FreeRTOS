################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/FUSION/COMPLEMENTARY/complementary_filter.c 

OBJS += \
./Core/FUSION/COMPLEMENTARY/complementary_filter.o 

C_DEPS += \
./Core/FUSION/COMPLEMENTARY/complementary_filter.d 


# Each subdirectory must supply rules for building sources it contributes
Core/FUSION/COMPLEMENTARY/%.o Core/FUSION/COMPLEMENTARY/%.su Core/FUSION/COMPLEMENTARY/%.cyclo: ../Core/FUSION/COMPLEMENTARY/%.c Core/FUSION/COMPLEMENTARY/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-FUSION-2f-COMPLEMENTARY

clean-Core-2f-FUSION-2f-COMPLEMENTARY:
	-$(RM) ./Core/FUSION/COMPLEMENTARY/complementary_filter.cyclo ./Core/FUSION/COMPLEMENTARY/complementary_filter.d ./Core/FUSION/COMPLEMENTARY/complementary_filter.o ./Core/FUSION/COMPLEMENTARY/complementary_filter.su

.PHONY: clean-Core-2f-FUSION-2f-COMPLEMENTARY

