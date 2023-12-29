################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Utilities/Fonts/font12.c \
../Drivers/Utilities/Fonts/font16.c \
../Drivers/Utilities/Fonts/font20.c \
../Drivers/Utilities/Fonts/font24.c \
../Drivers/Utilities/Fonts/font8.c 

OBJS += \
./Drivers/Utilities/Fonts/font12.o \
./Drivers/Utilities/Fonts/font16.o \
./Drivers/Utilities/Fonts/font20.o \
./Drivers/Utilities/Fonts/font24.o \
./Drivers/Utilities/Fonts/font8.o 

C_DEPS += \
./Drivers/Utilities/Fonts/font12.d \
./Drivers/Utilities/Fonts/font16.d \
./Drivers/Utilities/Fonts/font20.d \
./Drivers/Utilities/Fonts/font24.d \
./Drivers/Utilities/Fonts/font8.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Utilities/Fonts/%.o Drivers/Utilities/Fonts/%.su Drivers/Utilities/Fonts/%.cyclo: ../Drivers/Utilities/Fonts/%.c Drivers/Utilities/Fonts/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"D:/stm32cubeIDE/Labs/MiniProject/Drivers/STM32F4xx_HAL_Driver" -I"D:/stm32cubeIDE/Labs/MiniProject/Drivers/BSP/STM32F429I-Discovery" -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Utilities-2f-Fonts

clean-Drivers-2f-Utilities-2f-Fonts:
	-$(RM) ./Drivers/Utilities/Fonts/font12.cyclo ./Drivers/Utilities/Fonts/font12.d ./Drivers/Utilities/Fonts/font12.o ./Drivers/Utilities/Fonts/font12.su ./Drivers/Utilities/Fonts/font16.cyclo ./Drivers/Utilities/Fonts/font16.d ./Drivers/Utilities/Fonts/font16.o ./Drivers/Utilities/Fonts/font16.su ./Drivers/Utilities/Fonts/font20.cyclo ./Drivers/Utilities/Fonts/font20.d ./Drivers/Utilities/Fonts/font20.o ./Drivers/Utilities/Fonts/font20.su ./Drivers/Utilities/Fonts/font24.cyclo ./Drivers/Utilities/Fonts/font24.d ./Drivers/Utilities/Fonts/font24.o ./Drivers/Utilities/Fonts/font24.su ./Drivers/Utilities/Fonts/font8.cyclo ./Drivers/Utilities/Fonts/font8.d ./Drivers/Utilities/Fonts/font8.o ./Drivers/Utilities/Fonts/font8.su

.PHONY: clean-Drivers-2f-Utilities-2f-Fonts

