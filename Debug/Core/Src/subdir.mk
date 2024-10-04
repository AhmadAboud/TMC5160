################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/AD7794.c \
../Core/Src/hx711.c \
../Core/Src/hx711O.c \
../Core/Src/main.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_hal_timebase_tim.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c 

CPP_SRCS += \
../Core/Src/AD7794.cpp \
../Core/Src/DriveControllTMC.cpp \
../Core/Src/EventLoop.cpp \
../Core/Src/GpioPin.cpp \
../Core/Src/HX711_ADC.cpp \
../Core/Src/Pinout.cpp \
../Core/Src/SPIGpio.cpp \
../Core/Src/TimerGpio.cpp \
../Core/Src/inputsCat9555.cpp 

C_DEPS += \
./Core/Src/AD7794.d \
./Core/Src/hx711.d \
./Core/Src/hx711O.d \
./Core/Src/main.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_hal_timebase_tim.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d 

OBJS += \
./Core/Src/AD7794.o \
./Core/Src/DriveControllTMC.o \
./Core/Src/EventLoop.o \
./Core/Src/GpioPin.o \
./Core/Src/HX711_ADC.o \
./Core/Src/Pinout.o \
./Core/Src/SPIGpio.o \
./Core/Src/TimerGpio.o \
./Core/Src/hx711.o \
./Core/Src/hx711O.o \
./Core/Src/inputsCat9555.o \
./Core/Src/main.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_hal_timebase_tim.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o 

CPP_DEPS += \
./Core/Src/AD7794.d \
./Core/Src/DriveControllTMC.d \
./Core/Src/EventLoop.d \
./Core/Src/GpioPin.d \
./Core/Src/HX711_ADC.d \
./Core/Src/Pinout.d \
./Core/Src/SPIGpio.d \
./Core/Src/TimerGpio.d \
./Core/Src/inputsCat9555.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H725xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H725xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/AD7794.cyclo ./Core/Src/AD7794.d ./Core/Src/AD7794.o ./Core/Src/AD7794.su ./Core/Src/DriveControllTMC.cyclo ./Core/Src/DriveControllTMC.d ./Core/Src/DriveControllTMC.o ./Core/Src/DriveControllTMC.su ./Core/Src/EventLoop.cyclo ./Core/Src/EventLoop.d ./Core/Src/EventLoop.o ./Core/Src/EventLoop.su ./Core/Src/GpioPin.cyclo ./Core/Src/GpioPin.d ./Core/Src/GpioPin.o ./Core/Src/GpioPin.su ./Core/Src/HX711_ADC.cyclo ./Core/Src/HX711_ADC.d ./Core/Src/HX711_ADC.o ./Core/Src/HX711_ADC.su ./Core/Src/Pinout.cyclo ./Core/Src/Pinout.d ./Core/Src/Pinout.o ./Core/Src/Pinout.su ./Core/Src/SPIGpio.cyclo ./Core/Src/SPIGpio.d ./Core/Src/SPIGpio.o ./Core/Src/SPIGpio.su ./Core/Src/TimerGpio.cyclo ./Core/Src/TimerGpio.d ./Core/Src/TimerGpio.o ./Core/Src/TimerGpio.su ./Core/Src/hx711.cyclo ./Core/Src/hx711.d ./Core/Src/hx711.o ./Core/Src/hx711.su ./Core/Src/hx711O.cyclo ./Core/Src/hx711O.d ./Core/Src/hx711O.o ./Core/Src/hx711O.su ./Core/Src/inputsCat9555.cyclo ./Core/Src/inputsCat9555.d ./Core/Src/inputsCat9555.o ./Core/Src/inputsCat9555.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_hal_timebase_tim.cyclo ./Core/Src/stm32h7xx_hal_timebase_tim.d ./Core/Src/stm32h7xx_hal_timebase_tim.o ./Core/Src/stm32h7xx_hal_timebase_tim.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su

.PHONY: clean-Core-2f-Src

