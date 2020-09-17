################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../FATFS/Target/user_diskio.cpp \
../FATFS/Target/user_diskio_spi.cpp 

OBJS += \
./FATFS/Target/user_diskio.o \
./FATFS/Target/user_diskio_spi.o 

CPP_DEPS += \
./FATFS/Target/user_diskio.d \
./FATFS/Target/user_diskio_spi.d 


# Each subdirectory must supply rules for building sources it contributes
FATFS/Target/user_diskio.o: ../FATFS/Target/user_diskio.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -DDEBUG -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"C:/Users/lance/STM32CubeIDE/workspace_1.4.0/SD_Test_with_DMA_Class/Core/Src" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"FATFS/Target/user_diskio.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
FATFS/Target/user_diskio_spi.o: ../FATFS/Target/user_diskio_spi.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32H743xx -DDEBUG -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"C:/Users/lance/STM32CubeIDE/workspace_1.4.0/SD_Test_with_DMA_Class/Core/Src" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"FATFS/Target/user_diskio_spi.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

