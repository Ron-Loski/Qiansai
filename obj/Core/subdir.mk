################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/core_riscv.c 

C_DEPS += \
./Core/core_riscv.d 

OBJS += \
./Core/core_riscv.o 

DIR_OBJS += \
./Core/*.o \

DIR_DEPS += \
./Core/*.d \

DIR_EXPANDS += \
./Core/*.253r.expand \


# Each subdirectory must supply rules for building sources it contributes
Core/%.o: ../Core/%.c
	@	riscv-wch-elf-gcc -march=rv32imac_xw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/Users/15256/Desktop/QIANSAI/Debug" -I"c:/Users/15256/Desktop/QIANSAI/Core" -I"c:/Users/15256/Desktop/QIANSAI/User" -I"c:/Users/15256/Desktop/QIANSAI/Peripheral/inc" -I"c:/Users/15256/Desktop/QIANSAI/Hardware" -I"c:/Users/15256/Desktop/QIANSAI/Hardware/Inc" -I"c:/Users/15256/Desktop/QIANSAI/Hardware/Src" -isystem"c:/Users/15256/Desktop/QIANSAI/Hardware" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

