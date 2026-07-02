################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################
-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include User/subdir.mk
-include Startup/subdir.mk
-include Peripheral/src/subdir.mk
-include Hardware/Src/subdir.mk
-include Debug/subdir.mk
-include Core/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
QIANSAI.hex \

SECONDARY_LIST += \
QIANSAI.lst \

SECONDARY_SIZE += \
QIANSAI.siz \


# All Target
all: 
	$(MAKE) --no-print-directory main-build 

main-build: QIANSAI.elf secondary-outputs

# Tool invocations
QIANSAI.elf: $(OBJS) $(USER_OBJS_ESCAPE)
	@	riscv-wch-elf-gcc -march=rv32imac_xw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -T "c:\Users\15256\Desktop\QIANSAI\Ld\Link.ld" -nostartfiles -Xlinker --gc-sections -Wl,-Map,"QIANSAI.map" --specs=nano.specs --specs=nosys.specs -o "QIANSAI.elf" $(OBJS) $(LIBS) -lprintfloat
QIANSAI.hex: QIANSAI.elf
	@	riscv-wch-elf-objcopy -O ihex "QIANSAI.elf" "QIANSAI.hex"
QIANSAI.lst: QIANSAI.elf
	@	riscv-wch-elf-objdump --all-headers --demangle --disassemble -M xw "QIANSAI.elf" > "QIANSAI.lst"
QIANSAI.siz: QIANSAI.elf
	riscv-wch-elf-size --format=berkeley "QIANSAI.elf"

# Other Targets
clean:
	-$(RM) $(DIR_OBJS) $(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE) QIANSAI.elf
	-$(RM) $(DIR_EXPANDS) $(CALLGRAPH_DOT)
	-$(RM) $(DIR_DEPS)
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets