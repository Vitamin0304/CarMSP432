################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := D:/software/ccs930/ccs/tools/compiler/gcc-arm-none-eabi-9-2019-q4-major

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./main.o" \
"./startup_msp432p401r_gcc.o" \
"./system_msp432p401r.o" \
"./Apps/Car.o" \
"./Apps/Encoder.o" \
"./Apps/Motor.o" \
"./Apps/PIDController.o" \
"./Apps/Task.o" \
"./Apps/cJSON.o" \
"./Apps/motor_timer.o" \
"./Apps/my_uart.o" \
"./FuzzyControl/CarFuzzy.o" \
"./FuzzyControl/CarSim.o" \
"./FuzzyControl/FuzzyController.o" \
"./FuzzyControl/FuzzyVar.o" \
"./FuzzyControl/MembershipFuction.o" \
"./OLED/MyOLED.o" \
"./OLED/MyOledFont.o" \
"./PathTrack/PathGenerator.o" \
"./PathTrack/PathTrackSim.o" \
-Wl,-T"../msp432p401r.lds" \
$(GEN_CMDS__FLAG) \
-Wl,--start-group \
-lc \
-l:libc.a \
"D:/WorkSpace/ccs9/simplelink_msp432p4_sdk_3_40_01_02/source/ti/devices/msp432p4xx/driverlib/ccs/msp432p4xx_driverlib.lib" \
-Wl,--end-group \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include Apps/subdir_vars.mk
-include FuzzyControl/subdir_vars.mk
-include OLED/subdir_vars.mk
-include PathTrack/subdir_vars.mk
-include subdir_rules.mk
-include Apps/subdir_rules.mk
-include FuzzyControl/subdir_rules.mk
-include OLED/subdir_rules.mk
-include PathTrack/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
CarMSP432.out \

EXE_OUTPUTS__QUOTED += \
"CarMSP432.out" \

BIN_OUTPUTS += \
CarMSP432.hex \

BIN_OUTPUTS__QUOTED += \
"CarMSP432.hex" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(LDS_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "CarMSP432.out"

# Tool invocations
CarMSP432.out: $(OBJS) $(CMD_SRCS) $(LDS_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: GNU Linker'
	"D:/software/ccs930/ccs/tools/compiler/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-gcc-9.2.1.exe" -mfpu=fpv4-sp-d16 -D__MSP432P401R__ -Dccs -Dgcc -Os -Wall -mcpu=cortex-m4 -Wl,-Map,"CarMSP432.map" -mthumb -marm -mthumb -g -gstrict-dwarf -Wall -o"CarMSP432.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

CarMSP432.hex: $(EXE_OUTPUTS)
	@echo 'Building files: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo 'Invoking: GNU Objcopy Utility'
	"D:/software/ccs930/ccs/tools/compiler/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-objcopy.exe"  $(EXE_OUTPUTS__QUOTED) "CarMSP432.hex"
	@echo 'Finished building: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "main.o" "startup_msp432p401r_gcc.o" "system_msp432p401r.o" "Apps\Car.o" "Apps\Encoder.o" "Apps\Motor.o" "Apps\PIDController.o" "Apps\Task.o" "Apps\cJSON.o" "Apps\motor_timer.o" "Apps\my_uart.o" "FuzzyControl\CarFuzzy.o" "FuzzyControl\CarSim.o" "FuzzyControl\FuzzyController.o" "FuzzyControl\FuzzyVar.o" "FuzzyControl\MembershipFuction.o" "OLED\MyOLED.o" "OLED\MyOledFont.o" "PathTrack\PathGenerator.o" "PathTrack\PathTrackSim.o" 
	-$(RM) "startup_msp432p401r_gcc.d" "system_msp432p401r.d" "Apps\cJSON.d" "Apps\my_uart.d" "OLED\MyOLED.d" "OLED\MyOledFont.d" 
	-$(RM) "main.d" "Apps\Car.d" "Apps\Encoder.d" "Apps\Motor.d" "Apps\PIDController.d" "Apps\Task.d" "Apps\motor_timer.d" "FuzzyControl\CarFuzzy.d" "FuzzyControl\CarSim.d" "FuzzyControl\FuzzyController.d" "FuzzyControl\FuzzyVar.d" "FuzzyControl\MembershipFuction.d" "PathTrack\PathGenerator.d" "PathTrack\PathTrackSim.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

