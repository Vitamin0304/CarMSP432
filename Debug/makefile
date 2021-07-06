################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := D:/software/ccs930/ccs/tools/compiler/ti-cgt-arm_18.12.4.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./main.obj" \
"./startup_msp432p401r_ccs.obj" \
"./system_msp432p401r.obj" \
"./Apps/Car.obj" \
"./Apps/Encoder.obj" \
"./Apps/Motor.obj" \
"./Apps/PIDController.obj" \
"./Apps/Task.obj" \
"./Apps/cJSON.obj" \
"./Apps/motor_timer.obj" \
"./Apps/my_uart.obj" \
"./FuzzyControl/CarFuzzy.obj" \
"./FuzzyControl/CarSim.obj" \
"./FuzzyControl/FuzzyController.obj" \
"./FuzzyControl/FuzzyVar.obj" \
"./FuzzyControl/MembershipFuction.obj" \
"./OLED/MyOLED.obj" \
"./OLED/MyOledFont.obj" \
"./PathTrack/PathGenerator.obj" \
"./PathTrack/PathTrackSim.obj" \
"../msp432p401r.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \
-l"D:/WorkSpace/ccs9/simplelink_msp432p4_sdk_3_40_01_02/source/ti/devices/msp432p4xx/driverlib/ccs/msp432p4xx_driverlib.lib" \

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
	@echo 'Invoking: ARM Linker'
	"D:/software/ccs930/ccs/tools/compiler/ti-cgt-arm_18.12.4.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --opt_for_speed=2 --advice:power="all" --define=__MSP432P401R__ --define=ccs --define=EIGEN_DONT_ALIGN_STATICALLY -g --c11 --relaxed_ansi --rtti --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi -z -m"CarMSP432.map" --heap_size=16384 --stack_size=8192 -i"D:/software/ccs930/ccs/ccs_base/arm/include" -i"D:/WorkSpace/ccs9/simplelink_msp432p4_sdk_3_40_01_02/source/" -i"D:/software/ccs930/ccs/tools/compiler/ti-cgt-arm_18.12.4.LTS/lib" -i"D:/software/ccs930/ccs/tools/compiler/ti-cgt-arm_18.12.4.LTS/include" --reread_libs --diag_wrap=off --display_error_number --warn_sections --xml_link_info="CarMSP432_linkInfo.xml" --rom_model -o "CarMSP432.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

CarMSP432.hex: $(EXE_OUTPUTS)
	@echo 'Building files: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo 'Invoking: ARM Hex Utility'
	"D:/software/ccs930/ccs/tools/compiler/ti-cgt-arm_18.12.4.LTS/bin/armhex" --memwidth=8 --romwidth=8 -o "CarMSP432.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "main.obj" "startup_msp432p401r_ccs.obj" "system_msp432p401r.obj" "Apps\Car.obj" "Apps\Encoder.obj" "Apps\Motor.obj" "Apps\PIDController.obj" "Apps\Task.obj" "Apps\cJSON.obj" "Apps\motor_timer.obj" "Apps\my_uart.obj" "FuzzyControl\CarFuzzy.obj" "FuzzyControl\CarSim.obj" "FuzzyControl\FuzzyController.obj" "FuzzyControl\FuzzyVar.obj" "FuzzyControl\MembershipFuction.obj" "OLED\MyOLED.obj" "OLED\MyOledFont.obj" "PathTrack\PathGenerator.obj" "PathTrack\PathTrackSim.obj" 
	-$(RM) "startup_msp432p401r_ccs.d" "system_msp432p401r.d" "Apps\cJSON.d" "Apps\my_uart.d" "OLED\MyOLED.d" "OLED\MyOledFont.d" 
	-$(RM) "main.d" "Apps\Car.d" "Apps\Encoder.d" "Apps\Motor.d" "Apps\PIDController.d" "Apps\Task.d" "Apps\motor_timer.d" "FuzzyControl\CarFuzzy.d" "FuzzyControl\CarSim.d" "FuzzyControl\FuzzyController.d" "FuzzyControl\FuzzyVar.d" "FuzzyControl\MembershipFuction.d" "PathTrack\PathGenerator.d" "PathTrack\PathTrackSim.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

