################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
OLED/%.obj: ../OLED/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"D:/software/ccs930/ccs/tools/compiler/ti-cgt-arm_18.12.4.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="D:/software/ccs930/ccs/ccs_base/arm/include" --include_path="D:/WorkSpace/ccs9/simplelink_msp432p4_sdk_3_40_01_02/source" --include_path="D:/software/ccs930/ccs/ccs_base/arm/include/CMSIS" --include_path="D:/WorkSpace/ccs9/CarMSP432" --include_path="D:/software/ccs930/ccs/tools/compiler/ti-cgt-arm_18.12.4.LTS/include" --advice:power="all" --define=__MSP432P401R__ --define=ccs --define=EIGEN_DONT_ALIGN_STATICALLY -g --c11 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="OLED/$(basename $(<F)).d_raw" --obj_directory="OLED" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


