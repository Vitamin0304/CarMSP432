################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
Apps/%.o: ../Apps/%.cpp $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"D:/software/ccs930/ccs/tools/compiler/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-gcc-9.2.1.exe" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__MSP432P401R__ -Dccs -Dgcc -I"D:/software/ccs930/ccs/ccs_base/arm/include" -I"D:/WorkSpace/ccs9/simplelink_msp432p4_sdk_3_40_01_02/source" -I"D:/software/ccs930/ccs/ccs_base/arm/include/CMSIS" -I"D:/WorkSpace/ccs9/CarMSP432" -I"D:/software/ccs930/ccs/tools/compiler/gcc-arm-none-eabi-9-2019-q4-major/arm-none-eabi/include" -Os -Wall -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -std=c++17  -fno-threadsafe-statics $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Apps/%.o: ../Apps/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"D:/software/ccs930/ccs/tools/compiler/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-gcc-9.2.1.exe" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__MSP432P401R__ -Dccs -Dgcc -I"D:/software/ccs930/ccs/ccs_base/arm/include" -I"D:/WorkSpace/ccs9/simplelink_msp432p4_sdk_3_40_01_02/source" -I"D:/software/ccs930/ccs/ccs_base/arm/include/CMSIS" -I"D:/WorkSpace/ccs9/CarMSP432" -I"D:/software/ccs930/ccs/tools/compiler/gcc-arm-none-eabi-9-2019-q4-major/arm-none-eabi/include" -Os -Wall -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -std=c18 $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


