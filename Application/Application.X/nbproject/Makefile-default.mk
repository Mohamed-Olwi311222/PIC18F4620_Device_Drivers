#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/Application.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/Application.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=ECU_Layer/button/button.c ECU_Layer/Char_LCD/ecu_char_lcd.c ECU_Layer/DC_Motor/ecu_dc_motor.c ECU_Layer/keypad/ecu_keypad.c ECU_Layer/led/ecu_led.c ECU_Layer/Relay/ecu_relay.c ECU_Layer/seven_seg/ecu_seven_seg.c ECU_Layer/ecu_layer_init.c MCAL_Layer/ADC/hal_adc.c MCAL_Layer/CCPx/CCP1/hal_ccp1.c MCAL_Layer/CCPx/CCP2/hal_ccp2.c MCAL_Layer/EEPROM/mcal_eeprom.c MCAL_Layer/EUSART/hal_eusart.c MCAL_Layer/GPIO/hal_gpio.c MCAL_Layer/Interrupt/mcal_internal_interrupt.c MCAL_Layer/Interrupt/mcal_interrupt_manager.c MCAL_Layer/Interrupt/mcal_INTx_external_interrupt.c MCAL_Layer/Interrupt/mcal_RBx_external_interrupt.c MCAL_Layer/MSSP/SPI/mcal_spi.c MCAL_Layer/Timers/Timer0/hal_timer0.c MCAL_Layer/Timers/Timer1/hal_timer1.c MCAL_Layer/Timers/Timer2/hal_timer2.c MCAL_Layer/Timers/Timer3/hal_timer3.c device_config.c app.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/ECU_Layer/button/button.p1 ${OBJECTDIR}/ECU_Layer/Char_LCD/ecu_char_lcd.p1 ${OBJECTDIR}/ECU_Layer/DC_Motor/ecu_dc_motor.p1 ${OBJECTDIR}/ECU_Layer/keypad/ecu_keypad.p1 ${OBJECTDIR}/ECU_Layer/led/ecu_led.p1 ${OBJECTDIR}/ECU_Layer/Relay/ecu_relay.p1 ${OBJECTDIR}/ECU_Layer/seven_seg/ecu_seven_seg.p1 ${OBJECTDIR}/ECU_Layer/ecu_layer_init.p1 ${OBJECTDIR}/MCAL_Layer/ADC/hal_adc.p1 ${OBJECTDIR}/MCAL_Layer/CCPx/CCP1/hal_ccp1.p1 ${OBJECTDIR}/MCAL_Layer/CCPx/CCP2/hal_ccp2.p1 ${OBJECTDIR}/MCAL_Layer/EEPROM/mcal_eeprom.p1 ${OBJECTDIR}/MCAL_Layer/EUSART/hal_eusart.p1 ${OBJECTDIR}/MCAL_Layer/GPIO/hal_gpio.p1 ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_internal_interrupt.p1 ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_interrupt_manager.p1 ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_INTx_external_interrupt.p1 ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_RBx_external_interrupt.p1 ${OBJECTDIR}/MCAL_Layer/MSSP/SPI/mcal_spi.p1 ${OBJECTDIR}/MCAL_Layer/Timers/Timer0/hal_timer0.p1 ${OBJECTDIR}/MCAL_Layer/Timers/Timer1/hal_timer1.p1 ${OBJECTDIR}/MCAL_Layer/Timers/Timer2/hal_timer2.p1 ${OBJECTDIR}/MCAL_Layer/Timers/Timer3/hal_timer3.p1 ${OBJECTDIR}/device_config.p1 ${OBJECTDIR}/app.p1
POSSIBLE_DEPFILES=${OBJECTDIR}/ECU_Layer/button/button.p1.d ${OBJECTDIR}/ECU_Layer/Char_LCD/ecu_char_lcd.p1.d ${OBJECTDIR}/ECU_Layer/DC_Motor/ecu_dc_motor.p1.d ${OBJECTDIR}/ECU_Layer/keypad/ecu_keypad.p1.d ${OBJECTDIR}/ECU_Layer/led/ecu_led.p1.d ${OBJECTDIR}/ECU_Layer/Relay/ecu_relay.p1.d ${OBJECTDIR}/ECU_Layer/seven_seg/ecu_seven_seg.p1.d ${OBJECTDIR}/ECU_Layer/ecu_layer_init.p1.d ${OBJECTDIR}/MCAL_Layer/ADC/hal_adc.p1.d ${OBJECTDIR}/MCAL_Layer/CCPx/CCP1/hal_ccp1.p1.d ${OBJECTDIR}/MCAL_Layer/CCPx/CCP2/hal_ccp2.p1.d ${OBJECTDIR}/MCAL_Layer/EEPROM/mcal_eeprom.p1.d ${OBJECTDIR}/MCAL_Layer/EUSART/hal_eusart.p1.d ${OBJECTDIR}/MCAL_Layer/GPIO/hal_gpio.p1.d ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_internal_interrupt.p1.d ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_interrupt_manager.p1.d ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_INTx_external_interrupt.p1.d ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_RBx_external_interrupt.p1.d ${OBJECTDIR}/MCAL_Layer/MSSP/SPI/mcal_spi.p1.d ${OBJECTDIR}/MCAL_Layer/Timers/Timer0/hal_timer0.p1.d ${OBJECTDIR}/MCAL_Layer/Timers/Timer1/hal_timer1.p1.d ${OBJECTDIR}/MCAL_Layer/Timers/Timer2/hal_timer2.p1.d ${OBJECTDIR}/MCAL_Layer/Timers/Timer3/hal_timer3.p1.d ${OBJECTDIR}/device_config.p1.d ${OBJECTDIR}/app.p1.d

# Object Files
OBJECTFILES=${OBJECTDIR}/ECU_Layer/button/button.p1 ${OBJECTDIR}/ECU_Layer/Char_LCD/ecu_char_lcd.p1 ${OBJECTDIR}/ECU_Layer/DC_Motor/ecu_dc_motor.p1 ${OBJECTDIR}/ECU_Layer/keypad/ecu_keypad.p1 ${OBJECTDIR}/ECU_Layer/led/ecu_led.p1 ${OBJECTDIR}/ECU_Layer/Relay/ecu_relay.p1 ${OBJECTDIR}/ECU_Layer/seven_seg/ecu_seven_seg.p1 ${OBJECTDIR}/ECU_Layer/ecu_layer_init.p1 ${OBJECTDIR}/MCAL_Layer/ADC/hal_adc.p1 ${OBJECTDIR}/MCAL_Layer/CCPx/CCP1/hal_ccp1.p1 ${OBJECTDIR}/MCAL_Layer/CCPx/CCP2/hal_ccp2.p1 ${OBJECTDIR}/MCAL_Layer/EEPROM/mcal_eeprom.p1 ${OBJECTDIR}/MCAL_Layer/EUSART/hal_eusart.p1 ${OBJECTDIR}/MCAL_Layer/GPIO/hal_gpio.p1 ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_internal_interrupt.p1 ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_interrupt_manager.p1 ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_INTx_external_interrupt.p1 ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_RBx_external_interrupt.p1 ${OBJECTDIR}/MCAL_Layer/MSSP/SPI/mcal_spi.p1 ${OBJECTDIR}/MCAL_Layer/Timers/Timer0/hal_timer0.p1 ${OBJECTDIR}/MCAL_Layer/Timers/Timer1/hal_timer1.p1 ${OBJECTDIR}/MCAL_Layer/Timers/Timer2/hal_timer2.p1 ${OBJECTDIR}/MCAL_Layer/Timers/Timer3/hal_timer3.p1 ${OBJECTDIR}/device_config.p1 ${OBJECTDIR}/app.p1

# Source Files
SOURCEFILES=ECU_Layer/button/button.c ECU_Layer/Char_LCD/ecu_char_lcd.c ECU_Layer/DC_Motor/ecu_dc_motor.c ECU_Layer/keypad/ecu_keypad.c ECU_Layer/led/ecu_led.c ECU_Layer/Relay/ecu_relay.c ECU_Layer/seven_seg/ecu_seven_seg.c ECU_Layer/ecu_layer_init.c MCAL_Layer/ADC/hal_adc.c MCAL_Layer/CCPx/CCP1/hal_ccp1.c MCAL_Layer/CCPx/CCP2/hal_ccp2.c MCAL_Layer/EEPROM/mcal_eeprom.c MCAL_Layer/EUSART/hal_eusart.c MCAL_Layer/GPIO/hal_gpio.c MCAL_Layer/Interrupt/mcal_internal_interrupt.c MCAL_Layer/Interrupt/mcal_interrupt_manager.c MCAL_Layer/Interrupt/mcal_INTx_external_interrupt.c MCAL_Layer/Interrupt/mcal_RBx_external_interrupt.c MCAL_Layer/MSSP/SPI/mcal_spi.c MCAL_Layer/Timers/Timer0/hal_timer0.c MCAL_Layer/Timers/Timer1/hal_timer1.c MCAL_Layer/Timers/Timer2/hal_timer2.c MCAL_Layer/Timers/Timer3/hal_timer3.c device_config.c app.c



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk ${DISTDIR}/Application.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=18F4620
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/ECU_Layer/button/button.p1: ECU_Layer/button/button.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_Layer/button" 
	@${RM} ${OBJECTDIR}/ECU_Layer/button/button.p1.d 
	@${RM} ${OBJECTDIR}/ECU_Layer/button/button.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_Layer/button/button.p1 ECU_Layer/button/button.c 
	@-${MV} ${OBJECTDIR}/ECU_Layer/button/button.d ${OBJECTDIR}/ECU_Layer/button/button.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_Layer/button/button.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_Layer/Char_LCD/ecu_char_lcd.p1: ECU_Layer/Char_LCD/ecu_char_lcd.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_Layer/Char_LCD" 
	@${RM} ${OBJECTDIR}/ECU_Layer/Char_LCD/ecu_char_lcd.p1.d 
	@${RM} ${OBJECTDIR}/ECU_Layer/Char_LCD/ecu_char_lcd.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_Layer/Char_LCD/ecu_char_lcd.p1 ECU_Layer/Char_LCD/ecu_char_lcd.c 
	@-${MV} ${OBJECTDIR}/ECU_Layer/Char_LCD/ecu_char_lcd.d ${OBJECTDIR}/ECU_Layer/Char_LCD/ecu_char_lcd.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_Layer/Char_LCD/ecu_char_lcd.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_Layer/DC_Motor/ecu_dc_motor.p1: ECU_Layer/DC_Motor/ecu_dc_motor.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_Layer/DC_Motor" 
	@${RM} ${OBJECTDIR}/ECU_Layer/DC_Motor/ecu_dc_motor.p1.d 
	@${RM} ${OBJECTDIR}/ECU_Layer/DC_Motor/ecu_dc_motor.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_Layer/DC_Motor/ecu_dc_motor.p1 ECU_Layer/DC_Motor/ecu_dc_motor.c 
	@-${MV} ${OBJECTDIR}/ECU_Layer/DC_Motor/ecu_dc_motor.d ${OBJECTDIR}/ECU_Layer/DC_Motor/ecu_dc_motor.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_Layer/DC_Motor/ecu_dc_motor.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_Layer/keypad/ecu_keypad.p1: ECU_Layer/keypad/ecu_keypad.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_Layer/keypad" 
	@${RM} ${OBJECTDIR}/ECU_Layer/keypad/ecu_keypad.p1.d 
	@${RM} ${OBJECTDIR}/ECU_Layer/keypad/ecu_keypad.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_Layer/keypad/ecu_keypad.p1 ECU_Layer/keypad/ecu_keypad.c 
	@-${MV} ${OBJECTDIR}/ECU_Layer/keypad/ecu_keypad.d ${OBJECTDIR}/ECU_Layer/keypad/ecu_keypad.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_Layer/keypad/ecu_keypad.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_Layer/led/ecu_led.p1: ECU_Layer/led/ecu_led.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_Layer/led" 
	@${RM} ${OBJECTDIR}/ECU_Layer/led/ecu_led.p1.d 
	@${RM} ${OBJECTDIR}/ECU_Layer/led/ecu_led.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_Layer/led/ecu_led.p1 ECU_Layer/led/ecu_led.c 
	@-${MV} ${OBJECTDIR}/ECU_Layer/led/ecu_led.d ${OBJECTDIR}/ECU_Layer/led/ecu_led.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_Layer/led/ecu_led.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_Layer/Relay/ecu_relay.p1: ECU_Layer/Relay/ecu_relay.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_Layer/Relay" 
	@${RM} ${OBJECTDIR}/ECU_Layer/Relay/ecu_relay.p1.d 
	@${RM} ${OBJECTDIR}/ECU_Layer/Relay/ecu_relay.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_Layer/Relay/ecu_relay.p1 ECU_Layer/Relay/ecu_relay.c 
	@-${MV} ${OBJECTDIR}/ECU_Layer/Relay/ecu_relay.d ${OBJECTDIR}/ECU_Layer/Relay/ecu_relay.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_Layer/Relay/ecu_relay.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_Layer/seven_seg/ecu_seven_seg.p1: ECU_Layer/seven_seg/ecu_seven_seg.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_Layer/seven_seg" 
	@${RM} ${OBJECTDIR}/ECU_Layer/seven_seg/ecu_seven_seg.p1.d 
	@${RM} ${OBJECTDIR}/ECU_Layer/seven_seg/ecu_seven_seg.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_Layer/seven_seg/ecu_seven_seg.p1 ECU_Layer/seven_seg/ecu_seven_seg.c 
	@-${MV} ${OBJECTDIR}/ECU_Layer/seven_seg/ecu_seven_seg.d ${OBJECTDIR}/ECU_Layer/seven_seg/ecu_seven_seg.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_Layer/seven_seg/ecu_seven_seg.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_Layer/ecu_layer_init.p1: ECU_Layer/ecu_layer_init.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_Layer" 
	@${RM} ${OBJECTDIR}/ECU_Layer/ecu_layer_init.p1.d 
	@${RM} ${OBJECTDIR}/ECU_Layer/ecu_layer_init.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_Layer/ecu_layer_init.p1 ECU_Layer/ecu_layer_init.c 
	@-${MV} ${OBJECTDIR}/ECU_Layer/ecu_layer_init.d ${OBJECTDIR}/ECU_Layer/ecu_layer_init.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_Layer/ecu_layer_init.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/ADC/hal_adc.p1: MCAL_Layer/ADC/hal_adc.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/ADC" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/ADC/hal_adc.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/ADC/hal_adc.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/ADC/hal_adc.p1 MCAL_Layer/ADC/hal_adc.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/ADC/hal_adc.d ${OBJECTDIR}/MCAL_Layer/ADC/hal_adc.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/ADC/hal_adc.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/CCPx/CCP1/hal_ccp1.p1: MCAL_Layer/CCPx/CCP1/hal_ccp1.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/CCPx/CCP1" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/CCPx/CCP1/hal_ccp1.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/CCPx/CCP1/hal_ccp1.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/CCPx/CCP1/hal_ccp1.p1 MCAL_Layer/CCPx/CCP1/hal_ccp1.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/CCPx/CCP1/hal_ccp1.d ${OBJECTDIR}/MCAL_Layer/CCPx/CCP1/hal_ccp1.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/CCPx/CCP1/hal_ccp1.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/CCPx/CCP2/hal_ccp2.p1: MCAL_Layer/CCPx/CCP2/hal_ccp2.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/CCPx/CCP2" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/CCPx/CCP2/hal_ccp2.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/CCPx/CCP2/hal_ccp2.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/CCPx/CCP2/hal_ccp2.p1 MCAL_Layer/CCPx/CCP2/hal_ccp2.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/CCPx/CCP2/hal_ccp2.d ${OBJECTDIR}/MCAL_Layer/CCPx/CCP2/hal_ccp2.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/CCPx/CCP2/hal_ccp2.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/EEPROM/mcal_eeprom.p1: MCAL_Layer/EEPROM/mcal_eeprom.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/EEPROM" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/EEPROM/mcal_eeprom.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/EEPROM/mcal_eeprom.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/EEPROM/mcal_eeprom.p1 MCAL_Layer/EEPROM/mcal_eeprom.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/EEPROM/mcal_eeprom.d ${OBJECTDIR}/MCAL_Layer/EEPROM/mcal_eeprom.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/EEPROM/mcal_eeprom.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/EUSART/hal_eusart.p1: MCAL_Layer/EUSART/hal_eusart.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/EUSART" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/EUSART/hal_eusart.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/EUSART/hal_eusart.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/EUSART/hal_eusart.p1 MCAL_Layer/EUSART/hal_eusart.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/EUSART/hal_eusart.d ${OBJECTDIR}/MCAL_Layer/EUSART/hal_eusart.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/EUSART/hal_eusart.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/GPIO/hal_gpio.p1: MCAL_Layer/GPIO/hal_gpio.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/GPIO" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/GPIO/hal_gpio.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/GPIO/hal_gpio.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/GPIO/hal_gpio.p1 MCAL_Layer/GPIO/hal_gpio.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/GPIO/hal_gpio.d ${OBJECTDIR}/MCAL_Layer/GPIO/hal_gpio.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/GPIO/hal_gpio.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_internal_interrupt.p1: MCAL_Layer/Interrupt/mcal_internal_interrupt.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/Interrupt" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_internal_interrupt.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_internal_interrupt.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_internal_interrupt.p1 MCAL_Layer/Interrupt/mcal_internal_interrupt.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_internal_interrupt.d ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_internal_interrupt.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_internal_interrupt.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_interrupt_manager.p1: MCAL_Layer/Interrupt/mcal_interrupt_manager.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/Interrupt" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_interrupt_manager.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_interrupt_manager.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_interrupt_manager.p1 MCAL_Layer/Interrupt/mcal_interrupt_manager.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_interrupt_manager.d ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_interrupt_manager.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_interrupt_manager.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_INTx_external_interrupt.p1: MCAL_Layer/Interrupt/mcal_INTx_external_interrupt.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/Interrupt" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_INTx_external_interrupt.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_INTx_external_interrupt.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_INTx_external_interrupt.p1 MCAL_Layer/Interrupt/mcal_INTx_external_interrupt.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_INTx_external_interrupt.d ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_INTx_external_interrupt.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_INTx_external_interrupt.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_RBx_external_interrupt.p1: MCAL_Layer/Interrupt/mcal_RBx_external_interrupt.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/Interrupt" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_RBx_external_interrupt.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_RBx_external_interrupt.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_RBx_external_interrupt.p1 MCAL_Layer/Interrupt/mcal_RBx_external_interrupt.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_RBx_external_interrupt.d ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_RBx_external_interrupt.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_RBx_external_interrupt.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/MSSP/SPI/mcal_spi.p1: MCAL_Layer/MSSP/SPI/mcal_spi.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/MSSP/SPI" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/MSSP/SPI/mcal_spi.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/MSSP/SPI/mcal_spi.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/MSSP/SPI/mcal_spi.p1 MCAL_Layer/MSSP/SPI/mcal_spi.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/MSSP/SPI/mcal_spi.d ${OBJECTDIR}/MCAL_Layer/MSSP/SPI/mcal_spi.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/MSSP/SPI/mcal_spi.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/Timers/Timer0/hal_timer0.p1: MCAL_Layer/Timers/Timer0/hal_timer0.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/Timers/Timer0" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Timers/Timer0/hal_timer0.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Timers/Timer0/hal_timer0.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/Timers/Timer0/hal_timer0.p1 MCAL_Layer/Timers/Timer0/hal_timer0.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/Timers/Timer0/hal_timer0.d ${OBJECTDIR}/MCAL_Layer/Timers/Timer0/hal_timer0.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/Timers/Timer0/hal_timer0.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/Timers/Timer1/hal_timer1.p1: MCAL_Layer/Timers/Timer1/hal_timer1.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/Timers/Timer1" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Timers/Timer1/hal_timer1.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Timers/Timer1/hal_timer1.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/Timers/Timer1/hal_timer1.p1 MCAL_Layer/Timers/Timer1/hal_timer1.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/Timers/Timer1/hal_timer1.d ${OBJECTDIR}/MCAL_Layer/Timers/Timer1/hal_timer1.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/Timers/Timer1/hal_timer1.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/Timers/Timer2/hal_timer2.p1: MCAL_Layer/Timers/Timer2/hal_timer2.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/Timers/Timer2" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Timers/Timer2/hal_timer2.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Timers/Timer2/hal_timer2.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/Timers/Timer2/hal_timer2.p1 MCAL_Layer/Timers/Timer2/hal_timer2.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/Timers/Timer2/hal_timer2.d ${OBJECTDIR}/MCAL_Layer/Timers/Timer2/hal_timer2.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/Timers/Timer2/hal_timer2.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/Timers/Timer3/hal_timer3.p1: MCAL_Layer/Timers/Timer3/hal_timer3.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/Timers/Timer3" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Timers/Timer3/hal_timer3.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Timers/Timer3/hal_timer3.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/Timers/Timer3/hal_timer3.p1 MCAL_Layer/Timers/Timer3/hal_timer3.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/Timers/Timer3/hal_timer3.d ${OBJECTDIR}/MCAL_Layer/Timers/Timer3/hal_timer3.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/Timers/Timer3/hal_timer3.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/device_config.p1: device_config.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/device_config.p1.d 
	@${RM} ${OBJECTDIR}/device_config.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/device_config.p1 device_config.c 
	@-${MV} ${OBJECTDIR}/device_config.d ${OBJECTDIR}/device_config.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/device_config.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/app.p1: app.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/app.p1.d 
	@${RM} ${OBJECTDIR}/app.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/app.p1 app.c 
	@-${MV} ${OBJECTDIR}/app.d ${OBJECTDIR}/app.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/app.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
else
${OBJECTDIR}/ECU_Layer/button/button.p1: ECU_Layer/button/button.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_Layer/button" 
	@${RM} ${OBJECTDIR}/ECU_Layer/button/button.p1.d 
	@${RM} ${OBJECTDIR}/ECU_Layer/button/button.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_Layer/button/button.p1 ECU_Layer/button/button.c 
	@-${MV} ${OBJECTDIR}/ECU_Layer/button/button.d ${OBJECTDIR}/ECU_Layer/button/button.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_Layer/button/button.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_Layer/Char_LCD/ecu_char_lcd.p1: ECU_Layer/Char_LCD/ecu_char_lcd.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_Layer/Char_LCD" 
	@${RM} ${OBJECTDIR}/ECU_Layer/Char_LCD/ecu_char_lcd.p1.d 
	@${RM} ${OBJECTDIR}/ECU_Layer/Char_LCD/ecu_char_lcd.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_Layer/Char_LCD/ecu_char_lcd.p1 ECU_Layer/Char_LCD/ecu_char_lcd.c 
	@-${MV} ${OBJECTDIR}/ECU_Layer/Char_LCD/ecu_char_lcd.d ${OBJECTDIR}/ECU_Layer/Char_LCD/ecu_char_lcd.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_Layer/Char_LCD/ecu_char_lcd.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_Layer/DC_Motor/ecu_dc_motor.p1: ECU_Layer/DC_Motor/ecu_dc_motor.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_Layer/DC_Motor" 
	@${RM} ${OBJECTDIR}/ECU_Layer/DC_Motor/ecu_dc_motor.p1.d 
	@${RM} ${OBJECTDIR}/ECU_Layer/DC_Motor/ecu_dc_motor.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_Layer/DC_Motor/ecu_dc_motor.p1 ECU_Layer/DC_Motor/ecu_dc_motor.c 
	@-${MV} ${OBJECTDIR}/ECU_Layer/DC_Motor/ecu_dc_motor.d ${OBJECTDIR}/ECU_Layer/DC_Motor/ecu_dc_motor.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_Layer/DC_Motor/ecu_dc_motor.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_Layer/keypad/ecu_keypad.p1: ECU_Layer/keypad/ecu_keypad.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_Layer/keypad" 
	@${RM} ${OBJECTDIR}/ECU_Layer/keypad/ecu_keypad.p1.d 
	@${RM} ${OBJECTDIR}/ECU_Layer/keypad/ecu_keypad.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_Layer/keypad/ecu_keypad.p1 ECU_Layer/keypad/ecu_keypad.c 
	@-${MV} ${OBJECTDIR}/ECU_Layer/keypad/ecu_keypad.d ${OBJECTDIR}/ECU_Layer/keypad/ecu_keypad.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_Layer/keypad/ecu_keypad.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_Layer/led/ecu_led.p1: ECU_Layer/led/ecu_led.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_Layer/led" 
	@${RM} ${OBJECTDIR}/ECU_Layer/led/ecu_led.p1.d 
	@${RM} ${OBJECTDIR}/ECU_Layer/led/ecu_led.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_Layer/led/ecu_led.p1 ECU_Layer/led/ecu_led.c 
	@-${MV} ${OBJECTDIR}/ECU_Layer/led/ecu_led.d ${OBJECTDIR}/ECU_Layer/led/ecu_led.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_Layer/led/ecu_led.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_Layer/Relay/ecu_relay.p1: ECU_Layer/Relay/ecu_relay.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_Layer/Relay" 
	@${RM} ${OBJECTDIR}/ECU_Layer/Relay/ecu_relay.p1.d 
	@${RM} ${OBJECTDIR}/ECU_Layer/Relay/ecu_relay.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_Layer/Relay/ecu_relay.p1 ECU_Layer/Relay/ecu_relay.c 
	@-${MV} ${OBJECTDIR}/ECU_Layer/Relay/ecu_relay.d ${OBJECTDIR}/ECU_Layer/Relay/ecu_relay.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_Layer/Relay/ecu_relay.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_Layer/seven_seg/ecu_seven_seg.p1: ECU_Layer/seven_seg/ecu_seven_seg.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_Layer/seven_seg" 
	@${RM} ${OBJECTDIR}/ECU_Layer/seven_seg/ecu_seven_seg.p1.d 
	@${RM} ${OBJECTDIR}/ECU_Layer/seven_seg/ecu_seven_seg.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_Layer/seven_seg/ecu_seven_seg.p1 ECU_Layer/seven_seg/ecu_seven_seg.c 
	@-${MV} ${OBJECTDIR}/ECU_Layer/seven_seg/ecu_seven_seg.d ${OBJECTDIR}/ECU_Layer/seven_seg/ecu_seven_seg.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_Layer/seven_seg/ecu_seven_seg.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/ECU_Layer/ecu_layer_init.p1: ECU_Layer/ecu_layer_init.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/ECU_Layer" 
	@${RM} ${OBJECTDIR}/ECU_Layer/ecu_layer_init.p1.d 
	@${RM} ${OBJECTDIR}/ECU_Layer/ecu_layer_init.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/ECU_Layer/ecu_layer_init.p1 ECU_Layer/ecu_layer_init.c 
	@-${MV} ${OBJECTDIR}/ECU_Layer/ecu_layer_init.d ${OBJECTDIR}/ECU_Layer/ecu_layer_init.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/ECU_Layer/ecu_layer_init.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/ADC/hal_adc.p1: MCAL_Layer/ADC/hal_adc.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/ADC" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/ADC/hal_adc.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/ADC/hal_adc.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/ADC/hal_adc.p1 MCAL_Layer/ADC/hal_adc.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/ADC/hal_adc.d ${OBJECTDIR}/MCAL_Layer/ADC/hal_adc.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/ADC/hal_adc.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/CCPx/CCP1/hal_ccp1.p1: MCAL_Layer/CCPx/CCP1/hal_ccp1.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/CCPx/CCP1" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/CCPx/CCP1/hal_ccp1.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/CCPx/CCP1/hal_ccp1.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/CCPx/CCP1/hal_ccp1.p1 MCAL_Layer/CCPx/CCP1/hal_ccp1.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/CCPx/CCP1/hal_ccp1.d ${OBJECTDIR}/MCAL_Layer/CCPx/CCP1/hal_ccp1.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/CCPx/CCP1/hal_ccp1.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/CCPx/CCP2/hal_ccp2.p1: MCAL_Layer/CCPx/CCP2/hal_ccp2.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/CCPx/CCP2" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/CCPx/CCP2/hal_ccp2.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/CCPx/CCP2/hal_ccp2.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/CCPx/CCP2/hal_ccp2.p1 MCAL_Layer/CCPx/CCP2/hal_ccp2.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/CCPx/CCP2/hal_ccp2.d ${OBJECTDIR}/MCAL_Layer/CCPx/CCP2/hal_ccp2.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/CCPx/CCP2/hal_ccp2.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/EEPROM/mcal_eeprom.p1: MCAL_Layer/EEPROM/mcal_eeprom.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/EEPROM" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/EEPROM/mcal_eeprom.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/EEPROM/mcal_eeprom.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/EEPROM/mcal_eeprom.p1 MCAL_Layer/EEPROM/mcal_eeprom.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/EEPROM/mcal_eeprom.d ${OBJECTDIR}/MCAL_Layer/EEPROM/mcal_eeprom.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/EEPROM/mcal_eeprom.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/EUSART/hal_eusart.p1: MCAL_Layer/EUSART/hal_eusart.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/EUSART" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/EUSART/hal_eusart.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/EUSART/hal_eusart.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/EUSART/hal_eusart.p1 MCAL_Layer/EUSART/hal_eusart.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/EUSART/hal_eusart.d ${OBJECTDIR}/MCAL_Layer/EUSART/hal_eusart.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/EUSART/hal_eusart.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/GPIO/hal_gpio.p1: MCAL_Layer/GPIO/hal_gpio.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/GPIO" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/GPIO/hal_gpio.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/GPIO/hal_gpio.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/GPIO/hal_gpio.p1 MCAL_Layer/GPIO/hal_gpio.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/GPIO/hal_gpio.d ${OBJECTDIR}/MCAL_Layer/GPIO/hal_gpio.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/GPIO/hal_gpio.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_internal_interrupt.p1: MCAL_Layer/Interrupt/mcal_internal_interrupt.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/Interrupt" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_internal_interrupt.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_internal_interrupt.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_internal_interrupt.p1 MCAL_Layer/Interrupt/mcal_internal_interrupt.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_internal_interrupt.d ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_internal_interrupt.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_internal_interrupt.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_interrupt_manager.p1: MCAL_Layer/Interrupt/mcal_interrupt_manager.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/Interrupt" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_interrupt_manager.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_interrupt_manager.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_interrupt_manager.p1 MCAL_Layer/Interrupt/mcal_interrupt_manager.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_interrupt_manager.d ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_interrupt_manager.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_interrupt_manager.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_INTx_external_interrupt.p1: MCAL_Layer/Interrupt/mcal_INTx_external_interrupt.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/Interrupt" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_INTx_external_interrupt.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_INTx_external_interrupt.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_INTx_external_interrupt.p1 MCAL_Layer/Interrupt/mcal_INTx_external_interrupt.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_INTx_external_interrupt.d ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_INTx_external_interrupt.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_INTx_external_interrupt.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_RBx_external_interrupt.p1: MCAL_Layer/Interrupt/mcal_RBx_external_interrupt.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/Interrupt" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_RBx_external_interrupt.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_RBx_external_interrupt.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_RBx_external_interrupt.p1 MCAL_Layer/Interrupt/mcal_RBx_external_interrupt.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_RBx_external_interrupt.d ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_RBx_external_interrupt.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/Interrupt/mcal_RBx_external_interrupt.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/MSSP/SPI/mcal_spi.p1: MCAL_Layer/MSSP/SPI/mcal_spi.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/MSSP/SPI" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/MSSP/SPI/mcal_spi.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/MSSP/SPI/mcal_spi.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/MSSP/SPI/mcal_spi.p1 MCAL_Layer/MSSP/SPI/mcal_spi.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/MSSP/SPI/mcal_spi.d ${OBJECTDIR}/MCAL_Layer/MSSP/SPI/mcal_spi.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/MSSP/SPI/mcal_spi.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/Timers/Timer0/hal_timer0.p1: MCAL_Layer/Timers/Timer0/hal_timer0.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/Timers/Timer0" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Timers/Timer0/hal_timer0.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Timers/Timer0/hal_timer0.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/Timers/Timer0/hal_timer0.p1 MCAL_Layer/Timers/Timer0/hal_timer0.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/Timers/Timer0/hal_timer0.d ${OBJECTDIR}/MCAL_Layer/Timers/Timer0/hal_timer0.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/Timers/Timer0/hal_timer0.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/Timers/Timer1/hal_timer1.p1: MCAL_Layer/Timers/Timer1/hal_timer1.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/Timers/Timer1" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Timers/Timer1/hal_timer1.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Timers/Timer1/hal_timer1.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/Timers/Timer1/hal_timer1.p1 MCAL_Layer/Timers/Timer1/hal_timer1.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/Timers/Timer1/hal_timer1.d ${OBJECTDIR}/MCAL_Layer/Timers/Timer1/hal_timer1.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/Timers/Timer1/hal_timer1.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/Timers/Timer2/hal_timer2.p1: MCAL_Layer/Timers/Timer2/hal_timer2.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/Timers/Timer2" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Timers/Timer2/hal_timer2.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Timers/Timer2/hal_timer2.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/Timers/Timer2/hal_timer2.p1 MCAL_Layer/Timers/Timer2/hal_timer2.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/Timers/Timer2/hal_timer2.d ${OBJECTDIR}/MCAL_Layer/Timers/Timer2/hal_timer2.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/Timers/Timer2/hal_timer2.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCAL_Layer/Timers/Timer3/hal_timer3.p1: MCAL_Layer/Timers/Timer3/hal_timer3.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/MCAL_Layer/Timers/Timer3" 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Timers/Timer3/hal_timer3.p1.d 
	@${RM} ${OBJECTDIR}/MCAL_Layer/Timers/Timer3/hal_timer3.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/MCAL_Layer/Timers/Timer3/hal_timer3.p1 MCAL_Layer/Timers/Timer3/hal_timer3.c 
	@-${MV} ${OBJECTDIR}/MCAL_Layer/Timers/Timer3/hal_timer3.d ${OBJECTDIR}/MCAL_Layer/Timers/Timer3/hal_timer3.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCAL_Layer/Timers/Timer3/hal_timer3.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/device_config.p1: device_config.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/device_config.p1.d 
	@${RM} ${OBJECTDIR}/device_config.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/device_config.p1 device_config.c 
	@-${MV} ${OBJECTDIR}/device_config.d ${OBJECTDIR}/device_config.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/device_config.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/app.p1: app.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/app.p1.d 
	@${RM} ${OBJECTDIR}/app.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gcoff -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/app.p1 app.c 
	@-${MV} ${OBJECTDIR}/app.d ${OBJECTDIR}/app.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/app.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${DISTDIR}/Application.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -Wl,-Map=${DISTDIR}/Application.X.${IMAGE_TYPE}.map  -D__DEBUG=1  -mdebugger=none  -DXPRJ_default=$(CND_CONF)  -Wl,--defsym=__MPLAB_BUILD=1   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits -std=c99 -gcoff -mstack=compiled:auto:auto:auto        $(COMPARISON_BUILD) -Wl,--memorysummary,${DISTDIR}/memoryfile.xml -o ${DISTDIR}/Application.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}     
	@${RM} ${DISTDIR}/Application.X.${IMAGE_TYPE}.hex 
	
	
else
${DISTDIR}/Application.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -Wl,-Map=${DISTDIR}/Application.X.${IMAGE_TYPE}.map  -DXPRJ_default=$(CND_CONF)  -Wl,--defsym=__MPLAB_BUILD=1   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits -std=c99 -gcoff -mstack=compiled:auto:auto:auto     $(COMPARISON_BUILD) -Wl,--memorysummary,${DISTDIR}/memoryfile.xml -o ${DISTDIR}/Application.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}     
	
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${OBJECTDIR}
	${RM} -r ${DISTDIR}

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(wildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
