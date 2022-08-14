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
ifeq "$(wildcard nbproject/Makefile-local-PIC18F46J50_PIM.mk)" "nbproject/Makefile-local-PIC18F46J50_PIM.mk"
include nbproject/Makefile-local-PIC18F46J50_PIM.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=PIC18F46J50_PIM
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/PIC795_ChamberController.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/PIC795_ChamberController.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../main.c ../DelayPIC32.c ../LCD_4BIT_PIC32.c ../I2C_4BUS_EEPROM_PIC32.c ../DS3231.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1472/main.o ${OBJECTDIR}/_ext/1472/DelayPIC32.o ${OBJECTDIR}/_ext/1472/LCD_4BIT_PIC32.o ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o ${OBJECTDIR}/_ext/1472/DS3231.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1472/main.o.d ${OBJECTDIR}/_ext/1472/DelayPIC32.o.d ${OBJECTDIR}/_ext/1472/LCD_4BIT_PIC32.o.d ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o.d ${OBJECTDIR}/_ext/1472/DS3231.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1472/main.o ${OBJECTDIR}/_ext/1472/DelayPIC32.o ${OBJECTDIR}/_ext/1472/LCD_4BIT_PIC32.o ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o ${OBJECTDIR}/_ext/1472/DS3231.o

# Source Files
SOURCEFILES=../main.c ../DelayPIC32.c ../LCD_4BIT_PIC32.c ../I2C_4BUS_EEPROM_PIC32.c ../DS3231.c



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
	${MAKE}  -f nbproject/Makefile-PIC18F46J50_PIM.mk ${DISTDIR}/PIC795_ChamberController.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX795F512L
MP_LINKER_FILE_OPTION=
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
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1472/main.o: ../main.c  .generated_files/flags/PIC18F46J50_PIM/753cf16425235f595962c3399c5b91169e2d33d0 .generated_files/flags/PIC18F46J50_PIM/2f9ad26e3c1e4dd69f55a8e39ed1c84cc3c0c7ab
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d" -o ${OBJECTDIR}/_ext/1472/main.o ../main.c    -DXPRJ_PIC18F46J50_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/DelayPIC32.o: ../DelayPIC32.c  .generated_files/flags/PIC18F46J50_PIM/1139f4e24aacb8cd6450337b9c39ae18407043cb .generated_files/flags/PIC18F46J50_PIM/2f9ad26e3c1e4dd69f55a8e39ed1c84cc3c0c7ab
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/DelayPIC32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/DelayPIC32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/DelayPIC32.o.d" -o ${OBJECTDIR}/_ext/1472/DelayPIC32.o ../DelayPIC32.c    -DXPRJ_PIC18F46J50_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/LCD_4BIT_PIC32.o: ../LCD_4BIT_PIC32.c  .generated_files/flags/PIC18F46J50_PIM/52d55a916b9594ef3765912d4af12b3c7c2d6932 .generated_files/flags/PIC18F46J50_PIM/2f9ad26e3c1e4dd69f55a8e39ed1c84cc3c0c7ab
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/LCD_4BIT_PIC32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/LCD_4BIT_PIC32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/LCD_4BIT_PIC32.o.d" -o ${OBJECTDIR}/_ext/1472/LCD_4BIT_PIC32.o ../LCD_4BIT_PIC32.c    -DXPRJ_PIC18F46J50_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o: ../I2C_4BUS_EEPROM_PIC32.c  .generated_files/flags/PIC18F46J50_PIM/22997b0c18a58830d14395f84601afb3948c99a2 .generated_files/flags/PIC18F46J50_PIM/2f9ad26e3c1e4dd69f55a8e39ed1c84cc3c0c7ab
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o.d" -o ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o ../I2C_4BUS_EEPROM_PIC32.c    -DXPRJ_PIC18F46J50_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/DS3231.o: ../DS3231.c  .generated_files/flags/PIC18F46J50_PIM/7955420d2ed4aab40d597e14a2b44d07e41a748e .generated_files/flags/PIC18F46J50_PIM/2f9ad26e3c1e4dd69f55a8e39ed1c84cc3c0c7ab
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/DS3231.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/DS3231.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/DS3231.o.d" -o ${OBJECTDIR}/_ext/1472/DS3231.o ../DS3231.c    -DXPRJ_PIC18F46J50_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
else
${OBJECTDIR}/_ext/1472/main.o: ../main.c  .generated_files/flags/PIC18F46J50_PIM/21a910c19b7d4cc4f1a1a60c8f96ca759e064c .generated_files/flags/PIC18F46J50_PIM/2f9ad26e3c1e4dd69f55a8e39ed1c84cc3c0c7ab
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d" -o ${OBJECTDIR}/_ext/1472/main.o ../main.c    -DXPRJ_PIC18F46J50_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/DelayPIC32.o: ../DelayPIC32.c  .generated_files/flags/PIC18F46J50_PIM/8bd2ef0e6d44069b28a4b60a926b1f23e01283e1 .generated_files/flags/PIC18F46J50_PIM/2f9ad26e3c1e4dd69f55a8e39ed1c84cc3c0c7ab
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/DelayPIC32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/DelayPIC32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/DelayPIC32.o.d" -o ${OBJECTDIR}/_ext/1472/DelayPIC32.o ../DelayPIC32.c    -DXPRJ_PIC18F46J50_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/LCD_4BIT_PIC32.o: ../LCD_4BIT_PIC32.c  .generated_files/flags/PIC18F46J50_PIM/1d6ac2ffd6ed77ce8090e161762d22dcb6c1f1ff .generated_files/flags/PIC18F46J50_PIM/2f9ad26e3c1e4dd69f55a8e39ed1c84cc3c0c7ab
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/LCD_4BIT_PIC32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/LCD_4BIT_PIC32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/LCD_4BIT_PIC32.o.d" -o ${OBJECTDIR}/_ext/1472/LCD_4BIT_PIC32.o ../LCD_4BIT_PIC32.c    -DXPRJ_PIC18F46J50_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o: ../I2C_4BUS_EEPROM_PIC32.c  .generated_files/flags/PIC18F46J50_PIM/7069c7e4b9afda97cd76e444bd5e9c6fec3f852a .generated_files/flags/PIC18F46J50_PIM/2f9ad26e3c1e4dd69f55a8e39ed1c84cc3c0c7ab
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o.d" -o ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o ../I2C_4BUS_EEPROM_PIC32.c    -DXPRJ_PIC18F46J50_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/DS3231.o: ../DS3231.c  .generated_files/flags/PIC18F46J50_PIM/b1767e73974a9a56ebaa10745607763ff166c443 .generated_files/flags/PIC18F46J50_PIM/2f9ad26e3c1e4dd69f55a8e39ed1c84cc3c0c7ab
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/DS3231.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/DS3231.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/DS3231.o.d" -o ${OBJECTDIR}/_ext/1472/DS3231.o ../DS3231.c    -DXPRJ_PIC18F46J50_PIM=$(CND_CONF)    $(COMPARISON_BUILD)    
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${DISTDIR}/PIC795_ChamberController.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -g -mdebugger -D__MPLAB_DEBUGGER_ICD3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o ${DISTDIR}/PIC795_ChamberController.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_PIC18F46J50_PIM=$(CND_CONF)    $(COMPARISON_BUILD)   -mreserve=data@0x0:0x1FC -mreserve=boot@0x1FC02000:0x1FC02FEF -mreserve=boot@0x1FC02000:0x1FC024FF  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map" 
	
else
${DISTDIR}/PIC795_ChamberController.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o ${DISTDIR}/PIC795_ChamberController.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_PIC18F46J50_PIM=$(CND_CONF)    $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map" 
	${MP_CC_DIR}\\xc32-bin2hex ${DISTDIR}/PIC795_ChamberController.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
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

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
