# ***************************************************************
# *     Makefile for Cocos                                      *
# ***************************************************************

FWVERSION_SOFTWARE = 0x00000002

#################################################################
# Start of default section
#

TOOLCHAINDIR = ../Toolchain
CC       = $(TOOLCHAINDIR)/bin/arm-none-eabi-gcc.exe
CXX      = $(TOOLCHAINDIR)/bin/arm-none-eabi-g++
LD       = $(TOOLCHAINDIR)/bin/arm-none-eabi-ld.exe
AS       = $(TOOLCHAINDIR)/bin/arm-none-eabi-as.exe
AR       = $(TOOLCHAINDIR)/bin/arm-none-eabi-ar.exe
ELF2BIN  = $(TOOLCHAINDIR)/bin/arm-none-eabi-objcopy.exe
OBJDUMP  = $(TOOLCHAINDIR)/bin/arm-none-eabi-objdump.exe
GDB      = $(TOOLCHAINDIR)/bin/arm-none-eabi-gdb.exe
READELF  = $(TOOLCHAINDIR)/bin/arm-none-eabi-readelf.exe
REMOVE   = $(TOOLCHAINDIR)\utils\rm.exe
MKDIR    = $(TOOLCHAINDIR)\utils\mkdir.exe

#
# End of default section
#################################################################

#################################################################
# Define project name here
PROJECT = LPC1549

# Define linker script file here
LDSCRIPT = Projects/LPC1549/LPC1549.ld

# List all C define here, like -D_DEBUG=1
DEFS  = 
DEFS += -DCORE_M3
DEFS += -D__USE_LPCOPEN

# Define ASM defines here
ADEFS  =  

# List output directory here
OBJDIR  = object

# List source directory here
SRCDIR  = source

# List source directory here
RELDIR  = release

# List all user include directories here
INCLUDEDIR   = ./
INCLUDEDIR  += ./Source
INCLUDEDIR  += ./Source/Hardware/ARM-Cortex-M3
INCLUDEDIR  += ./Projects/LPC1549/Include
INCLUDEDIR  += ./Projects/LPC1549/lpc_chip_15xx/inc
INCLUDEDIR  += ./Projects/LPC1549/lpc_board_nxp_lpcxpresso_1549/inc

# List all library directories here
LIBRARYDIR  = ./Lib

# List all user libraries here (naming convention libxxxx.a where xxxx e.g. plus)
LIBRARY =

# List C source files here
SRC =
SRC += Source/AvRtos.C
SRC += Source/Hardware/ARM-Cortex-M3/AvRtosHw.C
SRC += Projects/LPC1549/Source/cr_startup_lpc15xx.C
SRC += Projects/LPC1549/Source/main.C
SRC += Projects/LPC1549/Source/sysinit.C

# LIBBOARD files used
SRC += Projects/LPC1549/lpc_board_nxp_lpcxpresso_1549/src/board.C
SRC += Projects/LPC1549/lpc_board_nxp_lpcxpresso_1549/src/board_sysinit.C

# LIBCHIP files used
SRC += Projects/LPC1549/lpc_chip_15xx/src/acmp_15xx.C
SRC += Projects/LPC1549/lpc_chip_15xx/src/adc_15xx.C
SRC += Projects/LPC1549/lpc_chip_15xx/src/chip_15xx.C
SRC += Projects/LPC1549/lpc_chip_15xx/src/clock_15xx.C
SRC += Projects/LPC1549/lpc_chip_15xx/src/crc_15xx.C
SRC += Projects/LPC1549/lpc_chip_15xx/src/dac_15xx.C
SRC += Projects/LPC1549/lpc_chip_15xx/src/dma_15xx.C
#SRC += Projects/LPC1549/lpc_chip_15xx/src/eeprom.C
SRC += Projects/LPC1549/lpc_chip_15xx/src/gpio_15xx.C
#SRC += Projects/LPC1549/lpc_chip_15xx/src/i2c_common_15xx.C
#SRC += Projects/LPC1549/lpc_chip_15xx/src/i2c_15xx.C
#SRC += Projects/LPC1549/lpc_chip_15xx/src/i2cm_15xx.C
#SRC += Projects/LPC1549/lpc_chip_15xx/src/i2cs_15xx.C
#SRC += Projects/LPC1549/lpc_chip_15xx/src/iap.C
SRC += Projects/LPC1549/lpc_chip_15xx/src/iocon_15xx.C
SRC += Projects/LPC1549/lpc_chip_15xx/src/pinint_15xx.C
SRC += Projects/LPC1549/lpc_chip_15xx/src/pmu_15xx.C
SRC += Projects/LPC1549/lpc_chip_15xx/src/ring_buffer.C
SRC += Projects/LPC1549/lpc_chip_15xx/src/ritimer_15xx.C
SRC += Projects/LPC1549/lpc_chip_15xx/src/rtc_15xx.C
SRC += Projects/LPC1549/lpc_chip_15xx/src/sct_15xx.C
SRC += Projects/LPC1549/lpc_chip_15xx/src/sct_pwm_15xx.C
SRC += Projects/LPC1549/lpc_chip_15xx/src/sctipu_15xx.C
SRC += Projects/LPC1549/lpc_chip_15xx/src/spi_15xx.C
SRC += Projects/LPC1549/lpc_chip_15xx/src/stopwatch_15xx.C
SRC += Projects/LPC1549/lpc_chip_15xx/src/swm_15xx.C
SRC += Projects/LPC1549/lpc_chip_15xx/src/sysctl_15xx.C
SRC += Projects/LPC1549/lpc_chip_15xx/src/sysinit_15xx.C
SRC += Projects/LPC1549/lpc_chip_15xx/src/uart_15xx.C

CXXSRC = 
       
# List ASM source files here
ASRC  = 
       
# Define warnings here
WARNING = error all

# Define Source optimisation level here
OPT = 

# Generate dependency information
DEP     = -MD -MP -MF .dep/$(@F).d

# Define CPU compiler flags
MCFLAGS = -mcpu=cortex-m3

#################################################################
# Define variables needed for compiler flags here
INCDIR  = $(patsubst %,-I%,$(INCLUDEDIR))
LIBDIR  = $(patsubst %,-L%,$(LIBRARYDIR))
LIBS    = $(patsubst %,$(LIBRARYDIR)/%,$(LIBRARY))
WARN    = $(patsubst %,-W%,$(WARNING))

# Define compiler flags here
ASFLAGS = -c -g $(MCFLAGS) $(ADEFS) -mfloat-abi=soft -mthumb
CPFLAGS = $(MCFLAGS) $(WARN) -mfloat-abi=soft -mthumb -ffunction-sections -fdata-sections $(OPT) -g $(DEFS) -std=gnu99
CXXFLAGS = $(CPFLAGS) -fno-strict-aliasing
LDFLAGS = $(MCFLAGS) -nostartfiles -mfloat-abi=soft -mthumb -ffunction-sections -fdata-sections -g -Wl,--gc-sections -static -T $(LDSCRIPT)

#################################################################
# Define all object files.
OBJ     = $(SRC:%.C=$(OBJDIR)/%.o) $(CXXSRC:%.CPP=$(OBJDIR)/%.o) $(ASRC:%.S=$(OBJDIR)/%.o)

# Define all subdirectories in output directory.
DIRS    = $(sort $(dir $(OBJ)))
DIRS   += .dep

#################################################################
#
# makefile targets
#
all: prebuild compile link postbuild

default : all
prebuild : $(DIRS)
compile : $(OBJ) 
link : $(PROJECT).elf

$(DIRS) : 
	$(MKDIR) -p $@

$(OBJDIR)/%.o : %.C
	@ echo ""
	@ echo "************** Compiling C-Files *************************"
	$(CC) -c -x c $(CPFLAGS) $(INCDIR) $(DEP) $< -o $@	 

$(OBJDIR)/%.o : %.S
	@ echo ""
	@ echo "************** Assembling S-Files ************************"
	$(CC) -c $(ASFLAGS) $(INCDIR) $< -o $@

$(PROJECT).elf: $(OBJ)
	@ echo ""
	@ echo "************** Linking Object-Files **********************"
	$(CC) $(LDFLAGS) $(OBJ) $(LIBS) -o $(OBJDIR)/$@

postbuild :  
	@ echo ""
	@ echo "************** Performing postbuild step *****************"
	$(ELF2BIN) --output-target=binary $(OBJDIR)/$(PROJECT).elf $(OBJDIR)/$(PROJECT).bin
	$(READELF) -S $(OBJDIR)/$(PROJECT).elf

size : 
	@ echo ""
	@ echo "************** Image size ********************************"
	$(SIZE) -A $(OBJDIR)/$(PROJECT).elf

clean:
	$(REMOVE) -fR .dep
	$(REMOVE) -fR $(OBJ)
	$(REMOVE) -f $(OBJDIR)/$(PROJECT).elf
	$(REMOVE) -f $(OBJDIR)/$(PROJECT).map
	$(REMOVE) -f $(OBJDIR)/$(PROJECT).bin

doxygen:
	@ echo "************** Build documentation ************************"
	@ $(TOOLCHAINDIR)/utils/doxygen Doxyfile
	@ echo "************** Documentation done *************************"

# 
# Include the dependency files, should be the last of the makefile
#
ifneq ($(MAKECMDGOALS),clean)
-include $(wildcard .dep/*)
endif
# *** EOF ***