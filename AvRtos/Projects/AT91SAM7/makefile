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
PROJECT = AT91SAM7

# Define linker script file here
LDSCRIPT = Projects/AT91SAM7/AT91SAM7.ld

# List all C define here, like -D_DEBUG=1
DEFS  = 

# Define ASM defines here
ADEFS  = -D__ASSEMBLY__

# List output directory here
OBJDIR  = Object

# List source directory here
SRCDIR  = Source

# List source directory here
RELDIR  = release

# List all user include directories here
INCLUDEDIR   = ./
INCLUDEDIR  += ./Source
INCLUDEDIR  += ./Source/Hardware/ARM-7TDMI
INCLUDEDIR  += ./Projects/AT91SAM7/Include

# List all library directories here
LIBRARYDIR  = ./Lib

# List all user libraries here (naming convention libxxxx.a where xxxx e.g. plus)
LIBRARY =

# List C source files here
SRC =
SRC += Source/AvRtos.C
SRC += Source/Hardware/ARM-7TDMI/AvRtosHw.C
SRC += Projects/AT91SAM7/Source/main.C
SRC += Projects/AT91SAM7/Source/Cstartup_SAM7.C

CXXSRC = 
       
# List ASM source files here
ASRC  = 
ASRC += Projects/AT91SAM7/Source/Cstartup.S
       
# Define warnings here
WARNING = error all

# Define Source optimisation level here
OPT = 

# Generate dependency information
DEP     = -MD -MP -MF .dep/$(@F).d

# Define CPU compiler flags
MCFLAGS = -mcpu=arm7tdmi

#################################################################
# Define variables needed for compiler flags here
INCDIR  = $(patsubst %,-I%,$(INCLUDEDIR))
LIBDIR  = $(patsubst %,-L%,$(LIBRARYDIR))
LIBS    = $(patsubst %,$(LIBRARYDIR)/%,$(LIBRARY))
WARN    = $(patsubst %,-W%,$(WARNING))

# Define compiler flags here
ASFLAGS = $(MCFLAGS) -g $(ADEFS)
CPFLAGS = $(MCFLAGS) -ffunction-sections -fno-strict-aliasing -fomit-frame-pointer $(WARN) $(OPT) -g $(DEFS) -std=gnu99
CXXFLAGS = $(CPFLAGS)
LDFLAGS = $(MCFLAGS) -nostartfiles -g -Wl,--gc-sections -static -T $(LDSCRIPT)

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