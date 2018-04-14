# ***************************************************************
# *     Makefile for Atmel AT91SAM7S256                         *
# ***************************************************************

#################################################################
# Start of default section
#
WINAVR_PATH = ../WinAVR

CC = $(WINAVR_PATH)/bin/avr-gcc
LD = $(WINAVR_PATH)/bin/avr-ld -v
AR = $(WINAVR_PATH)/bin/avr-ar
AS = $(WINAVR_PATH)/bin/avr-gcc
CP = $(WINAVR_PATH)/bin/avr-objcopy
OD = $(WINAVR_PATH)/bin/avr-objdump
SIZE = $(WINAVR_PATH)/bin/avr-size
REMOVE = $(WINAVR_PATH)/utils/bin/rm.exe
MKDIR  = $(WINAVR_PATH)/utils/bin/mkdir.exe

#
# End of default section
#################################################################

#################################################################
# Start of user section
#

# Define project name here
PROJECT = ATMEGA32

# Define linker script file here
LDSCRIPT = 

# List all user C define here, like -D_DEBUG=1
DEFS = 

# Define ASM defines here
ADEFS  =  

# List output directory here
OBJDIR  = Object

# List source directory here
RELDIR  = release

# List all user include directories here
INCLUDEDIR   = ./
INCLUDEDIR  += ./Source
INCLUDEDIR  += ./Source/Hardware/Atmega32
INCLUDEDIR  += ./Projects/Atmega32/Include

# List all library directories here
LIBRARYDIR  = ./Lib

# List all user libraries here (naming convention libxxxx.a where xxxx e.g. plus)
LIBRARY = 

# List C source files here
SRC =

# List C source files here
SRC += Source/AvRtos.C
SRC += Source/Hardware/Atmega32/AvRtosHw.C
SRC += Projects/Atmega32/Source/main.C

# List ASM source files here
ASRC = 

# Define optimisation level here
OPT = -O0

# Define warnings here
WARNING = all no-pointer-sign

# Generate dependency information
DEP     = -MD -MP -MF .dep/$(@F).d

# Define CPU compiler flags
MCFLAGS = -mmcu=atmega32

#################################################################
# Define variables needed for compiler flags here
INCDIR  = $(patsubst %,-I%,$(INCLUDEDIR))
LIBDIR  = $(patsubst %,-L%,$(LIBRARYDIR))
LIBS    = $(patsubst %,$(LIBRARYDIR)/%,$(LIBRARY))
WARN    = $(patsubst %,-W%,$(WARNING))

# Define compiler flags here
ASFLAGS = $(MCFLAGS) -g3 -ahls -Wa,-mapcs-32,-amhls=$(<:.S=.lst) $(ADEFS)
CPFLAGS = $(MCFLAGS) $(WARN) $(OPT) -g3 $(DEFS) -fpack-struct -fshort-enums -std=gnu99 -funsigned-char -funsigned-bitfields
LDFLAGS = $(MCFLAGS) -Wl,-Map,./Object/$(PROJECT).map --start-group -lm --end-group

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
all: prebuild compile link hex postbuild
prebuild : $(DIRS)
compile : $(OBJ) 
link : $(PROJECT).elf
hex : $(PROJECT).hex

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

$(PROJECT).hex: $(PROJECT).elf
	@ echo "************** Creating image **********************"
	$(CP) -R .eeprom -O ihex $(OBJDIR)/$(PROJECT).elf $(OBJDIR)/$(PROJECT).hex
	$(CP) -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex $(OBJDIR)/$(PROJECT).elf $(OBJDIR)/$(PROJECT).eep
	
postbuild:
	$(SIZE) --format=avr --mcu=$(MCU) $(OBJDIR)/$(PROJECT).elf

clean:
	$(REMOVE) -fR .dep
	$(REMOVE) -fR $(OBJ)
	$(REMOVE) -f $(OBJDIR)/$(PROJECT).elf
	$(REMOVE) -f $(OBJDIR)/$(PROJECT).map
	$(REMOVE) -f $(OBJDIR)/$(PROJECT).bin
	$(REMOVE) -f $(OBJDIR)/$(PROJECT).hex
	$(REMOVE) -f $(OBJDIR)/$(PROJECT).eep

program:
	$(WINAVR_PATH)/bin/avrdude.exe -c jtag1 -P COM4 -p m32 -U flash:w:$(OBJDIR)/$(PROJECT).hex

debug:
	$(WINAVR_PATH)/bin/avarice.exe -1 -j /dev/com4 -B 500 -P atmega32 :4242
	
dox:
	doxygen.exe Doxyfile

# 
# Include the dependency files, should be the last of the makefile
#
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***
