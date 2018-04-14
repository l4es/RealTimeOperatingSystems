# Makefile for ARM target

# Define directories.
	uSMARTX_INC_DIR = ../../../inc
	LDSCRIPT = ram.ld
	uSMARTX_LIB_DIR = ../lib/
	uSMARTX_LIB = usmartx
	
# Define programs.
    SHELL = sh
    AR = ar
    COMPILE = arm-elf-gcc
    ASSEMBLE = arm-elf-gcc -x assembler-with-cpp
    REMOVE = rm -f
    COPY = cp
    MOVE = mv
    OBJCOPY = arm-elf-objcopy
    OBJDUMP = arm-elf-objdump
    HEXSIZE = @arm-elf-size --target=$(FORMAT) $(TARGET).hex
    ELFSIZE = @arm-elf-size $(TARGET).elf

# MCU name
	MCU = arm7tdmi

# Output format. Can be [srec|ihex].
    FORMAT = ihex

# Target file name (without extension).
	TARGET = test

# List C source files here.
	SRC = ../../test/test.c
			
# List Assembler source files here.
	ASRC = crt0.s

# Compiler flags.
	CPFLAGS = -g -O0 -Wall -Wa,-ahlms=$(<:.c=.lst)

# Assembler flags.
    ASFLAGS = -Wa,-ahlms=$(<:.s=.lst),--gstabs 

# Linker flags (passed via GCC).
	LDFLAGS = -L$(uSMARTX_LIB_DIR) -nostartfiles -T$(LDSCRIPT) -Wl,-Map=$(TARGET).map,--cref

# Additional library flags (-lm = math library).
	LIBFLAGS = -l$(uSMARTX_LIB)

# Define all project specific object files.
	OBJ	= $(SRC:.c=.o) $(ASRC:.s=.o) 	
	
# Define all listing files.
	LST = $(ASRC:.s=.lst) $(SRC:.c=.lst)
	
# Compiler flags to generate dependency files.
	GENDEPFLAGS = -Wp,-M,-MP,-MT,$(*F).o,-MF,.dep/$(@F).d

# Add target processor to flags.
   	CPFLAGS += -mcpu=$(MCU) $(GENDEPFLAGS)
	ASFLAGS += -mcpu=$(MCU)
	LDFLAGS += -mcpu=$(MCU)	
  
.PHONY : Build
Build: $(TARGET).elf $(TARGET).hex $(TARGET).lss $(TARGET).bin line1 overallsize line2

.PHONY : BuildAll
BuildAll: clean $(TARGET).elf $(TARGET).hex $(TARGET).lss $(TARGET).bin  line1 overallsize line2

.PHONY : overallsize
overallsize:
	@echo Elf size:
	$(ELFSIZE)
	
%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

# Create final output files (.hex, .lss) from ELF output file.
%.hex: %.elf
	$(OBJCOPY) -O $(FORMAT) $< $@

# Create extended listing file from ELF output file.
%.lss: %.elf
	$(OBJDUMP) -h -S $< > $@

# Link: create ELF output file from object files.
.SECONDARY : $(TRG).elf
.PRECIOUS : $(OBJ)
%.elf: $(OBJ)
	$(COMPILE) $(LDFLAGS) $(OBJ) $(LIBFLAGS) --output $@
	
# Compile: create object files from C source files.
%.o : %.c
	$(COMPILE) -c $(CPFLAGS) -I$(uSMARTX_INC_DIR) $< -o $@ 

# Assemble: create object files from assembler files.

%.o : %.s
	$(ASSEMBLE) -c $(ASFLAGS) $< -o $@

# Target: line1 project.
.PHONY : line1
line1 :
	@echo ---------------------------------------------------------------------------------------------------

# Target: line2 project.
.PHONY : line2
line2 :
	@echo ---------------------------------------------------------------------------------------------------

# Target: clean project.
.SILENT : Clean
.PHONY : Clean
Clean :
	$(REMOVE) $(TARGET).hex
	$(REMOVE) $(TARGET).obj
	$(REMOVE) $(TARGET).elf
	$(REMOVE) $(TARGET).bin
	$(REMOVE) $(TARGET).map
	$(REMOVE) $(TARGET).obj
	$(REMOVE) $(TARGET).sym
	$(REMOVE) $(TARGET).lnk
	$(REMOVE) $(TARGET).lss
	$(REMOVE) $(OBJ)
	$(REMOVE) $(LST)
	$(REMOVE) $(SRC:.c=.s)
	
# Include the dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)


# List assembly only source file dependencies here:

