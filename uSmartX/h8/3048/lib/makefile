# Makefile for H8 300H target

# Define directories.	
	uSMARTX_INC_DIR = ../../../inc
	uSMARTX_SRC_DIR = ../../../src/
	uSMARTX_HAL_INC_DIR = ../inc
	
# Define programs.
	AR = h8300-coff-ar
    CC = h8300-coff-gcc
    ASSEMBLE = h8300-coff-gcc
    REMOVE = rm -f
    COPY = cp
    MOVE = mv

# Target file name (without extension).
	TARGET = libusmartx

# List C source files here.
	SRC =	$(uSMARTX_SRC_DIR)usmartx.c \
			$(uSMARTX_SRC_DIR)queue.c \
			$(uSMARTX_SRC_DIR)mailbox.c \
			$(uSMARTX_SRC_DIR)semaphore.c \
			$(uSMARTX_SRC_DIR)mballoc.c \
			$(uSMARTX_SRC_DIR)cycbuff.c \
			../../hal/hal.c

# Compiler flags.
	CPFLAGS = -nostartfiles -mh -c -O2 -Wall
	
# Assembler flags.
    ASFLAGS = -Wa,-ahlms=$(<:.s=.lst)

# Define all project specific object files.
	OBJ	= $(SRC:.c=.o) $(ASRC:.s=.o) 
	
# Define all listing files.
	LST = $(ASRC:.s=.lst) $(SRC:.c=.lst)
  
# Default target.
.PHONY : Lib
Lib: Clean $(TARGET).a clear_output

# Link: create .a output file from object files.
.PRECIOUS : $(OBJ)
%.a: $(OBJ)
	$(AR) rs $(TARGET).a $(OBJ)

# Compile: create object files from C source files.
%.o : %.c
	$(CC) $(CPFLAGS) -I$(uSMARTX_INC_DIR) -I$(uSMARTX_HAL_INC_DIR) $< -o $@ 

# Assemble: create object files from assembler files.
%.o : %.s
	$(ASSEMBLE) -c $(ASFLAGS) $< -o $@

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

# Target copy output files
.SILENT : move_output
.PHONY: move_output
move_output :
	$(MOVE) $(TARGET).a ../lib/$(TARGET).a
	
# Target clear output files
.SILENT : clear_output
.PHONY: clear_output
clear_output :
	$(RM)  $(uSMARTX_SRC_DIR)*.o
	$(RM)  *.o
	$(RM)  ../../hal/*.o
	$(RM)  ../../hal/*.lst

