#/****************************************************************
#KPIT Infosystems Ltd, Pune, India. - 13-Mar-2002.
#
#This program is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#
#*****************************************************************/

# H8-300H target directives

APPNAME = test

PREFIX=h8300-coff

# Define directories.
APP_DIR					= .

uSMARTX_INC_DIR			= ../../../inc
uSMARTX_LIB_DIR			= ../lib
uSMARTXLIB				= usmartx

# Files
APP_OBJECTS				= $(APPNAME).o start.o vects.o

OBJS					= $(APP_OBJECTS)

VPATH					= $(APP_DIR)

# Specify compiler to be used
CC = $(PREFIX)-gcc

# gcc options
OPTIONS =-nostartfiles -mh -O2 -Wall -I$(APP_DIR) -I$(uSMARTX_INC_DIR)

# Create srec file
all: $(APPNAME).mot
$(APPNAME).mot : $(APPNAME).out
	$(PREFIX)-objcopy -O srec $(APPNAME).out $@

# Link object files
$(APPNAME).out : $(OBJS) $(APPNAME).lnk makefile
	$(CC) $(OPTIONS) -L$(uSMARTX_LIB_DIR) -T$(APP_DIR)/$(APPNAME).lnk -Xlinker -Map -Xlinker $(APPNAME).map -o $@ $(OBJS) -l$(uSMARTXLIB)

# Compile .c files
%.o:	./%.c makefile $(HEADERS)
		$(CC) $(OPTIONS) -c $< -o $@
		
# Assemble .s files
%.o:	./%.s makefile $(HEADERS)
		$(CC) $(OPTIONS) -c $< -o $@
clean:
	rm -f  *.out *.mot *.map *.o *.o
