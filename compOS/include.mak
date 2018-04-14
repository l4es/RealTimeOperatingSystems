# Common definitions for all makefiles
# Based on the GeekOS makefile, see "COPYING-GEEKOS".

# we only support linux now!

CC := gcc
LD := ld
OBJCOPY := objcopy
CPLUS := g++

# Compiler flags.
CFLAGS := -O -Wall -D_COS_ -DPROTOTYPES -I../include -fno-builtin
CXXFLAGS := $(CFLAGS)
LDFLAGS := -nostdlib

# CFLAGS includes -DPROTOTYPES for bget

# Nasm.  This is written for version 0.98
NASM := nasm
NASMFLAGS := -f elf -i ../include

# Perl (needs to be version 5 or later).
PERL = perl

# Utility scripts
PAD := $(PERL) ../scripts/pad
NUMSECS := $(PERL) ../scripts/numsecs
CAT := cat
RM :=  rm -f
LS := ls

# directories in this project
dirs := . doc include kernel scripts
