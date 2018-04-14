# Makefile for COS, makes a floppy disk image
# Based on the GeekOS makefile, see "COPYING-GEEKOS".

include include.mak

all:
	make -C kernel

clean:
	make -C kernel clean
	
rm-backups:
	for dir in $(dirs); do $(RM) -v $$dir/*~; done
	
ls-backups:
	for dir in $(dirs); do $(LS) $$dir/*~; done; exit 0
