# Kernel makefile

include ../include.mak

MY_OBJECTS := boot.o kmain.o printk.o psnprintf.o sout.o string.o thread.o \
              debug.o mem.o int.o idt.o interrupt.o gdt.o tss.o sysinit.o \
	      kalloc.o heap.o page.o timer.o blockio.o vtext.o thr.o test.o

# rules to create objects
.SUFFIXES:
.SUFFIXES: .c .cxx .s .o

.c.o:
	$(CC) $(CFLAGS) -c $<

.cxx.o:
	$(CPLUS) $(CXXFLAGS) -c $<

.s.o:
	$(NASM) $(NASMFLAGS) -o $*.o $<

# targets

kernel.bin : $(MY_OBJECTS)
	$(LD) -M -o kernel.bin -T kernel.ld -e kstart $(MY_OBJECTS) \
		> storage.txt

clean :
	$(RM) *.o *.exe *.bin storage.txt

rm-backups:
	$(RM) *~
