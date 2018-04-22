#!/bin/sh

FILES="Makefile Makefile.in aclocal.m4 \
	config.h config.h.in config.log config.status config.sub config.guess \
	configure install-sh missing mkinstalldirs \
	stamp-h stamp-h.in stamp-h1 depcomp \
	intltool-extract.in intltool-merge.in intltool-update.in \
	unpluck/Makefile unpluck/Makefile.in"

DIRS="autom4te.cache"

echo "Removing auto-files..."
for i in $FILES; do
	echo -n "$i "
	rm -f $i
done

echo -e "\n\n"

echo "Removing auto-dirs..."
for i in $DIRS; do
	echo "$i "
	rm -f $i/*
	rmdir $i
done

echo -e "\n\n"
echo "Clean."
