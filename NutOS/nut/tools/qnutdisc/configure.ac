dnl **************************************************************************
dnl (c) 2012 Ole Reinhardt <ole.reinhardt@embedded-it.de>
dnl **************************************************************************
dnl Process this file with autoconf to produce a configure script.

AC_INIT(qnutdisc, 3.0)
m4_ifdef([AM_SILENT_RULES],[AM_SILENT_RULES([yes])])

AC_CONFIG_MACRO_DIR([m4])
AM_INIT_AUTOMAKE
AC_CONFIG_HEADERS([config.h])
AM_MAINTAINER_MODE

AC_PROG_CXX

AC_CHECK_PROGS([UIC], [uic-qt4 uic], [none])
if test "$UIC" == "none"; then
   AC_MSG_ERROR([Qt UI Compiler not found])
fi

AM_PROG_LIBTOOL



PKG_CHECK_MODULES(QNUTDISC, [QtCore QtGui QtNetwork])




AC_OUTPUT([
Makefile
src/Makefile

])

