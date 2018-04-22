#
# Copyright (C) 2012 by egnite GmbH
# Copyright (C) 2005 by egnite Software GmbH
# 
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holders nor the names of
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
# THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.
#
# For additional information see http://www.ethernut.de/
#
# $Id$
#

AC_INIT(nutdisc, 3.0.0)
m4_ifdef([AM_SILENT_RULES],[AM_SILENT_RULES([yes])])

AC_CONFIG_MACRO_DIR([m4])
AM_INIT_AUTOMAKE
AC_CONFIG_HEADERS([config.h])
AM_MAINTAINER_MODE

AC_PROG_CC

AC_ARG_ENABLE(nutdisc,
  AS_HELP_STRING([--disable-nutdisc],[disable NutOs Discoverer]),
  [],
  [enable_nutdisc=yes]
)

AC_ARG_ENABLE(wxpropgrid,
  AS_HELP_STRING([--disable-wxpropgrid],[disable xwProperty Grid]),
  [],
  [enable_wxpropgrid=yes]
)

# Path to wx-config by default or from command line.
WXCONFIG=wx-config
AC_ARG_WITH(wx-config,
[[  --with-wx-config=FILE
    Use the given path to wx-config when determining
    wxWidgets configuration; defaults to "wx-config"]],
[
    if test "$withval" != "yes" -a "$withval" != ""; then
	WXCONFIG=$withval
    fi
])

# Check wxWidgets' version.
AC_MSG_CHECKING([wxWidgets version])
if wxversion=`$WXCONFIG --version`; then
    AC_MSG_RESULT([$wxversion])
    wxwidgets_ok=yes
else
    AC_MSG_RESULT([not found])
    AC_MSG_ERROR([wxWidgets is required. Try --with-wx-config.])
    wxwidgets_ok=no
fi

# Check property grid path, may be either /usr/ or /usr/local/.
if test "x$wxwidgets_ok" = "xyes"; then 
  AC_MSG_NOTICE([checking for propgrid])
  if test "x$enable_wxpropgrid" = "xyes" ; then
    AC_MSG_NOTICE([wxpropgrid enabled])
    AC_MSG_NOTICE([checking for wxpropgrid])
    propgridpath1=/usr/include/wx-`wx-config --release`/wx/propgrid
    propgridpath2=/usr/local/include/wx-`wx-config --release`/wx/propgrid
    AC_CHECK_FILE($propgridpath1/propgrid.h,[propgrid_ok="yes"],[propgrid_ok="no"])
    if test "x$propgrid_ok" = "xno"; then
        AC_CHECK_FILE($propgridpath2/propgrid.h,[propgrid_ok="yes"],[propgrid_ok="no"])
    fi
    if test "x$propgrid_ok" = "xno"; then
      AC_MSG_WARN(wxpropgrid not found)
    else
      AC_MSG_NOTICE(wxpropgrid enabled)
    fi
  else
    propgrid_ok=no
    AC_MSG_WARN([wxpropgrid disabled])
  fi
fi

# Check requirements and enable or disable nutdisc build.
if test "x$wxwidgets_ok" = "xyes"; then 
  AC_MSG_NOTICE([checking for nutdisc])
  if test "x$enable_nutdisc" = "xno"; then
    AC_MSG_NOTICE([nutdisc disabled])
    nutdisc_ok=no
  else
    if test "x$propgrid_ok" = "xno"; then
      AC_MSG_NOTICE([nutdisc requires propgrid])
      nutdisc_ok=no
    else
      AC_MSG_NOTICE([nutdisc enabled])
      nutdisc_ok=yes
    fi
  fi
fi
 
WX_CPPFLAGS="`$WXCONFIG --cppflags`"
WX_CXXFLAGS="`$WXCONFIG --cxxflags`"
WX_LIBS="`$WXCONFIG --libs`"
WX_PROPGRID="`$WXCONFIG --optional-libs propgrid`"

AC_SUBST(WX_CPPFLAGS)
AC_SUBST(WX_CXXFLAGS)
AC_SUBST(WX_LIBS)
AC_SUBST(WX_PROPGRID)

AC_PROG_CPP
AC_PROG_CXX

AM_PROG_LIBTOOL

# Propagate our conditionals.
AM_CONDITIONAL(BUILD_NUTDISC, test x$nutdisc_ok = xyes)

AC_OUTPUT([
Makefile
src/Makefile

])

