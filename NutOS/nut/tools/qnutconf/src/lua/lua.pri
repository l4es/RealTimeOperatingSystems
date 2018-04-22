
DEPENDPATH += $$PWD/src
INCLUDEPATH += $$PWD/src

# Input
HEADERS += \
           $$PWD/src/lapi.h \
           $$PWD/src/lauxlib.h \
           $$PWD/src/lcode.h \
           $$PWD/src/ldebug.h \
           $$PWD/src/ldo.h \
           $$PWD/src/lfunc.h \
           $$PWD/src/lgc.h \
           $$PWD/src/llex.h \
           $$PWD/src/llimits.h \
           $$PWD/src/lmem.h \
           $$PWD/src/lobject.h \
           $$PWD/src/lopcodes.h \
           $$PWD/src/lparser.h \
           $$PWD/src/lstate.h \
           $$PWD/src/lstring.h \
           $$PWD/src/ltable.h \
           $$PWD/src/ltm.h \
           $$PWD/src/lua.h \
           $$PWD/src/luaconf.h \
           $$PWD/src/lualib.h \
           $$PWD/src/lundump.h \
           $$PWD/src/lvm.h \
           $$PWD/src/lzio.h 

SOURCES += \
           $$PWD/src/lapi.c \
           $$PWD/src/lauxlib.c \
           $$PWD/src/lbaselib.c \
           $$PWD/src/lcode.c \
           $$PWD/src/ldblib.c \
           $$PWD/src/ldebug.c \
           $$PWD/src/ldo.c \
           $$PWD/src/ldump.c \
           $$PWD/src/lfunc.c \
           $$PWD/src/lgc.c \
           $$PWD/src/linit.c \
           $$PWD/src/liolib.c \
           $$PWD/src/llex.c \
           $$PWD/src/lmathlib.c \
           $$PWD/src/lmem.c \
           $$PWD/src/loadlib.c \
           $$PWD/src/lobject.c \
           $$PWD/src/lopcodes.c \
           $$PWD/src/loslib.c \
           $$PWD/src/lparser.c \
           $$PWD/src/lstate.c \
           $$PWD/src/lstring.c \
           $$PWD/src/lstrlib.c \
           $$PWD/src/ltable.c \
           $$PWD/src/ltablib.c \
           $$PWD/src/ltm.c \
           $$PWD/src/lundump.c \
           $$PWD/src/lvm.c \
           $$PWD/src/lzio.c \
           $$PWD/src/print.c
