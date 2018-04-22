; General
;
SetCompressor /SOLID lzma

!define MULTIUSER_EXECUTIONLEVEL Admin
!define MULTIUSER_MUI
!define MULTIUSER_INSTALLMODE_COMMANDLINE

!define PRODUCT  "Nut/OS"
!define NUTVERSION  "5.2"
!define NUTRELEASE  "2"
!define INSTBUILD   "1"
!define SWREGKEY    "Software\egnite\Ethernut"

Name "${PRODUCT}"
OutFile "ethernut-${NUTVERSION}.${NUTRELEASE}-${INSTBUILD}.exe"
XPStyle off
BrandingText "${PRODUCT} ${NUTVERSION}.${NUTRELEASE}-${INSTBUILD}"

InstallDir c:\ethernut-${NUTVERSION}
InstallDirRegKey HKLM "${SWREGKEY}\${NUTVERSION}" ""

; Modern Interface 2
; 
!include "MUI2.nsh"
!include "library.nsh"

; Icons
;
;Icon nut2.ico
;UninstallIcon nutuni.ico

; Interface
!define MUI_ICON nut2.ico
!define MUI_UNICON nutuni.ico
!define MUI_ABORTWARNING

!define UMUI_USE_ALTERNATE_PAGE
!define UMUI_USE_UNALTERNATE_PAGE

!define MUI_COMPONENTSPAGE_SMALLDESC

;Remember the installer language
!define MUI_LANGDLL_ALWAYSSHOW
!define MUI_LANGDLL_REGISTRY_ROOT "HKLM"
!define MUI_LANGDLL_REGISTRY_KEY "${SWREGKEY}\${NUTVERSION}"
!define MUI_LANGDLL_REGISTRY_VALUENAME "Installer Language"

; Pages
!insertmacro MUI_PAGE_WELCOME
;!insertmacro MUI_PAGE_LICENSE "License.txt"
!insertmacro MUI_PAGE_COMPONENTS
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES

!define MUI_FINISHPAGE_LINK "http://www.ethernut.de"
!define MUI_FINISHPAGE_LINK_LOCATION "http://www.ethernut.de/"
!define MUI_FINISHPAGE_RUN "$INSTDIR\qnutconf.exe"
!define MUI_FINISHPAGE_RUN_TEXT "Start Nut/OS Configurator"
!define MUI_FINISHPAGE_RUN_NOTCHECKED

!insertmacro MUI_PAGE_FINISH
;!insertmacro MUI_PAGE_ABORT

!insertmacro MUI_UNPAGE_WELCOME
!insertmacro MUI_UNPAGE_CONFIRM
!insertmacro MUI_UNPAGE_INSTFILES
!insertmacro MUI_UNPAGE_FINISH

;Languages
;
!insertmacro MUI_LANGUAGE "English"
!insertmacro MUI_LANGUAGE "French"
!insertmacro MUI_LANGUAGE "German"

!insertmacro MUI_RESERVEFILE_LANGDLL

VIProductVersion "${NUTVERSION}.${NUTRELEASE}.${INSTBUILD}"
VIAddVersionKey /LANG=${LANG_ENGLISH} "ProductName" "Ethernut"
VIAddVersionKey /LANG=${LANG_ENGLISH} "Comments" "Open Source Software and Hardware Project"
VIAddVersionKey /LANG=${LANG_ENGLISH} "CompanyName" "egnite GmbH"
VIAddVersionKey /LANG=${LANG_ENGLISH} "LegalTrademarks" "Ethernut is a trademark of egnite GmbH"
VIAddVersionKey /LANG=${LANG_ENGLISH} "LegalCopyright" "Copyright ® 2001-2011 by egnite GmbH"
VIAddVersionKey /LANG=${LANG_ENGLISH} "FileDescription" "Embedded RTOS and TCP/IP Stack Installer"
VIAddVersionKey /LANG=${LANG_ENGLISH} "FileVersion" "${NUTVERSION}.${NUTRELEASE}"

; Installer Sections
;
InstType "Full"
InstType "Build with Configurator"
InstType "Build with WinAVR in Source Tree"

Section
  SetOutPath "$INSTDIR"
  File ..\..\..\..\AUTHORS
  File ..\..\..\..\COPYING
  File ..\..\..\..\INSTALL
  File ..\..\..\..\NEWS
  File ..\..\..\..\README

  SetOutPath "$INSTDIR\nut\appicc"
  File ..\..\..\appicc\README
  
  SetOutPath "$INSTDIR\nut\bin"
  File ..\..\..\bin\README

  SetOutPath "$INSTDIR\nut\bin\arm7tdmi"
  File ..\..\..\bin\arm7tdmi\README
  
  SetOutPath "$INSTDIR\nut\bin\arm9"
  File ..\..\..\bin\arm9\README
  
  SetOutPath "$INSTDIR\nut\bin\at90can128"
  File ..\..\..\bin\at90can128\README

  SetOutPath "$INSTDIR\nut\bin\atmega103"
  File ..\..\..\bin\atmega103\README
  
  SetOutPath "$INSTDIR\nut\bin\atmega128"
  File ..\..\..\bin\atmega128\README
  
  SetOutPath "$INSTDIR\nut\bin\atmega2561"
  File ..\..\..\bin\atmega2561\README

  SetOutPath "$INSTDIR\nut\doc"
  File ..\..\..\doc\nutos-sw-manual.odt
  File ..\..\..\doc\copying-gpl.txt
  File ..\..\..\doc\copying-liquorice.txt
  File ..\..\..\tools\packaging\nsis\license.txt

  SetOutPath "$INSTDIR\nut"
  File ..\..\..\nutsetup
  
  SetOutPath "$INSTDIR"
  WriteRegStr HKLM "${SWREGKEY}\${NUTVERSION}" "" $INSTDIR
  WriteUninstaller "$INSTDIR\Uninstall.exe"
SectionEnd

SectionGroup "Nut/OS"
  Section "Prebuild Libraries" SecOsLibs
    SectionIn 1
    SetOutPath "$INSTDIR\nut\lib\boards"
    File /r ..\..\..\lib\boards\*.o
    File /r ..\..\..\lib\boards\*.a
  SectionEnd

  Section "Header Files" SecOsHeaders
    SectionIn 1 2 3
    SetOutPath "$INSTDIR\nut\include"
    File /r ..\..\..\include\*.h
    File /r ..\..\..\include\*.txt
  SectionEnd

  Section "RTOS Source Code" SecOsSources
    SectionIn 1 2 3

    SetOutPath "$INSTDIR\nut"
    File ..\..\..\lib\Makefile
    File ..\..\..\*.all
    File ..\..\..\*.gcc
    File ..\..\..\*.arm-gcc
    File ..\..\..\*.arm-gccdbg
    File ..\..\..\*.arm-eCross-gcc
    File ..\..\..\*.arm-eCross-gccdbg
;    File ..\..\..\*.arm-icc
    File ..\..\..\*.arm-thumb-gcc
    File ..\..\..\*.arm-thumb-gccdbg
    File ..\..\..\*.avr-gcc
    File ..\..\..\*.avr-gccdbg
    File ..\..\..\*.avr32-gcc
    File ..\..\..\*.avr32-gccdbg
    File ..\..\..\*.avr-icc
    File ..\..\..\*.avr-icc7
    File ..\..\..\*.avrext-icc
    File ..\..\..\*.avrext-icc7
    File ..\..\..\*.cm3-ecross-gcc
    File ..\..\..\*.cm3-ecross-gccdbg
    File ..\..\..\*.cm3-gcc
    File ..\..\..\*.cm3-gccdbg
    File ..\..\..\*.licenses
    File ..\..\..\*.m68k-gcc
    File ..\..\..\*.m68k-gccdbg
    File ..\..\..\*.unix-gcc

    SetOutPath "$INSTDIR\nut\lib"
    File ..\..\..\lib\Makefile

    SetOutPath "$INSTDIR\nut\arch"
    File /r ..\..\..\arch\Makefile
    File /r ..\..\..\arch\*.c
    File /r ..\..\..\arch\*.h
    File /r ..\..\..\arch\*.S
    File /r ..\..\..\arch\*.ld

    SetOutPath "$INSTDIR\nut\c"
    File /r ..\..\..\c\*.c

    SetOutPath "$INSTDIR\nut\cpp"
    File ..\..\..\cpp\Makefile
    File ..\..\..\cpp\*.cc

    SetOutPath "$INSTDIR\nut\crt"
    File ..\..\..\crt\Makefile
    File ..\..\..\crt\*.c
    File ..\..\..\crt\*.h

    SetOutPath "$INSTDIR\nut\dev"
    File ..\..\..\dev\Makefile
    File ..\..\..\dev\*.c

    SetOutPath "$INSTDIR\nut\fs"
    File ..\..\..\fs\Makefile
    File ..\..\..\fs\*.c

    SetOutPath "$INSTDIR\nut\gorp"
    File /r ..\..\..\gorp\Makefile
    File /r ..\..\..\gorp\*.c

    SetOutPath "$INSTDIR\nut\lua"
    File ..\..\..\lua\Makefile
    File ..\..\..\lua\*.c

    SetOutPath "$INSTDIR\nut\os"
    File ..\..\..\os\Makefile
    File ..\..\..\os\*.c

  SectionEnd
SectionGroupEnd

SectionGroup "Nut/Net"
  Section "TCP/IP Source Code" SecNetSources
    SectionIn 1 2 3

    SetOutPath "$INSTDIR\nut\net"
    File ..\..\..\net\Makefile
    File ..\..\..\net\*.c

    SetOutPath "$INSTDIR\nut\pro"
    File ..\..\..\pro\Makefile
    File ..\..\..\pro\*.c
    File ..\..\..\pro\*.h
    SetOutPath "$INSTDIR\nut\pro\uhttp"
    File /r ..\..\..\pro\uhttp\*.c

  SectionEnd
SectionGroupEnd

SectionGroup "Contributions"
  Section "Non-BSDL Source Code" SecContribs
    SectionIn 1 2 3

    SetOutPath "$INSTDIR\nut\contrib"
    File /r ..\..\..\contrib\Makefile
    File /r ..\..\..\contrib\*.c
    File /r ..\..\..\contrib\*.S
    File /r ..\..\..\contrib\*.txt
  SectionEnd
SectionGroupEnd

SectionGroup "Manuals"
  Section "API Reference" SecDocApi
    SectionIn 1 2 3
    SetOutPath "$INSTDIR\nut\doc\en\html"
    File ..\..\..\doc\en\html\*.html
    File ..\..\..\doc\en\html\*.xml
    File ..\..\..\doc\en\html\*.css
    File ..\..\..\doc\en\html\*.map
    File ..\..\..\doc\en\html\*.md5
    File ..\..\..\doc\en\html\*.png
;    File ..\..\..\doc\en\html\*.gif
    File ..\..\..\doc\en\html\*.js
    SetOutPath "$INSTDIR\nut\doc\en\html\search"
    File ..\..\..\doc\en\html\search\*.html
    File ..\..\..\doc\en\html\search\*.css
    File ..\..\..\doc\en\html\search\*.js
    File ..\..\..\doc\en\html\search\*.png
    SetOutPath "$INSTDIR\nut\doc\gen"
    File ..\..\..\doc\gen\*.in
    File ..\..\..\doc\gen\*.cfg
    File ..\..\..\doc\gen\*.css
    File ..\..\..\doc\gen\*.html
    File ..\..\..\doc\gen\*.png
    File ..\..\..\doc\gen\*.txt
    SetOutPath "$INSTDIR\nut\doc\images"
    File ..\..\..\doc\images\*.gif
    SetOutPath "$INSTDIR"
    FILE ..\..\..\doc\en\chm\nutosapiref.chm
    FILE ..\..\..\doc\en\chm\nutosapiref.chi
    SetOutPath "$SMPROGRAMS\Ethernut ${NUTVERSION}"
    CreateShortCut "$SMPROGRAMS\Ethernut ${NUTVERSION}\WinHelp API Reference.lnk" "$INSTDIR\nutosapiref.chm"
    CreateShortCut "$SMPROGRAMS\Ethernut ${NUTVERSION}\HTML API Reference.lnk" "$INSTDIR\nut\doc\en\html\index.html"  
  SectionEnd
SectionGroupEnd

SectionGroup "Development Tools"
  Section "Nut/OS Configurator" SecConfig
    SectionIn 1 2
    SetOutPath "$INSTDIR"
    File ..\..\..\tools\win32\nutconf.exe
    File ..\..\..\tools\win32\nutconfigure.exe
    File ..\..\..\tools\win32\msvcr71.dll
    File ..\..\..\tools\win32\msvcp71.dll
    File ..\..\..\tools\win32\qnutconf.exe
    File ..\..\..\tools\win32\msvcr90.dll
    File ..\..\..\tools\win32\msvcp90.dll
    File ..\..\..\tools\win32\msvcr100.dll
    File ..\..\..\tools\win32\msvcp100.dll
    File ..\..\..\tools\win32\QtCore4.dll
    File ..\..\..\tools\win32\QtGui4.dll
    File ..\..\..\tools\win32\libusb0.dll
    File ..\..\..\tools\win32\lua51.dll
    File ..\..\..\tools\win32\msys-1.0.dll
    File ..\..\..\tools\win32\WinIo.dll
    File ..\..\..\tools\win32\Microsoft.VC90.CRT.manifest
    SetOutPath "$INSTDIR\nut\conf"
    File ..\..\..\conf\*.conf
    File /r ..\..\..\conf\*.nut
    SetOutPath "$INSTDIR"
    CreateShortCut "$SMPROGRAMS\Ethernut ${NUTVERSION}\Configurator.lnk" "$INSTDIR\qnutconf.exe"
    CreateShortCut "$SMPROGRAMS\Ethernut ${NUTVERSION}\Configurator (wxWidgets).lnk" "$INSTDIR\nutconf.exe"
  SectionEnd

  Section "Ethernut Discoverer" SecDiscover
    SectionIn 1 2
    SetOutPath "$INSTDIR"
    File ..\..\..\tools\win32\nutdisc.exe
    CreateShortCut "$SMPROGRAMS\Ethernut ${NUTVERSION}\Discoverer.lnk" "$INSTDIR\nutdisc.exe"
  SectionEnd

  Section "Compatibility Tools" SecCompat
    SectionIn 1 2
    SetOutPath "$INSTDIR\nut\tools\win32"
    File ..\..\..\tools\win32\cp.exe
    File ..\..\..\tools\win32\make.exe
    File ..\..\..\tools\win32\rm.exe
  SectionEnd

  Section "OpenOCD Configuration" SecOpenOcd
    SectionIn 1 2
    SetOutPath "$INSTDIR\nut\tools\win32"
    File ..\..\..\tools\win32\enut3ocd.cfg
    File ..\..\..\tools\win32\enut3ocd.ini
    File ..\..\..\tools\win32\eir1ocd.cfg
    File ..\..\..\tools\win32\eir1ocd.ini
    File ..\..\..\tools\win32\startoocd.bat
  SectionEnd

  Section "SAM-BA Support" SecSamBa
    SectionIn 1 2
    SetOutPath "$INSTDIR\nut\tools\win32"
    File ..\..\..\tools\win32\at91sam-sdram.tcl
  SectionEnd

  Section "AVRDUDE Programmer" SecAvrDude
    SectionIn 1 2
    SetOutPath "$INSTDIR\nut\tools\win32"
    File ..\..\..\tools\win32\avrdude.exe
    File ..\..\..\tools\win32\avrdude.conf
    File ..\..\..\tools\win32\loaddrv.exe
    File ..\..\..\tools\win32\install_giveio.bat
    File ..\..\..\tools\win32\remove_giveio.bat
    File ..\..\..\tools\win32\status_giveio.bat
    File ..\..\..\tools\win32\giveio.sys
    File ..\..\..\tools\win32\WinIo.sys
    File ..\..\..\tools\win32\libusb0.dll
    File ..\..\..\tools\win32\WinIo.dll
    File ..\..\..\tools\win32\msys-1.0.dll
  SectionEnd

  Section "JTAG-O-MAT Programmer" SecJtagomat
    SectionIn 1 2
    SetOutPath "$INSTDIR\nut\tools\win32"
    File ..\..\..\tools\win32\jtagomat.exe
    File ..\..\..\tools\win32\jtagomat-turtelizer.exe
    File ..\..\..\tools\win32\jtagomat-wiggler.exe
    File ..\..\..\tools\win32\jtagomat-openocd.exe
    File ..\..\..\tools\win32\flasher.bin
    File ..\..\..\tools\win32\flasher8.bin
    File ..\..\..\tools\win32\at91-erase.jom
    File ..\..\..\tools\win32\at91-fprobe.jom
    File ..\..\..\tools\win32\at91-remap.jom
    File ..\..\..\tools\win32\at91-upl.jom    
    File ..\..\..\tools\win32\at91-erase.bat
    File ..\..\..\tools\win32\at91-halt.bat
    File ..\..\..\tools\win32\at91w-erase.bat
    File ..\..\..\tools\win32\at91w-halt.bat
  SectionEnd

  Section "STK500 Programmer" SecStk500
    SectionIn 1 2
    SetOutPath "$INSTDIR\nut\tools\win32"
    File ..\..\..\tools\win32\Stk500.exe
    File ..\..\..\tools\win32\jtagiceii.exe
  SectionEnd

  Section "UROM Creator" SecUrom
    SectionIn 1 2 3
    SetOutPath "$INSTDIR\nut\tools\win32"
    File ..\..\..\tools\win32\crurom.exe
  SectionEnd

  Section "Raw Binary Creator" SecRobi
    SectionIn 1 2 3
    SetOutPath "$INSTDIR\nut\tools\win32"
    File ..\..\..\tools\win32\robi.exe
  SectionEnd

  Section "Source Code" SecToolSources
    SectionIn 1 3
    SetOutPath "$INSTDIR\nut\tools\crurom"
;    File ..\..\..\tools\crurom\Makefile
    File ..\..\..\tools\crurom\*.c
    SetOutPath "$INSTDIR\nut\tools\qnutconf\src"
    File ..\..\..\tools\qnutconf\src\*.c
    File ..\..\..\tools\qnutconf\src\*.cpp
    File ..\..\..\tools\qnutconf\src\*.h
    File ..\..\..\tools\qnutconf\src\*.rc
    File ..\..\..\tools\qnutconf\src\*.qrc
    File ..\..\..\tools\qnutconf\src\*.pro
    File ..\..\..\tools\qnutconf\src\*.ui
    SetOutPath "$INSTDIR\nut\tools\qnutconf\src\images"
    File ..\..\..\tools\qnutconf\src\images\*.bmp
    File ..\..\..\tools\qnutconf\src\images\*.ico
    File ..\..\..\tools\qnutconf\src\images\*.png
    File ..\..\..\tools\qnutconf\src\images\*.xpm
    SetOutPath "$INSTDIR\nut\tools\qnutconf\src\lua"
    File ..\..\..\tools\qnutconf\src\lua\*
    SetOutPath "$INSTDIR\nut\tools\qnutconf\src\lua\etc"
    File ..\..\..\tools\qnutconf\src\lua\etc\*
    SetOutPath "$INSTDIR\nut\tools\qnutconf\src\lua\src"
    File ..\..\..\tools\qnutconf\src\lua\src\*
    SetOutPath "$INSTDIR\nut\tools\nutconf\src"
    File ..\..\..\tools\nutconf\src\*.c
    File ..\..\..\tools\nutconf\src\*.cpp
    File ..\..\..\tools\nutconf\src\*.h
    File ..\..\..\tools\nutconf\src\*.rc
    SetOutPath "$INSTDIR\nut\tools\nutconf\src\bitmaps"
    File ..\..\..\tools\nutconf\src\bitmaps\*.bmp
    File ..\..\..\tools\nutconf\src\bitmaps\*.ico
    File ..\..\..\tools\nutconf\src\bitmaps\*.xpm
    SetOutPath "$INSTDIR\nut\tools\nutdisc\src"
    File ..\..\..\tools\nutdisc\src\*.cpp
    File ..\..\..\tools\nutdisc\src\*.h
    File ..\..\..\tools\nutdisc\src\*.rc
    SetOutPath "$INSTDIR\nut\tools\nutdisc\src\bitmaps"
    File ..\..\..\tools\nutdisc\src\bitmaps\*.ico
    File ..\..\..\tools\nutdisc\src\bitmaps\*.xpm
    SetOutPath "$INSTDIR\nut\tools\qnutdisc\src"
    File ..\..\..\tools\qnutdisc\src\*.cpp
    File ..\..\..\tools\qnutdisc\src\*.h
    File ..\..\..\tools\qnutdisc\src\*.pri
    File ..\..\..\tools\qnutdisc\src\*.pro
    File ..\..\..\tools\qnutdisc\src\*.qrc
    File ..\..\..\tools\qnutdisc\src\*.ui
    SetOutPath "$INSTDIR\nut\tools\include\win32"
    File ..\..\..\tools\include\win32\*.h
    SetOutPath "$INSTDIR\nut\tools\uhttp_win"
    File ..\..\..\tools\uhttp_win\README
    SetOutPath "$INSTDIR\nut\tools\uhttp_win\uhttp"
	File ..\..\..\tools\uhttp_win\uhttp\ReadMe.txt
	File ..\..\..\tools\uhttp_win\uhttp\uhttp.vcxproj.filters
	File ..\..\..\tools\uhttp_win\uhttp\uhttp.vcxproj.user
	File ..\..\..\tools\uhttp_win\uhttp\uhttp.vcxproj
    SetOutPath "$INSTDIR\nut\tools\uhttp_win\uhttp\include"
	File ..\..\..\tools\uhttp_win\uhttp\include\*.h
    SetOutPath "$INSTDIR\nut\tools\uhttp_win\uhttp\include\cfg"
	File ..\..\..\tools\uhttp_win\uhttp\include\cfg\*.h
    SetOutPath "$INSTDIR\nut\tools\uhttp_win\uhttp\include\isc"
	File ..\..\..\tools\uhttp_win\uhttp\include\isc\*.h
    SetOutPath "$INSTDIR\nut\tools\uhttp_win\uhttp\include\pro"
	File ..\..\..\tools\uhttp_win\uhttp\include\pro\*.h
    SetOutPath "$INSTDIR\nut\tools\uhttp_win\uhttp\include\pro\uhttp"
	File ..\..\..\tools\uhttp_win\uhttp\include\pro\uhttp\*.h
    SetOutPath "$INSTDIR\nut\tools\uhttp_win\uhttp\include\pro\uhttp\modules"
	File ..\..\..\tools\uhttp_win\uhttp\include\pro\uhttp\modules\*.h
    SetOutPath "$INSTDIR\nut\tools\uhttp_win\uhttp\include\pro\uhttp\os\win"
	File ..\..\..\tools\uhttp_win\uhttp\include\pro\uhttp\os\win\*.h
    SetOutPath "$INSTDIR\nut\tools\uhttp_win\uhttp_ajax"
	File ..\..\..\tools\uhttp_win\uhttp_ajax\uhttp_ajax.vcxproj.filters
	File ..\..\..\tools\uhttp_win\uhttp_ajax\uhttp_ajax.vcxproj.user
	File ..\..\..\tools\uhttp_win\uhttp_ajax\uhttp_ajax.vcxproj
    SetOutPath "$INSTDIR\nut\tools\uhttp_win\uhttp_auth"
	File ..\..\..\tools\uhttp_win\uhttp_auth\uhttp_auth.vcxproj.filters
	File ..\..\..\tools\uhttp_win\uhttp_auth\uhttp_auth.vcxproj.user
	File ..\..\..\tools\uhttp_win\uhttp_auth\uhttp_auth.vcxproj
    SetOutPath "$INSTDIR\nut\tools\uhttp_win\uhttp_form"
	File ..\..\..\tools\uhttp_win\uhttp_form\uhttp_form.vcxproj.filters
	File ..\..\..\tools\uhttp_win\uhttp_form\uhttp_form.vcxproj.user
	File ..\..\..\tools\uhttp_win\uhttp_form\uhttp_form.vcxproj
    SetOutPath "$INSTDIR\nut\tools\uhttp_win\uhttp_ssi"
	File ..\..\..\tools\uhttp_win\uhttp_ssi\uhttp_ssi.vcxproj.filters
	File ..\..\..\tools\uhttp_win\uhttp_ssi\uhttp_ssi.vcxproj.user
	File ..\..\..\tools\uhttp_win\uhttp_ssi\uhttp_ssi.vcxproj
    SetOutPath "$INSTDIR\nut\tools\uhttp_win\uhttp_tiny"
	File ..\..\..\tools\uhttp_win\uhttp_tiny\uhttp_tiny.vcxproj.filters
	File ..\..\..\tools\uhttp_win\uhttp_tiny\uhttp_tiny.vcxproj.user
	File ..\..\..\tools\uhttp_win\uhttp_tiny\uhttp_tiny.vcxproj
    SetOutPath "$INSTDIR\nut\tools\uhttp_win\uhttp_upload"
	File ..\..\..\tools\uhttp_win\uhttp_upload\uhttp_upload.vcxproj.filters
	File ..\..\..\tools\uhttp_win\uhttp_upload\uhttp_upload.vcxproj.user
	File ..\..\..\tools\uhttp_win\uhttp_upload\uhttp_upload.vcxproj
  SectionEnd
SectionGroupEnd


SectionGroup "Application Samples"
  Section "Ethernut Samples" SecSampleSources
    SectionIn 1 2 3

    SetOutPath "$INSTDIR\nut\app"
    File ..\..\..\app\Makefile
    File ..\..\..\app\Makeburn.*
    File ..\..\..\app\Makedefs.*
    File ..\..\..\app\Makerules.*
    File ..\..\..\app\Makevars.*

    SetOutPath "$INSTDIR\nut\app\caltime"
    File ..\..\..\app\caltime\Makefile
    File ..\..\..\app\caltime\*.c
    SetOutPath "$INSTDIR\nut\appicc\caltime"
    File ..\..\..\appicc\caltime\*.prj
    File ..\..\..\appicc\caltime\*.SRC
    
    SetOutPath "$INSTDIR\nut\app\cppdemo"
    File ..\..\..\app\cppdemo\Makefile
    File ..\..\..\app\cppdemo\*.cc

    SetOutPath "$INSTDIR\nut\app\editconf"
    File ..\..\..\app\editconf\Makefile
    File ..\..\..\app\editconf\*.c
    File ..\..\..\app\editconf\*.h

    SetOutPath "$INSTDIR\nut\app\events"
    File ..\..\..\app\events\Makefile
    File ..\..\..\app\events\*.c
    SetOutPath "$INSTDIR\nut\appicc\events"
    File ..\..\..\appicc\events\*.prj
    File ..\..\..\appicc\events\*.SRC
    
    SetOutPath "$INSTDIR\nut\app\ftpd"
    File ..\..\..\app\ftpd\Makefile
    File ..\..\..\app\ftpd\*.c
    SetOutPath "$INSTDIR\nut\appicc\ftpd"
    File ..\..\..\appicc\ftpd\*.prj
    File ..\..\..\appicc\ftpd\*.SRC

    SetOutPath "$INSTDIR\nut\app\httpd"
    File ..\..\..\app\httpd\Makefile
    File ..\..\..\app\httpd\*.c
    SetOutPath "$INSTDIR\nut\app\httpd\sample"
    File ..\..\..\app\httpd\sample\*.html
    File ..\..\..\app\httpd\sample\*.shtml
    File ..\..\..\app\httpd\sample\*.gif
    File ..\..\..\app\httpd\sample\*.asp
    File ..\..\..\app\httpd\sample\*.inc
    SetOutPath "$INSTDIR\nut\app\httpd\sample\flash"
    File ..\..\..\app\httpd\sample\flash\*.html
    File ..\..\..\app\httpd\sample\flash\*.swf
    SetOutPath "$INSTDIR\nut\appicc\httpd"
    File ..\..\..\appicc\httpd\*.prj
    File ..\..\..\appicc\httpd\*.SRC

    SetOutPath "$INSTDIR\nut\app\httpd_simple"
    File ..\..\..\app\httpd_simple\Makefile
    File ..\..\..\app\httpd_simple\*.c
    SetOutPath "$INSTDIR\nut\app\httpd_simple\urom"
    File ..\..\..\app\httpd_simple\urom\*.html
	
    SetOutPath "$INSTDIR\nut\app\httpd_upnp"
    File ..\..\..\app\httpd_upnp\Makefile
    File ..\..\..\app\httpd_upnp\*.c
    File ..\..\..\app\httpd_upnp\*.h
    SetOutPath "$INSTDIR\nut\app\httpd_upnp\sample"
    File ..\..\..\app\httpd_upnp\sample\*.html
    File ..\..\..\app\httpd_upnp\sample\*.gif
    SetOutPath "$INSTDIR\nut\app\httpd_upnp\sample\flash"
    File ..\..\..\app\httpd_upnp\sample\flash\*.html
    File ..\..\..\app\httpd_upnp\sample\flash\*.swf
    SetOutPath "$INSTDIR\nut\app\httpd_upnp\sample\upnp"
    File ..\..\..\app\httpd_upnp\sample\upnp\*.xml

    SetOutPath "$INSTDIR\nut\app\icmp-udp"
    File ..\..\..\app\icmp-udp\Makefile
    File ..\..\..\app\icmp-udp\*.c
    File ..\..\..\app\icmp-udp\*.class
    File ..\..\..\app\icmp-udp\*.java

    SetOutPath "$INSTDIR\nut\app\inetq"
    File ..\..\..\app\inetq\Makefile
    File ..\..\..\app\inetq\*.c
    SetOutPath "$INSTDIR\nut\appicc\inetq"
    File ..\..\..\appicc\inetq\*.prj
    File ..\..\..\appicc\inetq\*.SRC

    SetOutPath "$INSTDIR\nut\app\jtagtest"
    File ..\..\..\app\jtagtest\Makefile
    File ..\..\..\app\jtagtest\*.c

    SetOutPath "$INSTDIR\nut\app\logtime"
    File ..\..\..\app\logtime\Makefile
    File ..\..\..\app\logtime\*.c
    SetOutPath "$INSTDIR\nut\appicc\logtime"
    File ..\..\..\appicc\logtime\*.prj
    File ..\..\..\appicc\logtime\*.SRC

    SetOutPath "$INSTDIR\nut\app\lua"
    File ..\..\..\app\lua\Makefile
    File ..\..\..\app\lua\*.c

    SetOutPath "$INSTDIR\nut\app\owibus"
    File ..\..\..\app\owibus\Makefile
    File ..\..\..\app\owibus\*.c

    SetOutPath "$INSTDIR\nut\app\pingnet"
    File ..\..\..\app\pingnet\Makefile
    File ..\..\..\app\pingnet\*.c

    SetOutPath "$INSTDIR\nut\app\portdio"
    File ..\..\..\app\portdio\Makefile
    File ..\..\..\app\portdio\*.c
    SetOutPath "$INSTDIR\nut\appicc\portdio"
    File ..\..\..\appicc\portdio\*.prj
    File ..\..\..\appicc\portdio\*.SRC

    SetOutPath "$INSTDIR\nut\app\pppc"
    File ..\..\..\app\pppc\Makefile
    File ..\..\..\app\pppc\*.c
    SetOutPath "$INSTDIR\nut\appicc\pppc"
    File ..\..\..\appicc\pppc\*.prj
    File ..\..\..\appicc\pppc\*.SRC

    SetOutPath "$INSTDIR\nut\app\rs232d"
    File ..\..\..\app\rs232d\Makefile
    File ..\..\..\app\rs232d\*.c
    SetOutPath "$INSTDIR\nut\appicc\rs232d"
    File ..\..\..\appicc\rs232d\*.prj
    File ..\..\..\appicc\rs232d\*.SRC

    SetOutPath "$INSTDIR\nut\app\simple"
    File ..\..\..\app\simple\Makefile
    File ..\..\..\app\simple\*.c
    SetOutPath "$INSTDIR\nut\appicc\simple"
    File ..\..\..\appicc\simple\*.prj
    File ..\..\..\appicc\simple\*.SRC

    SetOutPath "$INSTDIR\nut\app\snmpd"
    File ..\..\..\app\snmpd\Makefile
    File ..\..\..\app\snmpd\*.c
    File ..\..\..\app\snmpd\*.h
    SetOutPath "$INSTDIR\nut\appicc\snmpd"
    File ..\..\..\appicc\snmpd\*.prj
    File ..\..\..\appicc\snmpd\*.SRC

    SetOutPath "$INSTDIR\nut\app\tcps"
    File ..\..\..\app\tcps\Makefile
    File ..\..\..\app\tcps\*.c
    SetOutPath "$INSTDIR\nut\appicc\tcps"
    File ..\..\..\appicc\tcps\*.prj
    File ..\..\..\appicc\tcps\*.SRC

    SetOutPath "$INSTDIR\nut\app\threads"
    File ..\..\..\app\threads\Makefile
    File ..\..\..\app\threads\*.c
    SetOutPath "$INSTDIR\nut\appicc\threads"
    File ..\..\..\appicc\threads\*.prj
    File ..\..\..\appicc\threads\*.SRC

    SetOutPath "$INSTDIR\nut\app\timers"
    File ..\..\..\app\timers\Makefile
    File ..\..\..\app\timers\*.c
    SetOutPath "$INSTDIR\nut\appicc\timers"
    File ..\..\..\appicc\timers\*.prj
    File ..\..\..\appicc\timers\*.SRC

    SetOutPath "$INSTDIR\nut\app\twitest"
    File ..\..\..\app\twitest\Makefile
    File ..\..\..\app\twitest\*.c

    SetOutPath "$INSTDIR\nut\app\uart"
    File ..\..\..\app\uart\Makefile
    File ..\..\..\app\uart\*.c

    SetOutPath "$INSTDIR\nut\appicc\uart"
    File ..\..\..\appicc\uart\*.prj
    File ..\..\..\appicc\uart\*.SRC

    SetOutPath "$INSTDIR\nut\app\uhttpd_ajax"
    File ..\..\..\app\uhttpd_ajax\Makefile
    File ..\..\..\app\uhttpd_ajax\*.c
    SetOutPath "$INSTDIR\nut\app\uhttpd_ajax\htdocs"
    File ..\..\..\app\uhttpd_ajax\htdocs\*.html
    File ..\..\..\app\uhttpd_ajax\htdocs\*.js

    SetOutPath "$INSTDIR\nut\app\uhttpd_auth"
    File ..\..\..\app\uhttpd_auth\Makefile
    File ..\..\..\app\uhttpd_auth\*.c
    SetOutPath "$INSTDIR\nut\app\uhttpd_auth\htdocs"
    File ..\..\..\app\uhttpd_auth\htdocs\*.html
    SetOutPath "$INSTDIR\nut\app\uhttpd_auth\htdocs\admin"
    File ..\..\..\app\uhttpd_auth\htdocs\admin\*.html
    SetOutPath "$INSTDIR\nut\app\uhttpd_auth\htdocs\user"
    File ..\..\..\app\uhttpd_auth\htdocs\user\*.html

    SetOutPath "$INSTDIR\nut\app\uhttpd_form"
    File ..\..\..\app\uhttpd_form\Makefile
    File ..\..\..\app\uhttpd_form\*.c
    SetOutPath "$INSTDIR\nut\app\uhttpd_form\htdocs"
    File ..\..\..\app\uhttpd_form\htdocs\*.html

    SetOutPath "$INSTDIR\nut\app\uhttpd_ssi"
    File ..\..\..\app\uhttpd_ssi\Makefile
    File ..\..\..\app\uhttpd_ssi\*.c
    SetOutPath "$INSTDIR\nut\app\uhttpd_ssi\htdocs"
    File ..\..\..\app\uhttpd_ssi\htdocs\*.shtml
    File ..\..\..\app\uhttpd_ssi\htdocs\*.inc
    File ..\..\..\app\uhttpd_ssi\htdocs\*.png

    SetOutPath "$INSTDIR\nut\app\uhttpd_tiny"
    File ..\..\..\app\uhttpd_tiny\Makefile
    File ..\..\..\app\uhttpd_tiny\*.c
    SetOutPath "$INSTDIR\nut\app\uhttpd_tiny\htdocs"
    File ..\..\..\app\uhttpd_tiny\htdocs\*.html
    File ..\..\..\app\uhttpd_tiny\htdocs\*.png

    SetOutPath "$INSTDIR\nut\app\uhttpd_upload"
    File ..\..\..\app\uhttpd_upload\Makefile
    File ..\..\..\app\uhttpd_upload\*.c
    SetOutPath "$INSTDIR\nut\app\uhttpd_upload\htdocs"
    File ..\..\..\app\uhttpd_upload\htdocs\*.html
    File ..\..\..\app\uhttpd_upload\htdocs\*.png

    SetOutPath "$INSTDIR\nut\app\xsvfexec"
    File ..\..\..\app\xsvfexec\*.c
    File ..\..\..\app\xsvfexec\*.h
	
    SetOutPath "$INSTDIR\nut\hwtest\arm\atmel\7segtst"
    File ..\..\..\hwtest\arm\atmel\7segtst\Makefile
    File ..\..\..\hwtest\arm\atmel\7segtst\*.c

    SetOutPath "$INSTDIR\nut\hwtest\arm\atmel\eeprom"
    File ..\..\..\hwtest\arm\atmel\eeprom\Makefile
    File ..\..\..\hwtest\arm\atmel\eeprom\*.c

    SetOutPath "$INSTDIR\nut\hwtest\arm\atmel\ioexpander"
    File ..\..\..\hwtest\arm\atmel\ioexpander\Makefile
    File ..\..\..\hwtest\arm\atmel\ioexpander\*.c

    SetOutPath "$INSTDIR\nut\hwtest\arm\atmel\led_key"
    File ..\..\..\hwtest\arm\atmel\led_key\Makefile
    File ..\..\..\hwtest\arm\atmel\led_key\*.c

    SetOutPath "$INSTDIR\nut\hwtest\audio\nutpiper"
    File ..\..\..\hwtest\audio\nutpiper\Makefile
    File ..\..\..\hwtest\audio\nutpiper\*.c
    File ..\..\..\hwtest\audio\nutpiper\*.h

    SetOutPath "$INSTDIR\nut\hwtest\audio\playmp3"
    File ..\..\..\hwtest\audio\playmp3\Makefile
    File ..\..\..\hwtest\audio\playmp3\*.c
    SetOutPath "$INSTDIR\nut\hwtest\audio\playmp3\sounds"
    File ..\..\..\hwtest\audio\playmp3\sounds\*.mp3

    SetOutPath "$INSTDIR\nut\hwtest\avr\basemon"
    File ..\..\..\hwtest\avr\basemon\Makefile
    File ..\..\..\hwtest\avr\basemon\*.c
    File ..\..\..\hwtest\avr\basemon\*.h
    SetOutPath "$INSTDIR\nut\hwtest\avr\basemon\html"
    File ..\..\..\hwtest\avr\basemon\html\*.html
    File ..\..\..\hwtest\avr\basemon\html\*.gif

    SetOutPath "$INSTDIR\nut\hwtest\avr\canbus"
    File ..\..\..\hwtest\avr\canbus\Makefile
    File ..\..\..\hwtest\avr\canbus\*.c
    SetOutPath "$INSTDIR\nut\appicc\canbus"
    File ..\..\..\appicc\canbus\*.prj
    File ..\..\..\appicc\canbus\*.SRC
    
    SetOutPath "$INSTDIR\nut\hwtest\avr\isp2"
    File ..\..\..\hwtest\avr\isp2\Makefile
    File ..\..\..\hwtest\avr\isp2\*.c
    File ..\..\..\hwtest\avr\isp2\*.S
    File ..\..\..\hwtest\avr\isp2\*.isp

    SetOutPath "$INSTDIR\nut\hwtest\cm3\stm\trampoline"
    File ..\..\..\hwtest\cm3\stm\trampoline\Makefile
    File ..\..\..\hwtest\cm3\stm\trampoline\*.c

    SetOutPath "$INSTDIR\nut\hwtest\cm3\stm\usb_test"
    File ..\..\..\hwtest\cm3\stm\usb_test\Makefile
    File ..\..\..\hwtest\cm3\stm\usb_test\*.c
  SectionEnd

  Section "Prebuild Samples" SecSampleBins
    SectionIn 1
    SetOutPath "$INSTDIR\nut\bin\boards"
    File /r ..\..\..\bin\boards\*.hex
    File /r ..\..\..\bin\boards\*.bin
    File /r ..\..\..\bin\boards\README
  SectionEnd
SectionGroupEnd

SectionGroup "Ethernut Bootloaders"
  Section "Ethernut 1.3 (eboot)" SecBoot13
    SectionIn 1 2 3
    SetOutPath "$INSTDIR\nut\boot\eboot"
    File ..\..\..\boot\eboot\Make*
    File ..\..\..\boot\eboot\*.c
    File ..\..\..\boot\eboot\*.h
    File ..\..\..\boot\eboot\*.s
  SectionEnd
  Section "Ethernut 2 (appload)" SecBoot21
    SectionIn 1 2 3
    SetOutPath "$INSTDIR\nut\boot\appload"
    File ..\..\..\boot\appload\Make*
    File ..\..\..\boot\appload\*.c
    File ..\..\..\boot\appload\*.h
    File ..\..\..\boot\appload\*.S
  SectionEnd
  Section "Ethernut 3 (bootmon)" SecBoot30
    SectionIn 1 2 3
    SetOutPath "$INSTDIR\nut\boot\bootmon"
    File ..\..\..\boot\bootmon\Make*
    File ..\..\..\boot\bootmon\*.c
    File ..\..\..\boot\bootmon\*.h
    File ..\..\..\boot\bootmon\*.S
    File ..\..\..\boot\bootmon\*.ld
    File ..\..\..\boot\bootmon\*.jom
  SectionEnd
  Section "AT91SAM7X (at91sam7x_bootloader)" SecBootSam7x
    SectionIn 1 2 3
    SetOutPath "$INSTDIR\nut\boot\at91sam7x_bootloader"
    File ..\..\..\boot\at91sam7x_bootloader\Make*
    File ..\..\..\boot\at91sam7x_bootloader\*.c
    File ..\..\..\boot\at91sam7x_bootloader\*.h
    File ..\..\..\boot\at91sam7x_bootloader\Readme.txt
    SetOutPath "$INSTDIR\nut\boot\at91sam7x_bootloader\demo"
    File ..\..\..\boot\at91sam7x_bootloader\demo\Make*
    File ..\..\..\boot\at91sam7x_bootloader\demo\*.c
  SectionEnd
  Section "AT91SAM7S (xloader7)" SecBootSam7s
    SectionIn 1 2 3
    SetOutPath "$INSTDIR\nut\boot\xloader7"
    File ..\..\..\boot\xloader7\Make*
    File ..\..\..\boot\xloader7\*.c
    File ..\..\..\boot\xloader7\*.h
    File ..\..\..\boot\xloader7\*.s
    File ..\..\..\boot\xloader7\*.ld
    File ..\..\..\boot\xloader7\README_FIRST.txt
  SectionEnd
SectionGroupEnd

Section -post
  SetOutPath $INSTDIR

  WriteRegStr HKLM SOFTWARE\nut "Install_Dir" $INSTDIR
  WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\ethernut-${NUTVERSION}" \
                   "DisplayName" "Ethernut ${NUTVERSION}.${NUTRELEASE} (remove only)"
  WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\ethernut-${NUTVERSION}" \
                   "UninstallString" '"$INSTDIR\Uninstall.exe"'
SectionEnd

/*
 * =================================================================================
 * Installer Callbacks.
 * =================================================================================
 */
Function .onInit
    InitPluginsDir
    !insertmacro MUI_LANGDLL_DISPLAY
FunctionEnd
    
; Descriptions
;
;Language strings

LangString DESC_SecBoot13 ${LANG_ENGLISH} "Will work with most AVR based Ethernut clones. Requires RTL8019AS."
LangString DESC_SecBoot13 ${LANG_FRENCH} "Will work with most AVR based Ethernut clones. Requires RTL8019AS."
LangString DESC_SecBoot13 ${LANG_GERMAN} "Funktioniert mit den meisten AVR basierten Ethernut Nachbauten. Benötigt RTL8019AS."

LangString DESC_SecBoot21 ${LANG_ENGLISH} "Ethernet/TFTP bootloader for AVR. Requires LAN91C111."
LangString DESC_SecBoot21 ${LANG_FRENCH} "Ethernet/TFTP bootloader for AVR. Requires LAN91C111."
LangString DESC_SecBoot21 ${LANG_GERMAN} "Ethernet/TFTP Bootloader für AVR. Benötigt LAN91C111."

LangString DESC_SecBoot30 ${LANG_ENGLISH} "Ethernet/TFTP bootloader for ARM7. Requires DM9000E."
LangString DESC_SecBoot30 ${LANG_FRENCH} "Ethernet/TFTP bootloader for ARM7. Requires DM9000E."
LangString DESC_SecBoot30 ${LANG_GERMAN} "Ethernet/TFTP Bootloader für ARM7. Benötigt DM9000E."

LangString DESC_SecCompat ${LANG_ENGLISH} "Provide Linux compatibility."
LangString DESC_SecCompat ${LANG_FRENCH} "Provide Linux compatibility."
LangString DESC_SecCompat ${LANG_GERMAN} "Bietet Kompatibilität mit Linux."

LangString DESC_SecConfig ${LANG_ENGLISH} "Graphic User Interface to configure and rebuild Nut/OS."
LangString DESC_SecConfig ${LANG_FRENCH} "Graphic User Interface to configure and rebuild Nut/OS."
LangString DESC_SecConfig ${LANG_GERMAN} "Graphische Oberfläche zum Konfigurieren von Nut/OS."

LangString DESC_SecDiscover ${LANG_ENGLISH} "Graphic User Interface to scan the local net for Ethernuts."
LangString DESC_SecDiscover ${LANG_FRENCH} "Graphic User Interface to scan the local net for Ethernuts."
LangString DESC_SecDiscover ${LANG_GERMAN} "Graphische Oberfläche, die installierte Ethernuts im lokalen Netz erkennt."

LangString DESC_SecContribs ${LANG_ENGLISH} "Contributions with strict copyright."
LangString DESC_SecContribs ${LANG_FRENCH} "Contributions with strict copyright."
LangString DESC_SecContribs ${LANG_GERMAN} "Beiträge mit eingeschränktem copyright."

LangString DESC_SecDocApi ${LANG_ENGLISH} "HTML files, created by Doxygen."
LangString DESC_SecDocApi ${LANG_FRENCH} "HTML files, created by Doxygen."
LangString DESC_SecDocApi ${LANG_GERMAN} "HTML Dateien, erstellt mit Doxygen."

LangString DESC_SecNetSources ${LANG_ENGLISH} "Complete source code, but no header files."
LangString DESC_SecNetSources ${LANG_FRENCH} "Complete source code, but no header files."
LangString DESC_SecNetSources ${LANG_GERMAN} "Vollständiger Quellcode, jedoch ohne Header Dateien."

LangString DESC_SecOpenOcd ${LANG_ENGLISH} "OpenOCD configuration files."
LangString DESC_SecOpenOcd ${LANG_FRENCH} "OpenOCD configuration files."
LangString DESC_SecOpenOcd ${LANG_GERMAN} "OpenOCD Konfiguration."

LangString DESC_SecSamBa ${LANG_ENGLISH} "SAM-BA Support."
LangString DESC_SecSamBa ${LANG_FRENCH} "SAM-BA Support."
LangString DESC_SecSamBa ${LANG_GERMAN} "SAM-BA Unterstützung."

LangString DESC_SecOsHeaders ${LANG_ENGLISH} "C and C++ header files."
LangString DESC_SecOsHeaders ${LANG_FRENCH} "C and C++ header files."
LangString DESC_SecOsHeaders ${LANG_GERMAN} "Header Dateien für C und C++."

LangString DESC_SecOsLibs ${LANG_ENGLISH} "Prebuild libraries."
LangString DESC_SecOsLibs ${LANG_FRENCH} "Prebuild libraries."
LangString DESC_SecOsLibs ${LANG_GERMAN} "Fertige Bibliotheken."

LangString DESC_SecOsSources ${LANG_ENGLISH} "Complete source code, but no header files."
LangString DESC_SecOsSources ${LANG_FRENCH} "Complete source code, but no header files."
LangString DESC_SecOsSources ${LANG_GERMAN} "Vollständiger Quellcode, aber ohne Header Dateien."

LangString DESC_SecRobi ${LANG_ENGLISH} "Creates raw binaries from Intel hex files. Nice with bootloaders."
LangString DESC_SecRobi ${LANG_FRENCH} "Creates raw binaries from Intel hex files. Nice with bootloaders."
LangString DESC_SecRobi ${LANG_GERMAN} "Erzeugt Binärdateien von Intel Hex Dateien. Gut für Bootloader."

LangString DESC_SecSampleSources ${LANG_ENGLISH} "Examples to learn from."
LangString DESC_SecSampleSources ${LANG_FRENCH} "Examples to learn from."
LangString DESC_SecSampleSources ${LANG_GERMAN} "Beispiele zum Lernen."

LangString DESC_SecSampleBins ${LANG_ENGLISH} "Prebuild samples."
LangString DESC_SecSampleBins ${LANG_FRENCH} "Prebuild samples."
LangString DESC_SecSampleBins ${LANG_GERMAN} "Fertig übersetzte Beispiele."

LangString DESC_SecStk500 ${LANG_ENGLISH} "Atmel's command line programmer. Use with ImageCraft."
LangString DESC_SecStk500 ${LANG_FRENCH} "Atmel's command line programmer. Use with ImageCraft."
LangString DESC_SecStk500 ${LANG_GERMAN} "Atmel's Programmierer für die Kommandozeile. Für ImageCraft."

LangString DESC_SecToolSources ${LANG_ENGLISH} "In case you want to modify them."
LangString DESC_SecToolSources ${LANG_FRENCH} "In case you want to modify them."
LangString DESC_SecToolSources ${LANG_GERMAN} "Falls Sie etwas ändern wollen."

LangString DESC_SecAvrDude ${LANG_ENGLISH} "Command line AVR programmer."
LangString DESC_SecAvrDude ${LANG_FRENCH} "Command line AVR programmer."
LangString DESC_SecAvrDude ${LANG_GERMAN} "AVR Programmierer für die Kommandozeile."

LangString DESC_SecJtagomat ${LANG_ENGLISH} "Command line ARM programmer."
LangString DESC_SecJtagomat ${LANG_FRENCH} "Command line ARM programmer."
LangString DESC_SecJtagomat ${LANG_GERMAN} "ARM JTAG Programmierer für die Kommandozeile."

LangString DESC_SecUrom ${LANG_ENGLISH} "Creates Nut/OS read-only filesystem."
LangString DESC_SecUrom ${LANG_FRENCH} "Creates Nut/OS read-only filesystem."
LangString DESC_SecUrom ${LANG_GERMAN} "Erzeugt Read-Only Dateisysteme."


;Assign language strings to sections
!insertmacro MUI_FUNCTION_DESCRIPTION_BEGIN

!insertmacro MUI_DESCRIPTION_TEXT ${SecBoot13} $(DESC_SecBoot13)
!insertmacro MUI_DESCRIPTION_TEXT ${SecBoot21} $(DESC_SecBoot21)
!insertmacro MUI_DESCRIPTION_TEXT ${SecCompat} $(DESC_SecCompat)
!insertmacro MUI_DESCRIPTION_TEXT ${SecConfig} $(DESC_SecConfig)
!insertmacro MUI_DESCRIPTION_TEXT ${SecDiscover} $(DESC_SecDiscover)
!insertmacro MUI_DESCRIPTION_TEXT ${SecDocApi} $(DESC_SecDocApi)
!insertmacro MUI_DESCRIPTION_TEXT ${SecContribs} $(DESC_SecContribs)
!insertmacro MUI_DESCRIPTION_TEXT ${SecNetSources} $(DESC_SecNetSources)
!insertmacro MUI_DESCRIPTION_TEXT ${SecOpenOcd} $(DESC_SecOpenOcd)
!insertmacro MUI_DESCRIPTION_TEXT ${SecOsHeaders} $(DESC_SecOsHeaders)
!insertmacro MUI_DESCRIPTION_TEXT ${SecOsLibs} $(DESC_SecOsLibs)
!insertmacro MUI_DESCRIPTION_TEXT ${SecOsSources} $(DESC_SecOsSources)
!insertmacro MUI_DESCRIPTION_TEXT ${SecRobi} $(DESC_SecRobi)
!insertmacro MUI_DESCRIPTION_TEXT ${SecSampleSources} $(DESC_SecSampleSources)
!insertmacro MUI_DESCRIPTION_TEXT ${SecSampleBins} $(DESC_SecSampleBins)
!insertmacro MUI_DESCRIPTION_TEXT ${SecStk500} $(DESC_SecStk500)
!insertmacro MUI_DESCRIPTION_TEXT ${SecToolSources} $(DESC_SecToolSources)
!insertmacro MUI_DESCRIPTION_TEXT ${SecAvrDude} $(DESC_SecAvrDude)
!insertmacro MUI_DESCRIPTION_TEXT ${SecJtagomat} $(DESC_SecJtagomat)
!insertmacro MUI_DESCRIPTION_TEXT ${SecUrom} $(DESC_SecUrom)

!insertmacro MUI_FUNCTION_DESCRIPTION_END

;Uninstaller Section
;
Section "Uninstall"

  SetDetailsPrint textonly
  DetailPrint "Deleting Files..."
  SetDetailsPrint listonly

  Delete "$INSTDIR\nutconf.exe"
  Delete "$INSTDIR\nutconfigure.exe"
  Delete "$INSTDIR\qnutconf.exe"
  Delete "$INSTDIR\nutdisc.exe"
  Delete "$INSTDIR\msvcr71.dll"
  Delete "$INSTDIR\msvcp71.dll"
  Delete "$INSTDIR\Microsoft.VC90.CRT.manifest"
  Delete "$INSTDIR\msvcr90.dll"
  Delete "$INSTDIR\msvcp90.dll"
  Delete "$INSTDIR\QtCore4.dll"
  Delete "$INSTDIR\QtGui4.dll"
;  Delete "$INSTDIR\nutoshelp.chm"
  Delete "$INSTDIR\ChangeLog"
  Delete "$INSTDIR\ChangeLog20090309"
  Delete "$INSTDIR\AUTHORS"
  Delete "$INSTDIR\COPYING"
  Delete "$INSTDIR\INSTALL"
  Delete "$INSTDIR\NEWS"
  Delete "$INSTDIR\README"
  RMDir /r "$INSTDIR\nut"

  Delete "$INSTDIR\Uninstall.exe"

  IfFileExists $INSTDIR 0 Removed
  MessageBox MB_YESNO|MB_ICONQUESTION "Completely remove the installation directory, including build trees and applications?" IDNO Removed
  RMDir /r "$INSTDIR"

  Removed:
  Delete "$SMPROGRAMS\Ethernut ${NUTVERSION}\*.*"
  RMDir "$SMPROGRAMS\Ethernut ${NUTVERSION}"
  DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\ethernut-${NUTVERSION}"
  DeleteRegKey HKLM "${SWREGKEY}\${NUTVERSION}"
  DeleteRegKey HKCU "Software\egnite\NutConf"
  DeleteRegKey HKCU "Software\egnite\nutdisc"

SectionEnd

/*
 * =================================================================================
 * Uninstaller Callbacks
 * =================================================================================
 */
Function un.onInit
  !insertmacro MUI_UNGETLANGUAGE
  
  InitPluginsDir
FunctionEnd

