cmake_minimum_required(VERSION 3.1.3)

# Check if environment variable QTDIR is set.
if (DEFINED ENV{QTDIR})
    set(CMAKE_PREFIX_PATH $ENV{QTDIR} ${CMAKE_PREFIX_PATH})
endif ()

project(qnutconf)
if ((CMAKE_COMPILER_IS_GNUCC) AND NOT (${MINGW}) )
  set( CMAKE_C_FLAGS   "-fstack-protector -fstack-protector-all" )
endif ()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5Widgets REQUIRED)
#find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(Lua)

if(NOT Lua_FOUND)
    message("System Lua not found. Using bunddled Lua")
    add_subdirectory(src/lua)
	set(LUA_LIBRARIES lua)
	set(LUA_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/lua/src)
	unset(LUA_NOTFOUND)
endif()

# Put cpp files here
set(QNUTCONF_SRCS
     src/builder.cpp
     src/dirtraverser.cpp
     src/finddialog.cpp
     src/main.cpp
     src/mainwindow.cpp
     src/../../common/nutcomponent.c
     src/nutcomponentdelegate.cpp
     src/nutcomponentmodel.cpp
     src/nutcomponentmodelfilterproxy.cpp
     src/nutcomponentdetailsmodel.cpp
     src/nutcomponentmodel_p.cpp
     src/settings.cpp
     src/settingsdialog.cpp
     src/systeminfo.cpp
)

set(QNUTCONF_HDRS
     src/builder.h
     src/dirtraverser.h
     src/finddialog.h
     src/mainwindow.h
     src/../../common/nutcomponent.h
     src/nutcomponentdelegate.h
     src/nutcomponentmodel.h
     src/nutcomponentmodelfilterproxy.h
     src/nutcomponentdetailsmodel.h
     src/nutcomponentmodel_p.h
     src/settings.h
     src/settingsdialog.h
     src/systeminfo.h
)

set(QNUTCONF_FORMS
     src/mainwindow.ui
     src/settingsdialog.ui
     src/finddialog.ui
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/win32
	${LUA_INCLUDE_DIR}
)

source_group("Form Files" FILES ${QNUTCONF_FORMS})
source_group("Header Files" FILES ${QNUTCONF_HDRS})
source_group("Generated Files" REGULAR_EXPRESSION .*moc_|.*ui_)
source_group("Resource Files" REGULAR_EXPRESSION .*\\.qrc)

qt5_add_resources(QRC_GENERATED src/qnutconf.qrc)
add_executable(qnutconf WIN32 ${QNUTCONF_SRCS} ${QNUTCONF_HDRS} ${UIC_GENERATED} ${QRC_GENERATED} ${PROTO_SRCS})
qt5_use_modules(qnutconf Widgets)
target_link_libraries(qnutconf ${LUA_LIBRARIES} Qt5::Widgets)

target_compile_features( qnutconf PRIVATE cxx_auto_type cxx_final cxx_lambdas cxx_nullptr cxx_range_for)
