cmake_minimum_required(VERSION 3.1.3)
add_definitions(-DNUT_CONFIGURE_EXEC)
project(NUTCONF)
if ((CMAKE_COMPILER_IS_GNUCC) AND NOT (${MINGW}) )
  set( CMAKE_C_FLAGS   "-fstack-protector -fstack-protector-all" )
endif ()

find_package(Lua)
if(NOT Lua_FOUND)
    message("System Lua not found. Using bundled lua")
    add_subdirectory(../qnutconf/src/lua lua)
	set(LUA_LIBRARIES lua -lm)
	set(LUA_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../qnutconf/src/lua/src)
	unset(LUA_NOTFOUND)
endif()

MESSAGE(STATUS "LUA_INCLUDE_DIR " ${LUA_INCLUDE_DIR} )

if (MSVC)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include/win32)
endif(MSVC)
include_directories(${LUA_INCLUDE_DIR})

# Put cpp files here
set(NUTCONF_SRCS
    ../common/nutcomponent.c
)

add_executable(nutconfigure ${NUTCONF_SRCS})
target_link_libraries(nutconfigure ${LUA_LIBRARIES})
