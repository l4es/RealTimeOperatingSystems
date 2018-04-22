cmake_minimum_required(VERSION 3.1.3)

project(crurom)
if (CMAKE_COMPILER_IS_GNUCC)
  set( CMAKE_C_FLAGS   "-fstack-protector -fstack-protector-all" )
endif ()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Put cpp files here
set(CRUROM_SRCS
     crurom.c
     dirent.c
	 getopt.c
)

if (WIN32)
	include_directories(
		${CMAKE_CURRENT_SOURCE_DIR}/../include/win32
	)
endif(WIN32)


add_executable(crurom ${CRUROM_SRCS})
