# defines:
#
# NM
# OBJCOPY
# LD
# CXX_COMPILER
# C_COMPILER
# CMAKE_SYSTEM_NAME
# CMAKE_SYSTEM_VERSION
# GENROMFS
# LINKER_FLAGS
# CMAKE_EXE_LINKER_FLAGS
# CMAKE_FIND_ROOT_PATH
# CMAKE_FIND_ROOT_PATH_MODE_PROGRAM
# CMAKE_FIND_ROOT_PATH_MODE_LIBRARY
# CMAKE_FIND_ROOT_PATH_MODE_INCLUDE

include(CMakeForceCompiler)

if ("$ENV{TI_SDK_DIR}" STREQUAL "")
        message(FATAL_ERROR "TI_SDK_DIR not set")
else()
        set(TI_SDK_DIR $ENV{TI_SDK_DIR})
endif()


# this one is important
set(CMAKE_SYSTEM_NAME Generic)

#this one not so much
set(CMAKE_SYSTEM_VERSION 1)

message(${TI_SDK_DIR}/linux-devkit/sysroots/x86_64-arago-linux/usr/bin)

# specify the cross compiler
find_program(C_COMPILER arm-linux-gnueabihf-gcc
	PATHS ${TI_SDK_DIR}/linux-devkit/sysroots/x86_64-arago-linux/usr/bin /usr/bin
	NO_DEFAULT_PATH
	)

if ("${TI_SYSROOT_DIR}" STREQUAL "")
	set(TI_SYSROOT_DIR ${TI_SDK_DIR}/filesystem/rootfs)
endif()


if(NOT C_COMPILER)
	message(FATAL_ERROR "could not find arm-linux-gnueabihf-gcc compiler")
endif()
cmake_force_c_compiler(${C_COMPILER} GNU)

find_program(CXX_COMPILER arm-linux-gnueabihf-g++
	PATHS ${TI_SDK_DIR}/linux-devkit/sysroots/x86_64-arago-linux/usr/bin  /usr/bin
	NO_DEFAULT_PATH
	)

if(NOT CXX_COMPILER)
	message(FATAL_ERROR "could not find arm-linux-gnueabihf-g++ compiler")
endif()
cmake_force_cxx_compiler(${CXX_COMPILER} GNU)

# compiler tools
foreach(tool objcopy nm ld)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} arm-linux-gnueabihf-${tool}
		PATHS ${TI_SDK_DIR}/linux-devkit/sysroots/x86_64-arago-linux/usr/bin  /usr/bin
		NO_DEFAULT_PATH
		)
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find arm-linux-gnueabihf-${tool}")
	endif()
endforeach()

# os tools
foreach(tool echo patch grep rm mkdir nm genromfs cp touch make unzip)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} ${tool})
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find ${TOOL}")
	endif()
endforeach()

add_definitions(
	-D __TI
	)

set(C_FLAGS "--sysroot=${TI_SYSROOT_DIR}")
set(LINKER_FLAGS "-Wl,-gc-sections")
set(CMAKE_EXE_LINKER_FLAGS ${LINKER_FLAGS})
set(CMAKE_C_FLAGS ${C_FLAGS})
set(CMAKE_CXX_LINKER_FLAGS ${C_FLAGS})

# where is the target environment
set(CMAKE_FIND_ROOT_PATH  get_file_component(${C_COMPILER} PATH))

# search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
