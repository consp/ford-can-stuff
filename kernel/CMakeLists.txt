# Module info
add_definitions(-D__KERNEL__ -DMODULE)

# Find the kernel release
execute_process(
        COMMAND uname -r
        OUTPUT_VARIABLE KERNEL_RELEASE
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

find_path(
        KERNELHEADERS_DIR
        Makefile
        PATHS /usr/src/linux-headers-${KERNEL_RELEASE}
)

message(STATUS "Kernel release: ${KERNEL_RELEASE}")
message(STATUS "Kernel headers: ${KERNELHEADERS_DIR}")

string(REPLACE "amd64" "common" KERNELHEADERS_COMMON_DIR ${KERNELHEADERS_DIR})
include_directories(
        ${KERNELHEADERS_DIR}/include
)

set(DRIVER_FILE slcan_multimode.ko)
set(KBUILD_CMD $(MAKE) -C ${KERNELHEADERS_DIR} M=${CMAKE_CURRENT_BINARY_DIR} src=${CMAKE_CURRENT_SOURCE_DIR} modules )

FILE(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/Kbuild "obj-m := slcan_multimode.o")
message(STATUS "Source dir ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "Binary dir ${CMAKE_CURRENT_BINARY_DIR}")

add_custom_command(OUTPUT ${DRIVER_FILE}
        COMMAND ${KBUILD_CMD}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        DEPENDS slcan_multimode.c VERBATIM)

    
add_custom_target(slcan_multimode ALL DEPENDS ${DRIVER_FILE})
#add_library(dummy_target slcan_multimode.c)


