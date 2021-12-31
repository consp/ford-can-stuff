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
set(KBUILD_CMD $(MAKE) -C ${KERNELHEADERS_DIR} M=${CMAKE_CURRENT_BINARY_DIR} src=${CMAKE_CURRENT_SOURCE_DIR} modules KBUILD_VERBOSE=1)
set(KBUILD_CLEAN $(MAKE) -C ${KERNELHEADERS_DIR} M=${CMAKE_CURRENT_BINARY_DIR} src=${CMAKE_CURRENT_SOURCE_DIR} clean )

FILE(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/Kbuild "obj-m := slcan_multimode.o")
message(STATUS "Source dir ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "Binary dir ${CMAKE_CURRENT_BINARY_DIR}")

add_custom_command(OUTPUT ${DRIVER_FILE}
        COMMAND ${KBUILD_CMD}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        DEPENDS slcan_multimode.c VERBATIM)

add_custom_target(mod_symvers
    COMMAND cp /usr/src/linux-headers-`uname -r`/Module.symvers ${CMAKE_CURRENT_BINARY_DIR}/
    )
    
add_custom_target(slcan_multimode ALL DEPENDS ${DRIVER_FILE} mod_symvers)
#add_library(dummy_target slcan_multimode.c)


add_custom_target(install_kmod
    COMMAND cp ${CMAKE_CURRENT_BINARY_DIR}/slcan_multimode.ko /lib/modules/`uname -r`/kernel/drivers/net/can/
    COMMAND depmod
    COMMENT "Installing kmod"
    )

add_custom_target(clean_kmod
    COMMAND ${KBUILD_CLEAN}
    COMMENT "Cleaning kmod"
    )
