cmake_minimum_required(VERSION 3.10)

project(Fiesta-tools VERSION 0.1)

add_subdirectory(atmega1284p)
add_subdirectory(kernel)
add_subdirectory(userspace)

add_custom_command(OUTPUT kernel_load_mod
    COMMAND sudo insmod kernel/slcan_multimode.ko
    COMMAND sudo ./userspace/slcand -F -o -t hw -S 2000000 /dev/ttyUSB0
)

add_custom_command(OUTPUT can_if_up
    COMMAND sudo ip link set slcan0d0 up
    COMMAND sudo ip link set slcan0d1 up
    COMMAND sudo ip link set slcan0d2 up
    )

add_custom_target(kernel_load kernel_load_mod)
add_custom_target(ifup can_if_up)
