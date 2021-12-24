
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DN_SLCAN=17")
include_directories (./include)

add_library (slcand_baudset slcand_baudset.c)

add_executable(slcand slcand.c)
target_link_libraries (slcand slcand_baudset)
