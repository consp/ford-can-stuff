
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DN_SLCAN=17")
include_directories (./include)

add_executable(slcand slcand.c)
target_link_libraries (slcand)
