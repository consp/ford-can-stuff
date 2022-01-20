
set (UPLOAD avrdude)
set (PROGRAMMER usbasp)

set (MCU atmega1284p)
set (E_FUSE 0xFF)
set (H_FUSE 0xD9)
set (L_FUSE 0xF7)
set (MCU_SPEED 24000000)


if (MCU MATCHES atmega1284p)
	set (MCU_DEFINE __AVR_ATmega1284P__)
endif (MCU MATCHES atmega1284p)

set (MCU_C_FLAGS "-D${MCU_DEFINE} -mmcu=${MCU}")
set (MCU_LINK_FLAGS "-mmcu=${MCU} -Wl,--gc-sections -mrelax -Wl,-Map,fiesta_can.map")

add_link_options (-mmcu=${MCU})
add_link_options(-Wl,--gc-sections -mrelax -Wl,-Map,fiesta_can.map)
add_link_options(-flto)
add_compile_options (-mmcu=${MCU})
add_compile_options (-fpack-struct)
add_compile_options (-fshort-enums)
add_compile_options (-Wall)
# add_compile_options (-Werror)
add_compile_options (-funsigned-char)
add_compile_options (-funsigned-bitfields)
add_compile_options (-ffunction-sections)
add_compile_options (-c)
add_compile_options (-O3)
add_compile_options (-g)
add_compile_options (--save-temps)
add_compile_options (-ffunction-sections)
add_compile_options (-fdata-sections)
add_compile_options (-MMD)
add_compile_options (-DUSB_VID=null -DUSB_PID=null -DARDUINO=106)
# add_compile_options (-S)
#add_compile_options (-flto) # disable if attempting to use usart global regs
add_compile_options (-ffixed-r2 -ffixed-r3 -ffixed-r4 -ffixed-r5 -ffixed-r6)
add_definitions (-DF_CPU=${MCU_SPEED}L)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories (include/ cores/ libraries/SPI/src/ libraries/Wire/src/ libraries/EEPROM/src/ board/)

set (AVRDUDE_CONFIG ${CMAKE_CURRENT_SOURCE_DIR}/board/avrdude.conf)

if (EXISTS ARDUINO_CORE)
set (SOURCE
		src/fiesta_can.cpp
		src/canmsg.cpp
        src/crc8.cpp
	)
else()
set (SOURCE
		src/fiesta_can.c
        src/crc8.c
    )
endif()

set (MCP2515_CAN_SOURCE
		src/mcp2515/mcp_can.c
	)

    #add_library(mcp2515can STATIC ${MCP2515_CAN_SOURCE})
add_executable (fiesta_can.elf ${SOURCE} ${CRC8_SOURCE} ${MCP2515_CAN_SOURCE})



find_program (AVR_CC avr-gcc REQUIRED)
find_program (AVR_CXX avr-g++ REQUIRED)
find_program (AVR_OBJCOPY avr-objcopy REQUIRED)
find_program (AVR_SIZE_TOOL avr-size REQUIRED)
find_program (AVR_OBJDUMP avr-objdump REQUIRED)
find_program (AVRDUDE avrdude REQUIRED)

set (CMAKE_SYSTEM_NAME Generic)
set (CMAKE_SYSTEM_PROCESSOR avr)
set (CMAKE_C_COMPILER ${AVR_CC})

if (EXISTS ARDUINO_CORE)
    set (ARDUINO_CORE_SOURCE
            WInterrupts.c
            wiring.c
            wiring_analog.c
            wiring_digital.c
            wiring_pulse.c
            wiring_shift.c
            Stream.cpp
            hooks.c
            new.cpp
            Print.cpp
            main.cpp
        )
    set (ARDUINO_CORE_SOURCE_DIR "cores/")
    list (TRANSFORM ARDUINO_CORE_SOURCE PREPEND ${ARDUINO_CORE_SOURCE_DIR})
    set (MIGHTY_CORE_SOURCE
        SPI/src/SPI.cpp
        )
    set (MIGHTY_CORE_SOURCE_DIR "libraries/")
    list (TRANSFORM MIGHTY_CORE_SOURCE PREPEND ${MIGHTY_CORE_SOURCE_DIR})
    add_definitions (-DARDUINO_CORE)
    add_library(arduino STATIC ${ARDUINO_CORE_SOURCE})
    add_library(mightycore STATIC ${MIGHTY_CORE_SOURCE})
    target_link_libraries (fiesta_can.elf arduino mightycore)
    target_link_libraries (mcp2515can arduino mightycore)
    set (CMAKE_CXX_COMPILER ${AVR_CXX})
else()
    set (CMAKE_CXX_COMPILER ${AVR_CC})
endif()
# HardwareSerial.cpp
#		HardwareSerial0.cpp
#		HardwareSerial1.cpp

add_custom_command (
		OUTPUT rom.eep
			COMMAND
				${AVR_OBJCOPY} -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 fiesta_can.elf rom.eep
			DEPENDS fiesta_can.elf
	)

add_custom_command (OUTPUT rom.hex
		COMMAND
			${AVR_OBJCOPY} -j .text -j .data -O ihex fiesta_can.elf rom.hex
		COMMAND
			${AVR_SIZE_TOOL} ${AVR_SIZE_ARGS} fiesta_can.elf
		DEPENDS fiesta_can.elf
	)

add_custom_target(
	upload
		COMMAND
		${AVRDUDE} -c usbasp -p ${MCU} -B 5 -U flash:w:rom.hex
	    DEPENDS rom.hex
		COMMENT "Uploading rom to ${MCU}"
   )

add_custom_target(
	fuses
		COMMAND
        ${AVRDUDE} -c usbasp -p ${MCU} -B 5 -U lfuse:w:${L_FUSE}:m -U hfuse:w:${H_FUSE}:m -U efuse:w:${E_FUSE}:m
	    DEPENDS rom.hex
		COMMENT "Uploading rom to ${MCU}"
   )

add_custom_target(
    reset
        COMMAND
        ${AVRDUDE} -p ${MCU} -c usbasp -B 5  -U lfuse:v:${L_FUSE}:m -U hfuse:v:${H_FUSE}:m -U efuse:v:${E_FUSE}:m
        COMMENT "Resetting ${MCU}"
    )
