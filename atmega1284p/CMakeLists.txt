
set (UPLOAD avrdude)
set (PROGRAMMER usbasp)

set (MCU atmega1284p)
set (E_FUSE 0xFD)
set (H_FUSE 0xD7)
set (L_FUSE 0xF7)
set (MCU_SPEED 20000000)


if (MCU MATCHES atmega1284p)
	set (MCU_DEFINE __AVR_ATmega1284P__)
endif (MCU MATCHES atmega1284p)

set (MCU_C_FLAGS "-D${MCU_DEFINE} -mmcu=${MCU}")
set (MCU_LINK_FLAGS "-mmcu=${MCU} -Wl,--gc-sections -mrelax -Wl,-Map,fiesta_can.map")

add_definitions ("-DF_CPU=${MCU_SPEED}L")
add_definitions ("-D${MCU_DEFINE}")
add_definitions ("-fpack-struct")
add_definitions ("-fshort-enums")
add_definitions ("-Wall")
add_definitions ("-Werror")
add_definitions ("-funsigned-char")
add_definitions ("-funsigned-bitfields")
add_definitions ("-ffunction-sections")
add_definitions ("-c")
add_definitions ("-O3")
add_definitions ("-ffunction-sections -fdata-sections -MMD")
add_definitions ("-DUSB_VID=null -DUSB_PID=null -DARDUINO=106")

include_directories (include/ cores/ libraries/SPI/src/ libraries/Wire/src/ libraries/EEPROM/src/ board/)

find_program (AVR_CC avr-gcc REQUIRED)
find_program (AVR_CXX avr-g++ REQUIRED)
find_program (AVR_OBJCOPY avr-objcopy REQUIRED)
find_program (AVR_SIZE_TOOL avr-size REQUIRED)
find_program (AVR_OBJDUMP avr-objdump REQUIRED)
find_program (AVRDUDE avrdude REQUIRED)

set (CMAKE_SYSTEM_NAME Generic)
set (CMAKE_SYSTEM_PROCESSOR avr)
set (CMAKE_C_COMPILER ${AVR_CC})
set (CMAKE_CXX_COMPILER ${AVR_CXX})

set (AVRDUDE_CONFIG ${CMAKE_CURRENT_SOURCE_DIR}/board/avrdude.conf)

set (SOURCE
		src/fiesta_can.cpp
		src/canmsg.cpp
	)

set (MCP2515_CAN_SOURCE
		src/mcp2515/mcp_can.cpp
	)

set (ARDUINO_CORE_SOURCE
		WInterrupts.c
		wiring.c
		wiring_analog.c
		wiring_digital.c
		wiring_pulse.c
		wiring_shift.c
		HardwareSerial.cpp
		HardwareSerial0.cpp
		HardwareSerial1.cpp
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

add_library (mcp2515can STATIC ${MCP2515_CAN_SOURCE})
add_library (arduino STATIC ${ARDUINO_CORE_SOURCE})
add_library (mightycore STATIC ${MIGHTY_CORE_SOURCE})

add_executable (fiesta_can.elf ${SOURCE})

set_target_properties (arduino PROPERTIES COMPILE_FLAGS "${MCU_C_FLAGS}" LINK_FLAGS "${MCU_LINK_FLAGS}")
set_target_properties (mightycore PROPERTIES COMPILE_FLAGS "${MCU_C_FLAGS}" LINK_FLAGS "${MCU_LINK_FLAGS}")
set_target_properties (mcp2515can PROPERTIES COMPILE_FLAGS "${MCU_C_FLAGS}" LINK_FLAGS "${MCU_LINK_FLAGS}")
set_target_properties (fiesta_can.elf PROPERTIES COMPILE_FLAGS "${MCU_C_FLAGS}" LINK_FLAGS "${MCU_LINK_FLAGS}")
target_link_libraries (mcp2515can arduino mightycore)
target_link_libraries (fiesta_can.elf mcp2515can arduino mightycore)

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
		${AVRDUDE} -C ${AVRDUDE_CONFIG} -c usbasp -p ${MCU} -U flash:w:rom.hex
	    DEPENDS rom.hex
		COMMENT "Uploading rom to ${MCU}"
   )
