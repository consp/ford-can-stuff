CAN Bus tool for modifying the 3 busses on a Ford Fiesta Mk7.5

Provided AS-IS. License is not Free, commerial use of code not covered under other Licences is prohibited.

Uses a ATMega1284p as controller, CN340N for USB to Serial and 3x MCP2515 with TC1051 as CAN controllers and trancievers.

Planned features:

- [ ] Bus transversion, e.g. HS to ICAN data transfer
- [ ] Bus translation, e.g. converting messages to a different format (mostly MSCAN to ICAN)
- [x] Debugging multiple busses via SLCAN protocol (extensions)
- [x] Binary mode for better bus usage
- [x] slcan compatible kernel driver for multiple busses

# Acknowledgements
- [MightyCore](https://github.com/MCUdude/MightyCore) for the arduino compatable stuff (which makes my life easier even if I do not use arduino tools)
- The [MCP_CAN_lib](https://github.com/coryjfowler/MCP_CAN_lib) fork for the mcp lib. This part is under GPLv3.
- SLCAN driver as in the upstream kernel.
- can-utils
- creator of simple crc8.c file for avr.

## Who this is for
This repository is the extension of my own project to get the front distance sensors working on my car. It collided with the idea of creating a translation module for Sync 4 to be fitted into Sync 3 capable vehicles. This repository is solely meant for development and debugging and not meant as a ready made tool to install and get going.


## Who this is not for
- If you do not know what a CAN BUS is, this is not for you
- If you do not know how to use a micro processor, this is not for you
- If you do not know how to order PCBs, Create your own board and put it all together, this is not for you
- If you just want fancy features on "car model XXX", this is not for you
- If you do not know how to compile kernel modules and/or have the knowledge to build them from scratch, this is not for you

If you still think this is for you: Welcome, you can always create pull requests if you want to change something. Issues will be ignored.

## Compile the arduino-style stuff

- Get all the needed tools: cmake, gcc-avr, avr-libc, avrdude.
- Get [mightycore for the atmega1284p](ihttps://github.com/MCUdude/MightyCore) core files and put them in the atmega1284p/cores directory
- Get the mightycore libraries and put them into atmega1284p/libraries
- Get the standard pinout file and the avrdude config file from mightycore and put them in the atmega1284p/boards directory.

Or collect the git submodules and run the batch file to create the softlinks.

Run `mkdir -p build; cd build; cmake ..`

## Compile the kernel module
Same as above but then `make slcan_multimode` and install with `make install_kmod`

## Known issues
- There is some memoy issue which causes the device to reset on receiving data and timing out afterwards.
