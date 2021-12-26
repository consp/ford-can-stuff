git submodule init
git submodule update
ln -s ./submodules/mightycore/avr/boards.txt ./atmega1284p/board/
ln -s ./submodules/mightycore/avr/avrdude.conf ./atmega1284p/board/
ln -s ./submodules/mightycore/avr/variants/standard/pins_arduino.h ./atmega1284p/board/
ln -s ./submodules/mightycore/avr/cores/MCUdude_corefiles/* ./atmega1284p/cores/
ln -s ./submodules/mightycore/avr/libraries/* ./atmega1284p/libraries/
