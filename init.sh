ORIGIN=$(pwd)
git submodule init
git submodule update
mkdir -p ./atmega1284p/board
mkdir -p ./atmega1284p/cores
mkdir -p ./atmega1284p/libraries
ln -s ../../submodules/mightycore/avr/boards.txt ./atmega1284p/board/
ln -s ../../submodules/mightycore/avr/avrdude.conf ./atmega1284p/board/
ln -s ../../submodules/mightycore/avr/variants/standard/pins_arduino.h ./atmega1284p/board/
cd $ORIGIN/atmega1284p/cores
ln -s ../../submodules/mightycore/avr/cores/MCUdude_corefiles/* ./
cd $ORIGIN/atmega1284p/libraries
ln -s ../../submodules/mightycore/avr/libraries/* ./
