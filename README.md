CAN Bus tool for modifying the 3 busses on a Ford Fiesta Mk7.5

Provided AS-IS. License is not Free, commerial use of code not covered under other Licences is prohibited.

Uses a ATMega1284p as controller, CN340N for USB to Serial and 3x MCP2515 with TC1051 as CAN controllers and trancievers.

Planned features:

- Bus transversion, e.g. HS to ICAN data transfer
- Bus translation, e.g. converting messages to a different format (mostly MSCAN to ICAN)
- Debugging multiple busses via SLCAN protocol (extensions)
