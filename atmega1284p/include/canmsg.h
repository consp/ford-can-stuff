#include <stdlib.h>
#include <stdint.h>

using namespace std;

typedef enum {
	HSCAN = 0,
	MSCAN = 1,
	ICAN = 2,
} CAN_BUS;

class canmsg {
	public:
		uint32_t id;
		uint8_t data[8];
        uint8_t bus;
        uint8_t len;
};

class apim_1 : public canmsg {
    public:
        uint32_t id = 0x048;
        uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0xE0, 0x00};
        uint8_t len = 8;
        uint8_t bus = 2;
};

class apim_2 : public canmsg {
    public:
        uint32_t id = 0x3B3;
        uint8_t data[8] = {0x41, 0x00, 0x00, 0x00, 0x4c, 0x00, 0xE0, 0x00};
        uint8_t len = 8;
        uint8_t bus = 2;
};

class apim_3 : public canmsg {
    public:
        uint32_t id = 0x109;
        uint8_t data[8] = {0x00, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x28};
        uint8_t len = 8;
        uint8_t bus = 2;
};

// bit positions of bco files:
//  7  6  5  4  3  2  1  0
// 15 14 13 12 11 10  9  8
// 23 22 21 20 19 18 17 16
// 31 30 29 28 27 26 25 24
// 39 38 37 36 35 34 33 32
// 47 46 45 44 43 42 41 40
// 55 54 53 52 51 50 49 48
// 63 62 61 60 59 58 57 56

class CAN_SteeringWheelData_2 : canmsg {
	public:
		uint32_t id = 0x081; // 129
		CAN_BUS bus = ICAN;
		union {
			uint8_t raw[8];
			struct {
				uint8_t cursor_l_rt : 1;
				uint8_t cursor_l_lt : 1;
				uint8_t cursor_l_dn : 1;
				uint8_t cursor_r_ok : 1;
				uint8_t cursor_r_up : 1;
				uint8_t cursor_r_rt : 1;
				uint8_t cursor_r_lt : 1;
				uint8_t cursor_r_dn : 1;
				uint8_t reserved0 : 6;
				uint8_t cursor_l_ok : 1;
				uint8_t cursor_l_up : 1;
				uint8_t reserved1 : 8;
				uint8_t reserved2 : 8;
				uint8_t reserved3 : 8;
				uint8_t reserved4 : 8;
				uint8_t reserved5 : 8;
				uint8_t reserved6 : 8;
			};
		};
};

