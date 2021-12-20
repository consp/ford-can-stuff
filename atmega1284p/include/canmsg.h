#include <stdlib.h>
#include <stdint.h>

typedef enum {

	HSCAN = 1,
	MSCAN = 2,
	ICAN = 3,
} CAN_BUS;

class canmsg {
	private:
		static const uint32_t identifier = 0; // should be set by children.
		static const CAN_BUS canbus = HSCAN;
	public:
		canmsg(uint32_t id, uint8_t data[8], uint32_t length);

		uint32_t id;
		uint8_t data[8];

		CAN_BUS bus();

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
	private:
		static const uint32_t identifier = 0x081; // 129
		static const CAN_BUS canbus = ICAN;
	public:
		explicit CAN_SteeringWheelData_2(uint8_t d[8]);
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

