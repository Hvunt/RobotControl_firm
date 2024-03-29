
#define MOTORS_FORWARD "FORWARD"
#define MOTORS_REVERSE "REVERSE"
#define MOTORS_LEFT "LEFT"
#define MOTORS_RIGHT "RIGHT"
#define MOTORS_STOP "STOP"

/*
 * Terminals definitions
 */
#define TERMINAL_ONE		 0b00000001
#define TERMINAL_TWO		 0b00000010
#define TERMINAL_THREE		 0b00000100
#define TERMINAL_FOUR		 0b00001000

/*
 * Commands definitions
 */
//SET
#define COMM_MOVE				0x50
#define COMM_SET_SERVOS_POS		0x80

//GET
#define COMM_GET_SPEED			0x70
#define COMM_GET_SERVOS_POS 	0xB0

enum {
	MOVE_FORWARD = 0x02,
	MOVE_REVERSE = 0x04,
	MOVE_LEFT = 0x20,
	MOVE_RIGHT = 0x22,
	MOVE_STOP = 0x44
};