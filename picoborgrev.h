/********************************************************************************/
/*																				*/
/*	PIC code for the PicoBorg Reverse advanced robot controller					*/
/*		https://www.piborg.org/picoborgrev										*/
/*																				*/
/*	Rewritten for sdcc by Torsten Kurbad <beaglebone@tk-webart.de>				*/
/*																				*/
/********************************************************************************/

/********************************************************************************/
/* Type definitions                                                           	*/
/********************************************************************************/

typedef enum { false, true } bool;

/********************************************************************************/
/* System #define Macros														*/
/********************************************************************************/

#define SYS_FREQ				32000000L
#define INSTR_PER_MS			(SYS_FREQ / 2000L)
#define LOOPS_PER_MS			(INSTR_PER_MS / 14U)	// For delay loop

/********************************************************************************/
/* User Level #define Macros													*/
/********************************************************************************/

#define PWM_MAX					(255)	// Maximum PWM duty cycle
#define I2C_MAX_LEN				(2)		// Maximum length of I2C data bytes
										//  Note, that this excludes address and
										//  command bytes, which are always sent
										//  by the I2C master.
#define FAILSAFE_LIMIT			(31)	// Approximately 0.25 seconds at a 122 Hz timer

#define I2C_ID_PICOBORG_REV		(0x15)	// Board identifier

#define COMMAND_NONE			(0)		// 'Empty' command

#define COMMAND_SET_LED			(1)		// Set the LED status
#define COMMAND_GET_LED			(2)		// Get the LED status
#define COMMAND_SET_A_FWD		(3)		// Set motor A PWM rate in a forwards direction
#define COMMAND_SET_A_REV		(4)		// Set motor A PWM rate in a reverse direction
#define COMMAND_GET_A			(5)		// Get motor A direction and PWM rate
#define COMMAND_SET_B_FWD		(6)		// Set motor B PWM rate in a forwards direction
#define COMMAND_SET_B_REV		(7)		// Set motor B PWM rate in a reverse direction
#define COMMAND_GET_B			(8)		// Get motor B direction and PWM rate
#define COMMAND_ALL_OFF			(9)		// Switch everything off
#define COMMAND_RESET_EPO		(10)	// Resets the EPO flag, use after EPO has been tripped and switch is now clear
#define COMMAND_GET_EPO			(11)	// Get the EPO latched flag
#define COMMAND_SET_EPO_IGNORE	(12)	// Set the EPO ignored flag, allows the system to run without an EPO
#define COMMAND_GET_EPO_IGNORE	(13)	// Get the EPO ignored flag
#define COMMAND_GET_DRIVE_FAULT	(14)	// Get the drive fault flag, indicates faults such as short-circuits and under voltage
#define COMMAND_SET_ALL_FWD		(15)	// Set all motors PWM rate in a forwards direction
#define COMMAND_SET_ALL_REV		(16)	// Set all motors PWM rate in a reverse direction
#define COMMAND_SET_FAILSAFE	(17)	// Set the failsafe flag, turns the motors off if communication is interrupted
#define COMMAND_GET_FAILSAFE	(18)	// Get the failsafe flag
#define COMMAND_SET_ENC_MODE	(19)	// Set the board into encoder or speed mode
#define COMMAND_GET_ENC_MODE	(20)	// Get the boards current mode, encoder or speed
#define COMMAND_MOVE_A_FWD		(21)	// Move motor A forward by n encoder ticks
#define COMMAND_MOVE_A_REV		(22)	// Move motor A reverse by n encoder ticks
#define COMMAND_MOVE_B_FWD		(23)	// Move motor B forward by n encoder ticks
#define COMMAND_MOVE_B_REV		(24)	// Move motor B reverse by n encoder ticks
#define COMMAND_MOVE_ALL_FWD	(25)	// Move all motors forward by n encoder ticks
#define COMMAND_MOVE_ALL_REV	(26)	// Move all motors reverse by n encoder ticks
#define COMMAND_GET_ENC_MOVING	(27)	// Get the status of encoders moving
#define COMMAND_SET_ENC_SPEED	(28)	// Set the maximum PWM rate in encoder mode
#define COMMAND_GET_ENC_SPEED	(29)	// Get the maximum PWM rate in encoder mode
#define COMMAND_GET_ID			(0x99)	// Get the board identifier
#define COMMAND_SET_I2C_ADD		(0xAA)	// Set a new I2C address

#define COMMAND_VALUE_FWD		(1)		// I2C value representing forward
#define COMMAND_VALUE_REV		(2)		// I2C value representing reverse

#define COMMAND_VALUE_ON		(1)		// I2C value representing on
#define COMMAND_VALUE_OFF		(0)		// I2C value representing off

#define EEPROM_I2C_ADDRESS		(0x00)	// Saved I2C address

#define I2C_DATA_NONE           (0x00)  // Empty data byte

/********************************************************************************/
/* Global variables																*/
/********************************************************************************/

// I2C address that was sent by the master
extern unsigned char i2cAddress;
// I2C command that was sent by the master
extern unsigned char i2cCommand;
// Array of data bytes received by the master
extern unsigned char i2cRXData[I2C_MAX_LEN];
// Byte count for the data array
extern int byteCount;
// Junk byte for excess/unnecessary reads
extern char junk;
// Has a GET_... command been received by the master that is yet to be answered?
extern bool readCommandPending;

// Has the Emergency Power Off switch been tripped?
extern bool epoTripped;
// Should the Emergency Power Off switch be ignored?
extern bool epoIgnored;
// Communications failsafe counter
extern int failsafeCounter;
// Is motor A moving?
extern bool movingA;
// Is motor B moving?
extern bool movingB;
// Motor A moving in reverse?
extern bool reverseA;
// Motor B moving in reverse?
extern bool reverseB;
// Maximum PWM duty cycle for encoder mode
extern int encLimit;
// Remaining encoder ticks for motor A
extern int remainingCountsA;
// Remaining encoder ticks for motor B
extern int remainingCountsB;

/********************************************************************************/
/* System Function Prototypes													*/
/********************************************************************************/

/* Configure the PIC's internal oswillator */
void ConfigureOscillator(void);
/* Delay processing for approximately 'ms' milliseconds */
void Delay_ms(unsigned short ms);

/********************************************************************************/
/* Interrupt Service Routine													*/
/********************************************************************************/

/* ISR to process the I2C commands and encoder interrupts */
void isr_i2c(void) __interrupt 0;

/********************************************************************************/
/* User Function Prototypes														*/
/********************************************************************************/

/* I/O and peripheral Initialization */
void InitApp(void);
/* Control movement of motor A */
void SetMotorA(bool reverse, int pwm);
/* Control movement of motor B */
void SetMotorB(bool reverse, int pwm);
/* Control movement of both motors */
void SetAllMotors(bool reverse, int pwm);
/* Enable/disable encoder mode */
void SetEncoderMode(bool enabled);
/* Move motor A by 'count' encoder ticks */
void MoveMotorA(bool reverse, int count);
/* Move motor B by 'count' encoder ticks */
void MoveMotorB(bool reverse, int count);
