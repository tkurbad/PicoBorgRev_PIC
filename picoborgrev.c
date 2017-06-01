/********************************************************************************/
/*																				*/
/*	PIC code for the PicoBorg Reverse advanced robot controller					*/
/*		https://www.piborg.org/picoborgrev										*/
/*																				*/
/*	Rewritten for sdcc by Torsten Kurbad <beaglebone@tk-webart.de>				*/
/*																				*/
/********************************************************************************/

/********************************************************************************/
/* Global #defines																*/
/********************************************************************************/

// Avoid polluting the global namespace with aliases for each pin.
#define NO_BIT_DEFINES


/********************************************************************************/
/* Includes																		*/
/********************************************************************************/

// PicoBorg Reverse uses a PIC16F1824 for I2C communication
#include "pic14/pic16regs.h"
#include "pic14/pic16f1824.h"

#include "picoborgrev.h"


/********************************************************************************/
/* PIC configuration															*/
/********************************************************************************/

__code unsigned short __at (_CONFIG1) configWord1 = (
	_FOSC_INTOSC &				// Internal oscillator enabled
	_WDTE_OFF &					// Watchdog timer is disabled
	_PWRTE_ON &					// Power up timer is enabled
	_MCLRE_ON &					// MCLR/RA3 pin function os MCLR
	_CP_OFF &					// Program memory protection disabled
	_CPD_OFF &					// Data memory protection disabled
	_BOREN_ON &					// Brown-out reset enabled
	_CLKOUTEN_OFF &				// Clock out disabled
	_IESO_OFF &					// Internal/external switchover disabled
	_FCMEN_ON					// Fail-Safe clock monitor enabled
	);

__code unsigned short __at (_CONFIG2) configWord2 = (
	_WRT_OFF &					// Flash memory write protection disabled
	_PLLEN_ON &					// 4xPLL enabled
	_STVREN_ON &				// Stack over- or underflow reset enabled
	_BORV_LO &					// Brown-out reset voltage set to 1.9V
	_DEBUG_OFF &				// In-circuit debugging disabled
	_LVP_OFF					// Low-voltage programming disabled
	);


/********************************************************************************/
/* System Functions																*/
/********************************************************************************/

/* Configure the PIC's internal oswillator */
void ConfigureOscillator(void) {
	OSCCONbits.SCS = 0b0;		// Set the clock to the CONFIG1 setting (internal oscillator)
	OSCCONbits.IRCF = 0b1110;	// Select the 8 MHz postscaler
	OSCCONbits.SPLLEN = 1;		// Enable the 4x PLL (overriden in CONFIG2 by PLLEN = ON)
	OSCTUNEbits.TUN = 0b011111;	// Set oscillator to max frequency
}

/* Delay processing for approximately 'ms' milliseconds */
void Delay_ms(unsigned short ms) {
	int u;
	while (ms--) {
		for(u = 0; u < LOOPS_PER_MS; u++) {
			Nop();
		}
	}
}


/********************************************************************************/
/* User Global Variable Initialization											*/
/********************************************************************************/

// I2C address that was sent by the master
unsigned char i2cAddress = 0x00;
// I2C command that was sent by the master
unsigned char i2cCommand = 0x00;
// Array of data bytes received by the master
unsigned char i2cRXData[I2C_MAX_LEN] = {0x00};
// Byte count for the data array
int byteCount = 0;
// Junk byte for excess/unnecessary reads
char junk = 0x00;
// Has a GET_... command been received by the master that is yet to be answered?
bool readCommandPending = false;

// Has the Emergency Power Off switch been tripped?
bool epoTripped = false;
// Should the Emergency Power Off switch be ignored?
bool epoIgnored = false;
// Communications failsafe counter
int failsafeCounter = 0;
// Is motor A moving?
bool movingA = false;
// Is motor B moving?
bool movingB = false;
// Motor A moving in reverse?
bool reverseA = false;
// Motor B moving in reverse?
bool reverseB = false;
// Set PWM duty cycle for encoder mode to PWM_MAX
int encLimit = PWM_MAX;
// Remaining encoder ticks for motor A
int remainingCountsA = 0;
// Remaining encoder ticks for motor B
int remainingCountsB = 0;

/********************************************************************************/
/* User Functions																*/
/********************************************************************************/

/* I/O and Peripheral Initialization */
void InitApp(void) {
	unsigned char value;

	/* Setup analog functionality and port direction */
	ANSELA = 0;						// All digital IO, port A
	ANSELC = 0;						// All digital IO, port C
	TRISAbits.TRISA0 = 1;			// Encoder line for B
	TRISAbits.TRISA1 = 1;			// Encoder line for A
	TRISAbits.TRISA2 = 1;   		// Error flag
	TRISAbits.TRISA4 = 0;   		// LED
	TRISAbits.TRISA5 = 1;   		// EPO
	TRISCbits.TRISC1 = 1;   		// I2C Data
	TRISCbits.TRISC0 = 1;   		// I2C Clock
	TRISCbits.TRISC3 = 0;   		// Motor A:1
	TRISCbits.TRISC2 = 0;   		// Motor A:2
	TRISCbits.TRISC5 = 0;   		// Motor B:1
	TRISCbits.TRISC4 = 0;   		// Motor B:2

	/* Initialize peripherals */

	// Ports
	PORTAbits.RA4 = 0;   			// LED On
	PORTCbits.RC3 = 0;   			// Motor A:1 Off
	PORTCbits.RC2 = 0;   			// Motor A:2 Off
	PORTCbits.RC5 = 0;   			// Motor B:1 Off
	PORTCbits.RC4 = 0;   			// Motor B:2 Off

	// MSSP (I2C)
	SSP1CON1bits.SSPM = 0b0110;		// I2C slave mode, 7b address (interrupts enabled manually)
	SSP1CON1bits.CKP = 1;			// Release clock line
	SSP1CON1bits.SSPOV = 0;			// Clear the overflow flag
	SSP1CON1bits.WCOL = 0;			// Clear the collision flag
	SSP1STATbits.SMP = 0;			// Slew rate control disabled (100 KHz I2C)
	SSP1STATbits.CKE = 0;			// Transmission on idle to active clock
	SSP1CON2bits.GCEN = 1;			// Enable listening for broadcasts
	SSP1CON2bits.SEN = 1;			// Clock stretching enabled
	SSP1CON3bits.PCIE = 1;			// Stop condition interrupts enabled
	SSP1CON3bits.SCIE = 0;			// Start condition interrupts disabled
	SSP1CON3bits.BOEN = 0;			// Buffer is only overwritten if SSPOV is clear
	SSP1CON3bits.SDAHT = 1;			// Minimum of 300ns SDA hold time after clock edge
	SSP1CON3bits.SBCDE = 0;			// Bus collision detection disabled
	SSP1CON3bits.AHEN = 0;			// Automatic holding of the clock after address disabled
	SSP1CON3bits.DHEN = 0;			// Automatic holding of the clock after data disabled
	SSP1MSK = 0xFE;					// Address mask against full 7 bits
	SSP1ADD = 0x88;					// Default I2C address
	SSP1CON1bits.SSPEN = 1;			// Enable the SDA and SCL pins as the port inputs (RC0 and RC1)

	// PWM
	CCP1CONbits.CCP1M = 0b0000;		// PWM 1 disabled
	CCP2CONbits.CCP2M = 0b0000;		// PWM 2 disabled
	CCP3CONbits.CCP3M = 0b0000;		// PWM 3 disabled
	CCP4CONbits.CCP4M = 0b0000;		// PWM 4 disabled
	TRISCbits.TRISC5 = 1;			// Disable PWM 1 output
	TRISCbits.TRISC3 = 1;			// Disable PWM 2 output
	PR2 = 0x3F;						// 8-bit resolution, 125 KHz at 1:1 prescale
	CCP1CONbits.P1M = 0b00;			// PWM 1 single output mode (P1A/RC5 only)
	CCP2CONbits.P2M = 0b00;			// PWM 2 single output mode (P2A/RC3 only)
	CCP1CONbits.CCP1M = 0b1100;		// PWM 1 set to PWM mode (B/C/D active high)
	CCP2CONbits.CCP2M = 0b1100;		// PWM 2 set to PWM mode (B/C/D active high)
	CCPR1L = 0x0;					// PWM 1 pulse width set to 0 (MSBs)
	CCP1CONbits.DC1B = 0b00;		// PWM 1 pulse width set to 0 (LSBs)
	CCPR2L = 0x0;					// PWM 2 pulse width set to 0 (MSBs)
	CCP2CONbits.DC2B = 0b00;		// PWM 2 pulse width set to 0 (LSBs)
	CCPTMRS0bits.C1TSEL = 0b00;		// PWM 1 set to period from Timer 2
	CCPTMRS0bits.C2TSEL = 0b00;		// PWM 2 set to period from Timer 2
	T2CONbits.T2OUTPS = 0b0000;		// Timer 2 1:1 postscaler
	T2CONbits.T2CKPS = 0b10;		// Timer 2 1:16 prescaler, 7.8125 KHz
	T2CONbits.TMR2ON = 1;			// Timer 2 enabled
	PIR1bits.TMR2IF = 0;			// Clear Timer 2 interrupt flag
	while (PIR1bits.TMR2IF == 0) ;	// Wait for Timer 2 to loop
	TRISCbits.TRISC5 = 0;			// Enable PWM 1 output
	TRISCbits.TRISC3 = 0;			// Enable PWM 2 output

	// Failsafe timer
	PR4 = 0x3F;						// 8-bit resolution, 125 KHz at 1:1 prescale
	T4CONbits.T4CKPS = 0b11;		// Timer 4 1:64 prescaler, 1,953.125 Hz
	T4CONbits.T4OUTPS = 0b1111;		// Timer 4 1:16 postscaler, ~122 Hz
	T4CONbits.TMR4ON = 0;			// Timer 4 disabled
	PIR3bits.TMR4IF = 0;			// Clear Timer 4 interrupt flag

	// OTHERS
	MDCONbits.MDEN = 0;				// Modulator disabled
	MDCONbits.MDOE = 0;				// Modulator ouput disabled
	SRCON0bits.SRLEN = 0;			// SR latch disabled
	SRCON0bits.SRQEN = 0;			// SR latch output disabled
	SRCON0bits.SRNQEN = 0;			// SR latch #output disabled
	CM1CON0bits.C1ON = 0;			// Comparator 1 disabled
	CM1CON0bits.C1OE = 0;			// Comparator 1 output disabled
	CM2CON0bits.C2ON = 0;			// Comparator 2 disabled
	CM2CON0bits.C2OE = 0;			// Comparator 2 output disabled
	RCSTAbits.SPEN = 0;				// UART disabled

	// Interrupts (global enable elsewhere)
	INTCONbits.PEIE = 1;			// Peripheral interrupts enabled
	PIE1bits.SSP1IE = 1;			// MSSP (I2C) interrupt enabled
	PIR1bits.SSP1IF = 0;			// Clear interrupt flag
	INTCONbits.IOCIE = 0;			// Interrupts on change disabled (enable when using encoder mode)
	IOCAN = 0;						// Disable all interrupts on falling pin changes
	IOCANbits.IOCAN0 = 1;			// Enable interrupts on falling A0 (Encoder line for B)
	IOCANbits.IOCAN1 = 1;			// Enable interrupts on falling A1 (Encoder line for A)
	IOCAP = 0;						// Disable all interrupts on rising pin changes
	IOCAPbits.IOCAP0 = 1;			// Enable interrupts on rising A0 (Encoder line for B)
	IOCAPbits.IOCAP1 = 1;			// Enable interrupts on rising A1 (Encoder line for A)
	INTCONbits.IOCIF = 0;			// Clear interrupt flag

	/* Read EEPROM settings */
	// Read the I2C address
	EEADRL = EEPROM_I2C_ADDRESS;
	EECON1bits.EEPGD = 0;
	EECON1bits.CFGS = 0;
	EECON1bits.RD = 1;
	value = EEDATL;
	// Check against limits before setting, if out of range keep with the default
	if ((value > 0x02) && (value < 0x78)) {
		SSP1ADD = value << 1;
	}
}

/* Control movement of motor A */
void SetMotorA(bool reverse, int pwm) {
	// Check if the EPO has been tripped
	if ((epoTripped == true) && (epoIgnored == false)) {
		reverse = false;
		pwm = 0;
	}
	// Work out the required settings
	if (pwm > PWM_MAX) pwm = PWM_MAX;
	if (reverse == true) {
		reverseA = true;				// Motor A is moving in reverse
		pwm = PWM_MAX - pwm;			// PWM pulse width inverted for reverse
		LATCbits.LATC2 = 1;				// Motor A:2 On
	} else {
		reverseA = false;				// Motor A is moving forward
		// PWM pulse width is not inverted for forward
		LATCbits.LATC2 = 0;				// Motor A:2 Off
	}
	// Set Motor A:1 to the pulse width
	CCPR2L = pwm >> 2;					// Set PWM 2 MSBs
	CCP2CONbits.DC2B = pwm & 0b11;		// Set PWM 2 LSBs
}


/* Control movement of motor B */
void SetMotorB(bool reverse, int pwm) {
	// Check if the EPO has been tripped
	if ((epoTripped == true) && (epoIgnored == false)) {
		reverse = false;
		pwm = 0;
	}
	// Work out the required settings
	if (pwm > PWM_MAX) pwm = PWM_MAX;
	if (reverse == true) {
		reverseB = true;				// Motor B is moving in reverse
		pwm = PWM_MAX - pwm;			// PWM pulse width inverted for reverse
		LATCbits.LATC4 = 1;				// Motor B:2 On
	} else {
		reverseB = false;				// Motor B is moving forward
		// PWM pulse width is not inverted for forward
		LATCbits.LATC4 = 0;				// Motor B:2 Off
	}
	// Set Motor B:1 to the pulse width
	CCPR1L = pwm >> 2;					// Set PWM 1 MSBs
	CCP1CONbits.DC1B = pwm & 0b11;		// Set PWM 1 LSBs
}

/* Control movement of both motors */
void SetAllMotors(bool reverse, int pwm) {
	// Check if the EPO has been tripped
	if ((epoTripped == true) && (epoIgnored == false)) {
		reverse = false;
		pwm = 0;
	}
	// Work out the required settings
	if (pwm > PWM_MAX) pwm = PWM_MAX;
	if (reverse == true) {
		reverseA = true;					// Motor A is moving in reverse
		reverseB = true;					// Motor B is moving in reverse
		pwm = PWM_MAX - pwm;				// PWM pulse width inverted for reverse
		LATC = PORTC | (_LATC2 | _LATC4);	// Motor A:2 and B:2 On
	} else {
		reverseA = false;					// Motor A is moving forward
		reverseB = false;					// Motor B is moving forward
		// PWM pulse width is not inverted for forward
		LATC = PORTC & ~(_LATC2 | _LATC4);	// Motor A:2 and B:2 Off
	}
	// Wait until it is safe to change the pulse widths
	PIR1bits.TMR2IF = 0;					// Clear Timer 2 interrupt flag
	while (PIR1bits.TMR2IF == 0) ;			// Wait for Timer 2 to loop
	// Set Motor A:1 and B:1 to the pulse width
	CCPR2L = pwm >> 2;						// Set PWM 2 MSBs
	CCPR1L = pwm >> 2;						// Set PWM 1 MSBs
	CCP2CONbits.DC2B = pwm & 0b11;			// Set PWM 2 LSBs
	CCP1CONbits.DC1B = pwm & 0b11;			// Set PWM 1 LSBs
}

/* Enable/disable encoder mode */
void SetEncoderMode(bool enabled) {
	// Disable any automatic routines
	movingA = false;
	movingB = false;

	// Turn off both motors
	SetAllMotors(false, 0);

	// Set the correct mode
	if (enabled == true) {
		INTCONbits.IOCIE = 1;				// Interrupts on change enabled
	} else {
		INTCONbits.IOCIE = 0;				// Interrupts on change disabled
	}
}

/* Move motor A by 'count' encoder ticks */
void MoveMotorA(bool reverse, int count) {
	// Make sure we are in encoder mode before starting
	if (INTCONbits.IOCIE == 0) {
		return;
	}

	// Set the distance to cover
	remainingCountsA = count;

	// Start moving
	SetMotorA(reverse, encLimit);

	// Set the moving flag
	movingA = true;
}

/* Move motor A by 'count' encoder ticks */
void MoveMotorB(bool reverse, int count) {
	// Make sure we are in encoder mode before starting
	if (INTCONbits.IOCIE == 0) {
		return;
	}

	// Set the distance to cover
	remainingCountsB = count;

	// Start moving
	SetMotorB(reverse, encLimit);
	
	// Set the moving flag
	movingB = true;
}


/********************************************************************************/
/* Interrupt Service Routine													*/
/********************************************************************************/

/* ISR to process the I2C commands and encoder interrupts */
void isr_i2c(void) __interrupt 0 {

	/* I2C interrupt processing */

	if (PIR1bits.SSP1IF) {						// I2C event occured?
		PIR1bits.SSP1IF = 0;					// Reset I2C event interrupt
		SSP1CON1bits.CKP = 0;					// Hold clock line
												//	(=clock stretching)

		if (SSP1STATbits.P) {					// I2C Stop bit received?
			if (readCommandPending == false) {	// Is no GET_... command pending?
				i2cCommand = COMMAND_NONE;		// Reset command
			}

			for (byteCount = 0; byteCount < I2C_MAX_LEN; ++byteCount) {
				i2cRXData[byteCount] = 0x00;	// Reset i2cRXData array unconditionally
			}
			byteCount = 0;						// Reset data byte count unconditionally

		} else {								// No I2C Stop bit received

			if (!SSP1STATbits.D_NOT_A) {		// Last byte sent by I2C master
												//	was an address, not data
				if (SSP1STATbits.R_NOT_W) {		// I2C master wants to receive data,
												//  i.e. sends a GET_... command

					switch(i2cCommand) {
						// A GET_... command has been initiated by the master.
						// First step: Send command byte for valid GET_...
						//	commands back to the
						//	I2C master
						case COMMAND_GET_LED:
							SSP1BUF = COMMAND_GET_LED;
							break;
						case COMMAND_GET_A:
							SSP1BUF = COMMAND_GET_A;
							break;
						case COMMAND_GET_B:
							SSP1BUF = COMMAND_GET_B;
							break;
						case COMMAND_GET_EPO:
							SSP1BUF = COMMAND_GET_EPO;
							break;
						case COMMAND_GET_EPO_IGNORE:
							SSP1BUF = COMMAND_GET_EPO_IGNORE;
							break;
						case COMMAND_GET_DRIVE_FAULT:
							SSP1BUF = COMMAND_GET_DRIVE_FAULT;
							break;
						case COMMAND_GET_FAILSAFE:
							SSP1BUF = COMMAND_GET_FAILSAFE;
							break;
						case COMMAND_GET_ENC_MODE:
							SSP1BUF = COMMAND_GET_ENC_MODE;
							break;
						case COMMAND_GET_ENC_MOVING:
							SSP1BUF = COMMAND_GET_ENC_MOVING;
							break;
						case COMMAND_GET_ENC_SPEED:
							SSP1BUF = COMMAND_GET_ENC_SPEED;
							break;
						case COMMAND_GET_ID:
							SSP1BUF = COMMAND_GET_ID;
							break;
						default:
							// For unknown commands, send 0x00
							SSP1BUF = I2C_DATA_NONE;
							break;
					}
				} else {										// I2C master wants to send data
					i2cAddress = SSP1BUF;						// Read I2C address sent by master
				}
			} else {											// Last byte sent by I2C master
																//	was data, not an address
				if (SSP1STATbits.R_NOT_W) {						// I2C master wants to receive data,
																//  i.e. sends a GET_... command.

					switch(i2cCommand) {
						/* A GET_... command has been initiated by the master.
						 * Second / third step: Send data byte(s) for valid GET_...
						 *	commands back to the
						 *	I2C master
						 */
						case COMMAND_GET_LED:					// LED status yields one data byte
							if (PORTAbits.RA4 == 1) {			// LED is off
								SSP1BUF = COMMAND_VALUE_OFF;	// Fill SSP1BUF with
																//	the byte value for 'off'.
																// I2C master will then
																//	read from this buffer.
							} else {							// LED is on
								SSP1BUF = COMMAND_VALUE_ON;
							}
							i2cCommand = COMMAND_NONE;			// Reset I2C command
							readCommandPending = false;			// GET_... command finished
							break;								// Done with this cycle
						case COMMAND_GET_A:						// Motor status yields two data bytes
							if (byteCount == 0) {				// First cycle?
																// First data byte represents direction
								if (PORTCbits.RC2 == 0) {		// Motor A forward?
																// Send byte value for 'forward'
									SSP1BUF = COMMAND_VALUE_FWD;
								} else {						// Motor A reverse?
																// Send byte value for 'reverse'
									SSP1BUF = COMMAND_VALUE_REV;
								}
								++byteCount;					// Increase byte counter
							} else {							// Second cycle?
																// Second data byte represents
																//	PWM value
								if (PORTCbits.RC2 == 0) {		// Motor A forward?
																// Send PWM value
									SSP1BUF = (char)(0xFF & ((CCPR2L << 2) | CCP2CONbits.DC2B));
								} else {						// Motor A reverse?
																// Send inverse PWM value
									SSP1BUF = PWM_MAX - (char)(0xFF & ((CCPR2L << 2) | CCP2CONbits.DC2B));
								}
								i2cCommand = COMMAND_NONE;		// After the second byte,
																//	reset I2C command,
								readCommandPending = false;		//	the GET_... command pending
																//	flag, and
								byteCount = 0;					//	the byte counter
							}
							break;								// Done with this cycle
						case COMMAND_GET_B:						// Motor status yields two data bytes
																// (cf. COMMAND_GET_A)
							if (byteCount == 0) {
								if (PORTCbits.RC4 == 0) {		// Motor B forward?
									SSP1BUF = COMMAND_VALUE_FWD;
								} else {						// Motor B reverse?
									SSP1BUF = COMMAND_VALUE_REV;
								}
								++byteCount;
							} else {
								if (PORTCbits.RC4 == 0) {
									SSP1BUF = (char)(0xFF & ((CCPR1L << 2) | CCP1CONbits.DC1B));
								} else {
									SSP1BUF = PWM_MAX - (char)(0xFF & ((CCPR1L << 2) | CCP1CONbits.DC1B));
								}
								i2cCommand = COMMAND_NONE;
								readCommandPending = false;
								byteCount = 0;
							}
							break;
						case COMMAND_GET_EPO:					// EPO tripped flag yields one data byte
							if (epoTripped == true) {
								SSP1BUF = COMMAND_VALUE_ON;
							} else {
								SSP1BUF = COMMAND_VALUE_OFF;
							}
							i2cCommand = COMMAND_NONE;
							readCommandPending = false;
							break;
						case COMMAND_GET_EPO_IGNORE:			// EPO ignore flag yields one data byte
							if (epoIgnored == true) {
								SSP1BUF = COMMAND_VALUE_ON;
							} else {
								SSP1BUF = COMMAND_VALUE_OFF;
							}
							i2cCommand = COMMAND_NONE;
							readCommandPending = false;
							break;
						case COMMAND_GET_DRIVE_FAULT:			// Fault flag yields one data byte
							if (PORTAbits.RA2 == 1) {
								SSP1BUF = COMMAND_VALUE_OFF;
							} else {
								SSP1BUF = COMMAND_VALUE_ON;
							}
							i2cCommand = COMMAND_NONE;
							readCommandPending = false;
							break;
						case COMMAND_GET_FAILSAFE:				// Failsafe flag yields one data byte
							if (T4CONbits.TMR4ON == 0) {
								SSP1BUF = COMMAND_VALUE_OFF;
							} else {
								SSP1BUF = COMMAND_VALUE_ON;
							}
							i2cCommand = COMMAND_NONE;
							readCommandPending = false;
							break;
						case COMMAND_GET_ENC_MODE:				// Encoder mode flag yields one data byte
							if (INTCONbits.IOCIE == 1) {
								SSP1BUF = COMMAND_VALUE_ON;
							} else {
								SSP1BUF = COMMAND_VALUE_OFF;
							}
							i2cCommand = COMMAND_NONE;
							readCommandPending = false;
							break;
						case COMMAND_GET_ENC_MOVING:			// Encoder moving flag yields one data byte
							if ((movingA == true) || (movingB == true)) {
								SSP1BUF = COMMAND_VALUE_ON;
							} else {
								SSP1BUF = COMMAND_VALUE_OFF;
							}
							i2cCommand = COMMAND_NONE;
							readCommandPending = false;
							break;
						case COMMAND_GET_ENC_SPEED:				// Encoder speed yields one data byte
							SSP1BUF = encLimit;
							i2cCommand = COMMAND_NONE;
							readCommandPending = false;
							break;
						case COMMAND_GET_ID:					// I2C ID is one data byte
							SSP1BUF = I2C_ID_PICOBORG_REV;
							i2cCommand = COMMAND_NONE;
							readCommandPending = false;
							break;
						default:
							// No valid command given
							SSP1BUF = I2C_DATA_NONE;			// Send empty reply
							readCommandPending = false;			// Reset GET_... command pending flag
							byteCount = 0;						//	and byte counter
							break;
					}
				} else {										// I2C master wants to send data
					if (i2cCommand == COMMAND_NONE) {			// If i2cCommand not (yet) set,
						i2cCommand = SSP1BUF;					//	get it from the I2C master

						switch(i2cCommand) {					// For all GET_... commands...
							case COMMAND_GET_LED:
							case COMMAND_GET_A:
							case COMMAND_GET_B:
							case COMMAND_GET_EPO:
							case COMMAND_GET_EPO_IGNORE:
							case COMMAND_GET_DRIVE_FAULT:
							case COMMAND_GET_FAILSAFE:
							case COMMAND_GET_ENC_MODE:
							case COMMAND_GET_ENC_MOVING:
							case COMMAND_GET_ENC_SPEED:
							case COMMAND_GET_ID:
								readCommandPending = true;		//	... set the readCommandPending
																//	flag
																// This way the data to send back
																//	to the master survives a
																//	I2C STOP/START or RE-START
																//	sequence.
								break;
							default:							// For all other commands...
								readCommandPending = false;		//	... clear the readCommandPending
																//	flag
								break;
						}
						
					} else {									// Master wants to send and i2cCommand
																//	is already set.
																//	-> Must be a SET_... command.
						if (byteCount > I2C_MAX_LEN) {			// First, make sure byteCount does not
							byteCount = I2C_MAX_LEN;			//	surpass the maximum.
						}

						switch(i2cCommand) {
							/* A SET_... command has been initiated by the master.
							 *	Get the appropriate number of data byte(s) for the
							 *	command from the master.
							 * Once all necessary data has been received for a
							 *	command, set the I/O pins and/or registers accordingly.
							*/
							case COMMAND_SET_LED:				// Setting the LED on/off
																//	needs one data byte
								if (byteCount == 0) {			// Handle only first byte
									i2cRXData[0] = SSP1BUF;		// Get the byte from the master
									if (i2cRXData[0] == COMMAND_VALUE_OFF) {
										LATAbits.LATA4 = 1;		// Set LED off
									} else {
										LATAbits.LATA4 = 0;		// Set LED on
									}
									i2cCommand = COMMAND_NONE;	// Reset I2C command
								}
								if (SSP1STATbits.BF) {			// If there is excess data
																//	for byteCount > 0,
									junk = SSP1BUF;				//	just clear the buffer
								}
								++byteCount;					// Increase byteCount
								break;							// Done with this cycle
							case COMMAND_SET_A_FWD:				// Running motor A forward
																//	needs one data byte
								if (byteCount == 0) {
									i2cRXData[0] = SSP1BUF;		// Read 'speed' byte from
																//	master and run motor A
																//	forward at desired speed
									SetMotorA(false, (int)i2cRXData[0]);
									i2cCommand = COMMAND_NONE;	// Reset, clear buffer, ...
								}								// (cf. COMMAND_SET_LED)
								if (SSP1STATbits.BF) {
									junk = SSP1BUF;
								}
								++byteCount;
								break;
							case COMMAND_SET_A_REV:				// Running motor A in reverse
																//	needs one data byte
								if (byteCount == 0) {
									i2cRXData[0] = SSP1BUF;		// Run motor A in reverse
																//	at desired speed
									SetMotorA(true, (int)i2cRXData[0]);
									i2cCommand = COMMAND_NONE;	// Reset, clear buffer, ...
								}
								if (SSP1STATbits.BF) {
									junk = SSP1BUF;
								}
								++byteCount;
								break;
							case COMMAND_SET_B_FWD:				// Running motor B forward
																//	needs one data byte
								if (byteCount == 0) {
									i2cRXData[0] = SSP1BUF;		// Run motor B forward
																//	at desired speed
									SetMotorB(false, (int)i2cRXData[0]);
									i2cCommand = COMMAND_NONE;	// Reset, clear buffer, ...
								}	
								if (SSP1STATbits.BF) {
									junk = SSP1BUF;
								}
								++byteCount;
								break;
							case COMMAND_SET_B_REV:				// Running motor B in reverse
																//	needs one data byte
								if (byteCount == 0) {
									i2cRXData[0] = SSP1BUF;		// Run motor B in reverse
																//	at desired speed
									SetMotorB(true, (int)i2cRXData[0]);
									i2cCommand = COMMAND_NONE;	// Reset, clear buffer, ...
								}
								if (SSP1STATbits.BF) {
									junk = SSP1BUF;
								}
								++byteCount;
								break;
							case COMMAND_ALL_OFF:				// Turning everything off
																//	needs one data byte
								if (byteCount == 0) {
									SetAllMotors(false, 0);		// Motors off
									LATAbits.LATA4 = 1;			// LED off
									movingA = false;			// Reset 'moving' flags
									movingB = false;
									i2cCommand = COMMAND_NONE;	// Reset, clear buffer, ...
								}
								if (SSP1STATbits.BF) {
									junk = SSP1BUF;
								}
								++byteCount;
								break;
							case COMMAND_RESET_EPO:				// Resetting the EPO switch
																//	needs one data byte
								if (byteCount == 0) {
									epoTripped = false;			// Reset EPO tripped flag
									i2cCommand = COMMAND_NONE;	// Reset, clear buffer, ...
								}
								if (SSP1STATbits.BF) {
									junk = SSP1BUF;
								}
								++byteCount;
								break;
							case COMMAND_SET_EPO_IGNORE:		// Setting the EPO ignore flag
																//	needs one data byte
								if (byteCount == 0) {
									i2cRXData[0] = SSP1BUF;
									if (i2cRXData[0] == COMMAND_VALUE_OFF) {
										epoIgnored = false;		// Do not ignore the EPO
									} else {
										epoIgnored = true;		// Ignore the EPO
									}
									i2cCommand = COMMAND_NONE;	// Reset, clear buffer, ...
								}
								if (SSP1STATbits.BF) {
									junk = SSP1BUF;
								}
								++byteCount;
								break;
							case COMMAND_SET_ALL_FWD:			// Running both motors forward
																//	needs one data byte
								if (byteCount == 0) {
									i2cRXData[0] = SSP1BUF;		// Run both motors forward
																//	at desired speed
									SetAllMotors(false, (int)i2cRXData[0]);
									i2cCommand = COMMAND_NONE;	// Reset, clear buffer, ...
								}
								if (SSP1STATbits.BF) {
									junk = SSP1BUF;
								}
								++byteCount;
								break;
							case COMMAND_SET_ALL_REV:			// Running both motors in
																//	reverse needs one data byte
								if (byteCount == 0) {
									i2cRXData[0] = SSP1BUF;		// Run both motors in reverse
																//	at desired speed
									SetAllMotors(true, (int)i2cRXData[0]);
									i2cCommand = COMMAND_NONE;	// Reset, clear buffer, ...
								}
								if (SSP1STATbits.BF) {
									junk = SSP1BUF;
								}
								++byteCount;
								break;
							case COMMAND_SET_FAILSAFE:			// Setting the fail safe flag
																//	needs one data byte
								if (byteCount == 0) {
									i2cRXData[0] = SSP1BUF;
									if (i2cRXData[0] == COMMAND_VALUE_OFF) {
										T4CONbits.TMR4ON = 0;	// Disable timer 4
																//	(fail safe timer)
									} else {
										T4CONbits.TMR4ON = 1;	// Enable timer 4
									}
									PIR3bits.TMR4IF = 0;		// Clear timer 4 interrupt flag
									i2cCommand = COMMAND_NONE;	// Reset, clear buffer, ...
								}
								if (SSP1STATbits.BF) {
									junk = SSP1BUF;
								}
								++byteCount;
								break;
							case COMMAND_SET_ENC_MODE:			// Turning encoder mode on/off
																//	needs one data byte
								if (byteCount == 0) {
									i2cRXData[0] = SSP1BUF;
									if (i2cRXData[0] == COMMAND_VALUE_OFF) {
										SetEncoderMode(false);	// Turn encoder mode off
									} else {
										SetEncoderMode(true);	// Turn encoder mode on
									}
									i2cCommand = COMMAND_NONE;	// Reset, clear buffer, ...
								}
								if (SSP1STATbits.BF) {
									junk = SSP1BUF;
								}
								++byteCount;
								break;
							case COMMAND_MOVE_A_FWD:			// Moving motor A forward
																//  by a given number of
																//	encoder steps
																//	needs two data bytes
								if (byteCount < I2C_MAX_LEN) {	// While less then I2C_MAX_LEN
																//	bytes have been received...
									i2cRXData[byteCount] = SSP1BUF;	// ... read one byte per
																	//	cycle from the master
																	//	and store it in the
																	//	rx array
									if (byteCount == 1) {		// As soon as the second byte
																//	has been received
																//	move motor A forward by
																//	'word' encoder ticks,
																//	where 'word' is composed of
																//	the two bytes received earlier
										MoveMotorA(false, ((int)i2cRXData[0] << 8) + (int)i2cRXData[1]);
																// Afterwards, reset, clear buffer, ...
										i2cCommand = COMMAND_NONE;
									}
								}
								if (SSP1STATbits.BF) {
									junk = SSP1BUF;
								}
								++byteCount;
								break;
							case COMMAND_MOVE_A_REV:			// Moving motor A in reverse
																//  by a given number of
																//	encoder steps
																//	needs two data bytes
								if (byteCount < I2C_MAX_LEN) {	// Read two bytes from the
																//	master
									i2cRXData[byteCount] = SSP1BUF;
									if (byteCount == 1) {
																// Move motor A in reverse
																//	by the given number of
																//	encoder ticks
																//	(cf. COMMAND_MOVE_A_FWD)
										MoveMotorA(true, ((int)i2cRXData[0] << 8) + (int)i2cRXData[1]);
																// Reset, clear buffer, ...
										i2cCommand = COMMAND_NONE;
									}
								}
								if (SSP1STATbits.BF) {
									junk = SSP1BUF;
								}
								++byteCount;
								break;
							case COMMAND_MOVE_B_FWD:			// Moving motor B forward
																//  by a given number of
																//	encoder steps
																//	needs two data bytes
								if (byteCount < I2C_MAX_LEN) {	// Read two bytes from the
																//	master
									i2cRXData[byteCount] = SSP1BUF;
									if (byteCount == 1) {
																// Move motor B forward
																//	by the given number of
																//	encoder ticks
										MoveMotorB(false, ((int)i2cRXData[0] << 8) + (int)i2cRXData[1]);
																// Reset, clear buffer, ...
										i2cCommand = COMMAND_NONE;
									}
								}
								if (SSP1STATbits.BF) {
									junk = SSP1BUF;
								}
								++byteCount;
								break;
							case COMMAND_MOVE_B_REV:			// Moving motor B in reverse
																//  by a given number of
																//	encoder steps
																//	needs two data bytes
								if (byteCount < I2C_MAX_LEN) {	// Read two bytes from the
																//	master
									i2cRXData[byteCount] = SSP1BUF;
									if (byteCount == 1) {
																// Move motor B in reverse
																//	by the given number of
																//	encoder ticks
										MoveMotorB(true, ((int)i2cRXData[0] << 8) + (int)i2cRXData[1]);
																// Reset, clear buffer, ...
										i2cCommand = COMMAND_NONE;
									}
								}
								if (SSP1STATbits.BF) {
									junk = SSP1BUF;
								}
								++byteCount;
								break;
							case COMMAND_MOVE_ALL_FWD:			// Moving both motors forward
																//  by a given number of
																//	encoder steps
																//	needs two data bytes
								if (byteCount < I2C_MAX_LEN) {	// Read two bytes from the
																//	master
									i2cRXData[byteCount] = SSP1BUF;
									if (byteCount == 1) {
																// Move both motors forward
																//	by the given number of
																//	encoder ticks
										MoveMotorA(false, ((int)i2cRXData[0] << 8) + (int)i2cRXData[1]);
										MoveMotorB(false, ((int)i2cRXData[0] << 8) + (int)i2cRXData[1]);
																// Reset, clear buffer, ...
										i2cCommand = COMMAND_NONE;
									}
								}
								if (SSP1STATbits.BF) {
									junk = SSP1BUF;
								}
								++byteCount;
								break;
							case COMMAND_MOVE_ALL_REV:			// Moving both motors in reverse
																//  by a given number of
																//	encoder steps
																//	needs two data bytes
								if (byteCount < I2C_MAX_LEN) {	// Read two bytes from the
																//	master
									i2cRXData[byteCount] = SSP1BUF;
									if (byteCount == 1) {
																// Move both motors in reverse
																//	by the given number of
																//	encoder ticks
										MoveMotorA(true, ((int)i2cRXData[0] << 8) + (int)i2cRXData[1]);
										MoveMotorB(true, ((int)i2cRXData[0] << 8) + (int)i2cRXData[1]);
																// Reset, clear buffer, ...
										i2cCommand = COMMAND_NONE;
									}
								}
								if (SSP1STATbits.BF) {
									junk = SSP1BUF;
								}
								++byteCount;
								break;
							case COMMAND_SET_ENC_SPEED:			// Setting encoder speed
																//	needs one data byte
								if (byteCount == 0) {
									encLimit = SSP1BUF;			// Set encoder speed
									if ((movingA == true) && (remainingCountsA > 0)) {
										SetMotorA(reverseA, encLimit);
									}
									if ((movingB == true) && (remainingCountsB > 0)) {
										SetMotorB(reverseB, encLimit);
									}
									i2cCommand = COMMAND_NONE;	// Reset, clear buffer, ...
								}
								if (SSP1STATbits.BF) {
									junk = SSP1BUF;
								}
								++byteCount;
								break;
							case COMMAND_SET_I2C_ADD:			// Setting a new I2C address
																//	needs one data byte
								if (byteCount == 0) {
									i2cRXData[0] = SSP1BUF;		// Get new address from master
									if ((i2cRXData[0] < 0x03) || (i2cRXData[0] > 0x77)) {
										/* New address is from reserved
										 * address space. Ignore it.
										 */
										i2cCommand = COMMAND_NONE;	// Reset, clear buffer, ...
										++byteCount;
										break;
									}
									SSP1ADD = i2cRXData[0] << 1;	// Set the live I2C address
									PIR2bits.EEIF = 0;				// Save the I2C address to EEPROM
									EECON1bits.WREN = 1;
									EEADRL = EEPROM_I2C_ADDRESS;
									EEDATL = i2cRXData[0];
									EECON2 = 0x55;
									EECON2 = 0xAA;
									EECON1bits.WR = 1;
									while (PIR2bits.EEIF == 0);
									PIR2bits.EEIF = 0;
									EECON1bits.WREN = 0;
									i2cCommand = COMMAND_NONE;	// Reset, clear buffer, ...
								}
								if (SSP1STATbits.BF) {
									junk = SSP1BUF;
								}
								++byteCount;
								break;
							default:							// No valid command received
								if (SSP1STATbits.BF) {			// Just clear the buffer
									junk = SSP1BUF;				//	if necessary
								}
								break;
						}
					}
				}
			}
		}
		SSP1CON1bits.CKP = 1;				// Release clock line
											//	(clock stretching ends)
	}

	/* Encoder interrupt processing */

	if (INTCONbits.IOCIF) {					// Pin change event occured?
		INTCONbits.IOCIF = 0;				// Clear the pin change interrupt
		if (IOCAFbits.IOCAF0) {				// Encoder for motor B moved one tick
			IOCAFbits.IOCAF0 = 0;			// Clear the encoder interrupt
			--remainingCountsB;				// Reduce remaining ticks for motor B
		}
		if (IOCAFbits.IOCAF1) {				// Encoder for motor A moved one tick
			IOCAFbits.IOCAF1 = 0;			// Clear the encoder interrupt
			--remainingCountsA;				// Reduce remaining ticks for motor A
		}
	}
}


/********************************************************************************/
/* Main Program																	*/
/********************************************************************************/

/* Main program */
void main(void) {
	// Configure the PIC's oscillator
	ConfigureOscillator();

	// Initialize I/O and peripherals for I2C mode
	InitApp();

	// 0.5 second LED pulse to say 'Hello'
	LATAbits.LATA4 = 0;					// LED On
	Delay_ms(500);
	LATAbits.LATA4 = 1;					// LED Off

	// Clear the I2C buffer
	do {
		PIR1bits.SSP1IF = 0;			// Clear the interrupt
		junk = SSP1BUF;					// Discard the transmitted byte
		SSP1CON1bits.CKP = 1;			// Release clock line
	} while (SSP1STATbits.BF);			// While there is a byte in the I2C buffer
	
	// Main loop
	INTCONbits.GIE = 1;					// Enable interrupts
	failsafeCounter = 0;				// Reset failsafe counter

	while (1) {							// Loop forever

		// Check the communications failsafe
		if (PIR3bits.TMR4IF) {
			PIR3bits.TMR4IF = 0;		// Clear the interrupt

			++failsafeCounter;			// Increment the counter
			if (failsafeCounter > FAILSAFE_LIMIT) {
				// Failsafe timeout exceeded, presume that communications have failed
				SetAllMotors(false, 0);
				// Invert the LED
				if (PORTAbits.RA4 == 1) {
					LATAbits.LATA4 = 0;
				} else {
					LATAbits.LATA4 = 1;
				}
				// Reset the counter
				failsafeCounter = 0;
			}
		}

		// Run the encoder loop
		if (INTCONbits.IOCIE == 1) {			// If encoder mode is enabled
			if (movingA && (remainingCountsA <= 0)) {
				SetMotorA(false, 0);	// Stop motor A if encoder ticks count reached
				movingA = false;
			}
			if (movingB && (remainingCountsB <= 0)) {
				SetMotorB(false, 0);	// Stop motor B if encoder ticks count reached
				movingB = false;
			}
		}

		if (epoIgnored == false) {		// If the EPO is not ignored
			// Check if the EPO has been tripped
			if (!epoTripped && (PORTAbits.RA5 == 1)) {
				epoTripped = true;
				SetMotorA(false , 0);	// Immediately stop motor A
				SetMotorB(false , 0);	//	and B
				movingA = false;
				movingB = false;
			}
		}

		// Kick the watchdog
		ClrWdt();
	}
}
