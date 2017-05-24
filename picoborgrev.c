// PIC code for the PicoBorg Reverse advanced robot controller
// https://www.piborg.org/picoborgrev
//
// Rewritten for sdcc by Torsten Kurbad <beaglebone@tk-webart.de>


// Avoid polluting the global namespace with aliases for each pin.
#define NO_BIT_DEFINES

// PicoBorg Reverse uses a PIC16F1824 for I2C communication
#include "pic14/pic16regs.h"
#include "pic14/pic16f1824.h"

#include "picoborgrev.h"

// Set the configuration words:
__code unsigned short __at (_CONFIG1) configWord1 = (
    _FOSC_INTOSC &
    _WDTE_OFF &
    _PWRTE_ON &
    _MCLRE_ON &
    _CP_OFF &
    _CPD_OFF &
    _BOREN_ON &
    _CLKOUTEN_OFF &
    _IESO_OFF &
    _FCMEN_ON
    );

__code unsigned short __at (_CONFIG2) configWord2 = (
    _WRT_OFF &
    _PLLEN_ON &
    _STVREN_ON &
    _BORV_LO &
    _DEBUG_OFF &
    _LVP_OFF
    );

/**********************************************************************/
/* System Functions                                                   */
/**********************************************************************/

void ConfigureOscillator(void) {
    OSCCONbits.SCS = 0b0;	    // Set the clock to the CONFIG1 setting (internal oscillator)
    OSCCONbits.IRCF = 0b1110;	    // Select the 8 MHz postscaler
    OSCCONbits.SPLLEN = 1;	    // Enable the 4x PLL (overriden in CONFIG2 by PLLEN = ON)
    OSCTUNEbits.TUN = 0b011111; // Set oscillator to max frequency
}

void Delay_ms(unsigned short ms) {
    int u;
    while (ms--) {
        for(u = 0; u < LOOPS_PER_MS; u++) {
            Nop();
        }
    }
}


/**********************************************************************/
/* User Global Variable Declaration                                   */
/**********************************************************************/

unsigned char i2cSend[I2C_MAX_LEN] = {0, 0, 0, 0};
unsigned char i2cRecv[I2C_MAX_LEN] = {0};
int i2cByte = 0;
char junk = 0x00;

unsigned char i2cAddress = 0x00;
unsigned char i2cCommand = 0x00;
unsigned char i2cRXData[I2C_MAX_LEN] = {0};
unsigned char i2cTXData[I2C_MAX_LEN] = {0};
int dataByte = 0;

bool epoTripped = false;
bool epoIgnored = false;
int failsafeCounter = 0;
bool encMode = false;
bool movingA = false;
bool movingB = false;
int encLimit = 255;
int remainingCountsA = 0;
int remainingCountsB = 0;

/**********************************************************************/
/* User Functions                                                     */
/**********************************************************************/

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
	CM1CON0bits.C1ON = 0;			// Comparitor 1 disabled
	CM1CON0bits.C1OE = 0;			// Comparitor 1 output disabled
	CM2CON0bits.C2ON = 0;			// Comparitor 2 disabled
	CM2CON0bits.C2OE = 0;			// Comparitor 2 output disabled
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

void SetMotorA(bool reverse, int pwm) {
	// Check if the EPO has been tripped
	if (epoTripped && !epoIgnored) {
		reverse = false;
		pwm = 0;
	}
	// Work out the required settings
	if (pwm > PWM_MAX) pwm = PWM_MAX;
	if (reverse) {
		pwm = PWM_MAX - pwm;			// PWM pulse width inverted for reverse
		LATCbits.LATC2 = 1;				// Motor A:2 On
	} else {
		// PWM pulse width is not inverted for forward
		LATCbits.LATC2 = 0;				// Motor A:2 Off
	}
	// Set Motor A:1 to the pulse width
	CCPR2L = pwm >> 2;				// Set PWM 2 MSBs
	CCP2CONbits.DC2B = pwm & 0b11;	// Set PWM 2 LSBs
}

void SetMotorB(bool reverse, int pwm) {
	// Check if the EPO has been tripped
	if (epoTripped && !epoIgnored) {
		reverse = false;
		pwm = 0;
	}
	// Work out the required settings
	if (pwm > PWM_MAX) pwm = PWM_MAX;
	if (reverse) {
		pwm = PWM_MAX - pwm;			// PWM pulse width inverted for reverse
		LATCbits.LATC4 = 1;				// Motor B:2 On
	} else {
		// PWM pulse width is not inverted for forward
		LATCbits.LATC4 = 0;				// Motor B:2 Off
	}
	// Set Motor B:1 to the pulse width
	CCPR1L = pwm >> 2;				// Set PWM 1 MSBs
	CCP1CONbits.DC1B = pwm & 0b11;	// Set PWM 1 LSBs
}

void SetAllMotors(bool reverse, int pwm) {
	// Check if the EPO has been tripped
	if (epoTripped && !epoIgnored) {
		reverse = false;
		pwm = 0;
	}
	// Work out the required settings
	if (pwm > PWM_MAX) pwm = PWM_MAX;
	if (reverse) {
		pwm = PWM_MAX - pwm;			// PWM pulse width inverted for reverse
		LATC = PORTC | (_LATC2 | _LATC4);	// Motor A:2 and B:2 On
	} else {
		// PWM pulse width is not inverted for forward
		LATC = PORTC & ~(_LATC2 | _LATC4);	// Motor A:2 and B:2 On
	}
	// Wait until it is safe to change the pulse widths
	PIR1bits.TMR2IF = 0;			// Clear Timer 2 interrupt flag
	while (PIR1bits.TMR2IF == 0) ;	// Wait for Timer 2 to loop
	// Set Motor A:1 and B:1 to the pulse width
	CCPR2L = pwm >> 2;				// Set PWM 2 MSBs
	CCPR1L = pwm >> 2;				// Set PWM 1 MSBs
	CCP2CONbits.DC2B = pwm & 0b11;	// Set PWM 2 LSBs
	CCP1CONbits.DC1B = pwm & 0b11;	// Set PWM 1 LSBs
}

void SetEncoderMode(bool enabled) {
	// Disable any automatic routines
	encMode = false;
	movingA = false;
	movingB = false;

	// Turn off both motors
	SetAllMotors(false, 0);

	// Set the correct mode
	encMode = enabled;
	if (encMode) {
		INTCONbits.IOCIE = 1;			// Interrupts on change enabled
	} else {
		INTCONbits.IOCIE = 0;			// Interrupts on change disabled
	}
}

void MoveMotorA(bool reverse, int count) {
	// Check we are in encoder mode before starting
	if (!encMode) {
		return;
	}

	// Set the distance to cover
	remainingCountsA = count;

	// Start moving
	SetMotorA(reverse, encLimit);

	// Set the moving flag
	movingA = true;
}

void MoveMotorB(bool reverse, int count) {
	// Check we are in encoder mode before starting
	if (!encMode) {
		return;
	}

	// Set the distance to cover
	remainingCountsB = count;

	// Start moving
	SetMotorB(reverse, encLimit);
	
	// Set the moving flag
	movingB = true;
}

/*
void ProcessI2C(int len) {
	int i;
	bool echo = false;

	// Clear the reply buffer
	for (i = 0; i < I2C_MAX_LEN; ++i) {
		i2cSend[i] = 0;
	}

	if (len < 2) return;

	if (i2cRecv[0] == 0x00) {
		// Broadcast message, only used for setting up the I2C address
		if ((i2cRecv[1] != COMMAND_SET_I2C_ADD) &&
			(i2cRecv[1] != COMMAND_GET_ID)) {
			return;
		}
	}

	switch (i2cRecv[1]) {
		case COMMAND_SET_LED:
			if (len < 3) break;
			if (i2cRecv[2] == COMMAND_VALUE_OFF) {
				LATAbits.LATA4 = 1;		// LED Off
			} else {
				LATAbits.LATA4 = 0;		// LED On
			}
			echo = true; len = 3;
			break;
		case COMMAND_SET_A_FWD:
			if (len < 3) break;
			SetMotorA(false, (int)i2cRecv[2]);
			echo = true; len = 3;
			break;
		case COMMAND_SET_A_REV:
			if (len < 3) break;
			SetMotorA(true, (int)i2cRecv[2]);
			echo = true; len = 3;
			break;
        case COMMAND_GET_A:
			i2cSend[0] = COMMAND_GET_A;
			if (PORTCbits.RC2 == 0) {
				i2cSend[1] = COMMAND_VALUE_FWD;
				i2cSend[2] = (char)(0xFF & ((CCPR2L << 2) | CCP2CONbits.DC2B));
			} else {
				i2cSend[1] = COMMAND_VALUE_REV;
				i2cSend[2] = PWM_MAX - (char)(0xFF & ((CCPR2L << 2) | CCP2CONbits.DC2B));
			}
			break;
		case COMMAND_SET_B_FWD:
			if (len < 3) break;
			SetMotorB(false, (int)i2cRecv[2]);
			echo = true; len = 3;
			break;
		case COMMAND_SET_B_REV:
			if (len < 3) break;
			SetMotorB(true, (int)i2cRecv[2]);
			echo = true; len = 3;
			break;
		case COMMAND_GET_B:
			i2cSend[0] = COMMAND_GET_B;
			if (PORTCbits.RC4 == 0) {
				i2cSend[1] = COMMAND_VALUE_FWD;
				i2cSend[2] = (char)(0xFF & ((CCPR1L << 2) | CCP1CONbits.DC1B));
			} else {
				i2cSend[1] = COMMAND_VALUE_REV;
				i2cSend[2] = PWM_MAX - (char)(0xFF & ((CCPR1L << 2) | CCP1CONbits.DC1B));
			}
			break;
		case COMMAND_ALL_OFF:
			SetAllMotors(false, 0);
			LATAbits.LATA4 = 1;		// LED Off
			movingA = false;
			movingB = false;
			echo = true; len = 3;
			break;
		case COMMAND_RESET_EPO:
			epoTripped = false;
			echo = true;
			break;
		case COMMAND_GET_EPO:
			i2cSend[0] = COMMAND_GET_EPO;
			if (epoTripped) {
				i2cSend[1] = COMMAND_VALUE_ON;
			} else {
				i2cSend[1] = COMMAND_VALUE_OFF;
			}
			break;
		case COMMAND_SET_EPO_IGNORE:
			if (len < 3) break;
			if (i2cRecv[2] == COMMAND_VALUE_OFF) {
				epoIgnored = false;
			} else {
				epoIgnored = true;
			}
			echo = true; len = 3;
			break;
		case COMMAND_GET_EPO_IGNORE:
			i2cSend[0] = COMMAND_GET_EPO_IGNORE;
			if (epoIgnored) {
				i2cSend[1] = COMMAND_VALUE_ON;
			} else {
				i2cSend[1] = COMMAND_VALUE_OFF;
			}
			break;
		case COMMAND_GET_DRIVE_FAULT:
			i2cSend[0] = COMMAND_GET_DRIVE_FAULT;
			if (PORTAbits.RA2 == 1) {
				i2cSend[1] = COMMAND_VALUE_OFF;
			} else {
				i2cSend[1] = COMMAND_VALUE_ON;
			}
			break;
		case COMMAND_SET_ALL_FWD:
			if (len < 3) break;
			SetAllMotors(false, (int)i2cRecv[2]);
			echo = true; len = 3;
			break;
		case COMMAND_SET_ALL_REV:
			if (len < 3) break;
			SetAllMotors(true, (int)i2cRecv[2]);
			echo = true; len = 3;
			break;
		case COMMAND_SET_FAILSAFE:
			if (len < 3) break;
			if (i2cRecv[2] == COMMAND_VALUE_OFF) {
				T4CONbits.TMR4ON = 0;			// Timer 4 disabled
			} else {
				T4CONbits.TMR4ON = 1;			// Timer 4 enabled
			}
			PIR3bits.TMR4IF = 0;			// Clear Timer 4 interrupt flag
			echo = true; len = 3;
			break;
		case COMMAND_GET_FAILSAFE:
			i2cSend[0] = COMMAND_GET_FAILSAFE;
			if (T4CONbits.TMR4ON == 0) {
				i2cSend[1] = COMMAND_VALUE_OFF;
			} else {
				i2cSend[1] = COMMAND_VALUE_ON;
			}
			break;
		case COMMAND_SET_ENC_MODE:
			if (len < 3) break;
			if (i2cRecv[2] == COMMAND_VALUE_OFF) {
				SetEncoderMode(false);
			} else {
				SetEncoderMode(true);
			}
			echo = true; len = 3;
			break;
		case COMMAND_GET_ENC_MODE:
			i2cSend[0] = COMMAND_GET_ENC_MODE;
			if (encMode) {
				i2cSend[1] = COMMAND_VALUE_ON;
			} else {
				i2cSend[1] = COMMAND_VALUE_OFF;
			}
			break;
		case COMMAND_MOVE_A_FWD:
			if (len < 4) break;
			MoveMotorA(false, ((int)i2cRecv[2] << 8) + (int)i2cRecv[3]);
			echo = true; len = 4;
			break;
		case COMMAND_MOVE_A_REV:
			if (len < 4) break;
			MoveMotorA(true, ((int)i2cRecv[2] << 8) + (int)i2cRecv[3]);
			echo = true; len = 4;
			break;
		case COMMAND_MOVE_B_FWD:
			if (len < 4) break;
			MoveMotorB(false, ((int)i2cRecv[2] << 8) + (int)i2cRecv[3]);
			echo = true; len = 4;
			break;
		case COMMAND_MOVE_B_REV:
			if (len < 4) break;
			MoveMotorB(true, ((int)i2cRecv[2] << 8) + (int)i2cRecv[3]);
			echo = true; len = 4;
			break;
		case COMMAND_MOVE_ALL_FWD:
			if (len < 4) break;
			MoveMotorA(false, ((int)i2cRecv[2] << 8) + (int)i2cRecv[3]);
			MoveMotorB(false, ((int)i2cRecv[2] << 8) + (int)i2cRecv[3]);
			echo = true; len = 4;
			break;
		case COMMAND_MOVE_ALL_REV:
			if (len < 4) break;
			MoveMotorA(true, ((int)i2cRecv[2] << 8) + (int)i2cRecv[3]);
			MoveMotorB(true, ((int)i2cRecv[2] << 8) + (int)i2cRecv[3]);
			echo = true; len = 4;
			break;
		case COMMAND_GET_ENC_MOVING:
			i2cSend[0] = COMMAND_GET_ENC_MOVING;
			if (movingA || movingB) {
				i2cSend[1] = COMMAND_VALUE_ON;
			} else {
				i2cSend[1] = COMMAND_VALUE_OFF;
			}
			break;
		case COMMAND_SET_ENC_SPEED:
			if (len < 3) break;
            encLimit = i2cRecv[2];
			echo = true; len = 3;
			break;
		case COMMAND_GET_ENC_SPEED:
			i2cSend[0] = COMMAND_GET_ENC_SPEED;
			i2cSend[1] = encLimit;
			break;
		case COMMAND_GET_ID:
            i2cSend[0] = COMMAND_GET_ID;
            i2cSend[1] = I2C_ID_PICOBORG_REV;
			break;
		case COMMAND_SET_I2C_ADD:
			// Set the live I2C address
			SSP1ADD = i2cRecv[2] << 1;
			// Save the I2C address to EEPROM
			PIR2bits.EEIF = 0;
			EECON1bits.WREN = 1;
			EEADRL = EEPROM_I2C_ADDRESS;
			EEDATL = i2cRecv[2];
			EECON2 = 0x55;
			EECON2 = 0xAA;
			EECON1bits.WR = 1;
			while (PIR2bits.EEIF == 0) ;
			PIR2bits.EEIF = 0;
			EECON1bits.WREN = 0;
			echo = true; len = 3;
			break;
        default:
			// Do nothing
			break;
	}

	// Echo sent command if required
	if (echo) {
		for (i = 1; i < len; ++i) {
			i2cSend[i - 1] = i2cRecv[i];
		}
	}
}
*/

/**********************************************************************/
/* Interrupt Service Routine                                          */
/**********************************************************************/

void isr_i2c(void) __interrupt 0 {
    // I2C event occured?
    if (PIR1bits.SSP1IF) {
        PIR1bits.SSP1IF = 0;
        SSP1CON1bits.CKP = 0;
        if (SSP1STATbits.P) {
            // Stop
            i2cCommand = COMMAND_NONE;
        }
        if (!SSP1STATbits.D_NOT_A) {
            if (SSP1STATbits.R_NOT_W) {
                // Master wants to read
                switch(i2cCommand) {
                    case COMMAND_GET_LED:
                        if (PORTAbits.RA4 == 1) {
                            SSP1BUF = COMMAND_VALUE_OFF;
                        } else {
                            SSP1BUF = COMMAND_VALUE_ON;
                        }
                        SSP1CON1bits.CKP = 1;
                        break;
                    case COMMAND_GET_A:
                        if (PORTCbits.RC2 == 0) {
                            SSP1BUF = COMMAND_VALUE_FWD;
                        } else {
                            SSP1BUF = COMMAND_VALUE_REV;
                        }
                        break;
                    case COMMAND_GET_ID:
                        SSP1BUF = I2C_ID_PICOBORG_REV;
                        SSP1CON1bits.CKP = 1;
                        break;
                    default:
                        SSP1BUF = 0x00;
                        SSP1CON1bits.CKP = 1;
                        break;
                }
            } else {
                // Master wants to write
                i2cAddress = SSP1BUF;
                SSP1CON1bits.CKP = 1;
            }
        } else {
            if (SSP1STATbits.R_NOT_W) {
                // Master wants to read

                // Send possible 2nd byte for read command
                switch(i2cCommand) {
                    case COMMAND_GET_LED:
                        break;
                    case COMMAND_GET_A:
                        if (PORTCbits.RC2 == 0) {
                            SSP1BUF = (char)(0xFF & ((CCPR2L << 2) | CCP2CONbits.DC2B));
                        } else {
                            SSP1BUF = PWM_MAX - (char)(0xFF & ((CCPR2L << 2) | CCP2CONbits.DC2B));
                        }
                        break;
                    case COMMAND_GET_ID:
                        break;
                    default:
                        break;
                }
                SSP1CON1bits.CKP = 1;
            } else {
                // Master wants to write
                if (i2cCommand == COMMAND_NONE) {
                    i2cCommand = SSP1BUF;                    
                }
                SSP1CON1bits.CKP = 1;
            }
        }
    }
}

/*
void isr_i2c(void) __interrupt 0 {

	// Check for I2C events
	if (PIR1bits.SSP1IF) {
        PIR1bits.SSP1IF = 0;

		// Analyse interrupt
		if (SSP1STATbits.P) {
			// Stop condition, process command
			if (SSP1STATbits.R_NOT_W) {
				// Master reading mode, do nothing
            } else {
				// Master writing mode, analyse the command
				if (i2cByte > I2C_MAX_LEN) i2cByte = I2C_MAX_LEN;
                ProcessI2C(i2cByte);
			}
			i2cByte = 0;
			failsafeCounter = 0;
		} else if (i2cByte < I2C_MAX_LEN) {
			// Within length limit
			if (SSP1STATbits.R_NOT_W) {
				// Master reading mode, forward the data bytes
                SSP1BUF = i2cSend[i2cByte];
            } else {
				// Master writing mode, read the transmitted byte
				i2cRecv[i2cByte] = SSP1BUF;
			}
			++i2cByte;
		} else {
			// Outside length limit
			if (SSP1STATbits.R_NOT_W) {
				// Master reading mode, send a junk symbol
				SSP1BUF = 0xcc;
			} else {
				// Master writing mode, discard the transmitted byte
				junk = SSP1BUF;
			}
		}

		SSP1CON1bits.CKP = 1;			// Release clock line
	}

	// Check for pin change events
	if (INTCONbits.IOCIF) {
		INTCONbits.IOCIF = 0;			// Clear the interrupt

		if (IOCAFbits.IOCAF0) {
			// Encoder for motor B
			IOCAFbits.IOCAF0 = 0;			// Clear the interrupt
			--remainingCountsB;
		}

		if (IOCAFbits.IOCAF1) {
			// Encoder for motor A
			IOCAFbits.IOCAF1 = 0;			// Clear the interrupt
			--remainingCountsA;
		}
	}
}
*/

/**********************************************************************/
/* Main Program                                                       */
/**********************************************************************/

void main(void) {
	// Configure the oscillator for the device
	ConfigureOscillator();

	// Initialize I/O and Peripherals for application
	InitApp();

	// Brief LED pulse
	LATAbits.LATA4 = 0;		// LED On
	Delay_ms(500);
	LATAbits.LATA4 = 1;		// LED Off
	
	// Clear the I2C buffer
	do {
		PIR1bits.SSP1IF = 0;			// Clear the interrupt
		junk = SSP1BUF;					// Discard the transmitted byte
		SSP1CON1bits.CKP = 1;			// Release clock line
	} while (SSP1STATbits.BF);
	
	// Main loop
	INTCONbits.GIE = 1;					// Enable interrupts
	failsafeCounter = 0;
	while (1) {
		// Check the communications failsafe
		if (PIR3bits.TMR4IF) {
			PIR3bits.TMR4IF = 0;			// Clear the interrupt

			++failsafeCounter;				// Increment the counter
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
		if (encMode) {
			if (movingA && (remainingCountsA <= 0)) {
				SetMotorA(false, 0);
				movingA = false;
			}
			if (movingB && (remainingCountsB <= 0)) {
				SetMotorB(false, 0);
				movingB = false;
			}
		}

		if (!epoIgnored) {
			// Check if the EPO has been tripped
			if (!epoTripped && (PORTAbits.RA5 == 1)) {
				epoTripped = true;
				SetMotorA(false , 0);
				SetMotorB(false , 0);
				movingA = false;
				movingB = false;
			}
		}

		// Kick the watchdog
		ClrWdt();
	}
}
