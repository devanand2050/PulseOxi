/*******************************************************************

              Microchip Pulse Oximeter Demo Main Code

********************************************************************
 FileName:        Main.c
 Description:     Pulse Oximeter Demo Main Code
 Processor:       dsPIC33FJ128GP802
 Compiler:        Microchip MPLAB C30 Compiler v3.30
 Company:         Microchip Technology Inc.
 Author:          Zhang Feng, Medical Products Group

 Software License Agreement

 The software supplied herewith by Microchip Technology Incorporated
 (the "Company") is intended and supplied to you, the Company's
 customer, for use solely and exclusively with products manufactured
 by the Company.

 The software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 MEDICAL DEMO WARNINGS, RESTRICTIONS AND DISCLAIMER

 This demo is intended solely for evaluation and development purposes.
 It is not intended for medical diagnostic use.

********************************************************************
 Change History:

 Author       Rev   Date         Description
 Zhang Feng   1.0   10/05/2012   Initial Release, for Schematics Rev 1.5


********************************************************************
 Additional Note:

 20MIPS clock

*******************************************************************/


//*****************************************************************************
// Include and Header files
//*****************************************************************************
#include "p33Fxxxx.h"
#include "dsp.h"
#include "math.h"
#include "GenericTypeDefs.h"
//#include "FIR_Filter.h"		//Don't #include "FIR_Filter.h" if #include "dsp.h"
#include "lcd.h"
#include "I2C_MCP4728.h"
#include "delay.h"
#include "string.h"
#include <stdint.h>

//*****************************************************************************
// PIC Configuration Bits
//*****************************************************************************
_FOSCSEL(FNOSC_FRC)					// Internal FRC Oscillator
_FOSC(FCKSM_CSECMD & OSCIOFNC_ON & POSCMD_NONE)		// Clock Switching is enable and Fail Safe Clock Monitor is disabled, OSC2 is digital IO, Primary Oscillator Mode: Disabled
_FWDT(FWDTEN_OFF)					// Watchdog timer enabled/disabled by user software
_FICD(ICS_PGD2 & JTAGEN_OFF)		// Communicate on PGC1/EMUC1 and PGD1/EMUD1, Disable JTAG
_FGS(GSS_STD & GCP_ON)				// User program memory is code-protected
//_FGS(GSS_OFF & GCP_OFF)

//*****************************************************************************
// Global Definitions
//*****************************************************************************
//#define WiFi_Enable
#define U1_waveform

#define Sample_Window_Interval			125		// For detecting max 4Hz(250ms) pulse rate, 250ms*0.5sample/ms(@2msPeriod)=125samples
#define Filter_Length					1792
#define DCVppHigh						200		// 2000
#define DCVppLow						100		// 1800
#define Finger_Present_Threshold		620		// 1200=0.60V/(2.048V/4096) DAC, 2482=2V/(3.3V/4096), 4000
#define initial_DAC_IRLed_intensity		2000	// 2000
#define initial_DAC_RedLed_intensity	2000	// 2000
#define dc_offset						400		// 400
#define duty_cycle						70		// 30=200uSD/C@10MIPS, 70=450uSD/C@40MIPS, 70=220usD/C@20MIPS

//*****************************************************************************
// Global Variables
//*****************************************************************************
extern FIRStruct LowPassFilter1, LowPassFilter2;
signed int FIR_output_IR[16];
signed int FIR_output_Red[16];
signed int FIR_input_IR[16];
signed int FIR_input_Red[16];

unsigned int Vref;
unsigned int DAC_IRLed_intensity, DAC_RedLed_intensity;

signed int CH0_ADRES_IR, CH0_ADRES_Red, CH1_ADRES_IR, CH1_ADRES_Red;
signed int CH1_ADRES_IR_max, CH1_ADRES_IR_min;
signed int CH1_ADRES_Red_max, CH1_ADRES_Red_min;
signed int IR_Max, IR_Min, Red_Max, Red_Min;
signed int IR_Vpp, Red_Vpp;
double IR_Vrms, Red_Vrms;
double Ratio;

signed int Baseline_Upper_Limit, Baseline_Lower_Limit, Baseline_ambient;

unsigned char Read_ADC_IR, Read_ADC_Red;
unsigned int Sample_Window_Counter;
unsigned char Find_MaxMin_State;
unsigned char Meter_State=0;
unsigned int Samples_Between_Pulses=0;
unsigned char Pulse_Rate, Pulse_Rate_previous;
unsigned char SpO2, SpO2_previous;
unsigned int maxCounter=0, minCounter=0;
double SpO2_temp;

unsigned char Detection_Done=0;
unsigned char goto_sleep=0;
unsigned char first_reading=0;
unsigned char RedReady=0, IRReady=0;

unsigned char SpO2_buffer[4] = {0};
unsigned char PR_buffer[4] = {0};
unsigned char avg_counter, avg_counter2;

const char mytext[] =  "Microchip  Pulse";
const char mytext1[] = " Oximeter  Demo ";
const char mytext2[] = " SPO2(%): ";
const char mytext3[] = " PR(bpm): ";
const char mytext4[] = "Check SpO2 Probe";
const char mytext5[] = "Goto Sleep...";
const char mytext6[] = "   Ratio: ";
const char mytext7[] = "%SPO2=";
const char mytext8[] = ",";
const char mytext9[] = "PR=";

#ifdef WiFi_Enable
//const char SSID[11] = {0x6a, 0x6f, 0x69, 0x6e, 0x20, 0x67, 0x75, 0x65, 0x73, 0x74, 0x0d};	// "join guest" cr
const char WF_join[5] = {0x6a, 0x6f, 0x69, 0x6e, 0x20};										// "join " command
const char set_wlan_phrase[16] = {0x73, 0x65, 0x74, 0x20, 0x77, 0x6c, 0x61, 0x6e, 0x20, 0x70, 0x68, 0x72, 0x61, 0x73, 0x65, 0x20};	// "set wlan phrase " command for WPA mode
const char SSID[] = "join mchpmpg";
//const char SSID[] = "mchpmpg";																// SSID(WiFi network name)
const char WF_password[] = "mchpmpg4";														// WiFi network password (WPA mode)
const char close_TCP[6] = {0x63, 0x6c, 0x6f, 0x73, 0x65, 0x0d};								// "close" cr
const char WF_sleep[6] = {0x73, 0x6c, 0x65, 0x65, 0x70, 0x0d};								// "sleep" cr
const char SPO2_symbol[9] = {0x53, 0x50, 0x4f, 0x32, 0x28, 0x25, 0x29, 0x3a, 0x20}; 		// "SPO2(%): "
const char PUL_symbol[9] = {0x50, 0x52, 0x28, 0x62, 0x70, 0x6d, 0x29, 0x3a, 0x20};			// "PR(bpm): "
unsigned char SPO2_String[4], PR_String[4];
unsigned char string_length, sl;
#endif

extern volatile unsigned char hunds;
extern volatile unsigned char tens;
extern volatile unsigned char ones;

unsigned char FirstByte;
unsigned char SecondByte_SingleWrite_A;
unsigned char SecondByte_MultiWrite_A;
unsigned char ThirdByte_A;
unsigned char FourthByte_A;
unsigned char SecondByte_SingleWrite_B;
unsigned char SecondByte_MultiWrite_B;
unsigned char ThirdByte_B;
unsigned char FourthByte_B;
unsigned char SecondByte_SingleWrite_C;
unsigned char SecondByte_MultiWrite_C;
unsigned char ThirdByte_C;
unsigned char FourthByte_C;
unsigned char SecondByte_SingleWrite_D;
unsigned char SecondByte_MultiWrite_D;
unsigned char ThirdByte_D;
unsigned char FourthByte_D;

unsigned int wf;
unsigned int delay2, delay_counter1;

//*****************************************************************************
// Local Function Prototypes
//*****************************************************************************
void Init(void);
void CH0_process(void);
void CH1_process(void);
void U1_write1byte(unsigned char value);
void send2U2(unsigned char* Uart_Output, unsigned int sz);
void sendChar2U2(unsigned char ch);
void Find_MaxMin(void);
void SpO2_Calculation(void);
void Pulse_Rate_Calculation(void);
void LCD_display(void);
void LCD_display_Ratio(void);
extern void hexdec( unsigned char count );
unsigned char Calibrate_IR(void);
unsigned char Calibrate_Red(void);
void str_Int(uint16_t i);
void uitoa(WORD Value, BYTE* Buffer);

/******************************************************************************
* Function:         Init (void) 
*
* Overview:         This function is called when the system first
*                   powers up and is responsible for initializing the
*                   variables, registers, and modules of the system.
*
* Note:             None
******************************************************************************/
void Init(void)
{
    //*****************************************************************************
	// Configure Oscillator to operate the device at 40MIPS where M=43,N1=N2=2
	// Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
	// Fosc= 7.37M*43/(2*2)=79.22Mhz for ~40MIPS input clock
	//
	// Configure Oscillator to operate the device at 10MIPS where M=11,N1=N2=2
	// Fosc= 7.37M*11/(2*2)=20.2675Mhz for ~10MIPS input clock
	//
	// Configure Oscillator to operate the device at 20MIPS where M=22,N1=N2=2
	// Fosc= 7.37M*22/(2*2)=20.2675Mhz for ~10MIPS input clock
    //*****************************************************************************
	PLLFBD = 20;							// 41=M=43@40MIPS, 9=1001->M=11@10MIPS, 20=10100->M=22@20MIPS
	CLKDIVbits.PLLPOST = 0;					// N1=2
	CLKDIVbits.PLLPRE = 0;					// N2=2
	OSCTUN = 0;								// Tune FRC oscillator, if FRC is used
	RCONbits.SWDTEN = 0;					// Disable Watch Dog Timer

	// Clock switch to incorporate PLL
	__builtin_write_OSCCONH(0x01);			// Initiate Clock Switch to FRC with PLL (NOSC=0b001)
	__builtin_write_OSCCONL(0x01);			// Start clock switching

	while (OSCCONbits.COSC != 0b001);		// Wait for Clock switch to occur
	while (OSCCONbits.LOCK != 1) {};		// Wait for PLL to lock

    //***************************
    // Initialize Ports
    //***************************
    PORTA = 0x0000;
    PORTB = 0x0000;
	LATA  = 0x0000;
	LATB  = 0x0000;
	TRISA = 0x0000;
	TRISB = 0x0000;
	AD1PCFGL = 0xffff;				// Set all AN ports digtial

    //*******************************************************************************************************************************************************
    // Initialize 12-bit ADC module, (AN0 = DC signal at 1st stage amplifier & AN1 = AC signal at 2nd stage amplifier)
    //*******************************************************************************************************************************************************
	TRISAbits.TRISA0 = 1;			// Set AN0 to input pin
	TRISAbits.TRISA1 = 1;			// Set AN1 to input pin
	AD1PCFGLbits.PCFG0 = 0;			// configure AN0/RA0 as Analog pin
	AD1PCFGLbits.PCFG1 = 0;			// configure AN1/RA1 as Analog pin
	AD1CON1 = 0x04e0;				// Select 12-bit 1-channel ADC operation, unsigned integer, internal counter auto-convert, Sampling begins when SAMP bit is set
	AD1CON2 = 0x0000;				// ADREF+ = AVdd, ADREF- = AVss
	AD1CON3 = 0x1f03;				// ADC clock derived from system clock
									// ADC Conversion clock Tad=Tcy*(ADCS+1) = (1/40M)*4 = 100ns
									// Auto sample time SAMC = 31*Tad = 31*100 = 3.1us
									// ADC Conversion Time for 12-bit Tc=14*Tad = 14*100 = 1.4us
	AD1CHS0 = 0x0001;				// initial positive input for Sample A is AN1, negative input is Vref-
	IPC3bits.AD1IP = 1;				// ADC1 conversion complete interrupt priority
	IFS0bits.AD1IF = 0;				// Clear the A/D interrupt flag bit
	IEC0bits.AD1IE = 0;				// Disable A/D interrupt
	AD1CON1bits.ADON = 1;			// Turn on the A/D converter

    //***************************
    // Initialize LCD Display
    //***************************
	Init_LCD();

    //*********************************************************************************************************
    // Initialize Timer 2 - IR light
    //*********************************************************************************************************
	T2CON = 0x0020;					// Stop 16-bit Timer2, 1:64(40MhzFosc, 1:8 for 8MHzFosc) Prescale, Internal clock (Fosc/2)
	TMR2 = 0x00;					// Clear timer register
	PR2 = 633;						// Load the period value, OCxRS <= PRx, 499=8MHzFosc(1ms), 623=40MHzFosc(1ms), 1246=40MHzFoc,2msPeriod, 2492=4msPeriod, 4984=40MHzFoc,8msPeriod, 312=2msPeriod@10MIPS
	IPC1bits.T2IP = 2;				// Set Timer2 Interrupt Priority Level
	IFS0bits.T2IF = 0;				// Clear Timer2 Interrupt Flag
	IEC0bits.T2IE = 1;				// Enable Timer2 Interrupt

    //*********************************************************************************************************
    // Initialize Timer 3 - Red light
    //*********************************************************************************************************
	T3CON = 0x0020;					// Stop 16-bit Timer3, 1:64(40MhzFosc, 1:8 for 8MHzFosc) Prescale, Internal clock (Fosc/2)
	TMR3 = 0x00;					// Clear timer register
	PR3 = 633;						// Load the period value, OCxRS <= PRx, 499=8MHzFosc(1ms period), 623=40MHzFosc(1ms period), 1246=40MHzFoc,2msPeriod, 2492=4msPeriod, 4984=40MHzFoc,8msPeriod, 312=2msPeriod@10MIPS
	IPC2bits.T3IP = 2;				// Set Timer3 Interrupt Priority Level
	IFS0bits.T3IF = 0;				// Clear Timer3 Interrupt Flag
	IEC0bits.T3IE = 1;				// Enable Timer3 Interrupt

    //*********************************************************************************************************
    // Initialize Output Compare 1 module in Continuous Pulse mode, OC1 controls IR LED switch
    //*********************************************************************************************************
	RPOR6bits.RP13R = 0b10010;		// RP13/RB13 tied to OC1 (IR)
	OC1CONbits.OCM = 0b000; 		// Disable Output Compare 1 Module
	OC1R = 0; 						// Write the duty cycle for the first PWM pulse, 24=8MHzFosc(50us), 30=40MHzFosc(50us), 600=40MHzFosc(1ms)
	OC1RS = duty_cycle; 			// Write the duty cycle for the second PWM pulse, OCxRS <= PRx, 499=8MHzFosc(1ms), 623=40MHzFosc(1ms), 1246=40MHzFoc,2msPeriod, 4984=40MHzFoc,8msPeriod, 280=450us D/C@40MHzFoc,2msPeriod,switch
	OC1CONbits.OCTSEL = 0; 			// Select Timer 2 as output compare time base

    //*********************************************************************************************************
    // Initialize Output Compare 2 module in Continuous Pulse mode, OC2 controls Red LED switch
    //*********************************************************************************************************
	RPOR6bits.RP12R = 0b10011;		// RP12/RB12 tied to OC2 (Red)
	OC2CONbits.OCM = 0b000; 		// Disable Output Compare 2 Module
	OC2R = 0; 						// Write the duty cycle for the first PWM pulse, 24=8MHzFosc, 30=40MHzFosc, 600=40MHzFosc(1ms)
	OC2RS = duty_cycle; 			// Write the duty cycle for the second PWM pulse, OCxRS <= PRx, 499=8MHzFosc(1ms), 623=40MHzFosc(1ms), 1246=40MHzFoc,2msPeriod, 4984=40MHzFoc,8msPeriod, 280=450us D/C@40MHzFoc,2msPeriod,switch
	OC2CONbits.OCTSEL = 1; 			// Select Timer 3 as output compare time base

    //*****************************************************************************************
    // Initialize UART1 module for outputing serial data or displaying waveform, 115200/8-N-1
    //*****************************************************************************************
	RPOR7bits.RP15R = 0b00011;		// RP15/RB15/Pin26 tied to U1TX
	U1MODEbits.STSEL = 0;			// 1 Stop bit
	U1MODEbits.PDSEL = 0;			// No Parity, 8 data bits
	U1MODEbits.ABAUD = 0;			// Auto-Baud Disabled
	U1MODEbits.BRGH = 1;			// High Speed mode
	U1BRG = 42;						// BAUD Rate Setting for 115200 BRGH=1 UxBRG=((Fcy/Baudrate)/4)-1=86, for 19200, BRGH=0 UxBRG = ((Fcy/Baudrate)/16)-1 = 40000000/19200/16-1 = 129, (UxBRG=259 for 9600), Fcy=40MHz, 21=10MIPS, 42=20MIPS
	U1STAbits.UTXISEL0 = 1;			// UTXISEL<1:0> = 01 = Interrupt when all transmit operations are completed.
	U1STAbits.UTXISEL1 = 0;
	IPC3bits.U1TXIP = 2;			// UART Transmitter interrupt priority
	IPC2bits.U1RXIP = 0;			// UART Receiver interrupt prioirty
	IFS0bits.U1TXIF = 0;			// Clear UART Transmitter interrupt flag
	IFS0bits.U1RXIF = 0;			// Clear UART Receiver interrupt flag
	IEC0bits.U1TXIE = 0;			// Disable UART Transmitter interrupt
	IEC0bits.U1RXIE = 0;			// Disable UART Receiver interrupt
	U1MODEbits.UARTEN = 1;			// Enable UART
	U1STAbits.UTXEN = 1;			// Enable UART TX, wait at least 104 usec (1/9600) before sending first char

    //********************************************************************************
    // Initialize UART2 module for WiFi module, 9600/8-N-1
    //********************************************************************************
#ifdef WiFi_Enable
	RPOR7bits.RP14R = 0b00101;		// RP14/RB14/Pin25 tied to U2TX
	U2MODEbits.STSEL = 0;			// 1 Stop bit
	U2MODEbits.PDSEL = 0;			// No Parity, 8 data bits
	U2MODEbits.ABAUD = 0;			// Auto-Baud Disabled
	U2MODEbits.BRGH = 0;			// Low Speed mode
	U2BRG = 129;					// BAUD Rate Setting for 19200, UxBRG = ((Fcy/Baudrate)/16)-1 = 40000000/19200/16-1 = 129, (UxBRG=259 for 9600), Fcy=40MHz, 64=10MIPS, 129=20MIPS
	U2STAbits.UTXISEL0 = 1;			// Not using transmission interrupt
	U2STAbits.UTXISEL1 = 1;
	IPC7bits.U2TXIP = 0;			// UART Transmitter interrupt priority
	IPC7bits.U2RXIP = 0;			// UART Receiver interrupt prioirty
	IFS1bits.U2TXIF = 0;			// Clear UART Transmitter interrupt flag
	IFS1bits.U2RXIF = 0;			// Clear UART Receiver interrupt flag
	IEC1bits.U2TXIE = 0;			// Disable UART Transmitter interrupt
	IEC1bits.U2RXIE = 0;			// Disable UART Receiver interrupt
	U2MODEbits.UARTEN = 1;			// Enable UART
	U2STAbits.UTXEN = 1;			// Enable UART TX, wait at least 104 usec (1/9600) before sending first char
#endif

    //*********************************************************************************
    // Initialize I2C for communicating with external DAC MCP4728
    //*********************************************************************************
	Init_I2C();

	//*****************************************************************************
	// Initialize MCP4728 DAC Output values
	//*****************************************************************************
	DAC_IRLed_intensity = initial_DAC_IRLed_intensity;		// (DAC-A) DAC output voltage for controlling IR LED current, 2000=1.0V (@1.5V IR)
	DAC_RedLed_intensity = initial_DAC_RedLed_intensity;	// (DAC-B) DAC output voltage for controlling Red LED current, 1500=0.75V (@1.5V Red)

	//Address Byte for MCP4728: 1100 000 0, DeviceCode=1100, AddressBits=000, R/nW=0(write)
	FirstByte = 0xc0;

	//Single Write Command (bit7-3) for DAC Input Register and EEPROM of selected channel (bit2,1 / DAC1,2)
	//2nd Byte format:  0     1    0    1   1    DAC1  DAC0  nUDAC
	//3rd Byte format:  Vref  PD1  PD0  Gx  D11  D10   D9    D8
	//4th Byte format:  D7    D6   D5   D4  D3   D2    D1    D0
	//DAC input register = 0x80 = internal Vref 2.048V

	SecondByte_SingleWrite_A = 0b01011000;	//Channel A
	SecondByte_SingleWrite_B = 0b01011010;	//Channel B
	SecondByte_SingleWrite_C = 0b01011100;	//Channel C
	SecondByte_SingleWrite_D = 0b01011110;	//Channel D

	ThirdByte_A = 0x80 + (DAC_IRLed_intensity>>8 & 0x0f);
	FourthByte_A = DAC_IRLed_intensity & 0x00ff;

	ThirdByte_B = 0x80 + (DAC_RedLed_intensity>>8 & 0x0f);
	FourthByte_B = DAC_RedLed_intensity & 0x00ff;

	ThirdByte_C = 0x80 + (dc_offset>>8 & 0x0f);				// (DAC-C) DC offset value for the second stage amplier
	FourthByte_C = dc_offset & 0x00ff;

	ThirdByte_D = 0x40;										// (DAC-D) Channel D is disabled
	FourthByte_D = 0x00;

	//Multi-Write Command for DAC Input Registier of selected channel only, EEPROM not affected.
	SecondByte_MultiWrite_A = 0b01000000;	//Channel A
	SecondByte_MultiWrite_B = 0b01000010;	//Channel B
	SecondByte_MultiWrite_C = 0b01000100;	//Channel C
	SecondByte_MultiWrite_D = 0b01000110;	//Channel D

	//Write Vref bits of each channel in this format: 1 0 0 X VrefA VrefB VrefC VrefD
	//SecondByte_WriteVref = 0x8e;			//Select Internal Vref 2.048V for channel A,B,C and Vdd for channel D (Vref=0 for Vdd, Vref=1 for Internal Ref)

	//Write Gain bits of each channel in this format: 1 1 0 X GxA GxB GxC GxD
	//SecondByte_WriteGain = 0xc0;			//Select Gain of 1 for all channels (Gx=0 for Gain 1, Gx=1 for Gain 2)

	//Write Power-Down bits of each channel in this format: 1 0 1 X PD1A PD0A PD1B PD0B, PD1C PD0C PD1D PD0D X X X X
	//SecondByte_WritePD = 0xa0;			//Select Normal Mode for channel A,B
	//ThirdByte_WritePD = 0x20;				//Select Normal Mode for channel C and Power-Down for channel D

	//Single Write Mode requires to connect and monitor the RDY/nBSY pin for EEPROM write time.
//	I2C_MCP4728_Write(FirstByte, SecondByte_SingleWrite_A, ThirdByte_A, FourthByte_A);
//	I2C_MCP4728_Write(FirstByte, SecondByte_SingleWrite_B, ThirdByte_B, FourthByte_B);
//	I2C_MCP4728_Write(FirstByte, SecondByte_SingleWrite_C, ThirdByte_C, FourthByte_C);
//	I2C_MCP4728_Write(FirstByte, SecondByte_SingleWrite_D, ThirdByte_D, FourthByte_D);

	I2C_MCP4728_Write(FirstByte, SecondByte_MultiWrite_A, ThirdByte_A, FourthByte_A);
	I2C_MCP4728_Write(FirstByte, SecondByte_MultiWrite_B, ThirdByte_B, FourthByte_B);
	I2C_MCP4728_Write(FirstByte, SecondByte_MultiWrite_C, ThirdByte_C, FourthByte_C);
	I2C_MCP4728_Write(FirstByte, SecondByte_MultiWrite_D, ThirdByte_D, FourthByte_D);

    //****************************************************************************************************************************************
    // Initialize FIR Lowpass Digital Filter (Generated by dsPIC Filter Design Software)
	//   Sampling Frequency = 500hz, Passband Frequency = 5hz, Stopband Frequency = 25, Passband Ripple = 0.1dB, Stopband Ripple = 50dB,
	//   Kaiser window with order of 75
    //****************************************************************************************************************************************
	FIRDelayInit(&LowPassFilter1);		// LowPassFilter1 (LowPass.s) is used for filtering IR signal.
	FIRDelayInit(&LowPassFilter2);		// LowPassFilter2 (LowPass2.s) is used for filtering Red signal.

}

/****************************************************************************
*
* MAIN FUNCTION
*
*****************************************************************************/
int main(void)
{
	unsigned int delay;

	//********** Initialization **********
	Init();

	//***** LCD Welcome message *****
	home_clr();
	puts_lcd( (unsigned char*) &mytext[0], sizeof(mytext) -1 );
	line_2();
	puts_lcd( (unsigned char*) &mytext1[0], sizeof(mytext1) -1 );

#ifdef WiFi_Enable			//Join WiFi network if WiFi module connected
	for (delay_counter1=0; delay_counter1<600; delay_counter1++)	//delay needed for WiFi module to power-up before sending command
	{
		for (delay2=0; delay2<30000; delay2++);
	}
	sendChar2U2(0x24);		//"$" character
	sendChar2U2(0x24);
	sendChar2U2(0x24);
	for (delay2=0; delay2<30000; delay2++);
	for (delay2=0; delay2<30000; delay2++);
	for (delay2=0; delay2<30000; delay2++);
	for (delay2=0; delay2<30000; delay2++);
	for (delay2=0; delay2<30000; delay2++);
	for (delay2=0; delay2<30000; delay2++);
//	send2U2 (SSID, 11);
//	send2U2 (set_wlan_phrase, 16);					//"set wlan phrase " command for WPA security
//	send2U2 (WF_password, sizeof(WF_password));		//WiFi network password for WPF mode
//	sendChar2U2(0x0d);								//cr, carriage return
//	for (delay2=0; delay2<15000; delay2++);
//	send2U2 (WF_join, 5);							//"join " command
	send2U2 (SSID, sizeof(SSID)-1);					//SSIN name
	sendChar2U2(0x0d);								//cr, carriage return
	for (delay2=0; delay2<30000; delay2++);
	for (delay2=0; delay2<30000; delay2++);
	for (delay2=0; delay2<30000; delay2++);
	for (delay2=0; delay2<30000; delay2++);
	for (delay2=0; delay2<30000; delay2++);
	for (delay2=0; delay2<30000; delay2++);
	for (delay2=0; delay2<30000; delay2++);
	for (delay2=0; delay2<30000; delay2++);
	for (delay2=0; delay2<30000; delay2++);
#endif

	//********** Enable OC1 & OC2 ouputs for IR & Red LED's on/off switch **********
	OC2CONbits.OCM = 0b101;				// Select the Output Compare 2 mode, Turn on Red LED
	T3CONbits.TON = 1;					// Start Timer3

	for (delay=0; delay<2200; delay++);

	OC1CONbits.OCM = 0b101;				// Select the Output Compare 1 mode, Turn on IR LED
	T2CONbits.TON = 1;					// Start Timer2

	goto_sleep = 0;

	//********** Main Loop **********
	while (1)
	{
		if (goto_sleep)
		{
			home_clr();
			puts_lcd( (unsigned char*) &mytext4[0], sizeof(mytext4) -1 );	//display "check spo2 probe" text on LCD
			line_2();
			puts_lcd( (unsigned char*) &mytext5[0], sizeof(mytext5) -1 );	//display "Goto Sleep..." text on LCD
			for (delay_counter1=0; delay_counter1<100; delay_counter1++)	//delay for displaying text on LCD
			{
				for (delay2=0; delay2<30000; delay2++);
			}

			I2C_MCP4728_Write(FirstByte, 0xa0, 0x40, 0x00);		//2nd_Byte = 0b10100000(power-down ch_A), 3rd_Byte = 0b01000000(Vref=VDD 100kOhm)
			I2C_MCP4728_Write(FirstByte, 0xa2, 0x40, 0x00);		//2nd_Byte = 0b10100010(power-down ch_B), 3rd_Byte = 0b01000000(Vref=VDD 100kOhm)
			I2C_MCP4728_Write(FirstByte, 0xa4, 0x40, 0x00);		//2nd_Byte = 0b10100100(power-down ch_C), 3rd_Byte = 0b01000000(Vref=VDD 100kOhm)
			I2C_MCP4728_Write(FirstByte, 0xa6, 0x40, 0x00);		//2nd_Byte = 0b10100110(power-down ch_D), 3rd_Byte = 0b01000000(Vref=VDD 100kOhm)

			#ifdef WiFi_Enable
			send2U2 (close_TCP, 6);		//disconnect a TCP connection
			for (delay2=0; delay2<30000; delay2++);
			for (delay2=0; delay2<30000; delay2++);
			for (delay2=0; delay2<30000; delay2++);
			for (delay2=0; delay2<30000; delay2++);
			for (delay2=0; delay2<30000; delay2++);
			for (delay2=0; delay2<30000; delay2++);
			for (delay2=0; delay2<30000; delay2++);
			for (delay2=0; delay2<30000; delay2++);
			send2U2 (WF_sleep, 6);		//put the WiFi module to sleep mode
			#endif

			home_clr();
			PMD1 = 0xffff;
			PMD2 = 0xffff;
			PMD3 = 0xffff;
			Nop();

			while (1)
			{
				Sleep();					// Put MCU into sleep
				Nop();
			}
		}

		//--------- Main State Machine starts here ---------
		if (RedReady && IRReady)
		{
			RedReady = 0;
			IRReady = 0;

//			LATBbits.LATB10 = 1;			//for debugging
			FIR(1, &FIR_output_IR[0], &FIR_input_IR[0], &LowPassFilter1);
			FIR(1, &FIR_output_Red[0], &FIR_input_Red[0], &LowPassFilter2);

			CH1_ADRES_IR = FIR_output_IR[0];
			CH1_ADRES_Red = FIR_output_Red[0];

			#ifdef U1_waveform				//using mTouch Sensing Solutions GUI
//			LATBbits.LATB10 = 1;			//for debugging
			str_Int(1);						//Turn on LED1, 3=Turn on LED1(RED=Red) & LED2(IR=Blue)
//			str_Int(CH1_ADRES_Red);			//convert unsigned int to ascii - RED
			str_Int(CH1_ADRES_IR);			//convert unsigned int to ascii
			U1_write1byte(0x0d);			//footer1
			U1_write1byte(0x0a);			//footer2
//			LATBbits.LATB10 = 0;			//for debugging
			#endif

			Find_MaxMin();

			if (maxCounter == 1)
				Samples_Between_Pulses++;			//Counting samples between two continuous pulses' peak for PR

			if (Detection_Done)
			{
				//Max & Min are all found. Calculate SpO2 & Pulse Rate
				SpO2_Calculation();				//calculate SpO2
				Pulse_Rate_Calculation();		//calculate pulse rate

				if (first_reading==1)
				{
					SpO2_previous = SpO2;
					Pulse_Rate_previous = Pulse_Rate;
					first_reading = 2;
				}
				else if (first_reading==2)
				{
					if (SpO2 > SpO2_previous)
					{
						SpO2 = SpO2_previous + 1;
						SpO2_previous = SpO2;
					}
					else if (SpO2 < SpO2_previous)
					{
						SpO2 = SpO2_previous - 1;
						SpO2_previous = SpO2;
					}
					if (Pulse_Rate > Pulse_Rate_previous)
					{
						Pulse_Rate = Pulse_Rate_previous + 1;
						Pulse_Rate_previous = Pulse_Rate;
					}
					else if (Pulse_Rate < Pulse_Rate_previous)
					{
						Pulse_Rate = Pulse_Rate_previous - 1;
						Pulse_Rate_previous = Pulse_Rate;
					}
				}	
				if (first_reading>0)				//First reading is no good, not display.
				{
					LCD_display();				//Send the result to LCD display
//					LCD_display_Ratio();		//for debugging
				}
				if (first_reading != 2)
					first_reading = 1;

				#ifdef WiFi_Enable		//send result to wifi module
				uitoa(SpO2, SPO2_String);
				uitoa(Pulse_Rate, PR_String);

				send2U2(SPO2_symbol, 9);
				string_length = strlen(SPO2_String);
				for (sl=0; sl<string_length; sl++) {
					sendChar2U2(SPO2_String[sl]);
				}
				sendChar2U2(0x0d);			//cr

				send2U2(PUL_symbol, 9);
				string_length = strlen(PR_String);
				for (sl=0; sl<string_length; sl++) {
					sendChar2U2(PR_String[sl]);
				}
				sendChar2U2(0x0d);			//cr

				sendChar2U2(0x0d);			//cr
				#endif

				Detection_Done = 0;
				maxCounter = 0;
				Samples_Between_Pulses = 0;
			}
//			LATBbits.LATB10 = 0;			//for debugging
		}
	}
}

/*******************************************************************************
 * Function Name: Find_MaxMin()
 * Specification: Detecte the max & min value of the IR & Red AC signals at AN1
 *******************************************************************************/
void Find_MaxMin(void)
{
	switch (Find_MaxMin_State)
	{
		case 0:
			CH1_ADRES_IR_max = CH1_ADRES_IR;
			Find_MaxMin_State = 1;
			break;

		case 1:
			if (CH1_ADRES_IR > CH1_ADRES_IR_max)
			{
				CH1_ADRES_IR_max = CH1_ADRES_IR;
				Find_MaxMin_State = 2;
				}
			else
			{
				Find_MaxMin_State = 0;
			}
			break;

		case 2:		//Finding Max
			if (CH1_ADRES_IR > CH1_ADRES_IR_max)
			{
				CH1_ADRES_IR_max = CH1_ADRES_IR;
				CH1_ADRES_Red_max = CH1_ADRES_Red;
				Sample_Window_Counter = Sample_Window_Interval;		//restart counter
			}
			else
			{
				Sample_Window_Counter--;			//signal is now going down
				if (Sample_Window_Counter == 0)		//no more max peaks detected in the sampling window interval, go to detect min
				{
					Find_MaxMin_State = 3;			//go to next state - Finding Min state
					maxCounter++;
					CH1_ADRES_IR_min = CH1_ADRES_IR_max;
					CH1_ADRES_Red_min = CH1_ADRES_Red_max;
					IR_Max = CH1_ADRES_IR_max;
					Red_Max = CH1_ADRES_Red_max;
				}
			}
			break;

		case 3:		//Finding Min
			if (CH1_ADRES_IR < CH1_ADRES_IR_min)
			{
				CH1_ADRES_IR_min = CH1_ADRES_IR;
				CH1_ADRES_Red_min = CH1_ADRES_Red;
				Sample_Window_Counter = Sample_Window_Interval;		//restart counter
			}
			else
			{
				Sample_Window_Counter--;			//signal is now going up
				if (Sample_Window_Counter == 0)		//no more min peaks detected in the sampling window interval, go to detect max
				{
					Find_MaxMin_State = 4;			//go to next state
					minCounter++;
					CH1_ADRES_IR_max = CH1_ADRES_IR_min;
					CH1_ADRES_Red_max = CH1_ADRES_Red_min;
					IR_Min = CH1_ADRES_IR_min;
					Red_Min = CH1_ADRES_Red_min;
				}
			}
			break;

		case 4:		//Finding Second Max
			if (CH1_ADRES_IR > CH1_ADRES_IR_max)
			{
				CH1_ADRES_IR_max = CH1_ADRES_IR;
				CH1_ADRES_Red_max = CH1_ADRES_Red;
				Sample_Window_Counter = Sample_Window_Interval;		//restart counter
			}
			else
			{
				Sample_Window_Counter--;			//signal is now going down
				if (Sample_Window_Counter == 0)		//no more max peaks detected in the sampling window interval, go to detect min
				{
					Find_MaxMin_State = 0;			//go to next state - Finding Min state
					maxCounter++;
					Detection_Done = 1;				//Found Max & Min
				}
			}
			break;
	}
}

/*****************************************************************************
 * Function Name: SpO2_Calculation()
 * Specification: Calculate the %SpO2
 *****************************************************************************/
void SpO2_Calculation (void)
{
	IR_Vpp = fabs(IR_Max - IR_Min);
	Red_Vpp = fabs(Red_Max - Red_Min);

	IR_Vrms = IR_Vpp / sqrt(8);
	Red_Vrms = Red_Vpp / sqrt(8);

//	SpO2 = log10(Red_Vrms) / log10(IR_Vrms) * 100;
//	if (SpO2 > 100)
//	{
//		SpO2 = 100;
//	}

	// Using lookup talbe to calculate SpO2
	Ratio = (Red_Vrms/CH0_ADRES_Red) / (IR_Vrms/CH0_ADRES_IR);

	if (Ratio == 0)
		Nop();
	else if (Ratio < 0.40)
		SpO2 = 99;
	else if (Ratio < 0.58)
		SpO2 = 99;
	else if (Ratio < 0.63)
		SpO2 = 98;
	else if (Ratio < 0.635)
		SpO2 = 97;
	else if (Ratio < 0.64)
		SpO2 = 96;
	else if (Ratio < 0.645)
		SpO2 = 95;
	else if (Ratio < 0.65)
		SpO2 = 94;
	else if (Ratio < 0.66)
		SpO2 = 93;
	else if (Ratio < 0.67)
		SpO2 = 92;
	else if (Ratio < 0.70)
		SpO2 = 91;
	else if (Ratio < 0.71)
		SpO2 = 90;
	else if (Ratio < 0.77)
		SpO2 = 89;
	else if (Ratio < 0.79)
		SpO2 = 88;
	else if (Ratio < 0.81)
		SpO2 = 87;
	else if (Ratio < 0.83)
		SpO2 = 86;
	else if (Ratio < 0.85)
		SpO2 = 85;
	else if (Ratio < 0.86)
		SpO2 = 84;
	else if (Ratio < 0.87)
		SpO2 = 83;
	else if (Ratio < 0.89)
		SpO2 = 82;
	else if (Ratio < 0.91)
		SpO2 = 81;
	else if (Ratio < 0.93)
		SpO2 = 80;
	else if (Ratio < 0.95)
		SpO2 = 79;
	else
		SpO2 = 70;
}

/*****************************************************************************
 * Function Name: Pulse_Rate_Calculation()
 * Specification: Calculate the pulse rate (Beats Per Minute, bpm)
 *****************************************************************************/
void Pulse_Rate_Calculation (void)
{
	Pulse_Rate = 30000 / Samples_Between_Pulses;	//PR(bpm) = 500sps x 60s / (Samples Count)
	Samples_Between_Pulses = 0;		//reset the counter
}

/*****************************************************************************
 * Function Name: U1_write1byte(unsigned char value)
 * Specification: Write one byte to UART serial port
 *****************************************************************************/
void U1_write1byte (unsigned char value)
{
	while (!U1STAbits.TRMT);		//wait until Transmit Shift Register is empty
	U1TXREG = value;				//write 1 byte to serial port
}

/*****************************************************************************
 * Function Name: LCD_display(void)
 * Specification: Display SPO2 & Pulse Rate results on LCD
 *****************************************************************************/
void LCD_display (void)
{
	hexdec(SpO2);
	home_clr();
	puts_lcd( (unsigned char*) &mytext2[0], sizeof(mytext2) -1 );
	lcd_data(hunds + 0x30);
	lcd_data(tens + 0x30);
	lcd_data(ones + 0x30);

	hexdec(Pulse_Rate);
	line_2();
	puts_lcd( (unsigned char*) &mytext3[0], sizeof(mytext3) -1 );
	lcd_data(hunds + 0x30);
	lcd_data(tens + 0x30);
	lcd_data(ones + 0x30);
}

/*****************************************************************************
 * Function Name: LCD_display_Ratio(void)
 * Specification: Display Ratio results on LCD for debugging
 *****************************************************************************/
void LCD_display_Ratio (void)
{
	hexdec(SpO2);
	home_clr();
	puts_lcd( (unsigned char*) &mytext2[0], sizeof(mytext2) -1 );
	lcd_data(hunds + 0x30);
	lcd_data(tens + 0x30);
	lcd_data(ones + 0x30);

	Ratio = Ratio * 100;
	hexdec(Ratio);
	line_2();
	puts_lcd( (unsigned char*) &mytext6[0], sizeof(mytext6) -1 );
	lcd_data(hunds + 0x30);
	lcd_data(tens + 0x30);
	lcd_data(ones + 0x30);
}

/*****************************************************************************
 * Function Name: unsigned char Calibrate_IR (void)
 * Specification: Calibrate IR LED intensity.
 *****************************************************************************/
unsigned char Calibrate_IR (void)
{
	unsigned char MeterState;

	MeterState = Meter_State;

	if (CH0_ADRES_IR > Baseline_Upper_Limit)
	{
		if (DAC_IRLed_intensity > 0)
		{
			DAC_IRLed_intensity--;
			ThirdByte_A = 0x80 + (DAC_IRLed_intensity>>8 & 0x0f);
			FourthByte_A = DAC_IRLed_intensity & 0x00ff;
			I2C_MCP4728_Write(FirstByte, SecondByte_MultiWrite_A, ThirdByte_A, FourthByte_A);
		}
		IRReady = 0;
	}
	else if (CH0_ADRES_IR < Baseline_Lower_Limit)
	{
		if (DAC_IRLed_intensity < 4095)
		{
			DAC_IRLed_intensity++;
			ThirdByte_A = 0x80 + (DAC_IRLed_intensity>>8 & 0x0f);
			FourthByte_A = DAC_IRLed_intensity & 0x00ff;
			I2C_MCP4728_Write(FirstByte, SecondByte_MultiWrite_A, ThirdByte_A, FourthByte_A);
		}
		IRReady = 0;
	}
	else
	{
		IRReady = 1;
	}

	return(MeterState);
}

/*****************************************************************************
 * Function Name: unsigned char Calibrate_Red (void)
 * Specification: Calibrate Red LED intensity.
 *****************************************************************************/
unsigned char Calibrate_Red (void)
{
	unsigned char MeterState;

	MeterState = Meter_State;

	if (CH0_ADRES_Red > Baseline_Upper_Limit)
	{
		if (DAC_RedLed_intensity > 0)
		{
			DAC_RedLed_intensity--;
			ThirdByte_B = 0x80 + (DAC_RedLed_intensity>>8 & 0x0f);
			FourthByte_B = DAC_RedLed_intensity & 0x00ff;
			I2C_MCP4728_Write(FirstByte, SecondByte_MultiWrite_B, ThirdByte_B, FourthByte_B);
		}
		RedReady = 0;
	}
	else if (CH0_ADRES_Red < Baseline_Lower_Limit)
	{
		if (DAC_RedLed_intensity < 4095)
		{
			DAC_RedLed_intensity++;
			ThirdByte_B = 0x80 + (DAC_RedLed_intensity>>8 & 0x0f);
			FourthByte_B = DAC_RedLed_intensity & 0x00ff;
			I2C_MCP4728_Write(FirstByte, SecondByte_MultiWrite_B, ThirdByte_B, FourthByte_B);
		}
		RedReady = 0;
	}
	else
	{
		RedReady = 1;
	}

	return(MeterState);
}

/*****************************************************************************
 * Function Name: sendChar2U2(unsigned char ch)
 * Specification: Write one byte to UART serial port
 *****************************************************************************/
void sendChar2U2(unsigned char ch)
{
	while (U2STAbits.UTXBF); // wait when TX buffer full
	U2TXREG = ch;
}

/*****************************************************************************
 * Function Name: send2U2(unsigned char* Uart_Output, unsigned int sz)
 * Specification: Write a string to UART serial port
 *****************************************************************************/
void send2U2(unsigned char* Uart_Output, unsigned int sz)  // str is "\0" terminated string
{
	while (sz) {
		while (U2STAbits.UTXBF);	// wait when TX buffer full
		U2TXREG = *Uart_Output++;
		sz--;
		Nop(); Nop();		
	}
}

/*****************************************************************************
  Function:
	void uitoa(WORD Value, BYTE* Buffer)
  Summary:
	Converts an unsigned integer to a decimal string.
  Description:
	Converts a 16-bit unsigned integer to a null-terminated decimal string.
  Precondition:
	None
  Parameters:
	Value	- The number to be converted
	Buffer	- Pointer in which to store the converted string
  Returns:
  	None
  ***************************************************************************/
void uitoa(WORD Value, BYTE* Buffer)
{
	BYTE i;
	WORD Digit;
	WORD Divisor;
	BOOL Printed = FALSE;

	if(Value)
	{
		for(i = 0, Divisor = 10000; i < 5u; i++)
		{
			Digit = Value/Divisor;
			if(Digit || Printed)
			{
				*Buffer++ = '0' + Digit;
				Value -= Digit*Divisor;
				Printed = TRUE;
			}
			Divisor /= 10;
		}
	}
	else
	{
		*Buffer++ = '0';
	}

	*Buffer = '\0';
}

/*****************************************************************************
 * Function Name: void str_Int(uint16_t i)
 * Specification: Converts integer to ASCII.
 *****************************************************************************/
void str_Int(uint16_t i)
{       uint8_t ctr = 0;

        // Decimal Output
        while (i >= 10000)  { i -= 10000; ctr++; } U1_write1byte(ctr + 0x30); ctr = 0;
        while (i >=  1000)  { i -=  1000; ctr++; } U1_write1byte(ctr + 0x30); ctr = 0;
        while (i >=   100)  { i -=   100; ctr++; } U1_write1byte(ctr + 0x30); ctr = 0;
        while (i >=    10)  { i -=    10; ctr++; } U1_write1byte(ctr + 0x30); ctr = 0;
        while (i >=     1)  { i -=     1; ctr++; } U1_write1byte(ctr + 0x30);

         U1_write1byte(0x3b);
}


//=============== Interrupt Service Routine (ISR) ========================================

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void)		//Read IR DC & AC signals from AN0 & AN1
{
	int delay;

	Read_ADC_IR = 1;

	for (delay=0; delay<500; delay++);	//2000=delayed 256us before read ADC, 200, 500
//	LATBbits.LATB10 = 1;			// for debugging

	//Acquires IR-DC from Channel0 (AN0)
	AD1CHS0bits.CH0SA = 0x00;		// Select AN0
	AD1CON1bits.SAMP = 1;			// Begin sampling
	while(!AD1CON1bits.DONE);		// Waiting for ADC completed
	AD1CON1bits.DONE = 0;			// Clear conversion done status bit
	CH0_ADRES_IR = ADC1BUF0;		// Read ADC result

	for (delay=0; delay<2; delay++);	//500

	//Acquires IR-AC from Channel1 (AN1)
	AD1CHS0bits.CH0SA = 0x01;		// Select AN1
	AD1CON1bits.SAMP = 1;			// Begin sampling
	while(!AD1CON1bits.DONE);		// Waiting for ADC completed
	AD1CON1bits.DONE = 0;			// Clear conversion done status bit
	FIR_input_IR[0] = ADC1BUF0;
	CH1_ADRES_IR = ADC1BUF0;

	Meter_State = Calibrate_IR();

//	LATBbits.LATB10 = 0;			// for debugging

	OC1RS = duty_cycle; 			// Write Duty Cycle value for next PWM cycle
	IFS0bits.T2IF = 0;				// Clear Timer2 Interrupt Flag
}

void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void)		//Read Red DC & AC signals from AN0 & AN1
{
	int delay;

	Read_ADC_Red = 1;

	for (delay=0; delay<500; delay++);	//2000=delayed 256us before read ADC
//	LATBbits.LATB10 = 1;			// for debugging

	//Acquires Red-DC from Channel0 (AN0)
	AD1CHS0bits.CH0SA = 0x00;		// Select AN0
	AD1CON1bits.SAMP = 1;			// Begin sampling
	while(!AD1CON1bits.DONE);		// Waiting for ADC completed
	AD1CON1bits.DONE = 0;			// Clear conversion done status bit
	CH0_ADRES_Red = ADC1BUF0;		// Read ADC result

	for (delay=0; delay<2; delay++);

	//Acquires Red-AC from Channel1 (AN1)
	AD1CHS0bits.CH0SA = 0x01;		// Select AN1
	AD1CON1bits.SAMP = 1;			// Begin sampling
	while(!AD1CON1bits.DONE);		// Waiting for ADC completed
	AD1CON1bits.DONE = 0;			// Clear conversion done status bit
	FIR_input_Red[0] = ADC1BUF0;
	CH1_ADRES_Red = ADC1BUF0;

	if (CH0_ADRES_Red > Finger_Present_Threshold)		//if no finger present then goto sleep
	{
		goto_sleep = 1;
	}
	if (CH0_ADRES_Red<=10 && CH1_ADRES_Red>=4000)		//if spo2 probe is not connected
	{
		goto_sleep = 1;
	}
//	LATBbits.LATB10 = 0;			// for debugging

	for (delay=0; delay<1000; delay++);	//2000=delayed 256us before read ADC
//	LATBbits.LATB10 = 1;			// for debugging
	//Acquires Red-DC baseline from Channel0 (AN0)
	AD1CHS0bits.CH0SA = 0x00;		// Select AN0
	AD1CON1bits.SAMP = 1;			// Begin sampling
	while(!AD1CON1bits.DONE);		// Waiting for ADC completed
	AD1CON1bits.DONE = 0;			// Clear conversion done status bit
	Baseline_ambient = ADC1BUF0;
	Baseline_Upper_Limit = Baseline_ambient + DCVppHigh;
	Baseline_Lower_Limit = Baseline_ambient + DCVppLow;

	Meter_State = Calibrate_Red();

//	LATBbits.LATB10 = 0;			// for debugging

	OC2RS = duty_cycle;				// Write Duty Cycle value for next PWM cycle
	IFS0bits.T3IF = 0;				// Clear Timer3 Interrupt Flag
}

void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void)		//UART1 TX Interrupt
{
//	LATBbits.LATB10 = 0;			// for debugging
	IFS0bits.U1TXIF = 0;			// clear interrupt flag
}
