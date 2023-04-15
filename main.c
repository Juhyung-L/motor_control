//////////////////////////////////////////////////////////////////////////////////////////
//
// File name: main.c
// Author: Juhyung Lee
// Date: 12/9/22
// Description:     
//
//////////////////////////////////////////////////////////////////////////////////////////

#include <xc.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <string.h>
#include "lcd_display_driver.h"

#pragma config POSCMOD = HS // Primary Oscillator Mode: High Speed crystal
#pragma config FNOSC = PRIPLL // Oscillator Selection: Primary oscillation w/ PLL
#pragma config FPLLMUL = MUL_20 // PLL Multiplier: Multiply by 20
#pragma config FPLLIDIV = DIV_2 // PLL Input Divider: Divide by 2
#pragma config FPLLODIV = DIV_1 // PLL Output Divider: Divide by 1
#pragma config FPBDIV = DIV_1 // Peripheral Bus Block: Divide by 1

// enum for determining the mode of the controller
// push button S3 = angle_gap_mode
// push button S4 = P_mode
// push button S5 = PI_mode
enum mode {angle_gap_mode, P_mode, PI_mode};
static volatile enum mode mode = angle_gap_mode; // start at angle_gap_mode
float kp = 10;
float ki = 0.01;

#define SYS_FREQ 80000000ul // 80 MHz
// P49 RF4 U2RX
// P50 RF5 U2TX

static volatile float desired_angle = 0;
static volatile int angle_received = 0; // status variable to check if target angle was received

// state functions
void state00();
void state01();
void state11();
void state10();

// Quadrature signal A (CN8, pin 10)
// Quadrature signal B (CN9, pin 11)
static volatile int count = 0; // initialize as zero because this is a relative encoder
static volatile int quadA = 0;
static volatile int quadB = 0;
void (*state)();

static volatile float current_angle;
static volatile float eint;

// helper function to set the direction of the motor
void setDirection(float direction) {
    if (direction > 0) { // drive motor forwards (increase variable "count")
        LATFbits.LATF1 = 1;
        LATFbits.LATF0 = 0;
    }
    else if (direction < 0) { // drive motor backwards (decrease variable "count")
        LATFbits.LATF1 = 0;
        LATFbits.LATF0 = 1;
    }
    else { // direction == 0 -> stop motor
        LATFbits.LATF1 = 0;
        LATFbits.LATF0 = 0;
    }
}

// helper function to set output compare 4 to change PWM signal
// the value will be capped at 0 to 1023
void setOC4(float input) {
    if (input > 1023) {
        input = 1023;
    }
    else if (input < 0) {
        input = 0;
    }
    OC4RS = input;
}

void __ISR(_EXTERNAL_1_VECTOR, IPL4SOFT) EXISR1(void) { // external interrupt for proportional controller mode (S4)
    mode = P_mode;
    IFS0bits.INT1IF = 0;
}

void __ISR(_EXTERNAL_2_VECTOR, IPL4SOFT) EXISR2(void) { // external interrupt for proportional-integral controller mode (S5)
    mode = PI_mode;
    IFS0bits.INT2IF = 0;
}

// reading from an analog source
unsigned int adc_sample_convert(int pin) // sample & convert the value on the given
{
	// adc pin the pin should be configured as an analog input in AD1PCFG
	unsigned int elapsed = 0, finish_time = 0;
	AD1CHSbits.CH0SA = pin; // connect chosen pin to MUXA for sampling
	AD1CON1bits.SAMP = 1; // start sampling
	elapsed = _CP0_GET_COUNT();
	finish_time = elapsed + 1000;
	while (_CP0_GET_COUNT() < finish_time) // stay in loop for the duration of SAMPLE_TIME
	{ }
	AD1CON1bits.SAMP = 0; // stop sampling and start converting
	while (!AD1CON1bits.DONE) // stay in loop until conversion is done
	{ }
	return ADC1BUF0; // read the buffer with the result
}

void __ISR(_CHANGE_NOTICE_VECTOR, IPL3SOFT) CNISR(void) // change notification interrupt
{
    if (PORTDbits.RD6 = 0) { // push button S3 pressed -> switch to angle_gap_mode
        mode = angle_gap_mode;
        IFS1bits.CNIF = 0;
    }
    // read from quadrature signal A and B
    quadA = PORTGbits.RG6; // pin 10
    quadB = PORTGbits.RG7; // pin 11
    
    state();
    IFS1bits.CNIF = 0; // clear the interrupt flag
}

void readUART2(char* message, int maxLength) {
    char data = 0;
    int complete = 0, num_bytes = 0;
    while(!complete) {
        if(U2STAbits.URXDA) { // if data is available
            data = U2RXREG; // read the data
            if(data == '\n' || data=='\r' || num_bytes >= maxLength-1) {
                complete = 1;
            }
            else {
                message[num_bytes] = data;
                ++num_bytes;
            }
        }
    }
}

void writeUART2(const char * string) {
    while (*string != '\0') {
        while (U2STAbits.UTXBF) 
        { } // wait until tx buffer isn't full
        U2TXREG = *string;
        ++string;
    }
}

void __ISR(_UART_2_VECTOR, IPL3SOFT) uartInterrupt(void) {
    if(IFS1bits.U2RXIF) { // rx interrupt
        char message[10];
        readUART2(message, 10);
        desired_angle = atof(message);
        angle_received = 1; // set status flag to true
        _CP0_SET_COUNT(0); // start timer
        IFS1bits.U2RXIF = 0; // clear the status flag
    }
    else if(IFS1bits.U2TXIF) { // tx interrupt
        IFS1bits.U2TXIF = 0;
    }
    else if(IFS1bits.U2EIF) { // UART error interrupt
        IFS1bits.U2EIF = 0;
    }
}

void __ISR(_TIMER_5_VECTOR, IPL2SOFT) timer5ISR(void) {
    if(angle_received) {
        char desired_angle_string[1000];
        char current_angle_string[1000];
        char time[1000];
        
        sprintf(desired_angle_string, "%0.4f\r\n", desired_angle);
        sprintf(current_angle_string, "%0.4f\r\n", (360.0*count)/(99.0*48.0));
        sprintf(time, "%0.4f\r\n", _CP0_GET_COUNT()/40000000.0); // 40M ticks per second
        
        writeUART2(time);
        writeUART2(desired_angle_string);
        writeUART2(current_angle_string);
    }
    IFS0bits.T5IF = 0;
    // timer 5 interrupt at 50 Hz
}

void __ISR(_TIMER_2_VECTOR, IPL2SOFT) timer2ISR(void) {
    current_angle = (360.0*count)/(99.0*48.0); // get the current angle
    
    if (mode == angle_gap_mode) { // angle gap mode
        // read from potentiometer and set the duty cycle of the motor
        // higher potentiometer reading = higher duty cycle
        unsigned int pot_data = 0;
        int pot_pin = 2;
        pot_data = adc_sample_convert(pot_pin); // reading from potentiometer
        OC4RS = pot_data; // this changes the duty cycle
        // set direction of motor
        // set P88 and P87 until ref angle - current angle < 7.5
        if((desired_angle - current_angle) > 7.5) { // desired angle is more positive -> increase count
            setDirection(1);
        }
        else if((desired_angle - current_angle) < -7.5) { // current angle is more positive -> decrease count
            setDirection(-1);
        }
        else { // stop the motor
            setDirection(0);
        }
        LATAbits.LATA0 = 1;
        IFS0bits.T2IF = 0;
    }
    else if (mode == P_mode) { // proportional control mode
        float e = desired_angle - current_angle; // get error
        setDirection(kp * e); // set direction as gain * error (only the sign of gain * error matters)
        setOC4(abs(kp * e)); // set pwm signal to magnitude of gain * error
        LATAbits.LATA1 = 1;
        IFS0bits.T2IF = 0;
    }
    else if (mode == PI_mode) { // proportional-integral control mode
        float e = desired_angle - current_angle;
        eint += e; // integrating error (summation in the discrete world)
        setDirection(kp*e + ki*eint);
        setOC4(abs(kp*e + ki*eint));
        LATAbits.LATA2 = 1;
        IFS0bits.T2IF = 0;
    }
    else {
        IFS0bits.T2IF = 0;
    } // do nothing (probably won't ever reach this if statement but just in case)
}

int main(void) {
    
    // debug
    DDPCONbits.JTAGEN = 0; // disable JTAG
    TRISAbits.TRISA0 = 0;
    TRISAbits.TRISA1 = 0;
    TRISAbits.TRISA2 = 0;
    LATAbits.LATA0 = 0;
    LATAbits.LATA1 = 0;
    LATAbits.LATA2 = 0;
    
    INTCONbits.MVEC = 0x1; // enable multi-vector mode


    ////////////////////////////////////////// UART ///////////////////////////////////////////
    // initialize UART2
    U2MODEbits.PDSEL = 0b00; // 8 data bits, no parity
    U2MODEbits.STSEL = 0; // 1 stop bit
    U2STAbits.UTXEN = 1; // turn on receiver
    U2STAbits.URXEN = 1; // turn on transmitter
    U2MODEbits.UEN = 0; // disable hardware control flow
    
    // desired baud rate = 230400
    U2MODEbits.BRGH = 0; // set M = 16
    U2BRG = ((80000000 / 230400) / 16) - 1; // set the baud rate to 230400
    U2MODEbits.ON = 1; // turn on UART2
    
    //initialize UART2 interrupt
    __builtin_disable_interrupts();
    U2STAbits.URXISEL = 0b00; // fire interrupt whenever RX FIFO contains six or more characters
    IFS1bits.U2RXIF = 0; // clear the rx interrupt flag
    IPC8bits.U2IP = 3; // set interrupt priority
    IPC8bits.U2IS = 1; // set interrupt sub priority
    IEC1bits.U2RXIE = 1; // enable the rx interrupt
    __builtin_enable_interrupts();
    ///////////////////////////////////////////////////////////////////////////////////////////


    ///////////////////////////////////////// TIMERS //////////////////////////////////////////
    // set up timer 5 (50 Hz)
    T5CONbits.TCKPS = 0b111; // set pre-scaler to 256
    T5CONbits.TCS = 0; // input to timer 5 is PBCLK
    T5CONbits.ON = 1; // turn on timer 5
    TMR5 = 0; // initialize timer 5 count to 0

    // desired_frequency = PBCLK / ((PRx + 1) * pre-scaler)
    // PR5 = 80 M / ((50 * 256) - 1) = 6249
    PR5 = 6240;
    
    // set up timer 5 interrupt
    __builtin_disable_interrupts();
    IPC5bits.T5IP = 2; // set priority to 2
    IPC5bits.T5IS = 0; // set sub priority
    IFS0bits.T5IF = 0; // clear interrupt status flag
    IEC0bits.T5IE = 1; // enable timer 5 interrupt
    __builtin_enable_interrupts();

    // set up timer 2 (50 Hz)
    T2CONbits.TCKPS = 0b111; // set prescaler to 256
    T2CONbits.TCS = 0; // input to timer 2 is PBLCK
    T2CONbits.ON = 1;
    TMR2 = 0;

    // desired_frequency = PBCLK / ((PRx + 1) * pre-scaler)
    // PR5 = 80 M / ((50 * 256) - 1) = 6249
    PR2 = 6249;

    // set up timer 2 interrupt
    __builtin_disable_interrupts();
    IPC2bits.T2IP = 2; // set priority to 2
    IPC2bits.T2IS = 0; // set sub priority
    IFS0bits.T2IF = 0; // clear interrupt status flag
    IEC0bits.T2IE = 1; // enable timer 5 interrupt
    __builtin_enable_interrupts();

    // set up timer 3 to generate PWM signal
    T3CONbits.TCKPS = 0b011; // prescaler = 8
    TMR3 = 0;
    PR3 = 1023;

    // set up OC4
    OC4CONbits.OCM = 0b110; // PWM mode without fault pin 
    OC4CONbits.OCTSEL = 1; // OC4 uses Timer 3
    OC4RS = 1023;
    OC4R = 1023;

    T3CONbits.ON = 1; // turn on timer 3
    OC4CONbits.ON = 1; // turn on OC4
    ///////////////////////////////////////////////////////////////////////////////////////////


    /////////////////////////////// READING FROM POTENTIOMETER ////////////////////////////////
	AD1PCFGbits.PCFG2 = 0; // AN2 is analog pin (potentiometer)
    AD1PCFGbits.PCFG4 = 0; // AN4 is analog pin
	AD1CON3bits.ADCS = 2;
    AD1CON1bits.ON = 1; // turn on A/D converter (or just bits.ON)
    ///////////////////////////////////////////////////////////////////////////////////////////


    ///////////////////////////////////// ENCODER READING /////////////////////////////////////
    // set up CN interrupt for reading encoder
	__builtin_disable_interrupts(); // disable CPU interrupts
	CNCONbits.ON = 1; // turn on CN

	// use CN8 and CN8 pins as change notification
	CNENbits.CNEN8 = 1;
	CNENbits.CNEN9 = 1;

    // CN15 for switching back to angle_gap_mode
    CNENbits.CNEN15 = 1;

	IPC6bits.CNIP = 3; // interrupt priority = 3
	IPC6bits.CNIS = 2; // sub priority = 2
	IFS1bits.CNIF = 0; // clear the interrupt flag
	IEC1bits.CNIE = 1; // enable the CN interrupt
	__builtin_enable_interrupts(); // enable CPU interrupts
    ///////////////////////////////////////////////////////////////////////////////////////////


    // S3 = P83 = CN15 = RD6
    // S4 = P80 = P18 = INT1 = RE8
    // S5 = P92 = P19 = INT2 = RE9
    ///////////////////////////////// SETTING UP PUSH BUTTONS /////////////////////////////////
    TRISDbits.TRISD6 = 1;
    TRISEbits.TRISE8 = 1;
    TRISEbits.TRISE9 = 1;

    // set up external interrupt 1 for proportional controller mode (S4)
    __builtin_disable_interrupts();
    INTCONbits.INT1EP = 0; // INT2 triggers on a falling edge
    IPC1bits.INT1IP = 4; // interrupt priority = 3
    IPC1bits.INT1IS = 2; // sub priority = 2
    IFS0bits.INT1IF = 0; // clear the interrupt flag
    IEC0bits.INT1IE =  1; // enable the interrupt
    __builtin_enable_interrupts();

    // set up external interrupt 2 for proportional-integeral controller mode (S5)
    __builtin_disable_interrupts();
    INTCONbits.INT2EP = 0; // INT2 triggers on a falling edge
    IPC2bits.INT2IP = 4; // interrupt priority = 3
    IPC2bits.INT2IS = 2; // sub priority = 2
    IFS0bits.INT2IF = 0; // clear the interrupt flag
    IEC0bits.INT2IE =  1; // enable the interrupt
    __builtin_enable_interrupts();
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    eint = 0;

    // setting CN8 (Quad signal A, G6) and CN9 (Quad signal B, G7) to input to read from them
    TRISGbits.TRISG6 = 1;
    TRISGbits.TRISG7 = 1;
    state = &state00;
    
    // setting P88 (F1) and P87 (F0) as digitial output
    TRISFbits.TRISF1 = 0;
    TRISFbits.TRISF0 = 0;
    LATFbits.LATF1 = 0;
    LATFbits.LATF0 = 0;
    
    char desired_angle_string[20];
    char current_angle_string[20];
     lcd_display_driver_initialize(); // initialize lcd display   
    while(1) {
        sprintf(desired_angle_string, "Ref: %8.2f%c", desired_angle, 0xDF);
        sprintf(current_angle_string, "Cur: %8.2f%c", (360.0*count)/(99.0*48.0), 0xDF);
        display_driver_use_first_line();
        lcd_display_driver_write(desired_angle_string, 14);
        display_driver_use_second_line();
        lcd_display_driver_write(current_angle_string, 14);
    }
	return 0;
}

// state functions
void state00()
{
    if(quadA == 0 && quadB == 1)
    {
        ++count;
        state = &state01;
    }
    else if(quadA == 1 && quadB == 0)
    {
        --count;
        state = &state10;
    }
}

void state01()
{
    if(quadA == 1 && quadB == 1)
    {
        ++count;
        state = &state11;
    }
    else if(quadA == 0 && quadB == 0)
    {
        --count;
        state = &state00;
    }
}

void state11()
{
    if(quadA == 1 && quadB == 0)
    {
        ++count;
        state = &state10;
    }
    else if(quadA == 0 && quadB == 1)
    {
        --count;
        state = &state01;
    }
}

void state10()
{
    if(quadA == 0 && quadB == 0)
    {
        ++count;
        state = &state00;
    }
    else if(quadA == 1 && quadB == 1)
    {
        --count;
        state = &state11;
    }
}