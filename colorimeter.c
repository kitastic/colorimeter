/*
 * NGOC PHAM        ID: 1000757143      CSE3442-003
 *
 * Colorimeter
 */

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   M0PWM3 (PB5) drives an PNP transistor that powers the red LED
// Green LED:
//   M0PWM5 (PE5) drives an PNP transistor that powers the green LED
// Blue LED:
//   M0PWM4 (PE4) drives an PNP transistor that powers the blue LED
// Green LED (on-board):
//   (PF3) Digitally enabled for flashing on every specified period
// PUSH_BUTTON:
//   SW1 (PF4) internal pull-up push button used for "button" cmd
// Light Sensor:
//   (PE3) [AINO] uses sample sequencer 3 (SS3) and takes one sample at a time
//   measures light and returns voltage value
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1


//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "wait.h"
#include <math.h>
#include "eeprom.h"
#include "colorimeter.h"

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------


uint8_t fieldCount=0;              // string_token: number of fields
uint16_t T = 2047;                 // Threshold value (2^n)-1
uint16_t red = 0;
uint16_t green = 0;
uint16_t blue = 0;
uint16_t E = 0;                    // match E command
uint16_t D = 0;                    // delta D command
float iir = 0;
bool matchFlag = false;            // match mode indicator
bool ledSample = false;            // a flag to tell LED interrupt to flash 
bool deltaFlag = false;            // delta mode indicator


//-----------------------------------------------------------------------------
// Initialize Hardware
//-----------------------------------------------------------------------------

void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    // PWM is system clock / 2
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S)
                | SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_2;
    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clock gating
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOE;  // GPIO port A, B, E peripherals
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;           // enable port f
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;          // turn-on SSI2 clocking
    SYSCTL_RCGC0_R |= SYSCTL_RCGC0_PWM0;            // turn-on PWM0 module
    SYSCTL_RCGCADC_R |= 1;                          // turn on ADC module 0 clocking
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;      // turn on timer 1

    // Configure switch 1, aka push button 1 on port f4
    GPIO_PORTF_DEN_R |= 0x10;                       // enable bit 16 (1 left-shifted 4)
    GPIO_PORTF_PUR_R |= 0x10;                       // enable internal pull-up for PB1

    // Configure green LED on board [PF3]
    GPIO_PORTF_DIR_R |= 1 << 3;     // set bit 3 to output
    GPIO_PORTF_DR2R_R |= 1 << 3;    // set drive strenth to 2mA (default)
    GPIO_PORTF_DEN_R |= 1 << 3;     // enable green LED     

    // Configure three backlight LEDs
    GPIO_PORTB_DIR_R |= 0x20;   // make bit5 an output
    GPIO_PORTB_DR2R_R |= 0x20;  // set drive strength to 2mA
    GPIO_PORTB_DEN_R |= 0x20;   // enable bit5 for digital
    GPIO_PORTB_ODR_R |= 0x20;
    GPIO_PORTB_AFSEL_R |= 0x20; // select auxilary function for bit 5
    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB5_M0PWM3; // enable PWM on bit 5
    GPIO_PORTE_DIR_R |= 0x30;   // make bits 4 and 5 outputs
    GPIO_PORTE_DR2R_R |= 0x30;  // set drive strength to 2mA
    GPIO_PORTE_DEN_R |= 0x30;   // enable bits 4 and 5 for digital
    GPIO_PORTE_ODR_R |= 0x30;
    GPIO_PORTE_AFSEL_R |= 0x30; // select auxilary function for bits 4 and 5
    GPIO_PORTE_PCTL_R = GPIO_PCTL_PE4_M0PWM4 | GPIO_PCTL_PE5_M0PWM5; // enable PWM on bits 4 and 5

    // Configure AIN0 as an analog input
    GPIO_PORTE_AFSEL_R |= 0X08;                     // select alternative functions for AIN0 (PE3)
    GPIO_PORTE_DEN_R &= ~0X08;                      // turn off digital operation on pin PE3
    GPIO_PORTE_AMSEL_R |= 0X08;                     // turn on analog operation on pin PE3

    // Configure ADC
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                  // select PLL as base time (not needed, already default)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;              // disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;          // select SS3 bit in ADCPSSI as trigger
    ADC0_SSMUX3_R = 0;                              // set first sample to AIN0
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;                // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

    // Configure UART0 pins
    GPIO_PORTA_DIR_R |= 2;                           // enable output on UART0 TX pin: default, added for clarity
    GPIO_PORTA_DEN_R |= 3;                           // enable digital on UART0 pins: default, added for clarity
	GPIO_PORTA_AFSEL_R |= 3;                         // use peripheral to drive PA0, PA1: default, added for clarity
	GPIO_PORTA_PCTL_R &= 0xFFFFFF00;                 // set fields for PA0 and PA1 to zero
	GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                     // select UART0 to drive pins PA0 and PA1: default, added for clarity

   	// Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
	UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // Configure PWM module0 to drive RGB backlight
    // RED   on M0PWM3 (PB5), M0PWM1b
    // BLUE  on M0PWM4 (PE4), M0PWM2a
    // GREEN on M0PWM5 (PE5), M0PWM2b
    __asm(" NOP");                                   // wait 3 clocks
    __asm(" NOP");
    __asm(" NOP");
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                // reset PWM0 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM0_1_CTL_R = 0;                                // turn-off PWM0 generator 1
    PWM0_2_CTL_R = 0;                                // turn-off PWM0 generator 2
    PWM0_1_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE;
                                                     // output 3 on PWM0, gen 1b, cmpb
    PWM0_2_GENA_R = PWM_0_GENA_ACTCMPAD_ZERO | PWM_0_GENA_ACTLOAD_ONE;
                                                     // output 4 on PWM0, gen 2a, cmpa
    PWM0_2_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE;
                                                     // output 5 on PWM0, gen 2b, cmpb
    PWM0_1_LOAD_R = 1024;                            // set period to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM0_2_LOAD_R = 1024;
    //PWM0_INVERT_R = PWM_INVERT_PWM3INV | PWM_INVERT_PWM4INV | PWM_INVERT_PWM5INV;
                                                     // invert outputs for duty cycle increases with increasing compare values
    PWM0_1_CMPB_R = 0;                               // red off (0=always low, 1023=always high)
    PWM0_2_CMPB_R = 0;                               // green off
    PWM0_2_CMPA_R = 0;                               // blue off

    PWM0_1_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 1
    PWM0_2_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 2
    PWM0_ENABLE_R = PWM_ENABLE_PWM3EN | PWM_ENABLE_PWM4EN | PWM_ENABLE_PWM5EN;
                                                     // enable outputs

    // Configure Timer 1 for periodic interrupt service [periodIsr()]
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 0x30D40;                        // set load value to 2e5 for 200 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    //NVIC_EN0_R |= 1 << (INT_TIMER1A-16);           // turn-on interrupt 37 (TIMER1A)
    //TIMER1_CTL_R |= TIMER_CTL_TAEN;                // turn-on timer
}

//-----------------------------------------------------------------------------
// String/Tokenizing fuctions
//-----------------------------------------------------------------------------

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
	while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
	UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
	uint8_t i;
    for (i = 0; str[i] != 0; i++)
	  putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
	while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
	return UART0_DR_R & 0xFF;                        // get character from fifo
}

void getsUart0(char* str)
{
    uint8_t counter = 0;                             // also index for string
    char c;

    while(true)
    {
        c = getcUart0();
        if (c == 8)                                 // if c = backspace
        {
            if(counter == 0)                        // if string is already empty
                counter = 0;                        // set index to 0
            else
                --counter;                          // decrement counter
        }
        else if(c == 13)                            // if c = enter key
        {
            str[counter++] = '\0';                  // null terminate
            getcUart0();                            // get remaining line feed char from remaining Uart0 buffer
            return;
        }
        else
        {
            if (c >= 65  && c <= 90)                // if upper case letter
                c += 32;                            // convert to lower case letter

            str[counter] = c;                       // place character into string
            counter++;                              // increment counter
            if (counter == (MAX_CHARS - 1))         // if max input is reached
            {
                str[counter++] = '\0';              // null terminate
                return;
            }
        }
    }
}

bool isChar(const char c)
{
    if((c >= 'A' && c <='Z') || (c >= 'a' && c <= 'z'))
        return true;
    else
        return false;
}

bool isNum(const char c)
{
    if (c >= 48 && c <= 57)     // ascii values for numbers 0-9
        return true;
    else if(c == '.' || c == '-')
        return true;
    else
        return false;
}

bool isDelimit(const char c)
{
    bool result = true;
    if(isNum(c))
        result = false;
    if(isChar(c))
        result = false;
    return result;
}

void tokenizeStr()
{
    uint8_t i=0;            			// string index
    uint8_t j=0;            			// position index
    uint8_t status=0;       			// 0=reset, 1=delimiter, 2=alpha, 3=numeric
    fieldCount = 0;

    // finding positions and number of fields
    for(i=0; strInput[i] != 0; i++)
    {
        if(i==0)                        // evaluate first character
        {
            if(isDelimit(strInput[i]))  // if char is delimiter
            {
                status = 1;             // change status to delimiter
            }
            else
            {
                pos[j++] = i;           // update starting field index
                ++fieldCount;           // increment field count

                if(isChar(strInput[i])) // if char is alphabet  
                    status = 2;         // update status to alphabet
                else                    // if number                      
                    status = 3;         // update status to numeric
            }
        }
        else
        {
            if(isDelimit(strInput[i]))
            {
                status = 1;
            }
            else if(isChar(strInput[i]))
            {
                if(status==1)           // if previous char was delimiter
                {
                    pos[j++] = i;       // update starting field position
                    ++fieldCount;       // increment field count
                }
                status = 2;
            }
            else if(isNum(strInput[i]))
            {
                if (status==1)          // if previous char was delimiter
                {
                    pos[j++] = i;       // update starting field position
                    ++fieldCount;       // increment field count
                }
                status = 3;
            }
        }
    }

    // determine field types
    for(i=0; i<fieldCount; i++)
    {
        if(isNum(strInput[pos[i]]))
            type[i] = 2;
        else
            type[i] = 1;
    }
}


// Gets command from first field
void parseCmd(uint8_t n)
{
    uint8_t index=0;
    uint8_t i;

    for(i=pos[n]; !isDelimit(strInput[i]); i++)
    {
        cmd[index++] = strInput[i];
    }
    cmd[index] = '\0';     				// null terminate string
}

// gets argument at specific position
void parseArg(uint8_t n)
{
    uint8_t i, index=0;

    for(i=pos[n]; !isDelimit(strInput[i]); i++)
    {
        arg[index++] = strInput[i];
    }
    arg[index] = '\0';
}

// n is used as the element number in position array
uint16_t getValue(uint8_t n)
{
    uint8_t i;
    uint8_t j=0;                		// temp index
    uint16_t result;	
    char temp[4];               		// to store rgb text values
    
    // get red value
    for(i=pos[n]; !isDelimit(strInput[i]); i++)
    {
        temp[j++] = strInput[i];
    }
    temp[j] = '\0';
    result = atoi(temp);
    return result;
}

bool isCommand(const char* str)
{
    bool result = false;
    
    if(strcmp(str, "help") == 0 || strcmp(str, "menu") == 0)
    {
        // help command has no args
        if(strcmp(str,cmd) == 0 && fieldCount == 1)
            result = true;
    }
    else if(strcmp(str, "rgb") == 0)
    {
        // rgb command with three number args
        if(strcmp(str,cmd) == 0 && fieldCount == 4)
            result = true;
        // if rgb off command    
        else if(strcmp(str,cmd) == 0 && fieldCount == 2)
            result = true;
    }
    else if(strcmp(str, "light") == 0)
    {
        // if light command and no arg
        if(strcmp(str, cmd) == 0 && fieldCount == 1)
            result = true;
    }
    else if(strcmp(str, "ramp") == 0)
    {
        // if ramp command with second arg being a color name
       if(strcmp(str, cmd) == 0 && fieldCount == 4 && type[1] == 1 && type[2] == 1 && type[3] == 1)
            result = true;
    }
    else if(strcmp(str, "test") == 0)
    {
        if(strcmp(str, cmd) == 0 && fieldCount == 1)
            result = true;        
    }
    else if(strcmp(str, "calibrate") == 0)
    {
        if(strcmp(str, cmd) == 0 && fieldCount == 1)
            result = true;
    }
    else if(strcmp(str, "trigger") == 0)
    {
        if(strcmp(str, cmd) == 0 && fieldCount == 1)
            result = true;
    }
    else if(strcmp(str, "button") == 0)
    {
        if(strcmp(str, cmd) == 0 && fieldCount == 1)
            result = true;
    }
    else if(strcmp(str, "periodic") == 0)
    {
        if(strcmp(str, cmd) == 0 && fieldCount == 2)
            result = true;
    }
    else if(strcmp(str, "led") == 0)
    {
        if(strcmp(str, cmd) == 0 && fieldCount == 2)
            result = true;
    }
    else if(strcmp(str, "color") == 0)
    {
        if(strcmp(str, cmd) == 0 && fieldCount == 2 && type[1] == 2)
            result = true;
    }
    else if(strcmp(str, "show") == 0)
    {
        if(strcmp(str, cmd) == 0 && fieldCount == 2 && type[1] == 2)
            result = true;
    }
    else if(strcmp(str, "erase") == 0)
    {
        if(strcmp(str, cmd) == 0 && fieldCount == 2 && type[1] == 2)
            result = true;
    }
    else if(strcmp(str, "match") == 0)
    {
        if(strcmp(str, cmd) == 0 && fieldCount == 2 && type[1] == 2)
            result = true;
    }
     else if(strcmp(str, "delta") == 0)
    {
        if(strcmp(str, cmd) == 0 && fieldCount == 2)
            result = true;
    }

    return result;
}

//-----------------------------------------------------------------------------
// EEPROM functions
//-----------------------------------------------------------------------------

void promMenu()
{
    putsUart0("\r\n");
    putsUart0("=================================================================================\r\n");
    putsUart0("                               EEPROM MENU\r\n");
    putsUart0("=================================================================================\r\n");
    putsUart0("promCalibration      - shows calibrated rgb values\r\n");
    putsUart0("promShowColors       - lists valid colors in EEPROM\r\n");
    putsUart0("promErase            - erases EEPROM to factory default\r\n");
}

// Initialize EEPROM
uint16_t enableEeprom()
{
    SYSCTL_RCGCEEPROM_R = 0x01;
    uint16_t status = EEPROMInit();
    while(status != EEPROM_INIT_OK)
    {
        status = EEPROMInit();
    }
    return status;
}

void saveColorToProm()
{
    uint32_t result = EEPROMProgram(colors, 0x0, sizeof(colors));
    if (result != 0)
        putsUart0("Status: failed to save color to EEPROM\r\n");
}

void saveCalibrationToProm()
{
    uint32_t result = EEPROMProgram(calibration, 0x400, sizeof(calibration));
    if (result != 0)
        putsUart0("Status: failed to save to calibration EEPROM\r\n");   
}

void readFromProm()
{
    char str[60];
    uint8_t i, colorCount=0;

    // read colors at address 0x0
    EEPROMRead(promColors, 0x0, sizeof(promColors));
    memcpy(colors, promColors, sizeof(promColors));

    // read calibration at address 0x400 (address/32blocks = block 32, 0 offset)
    EEPROMRead(promCalibration, 0x400, sizeof(promCalibration));
    memcpy(calibration, promCalibration, sizeof(calibration));

    // if values read all ones, then default from EEPROM and nothing written
    if(calibration[0] == 0xFFFFFFFF && calibration[1] == 0xFFFFFFFF & calibration[2] == 0xFFFFFFFF)
    {
        calibration[0] = 0;
        calibration[1] = 0;
        calibration[2] = 0;
    }

    if(calibration[0] == 0 && calibration[1] == 0 && calibration[2] == 0)
    {
        putsUart0("Status: not yet calibrated\r\n");
    }
    else
    {
        sprintf(str, "Status: Calibration restored; (%u, %u, %u)\r\n", calibration[0], calibration[1], calibration[2]);
        putsUart0(str);
    }

    for(i=0; i<16; i++)
    {
        if(colors[i][0] == 0)
        {
            colorCount++;
        }
    }
    sprintf(str, "Status: restored %u colors.\r\n", colorCount);
    putsUart0(str);
}

void promErase()
{
    char str[40] = "";
    uint32_t result = EEPROMMassErase();
    if(result == 0)
        putsUart0("Status: EEPROM erased\r\n");
    else
        sprintf(str, "Status: error code - %u\r\n", result);
        putsUart0(str);
}

void promShowColors()
{
    EEPROMRead(promColors, 0x0, sizeof(promColors));
    uint8_t i;
    char str[40];
    uint8_t colorCount = 0;
    for(i=0; i<16; i = i + 1)
    {
        if(promColors[i][0] == 0)
        {
            colorCount++;
            sprintf(str, "Color %2u: (%3u, %3u, %3u)\r\n", i, promColors[i][1], promColors[i][2], promColors[i][3]);
            putsUart0(str);
        }
    }

    if(colorCount == 0)
        putsUart0("Status: no colors saved\r\n");
}

void promShowCalibration()
{
    EEPROMRead((uint32_t*)&promCalibration, 0x400, sizeof(promCalibration));
    char str[40];

    if(promCalibration[0] == 0xFFFFFFFF && promCalibration[1] == 0xFFFFFFFF && promCalibration[2] == 0xFFFFFFFF)
    {
        putsUart0("Status: no calibration saved in EEPROM\r\n");
    }
    else
    {
        sprintf(str, "Calibration: (%u, %u, %u)\r\n", promCalibration[0], promCalibration[1], promCalibration[2]);
        putsUart0(str);
    }   
}

//-----------------------------------------------------------------------------
// Utility functions
//-----------------------------------------------------------------------------

void setRgbColor(uint16_t red, uint16_t green, uint16_t blue)
{
    PWM0_1_CMPB_R = red;
    PWM0_2_CMPA_R = blue;
    PWM0_2_CMPB_R = green;
}

uint16_t readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                    // set start bit
    while(ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                          // get single result from the FIFO
}

void waitPb1()
{
    while(PUSH_BUTTON1);
}

bool notCalibrated()
{
    if(calibration[0] == 0 && calibration[1] == 0 && calibration[2] == 0)
    {
        putsUart0("\r\n*** Incomplete calibration ***\r\n");
        return true;
    }
    return false;
}

//-----------------------------------------------------------------------------
// Command functions
//-----------------------------------------------------------------------------

void showMenu()
{
    putsUart0("\r\n");
    putsUart0("=================================================================================\r\n");
    putsUart0("                               MAIN MENU\r\n");
    putsUart0("=================================================================================\r\n");
    putsUart0("rgb [#] [#] [#]              (changes rgb to specified value)\r\n");
    putsUart0("rgb off                      (turns off all rgb)\r\n");
    putsUart0("light                        (measures light intensity)\r\n");
    putsUart0("ramp red|green|blue          (ramps up each rgb)\r\n");
    putsUart0("test                         (ramps and measures all rgb values)\r\n");
    putsUart0("calibrate                    (gets redPwm, greenPwm, and bluePwm values))\r\n");
    putsUart0("trigger                      (turns on led light calibrated rgb and displays R values)\r\n");
    putsUart0("button                       (uses SW1 to perform trigger function)\r\n");
    putsUart0("led x                        (x = on, off, or sample)\r\n");
    putsUart0("periodic T                   (T = 0 - 255 or off)\r\n");
    putsUart0("delta D                      (D = 0 - 255 or off)\r\n");
    putsUart0("match E                      (E = 0 - 255 or off)\r\n");
    putsUart0("showColors                   (shows colors saved)\r\n");
    putsUart0("help                         (show main menu)\r\n");
}

void rgbLight()
{
    uint16_t red, green, blue;      // rgb values
    red = getValue(1);
    green = getValue(2);
    blue = getValue(3);
    setRgbColor(red, green, blue);
}

void rgbOff()
{
    parseArg(1);
    if(strcmp("off",arg) == 0)
        setRgbColor(0, 0, 0);
    else
        putsUart0("\r\nStatus: Arg not off\r\n");
}

void light()
{
    uint16_t raw;
    char str[20];
    raw = readAdc0Ss3();
    sprintf(str, "ADC:  %u\r\n", raw);
    putsUart0(str);
}

void ramp()
{
    uint16_t i;

    for(i=0; i<1024; i++)
    {
        setRgbColor(i, 0, 0);
        waitMicrosecond(5000);
    }
    for(i=0; i<1024; i++)
    {
        setRgbColor(0, 0, i);
        waitMicrosecond(5000);
    }
    for(i=0; i<1024; i++)
    {
        setRgbColor(0, i, 0);
        waitMicrosecond(5000);
    }
    setRgbColor(0,0,0);
}

void test()
{
    uint16_t i;
    uint16_t raw;
    char str[40];

    for(i=0; i<1024; i++)
    {
        setRgbColor(i, 0, 0);
        waitMicrosecond(10000);
        raw = readAdc0Ss3();
        sprintf(str, "%u, 0, 0, %u\r\n", i, raw);
        putsUart0(str);
    }
    for(i=0; i<1024; i++)
    {
        setRgbColor(0, 0, i);
        waitMicrosecond(10000);
        raw = readAdc0Ss3();
        sprintf(str, "0, 0, %u, %u\r\n", i, raw);
        putsUart0(str);
    }
    for(i=0; i<1024; i++)
    {
        setRgbColor(0, i, 0);
        waitMicrosecond(10000);
        raw = readAdc0Ss3();
        sprintf(str, "0, %u, 0, %u\r\n", i, raw);
        putsUart0(str);
    }
    setRgbColor(0,0,0);
}

void calibrate()
{
    uint16_t i;
    uint16_t raw;
    char str[40];
    bool redStatus = false;
    bool greenStatus = false;
    bool blueStatus = false;

    for(i=0; i<1024; i++)
    {
        setRgbColor(i, 0, 0);
        waitMicrosecond(10000);
        raw = readAdc0Ss3();
        if (raw > T)                			// if light value reaches threshold
        {    
            calibration[0] = i - 1;     		// save redPwm value
            redStatus = true;
            break;
        }
    }
    for(i=0; i<1024; i++)
    {
        setRgbColor(0, i, 0);
        waitMicrosecond(10000);
        raw = readAdc0Ss3();
        if (raw > T)                			// if light value reaches threshold
        {
            calibration[1] = i - 1;   			// save greenPwm value
            greenStatus = true;
            break;
        }
    }
    for(i=0; i<1024; i++)
    {
        setRgbColor(0, 0, i);
        waitMicrosecond(10000);
        raw = readAdc0Ss3();
        if (raw > T)                			// if light value reaches threshold
        {
            calibration[2] = i - 1;    			// save bluePwm value
            blueStatus = true;
            break;
        }
    }
    setRgbColor(0,0,0);
    if(redStatus && greenStatus && blueStatus)
    {
        saveCalibrationToProm();
        sprintf(str, "(%u, %u, %u)\r\n", calibration[0], calibration[1], calibration[2]);
        putsUart0(str);
    }
    else
    {
        putsUart0("\r\nStatus: error calibrating\r\n");
    }
}

// shows RGB triplet raw form, capped at T value
void trigger()
{
    NVIC_EN0_R |= 0 << (INT_TIMER1A-16);     // turn-off interrupt 37 (TIMER1A)
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;         // turn-off timer
    uint16_t red, green, blue;
    char str[40];

    if(notCalibrated())
        return;

    setRgbColor(calibration[0], 0, 0);
    waitMicrosecond(10000);
    red = readAdc0Ss3();
    setRgbColor(0, calibration[1], 0);
    waitMicrosecond(10000);
    green = readAdc0Ss3();
    setRgbColor(0, 0, calibration[2]);
    waitMicrosecond(10000);
    blue = readAdc0Ss3();
    setRgbColor(0,0,0);
    sprintf(str, "(%u, %u, %u)\r\n", red, green, blue);
    putsUart0(str);

}

// shows RGB triplet in calibrated 8-bit format
void trigger2()
{
    uint16_t red, green, blue;  
    char str[40];

    // ">> 3" convert raw value of 11 bits to 8 bits
    setRgbColor(calibration[0], 0, 0);
    waitMicrosecond(10000);
    red = readAdc0Ss3() >> 3;
    setRgbColor(0, calibration[1], 0);
    waitMicrosecond(10000);
    green = readAdc0Ss3() >> 3;
    setRgbColor(0, 0, calibration[2]);
    waitMicrosecond(10000);
    blue = readAdc0Ss3() >> 3;
    setRgbColor(0,0,0);
    sprintf(str, "\r\n(%u, %u, %u)\r\n\r\n", red, green, blue);
    putsUart0(str);

}

// shows RGB triplet raw form, capped at T value
void button()
{
    NVIC_EN0_R |= 0 << (INT_TIMER1A-16);   // turn-off interrupt 37 (TIMER1A)
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;       // turn-off timer
    uint16_t red, green, blue;
    char str[40];

    if(notCalibrated())
        return;

    sprintf(str, "Press SW1 to measure\r\n");
    putsUart0(str);
    waitPb1();
    setRgbColor(calibration[0], 0, 0);
    waitMicrosecond(50000);
    red = readAdc0Ss3();
    setRgbColor(0, calibration[1], 0);
    waitMicrosecond(10000);
    green = readAdc0Ss3();
    setRgbColor(0, 0, calibration[2]);
    waitMicrosecond(10000);
    blue = readAdc0Ss3();
    setRgbColor(0,0,0);
    sprintf(str, "(%u, %u, %u)\r\n", red, green, blue);
    putsUart0(str);

}

// this function verifies parameters before calling the interrupt
void periodic()
{
    if(notCalibrated())
    {
        return;
    }
    else
    {
        if(type[1] == 1)                        // if second field is alphabetic i.e."off"
        {
            NVIC_EN0_R |= 0 << (INT_TIMER1A-16);// turn-off interrupt 37 (TIMER1A)
            TIMER1_CTL_R &= ~TIMER_CTL_TAEN;    // turn-off timer
            putsUart0("Status: periodic mode off\r\n");
        }
        else                                    // if second field is numeric
        {
            uint32_t t = getValue(1);
            if (t == 0)
            {
                NVIC_EN0_R |= 0 << (INT_TIMER1A-16);// turn-off interrupt 37 (TIMER1A)
                TIMER1_CTL_R &= ~TIMER_CTL_TAEN;    // turn-off timer
                putsUart0("Status: periodic mode off\r\n");
            }
            else
            {
                putsUart0("Status: periodic mode on\r\n");
                t = 40000000 * 0.1 * t;             // 40Mhz * units of 0.1 seconds of t
                TIMER1_TAILR_R = t;                 // set new calculated load value
                NVIC_EN0_R |= 1 << (INT_TIMER1A-16);// turn-on interrupt 37 (TIMER1A)
                TIMER1_CTL_R |= TIMER_CTL_TAEN;     // turn-on timer
            }
        }
    }
}

void periodIsr()
{
    char str[40];

    if(ledSample)
    {
        GREEN_LED = 1;
        waitMicrosecond(5000);
        GREEN_LED = 0;
    }

    setRgbColor(calibration[0], 0, 0);
    waitMicrosecond(10000);
    red = readAdc0Ss3() >> 3;
    setRgbColor(0, calibration[1], 0);
    waitMicrosecond(10000);
    green = readAdc0Ss3() >> 3;
    setRgbColor(0, 0, calibration[2]);
    waitMicrosecond(10000);
    blue = readAdc0Ss3() >> 3;
    setRgbColor(0,0,0);

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;              // clear bit (processed interrupt)

    if(!matchFlag && !deltaFlag)
    {
        sprintf(str, "\r\n(%u, %u, %u)\r\n", red, green, blue);
        putsUart0(str);
        return;
    }

    if(matchFlag)
        match();

    if(deltaFlag)
        delta();
}

void led()
{
    parseArg(1);
    if(strcmp("on", arg) == 0)
    {
        putsUart0("Status: led on\r\n");
        GREEN_LED = 1;
    }
    else if(strcmp("off", arg) == 0)
    {
        putsUart0("Status: led off\r\n");
        GREEN_LED = 0;
    }
    else if(strcmp("sample", arg) == 0)
    {
        putsUart0("Status: led sample on\r\n");
        ledSample = true;          // set flag for periodic ISR to blink light
    }
    else
        putsUart0("\r\nStatus: invalid \"led\" argument\r\n");
}

void colorN()
{
    char str[50];

    if(notCalibrated())
        return;

    uint16_t n;
    n = getValue(1);
    setRgbColor(calibration[0], 0, 0);
    waitMicrosecond(10000);
    red = readAdc0Ss3() >> 3;
    setRgbColor(0, calibration[1], 0);
    waitMicrosecond(10000);
    green = readAdc0Ss3() >> 3;
    setRgbColor(0, 0, calibration[2]);
    waitMicrosecond(10000);
    blue = readAdc0Ss3() >> 3;
    setRgbColor(0,0,0);

    // store valid bit and rgb values at index n
    colors[n][0] = 0;
    colors[n][1] = red;
    colors[n][2] = green;
    colors[n][3] = blue;
    saveColorToProm();
    sprintf(str, "Status: saved (%u, %u, %u) at index %u\r\n", red, green, blue, n);
    putsUart0(str);
}

// lights led to color N's rgb values
void showN()
{
    uint16_t n;
    n = getValue(1);
    setRgbColor(colors[n][1], colors[n][2], colors[n][3]);
    putsUart0("\r\nPress any key to continue\r\n");
    getcUart0();
    setRgbColor(0, 0, 0);
}


// list color indexes of valid colors
void showColors()
{
    uint8_t i;
    uint8_t colorCount = 0;
    char str[50];
    putsUart0("Current saved colors:\r\n");
    for(i=0; i<16; i++)
    {
        if(colors[i][0] == 0)
        {
            colorCount++;
            sprintf(str,"Color %2u:  (%3u, %3u, %3u)\r\n", i, colors[i][1], colors[i][2], colors[i][3]);
            putsUart0(str);
        }
    }

    if(colorCount == 0)
        putsUart0("Status: no colors indexed\r\n");
}

void eraseN()
{
    uint8_t n = getValue(1);
    colors[n][0] = 0xFFFFFFFF;           // 0xFFFFFFFF = invalid
}

// for current sample's distance vector, it will compare with all stored color's
// distance vector and if the difference is less than variable E, it will display
// color index
void match()
{
    float Ei;
    uint8_t i;
    uint16_t reds, greens, blues;
    char str[40];

    if(notCalibrated())
        return;

    for(i=0; i<16; i++)
    {
        if(colors[i][0] == 0)               // if valid color
        {
            // r - ri
            reds = abs(red - colors[i][1]);
            greens = abs(green - colors[i][2]);
            blues = abs(blue - colors[i][3]);
            // (r - ri)^2
            reds = pow(reds, 2);
            greens = pow(greens, 2);
            blues = pow(blues, 2);
            Ei = sqrt(reds + greens + blues);
            if(Ei < E)
            {
                sprintf(str, "Color %u\r\n", i);
                putsUart0(str);
            }
        }
    }
}

// for each sample taken periodically, if the difference between the sample and 
// the current infinite impulse response is more than variable D, it will 
// display (r, g, b) values
void delta()
{
    char str[40];
    float v, result;
    float alpha = 0.9;

    if(notCalibrated())
        return;

    // deltaD command
    v = sqrt(pow(red, 2) + pow(green, 2) + pow(blue, 2));
    iir = (alpha * iir) + (1-alpha) * v;
    result = fabs(v - iir);
    if(result > D)
    {
        sprintf(str, "(%u, %u, %u)\r\n", red, green, blue);
        putsUart0(str);
    }
}


//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    uint16_t promSuccess = 0;

    // Initialize hardware
	initHw();
    
    // Initialize EEPROM
    promSuccess = enableEeprom();
    while(promSuccess != 0)
    {
        promSuccess = enableEeprom();
    }
    readFromProm();
    
	showMenu();
    while(true)
    {
        char str[40];
        bool status = false;
        putsUart0("\r\n");
        putsUart0("Enter command: ");
        getsUart0(strInput);
        tokenizeStr();
        parseCmd(0);
        if(isCommand("help") || isCommand("menu"))
        {
            showMenu();
            status = true;
        }
        else if(isCommand("rgb"))
        {
            if(fieldCount == 4)
                rgbLight();
            else
                rgbOff();
            status = true;
        }
        else if(isCommand("light"))
        {
            light();
            status = true;
        }
        else if(isCommand("ramp"))
        {
            ramp();
            status = true;
        }
        else if(isCommand("test"))
        {
        	test();
            status = true;
        }
        else if(isCommand("calibrate"))
        {
            calibrate();
        	status = true;
        }
        else if(isCommand("trigger"))
        {
        	trigger();
        	status = true;
        }
        else if(isCommand("button"))
        {
    		button();
    		status = true;
        }
        else if(isCommand("periodic"))
        {
            periodic();
            status = true;
        }
        else if(isCommand("led"))
        {
            led();
            status = true;
        }
        else if(isCommand("color"))
        {
            colorN();
            status = true;
        }
        else if(isCommand("show"))
        {
            showN();
            status = true;
        }
        else if(isCommand("erase"))
        {
            eraseN();
            status = true;
        }
        else if(isCommand("match"))
        {
            if(type[1] == 1)
            {
                matchFlag = false;  // turn match mode off
            }
            else
            {
                matchFlag = true;   // turn match mode on
                E = getValue(1);
            }
            status = true;
        }
        else if(isCommand("delta"))
        {
            if(type[1] == 1)        // second argument is alphabetic
            {
                deltaFlag = false;  // turn delta mode off
            }
            else
            {
                deltaFlag = true;
                iir = 0;            // reset average values
                D = getValue(1);
            }
            status = true;
        }
        else if(strcmp(cmd, "showcolors") == 0)
        {
            showColors();
            status = true;
        }
        else if(strcmp(cmd,"prommenu") == 0)
        {
            promMenu();
            status = true;
        }
        else if(strcmp(cmd, "promerase") == 0)
        {
            promErase();
            status = true;
        }
        else if(strcmp(cmd, "promshowcolors") == 0)
        {
            promShowColors();
            status = true;
        }
        else if(strcmp(cmd, "promcalibration") == 0)
        {
            promShowCalibration();
            status = true;
        }


        if(!status){
            putsUart0("\r\n*** Unknown command ***\r\n");
        }

        // reset all variables
        fieldCount = 0;
        memset(pos, 0, MAX_FIELDS*sizeof(uint8_t));
        memset(type, 0, MAX_FIELDS*sizeof(uint8_t));
        memset(cmd, 0, 20*sizeof(char));
        memset(arg, 0, 20*sizeof(char));
        memset(strInput,0,(MAX_CHARS+1)*sizeof(char));
        memset(str, 0, 40*sizeof(char));
    }
}


