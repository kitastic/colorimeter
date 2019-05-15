#ifndef COLORIMETER_H__
#define COLORIMETER_H__

#define MAX_CHARS 80        // max number of chars from user input
#define MAX_FIELDS 5
#define PUSH_BUTTON1    (*((volatile uint32_t*)(0x42000000 + (0x400253FC - 0x40000000)*32 + 4*4)))
#define GREEN_LED       (*((volatile uint32_t*)(0x42000000 + (0x400253FC - 0x40000000)*32 + 3*4)))

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

char strInput[MAX_CHARS+1];          // plus 1 for \0
char cmd[20];                       // parsed command
char arg[20];                       // string arument
uint8_t pos[MAX_FIELDS];            // string_token: start index of each field
uint8_t type[MAX_FIELDS];           // string_token: 1=alphabet, 2=number
uint8_t fieldCount;                 // string_token: number of fields
uint16_t T;                         // Threshold value (2^n)-1
uint16_t red;
uint16_t green;
uint16_t blue;
uint32_t colors[16][4];             // columns: valid, red, green, blue
uint32_t promColors[16][4];
uint32_t calibration[3];            // redPwm, greenPwm, bluePwm
uint32_t promCalibration[3];
uint16_t E;                          // match E command
uint16_t D;                          // delta D command
float iir;
bool ledSample;                     // a flag to tell LED interrupt to flash 
bool matchFlag;                     // match mode indicator
bool deltaFlag;                     // delta mode indicator


//-----------------------------------------------------------------------------
// Initialize Hardware
//-----------------------------------------------------------------------------

void initHw();

//-----------------------------------------------------------------------------
// String/Tokenizing fuctions
//-----------------------------------------------------------------------------

void putcUart0(char);
void putsUart0(char*);
char getcUart0();
void getsUart0(char*);
bool ischar(const char);
bool isNum(const char);
bool isDelimit(const char);
void tokenizeStr();
void parseCmd(uint8_t);
void parseArg(uint8_t);
uint16_t getValue(uint8_t);
bool isCommand(const char*);

//-----------------------------------------------------------------------------
// EEPROM functions
//-----------------------------------------------------------------------------

void promMenu();
uint16_t enableEeprom();
void saveColorToProm();
void saveCalibrationToProm();
void readFromProm();
void promErase();
void promShowColors();
void promShowCalibration();

//-----------------------------------------------------------------------------
// Utility functions
//-----------------------------------------------------------------------------

void setRgbColor(uint16_t, uint16_t, uint16_t);
uint16_t readAdc0Ss3();
void waitPb1();
bool notCalibrated();

//-----------------------------------------------------------------------------
// Command functions
//-----------------------------------------------------------------------------

void showMenu();
void rgbLight();
void rgbOff();
void light();
void ramp();
void test();
void calibrate();
void trigger();
void trigger2();
void button();
void periodic();
void periodIsr();
void led();
void colorN();
void showN();
void showColors();
void eraseN();
void match();
void delta();

#endif
