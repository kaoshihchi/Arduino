
#define COMMANDLENGTH 15
#define RESPONDLENGTH 10

// serial port parameters------------------------------------------------------------------------------------
unsigned char Command[COMMANDLENGTH];       // The Current Command For The Arduino To Process
byte Respond[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // The Current Respond For The Arduino To Process

//-----------------------------------------------------------------------------------------------------------

// DueFlashStorage parameters------------------------------------------------------------------------------------
// name of the controller
unsigned char name_due = 0x01;
//-----------------------------------------------------------------------------------------------------------

// motor control parameters-------------------------------------------------------------------------------------------------
byte U8_a, U8_b, U8_c, U8_d;
byte position_direction = 0;
byte channel_num = 1;

int thresholdValue = 1;
byte fullpowercount = 0;

int errorNumber1, errorNumber2 = 0;
bool errorDirection = false;
float pNumber, iNumber, dNumber, pwmNumber = 0;
byte pwmSpeedMax = 0;
byte pwmSpeedMin = 0;
byte pwmSpeedValue = 1;
byte Speed_lowest = 3;
int time_loop = 20;
byte threshold = 10;
int slowarea_num = 500;

// Using pointer to send encoder signal to each channels
int *encoderValue;
int stateEncoderB_running = 0;
int stateEncoderA_running = 0;
int valueEncoder_present = 2; 
int valueEncoder_previous = 1; 
int diffEncoder = 1; 
bool runningstatus         = false;
bool motorlimit            = false;

void count(void);
void MotorRun(void);
void processSerialCommands(Stream& serialPort);

int encoderValue_ch1 = 0;
int encoderValue_ch2 = 0;
int encoderValue_ch3 = 0;
int encoderValue_ch4 = 0;
int encoderValue_ch5 = 0;
int encoderValue_ch6 = 0;

//-----------------------------------------------------------------------------------------------------------

//Pin Number-------------------------------------------------------------------------------------------------
byte pinEncoderA_running = 30;
byte pinEncoderB_running = 31;
byte pinMotorMinus_running = 2;
byte pinMotorPlus_running = 3;
byte pinEn_running = 42;
int targetValue = 0;

byte pinEncoderA_ch1 = 30;
byte pinEncoderB_ch1 = 31;
byte pinMotorMinus_ch1 = 2;
byte pinMotorPlus_ch1 = 3;
byte pinEn_ch1 = 42;

byte pinEncoderA_ch2 = 32;
byte pinEncoderB_ch2 = 33;
byte pinMotorMinus_ch2 = 4;
byte pinMotorPlus_ch2 = 5;
byte pinEn_ch2 = 43;

byte pinEncoderA_ch3 = 34;
byte pinEncoderB_ch3 = 35;
byte pinMotorMinus_ch3 = 6;
byte pinMotorPlus_ch3 = 7;
byte pinEn_ch3 = 44;

byte pinEncoderA_ch4 = 36;
byte pinEncoderB_ch4 = 37;
byte pinMotorMinus_ch4 = 8;
byte pinMotorPlus_ch4 = 9;
byte pinEn_ch4 = 45;

byte pinEncoderA_ch5 = 38;
byte pinEncoderB_ch5 = 39;
byte pinMotorMinus_ch5 = 10;
byte pinMotorPlus_ch5 = 11;
byte pinEn_ch5 = 46;

byte pinEncoderA_ch6 = 40;
byte pinEncoderB_ch6 = 41;
byte pinMotorMinus_ch6 = 12;
byte pinMotorPlus_ch6 = 13;
byte pinEn_ch6 = 47;

byte HbridgeHigh = pinMotorMinus_running;
byte HbridgeLow = pinMotorPlus_running;
//-----------------------------------------------------------------------------------------------------------
// --- ADD to motor control parameters in variables.h ---

// CONSTANT: Adjust this value based on your stage's measured backlash (steps). 
// It must be greater than the maximum mechanical backlash.
const int BACKLASH_OVERSHOOT_STEPS = 200; 

// VARIABLE: Holds the temporary target during the two-step move.
int tempTarget = 0;
