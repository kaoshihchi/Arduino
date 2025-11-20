// from 20190926
// final edited 202205
// note: 
//  20190926, Multiple controller: Use Command[1] as controller name, so we should shift all of the parameters 1 register. 
//  0x00 is the administor name. Use this name could controll all of the controller. 
// ###############################################################################################################################################
// note of Oriel controller library: 
// Command[1] is the key of the command. Command[2] is the name of the controller. 
// 0x00: Move Stage during a while[Controller Name][Traveling Time][Direction][OutputPWMPower], Applying voltage to [Direction] (0, 1) channel within [Traveling Time] (0 ~ 255 ms)
// 0x01: [Controller Name] Send back current position (steps)
// 0x02: Target Position [Controller Name]{Command[2], Command[3], Command[4], Command[5]}, Direction Command[6], Moving to target position (steps)
// 0x03: Changing PID parameters [Controller Name]
// 0x04: Changing running motor to Command[3] channel [Controller Name]
// 0x05: Write position (step){Command[3]} [Controller Name]
// 0x06: Immergency stop [Controller Name]
// 0x07: Assign connected controller name [Controller Name], save name to address 0. 

// 20200820: PWM function may cause the controller miss encoder signal. 
// 20200915: Abandon PWM function
// 20201203: Fix 0x01 function, it will send runningstatus, channelnumber, position. 
// 20201228: Respond controller status to PC. Fix encoder signal missing problem. Improve accuracy by PID parameters. 
// 20210929: Edited from PID control fine tune. 0x03 Changing PID parameters [Controller Name]
// 20210930: // 0x08: Changing Motor Limit [FF][Controller Name][08][3. threshold]
//           // 0x09: Interrupts eable/disable [FF][Controller Name][09][3. Interrupt eable/disable]
//20220525: attachInterrupt when enable the motor. 

// #include <EEPROM.h>, due do not have EEPROM, so we should use dueFlashStorage.write(0,123);
#include <DueFlashStorage.h>
#include "variables.h"
DueFlashStorage dueFlashStorage;

// In Oriel_Controller_PID_6Channels_V9_RS232.ino, before setup()
void processSerialCommands(Stream& serialPort);

// the setup routine runs once when you press reset:
void setup() {
  // initialize Serial1 communication at 9600 bits per second:
  Serial.begin(9600);
  Serial1.begin(19200);
  Respond[0] = 255; 

  // initialize the name of the controller. load the name from flash. 
  name_due = dueFlashStorage.read(0); 

  // Define Pin Mode
  pinMode(pinMotorMinus_ch1, OUTPUT);
  pinMode(pinMotorPlus_ch1, OUTPUT);
  pinMode(pinEncoderA_ch1, INPUT);
  pinMode(pinEncoderB_ch1, INPUT);
  pinMode(pinEn_ch1, OUTPUT);

  pinMode(pinMotorMinus_ch2, OUTPUT);
  pinMode(pinMotorPlus_ch2, OUTPUT);
  pinMode(pinEncoderA_ch2, INPUT);
  pinMode(pinEncoderB_ch2, INPUT);
  pinMode(pinEn_ch2, OUTPUT);

  pinMode(pinMotorMinus_ch3, OUTPUT);
  pinMode(pinMotorPlus_ch3, OUTPUT);
  pinMode(pinEncoderA_ch3, INPUT);
  pinMode(pinEncoderB_ch3, INPUT);
  pinMode(pinEn_ch3, OUTPUT);

  pinMode(pinMotorMinus_ch4, OUTPUT);
  pinMode(pinMotorPlus_ch4, OUTPUT);
  pinMode(pinEncoderA_ch4, INPUT);
  pinMode(pinEncoderB_ch4, INPUT);
  pinMode(pinEn_ch4, OUTPUT);

  pinMode(pinMotorMinus_ch5, OUTPUT);
  pinMode(pinMotorPlus_ch5, OUTPUT);
  pinMode(pinEncoderA_ch5, INPUT);
  pinMode(pinEncoderB_ch5, INPUT);
  pinMode(pinEn_ch5, OUTPUT);

  pinMode(pinMotorMinus_ch6, OUTPUT);
  pinMode(pinMotorPlus_ch6, OUTPUT);
  pinMode(pinEncoderA_ch6, INPUT);
  pinMode(pinEncoderB_ch6, INPUT);
  pinMode(pinEn_ch6, OUTPUT);

  // Initial Parameters to channel 1:
  encoderValue = &encoderValue_ch1;
  runningstatus = false;

  channel_num = 1; 
  pinEncoderA_running = pinEncoderA_ch1;
  pinEncoderB_running = pinEncoderB_ch1;
  pinMotorMinus_running = pinMotorMinus_ch1;
  pinMotorPlus_running = pinMotorPlus_ch1;
  pinEn_running = pinEn_ch1;

  encoderValue = &encoderValue_ch1;
  
  InitialEncoderState();
  attachInterrupt(digitalPinToInterrupt(pinEncoderA_ch1), ReadEncoderState, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncoderB_ch1), ReadEncoderState, CHANGE);

  // PID Controll Parameters:
  pNumber = 20 * 0.001;
  dNumber = 0 * 0.001;
  iNumber = 0;
  pwmSpeedMax = 50;
  pwmSpeedMin = pwmSpeedValue;

  // Test serial port
  // Serial1.println("Hello Serial 1");
  // Serial.println("Hello Serial 0");

}

// the loop routine runs over and over again forever:
void loop() {
  
  // Check Serial Port 0 (USB/Programming Port)
  if (Serial.available() >= COMMANDLENGTH) {
    processSerialCommands(Serial);
  }

  // Check Serial Port 1 (RS-232 Port)
  if (Serial1.available() >= COMMANDLENGTH) {
    processSerialCommands(Serial1);
  }

  // Motor control loop runs regardless of serial communication
  MotorRun(); 
}
