// from 20190926
// final edited 202205
// 2023-10 Update: Implemented Directional PID and Non-Blocking PWM

#include <DueFlashStorage.h>
#include "variables.h"
DueFlashStorage dueFlashStorage;

// the setup routine runs once when you press reset:
void setup() {
  // initialize Serial1 communication at 9600 bits per second:
  Serial.begin(9600);
  Serial1.begin(19200);
  Respond[0] = 255; 

  // initialize the name of the controller. load the name from flash. 
  name_due = dueFlashStorage.read(0); 

  // Define Pin Mode for all channels
  pinMode(pinMotorMinus_ch1, OUTPUT); pinMode(pinMotorPlus_ch1, OUTPUT);
  pinMode(pinEncoderA_ch1, INPUT);    pinMode(pinEncoderB_ch1, INPUT);
  pinMode(pinEn_ch1, OUTPUT);

  pinMode(pinMotorMinus_ch2, OUTPUT); pinMode(pinMotorPlus_ch2, OUTPUT);
  pinMode(pinEncoderA_ch2, INPUT);    pinMode(pinEncoderB_ch2, INPUT);
  pinMode(pinEn_ch2, OUTPUT);

  pinMode(pinMotorMinus_ch3, OUTPUT); pinMode(pinMotorPlus_ch3, OUTPUT);
  pinMode(pinEncoderA_ch3, INPUT);    pinMode(pinEncoderB_ch3, INPUT);
  pinMode(pinEn_ch3, OUTPUT);

  pinMode(pinMotorMinus_ch4, OUTPUT); pinMode(pinMotorPlus_ch4, OUTPUT);
  pinMode(pinEncoderA_ch4, INPUT);    pinMode(pinEncoderB_ch4, INPUT);
  pinMode(pinEn_ch4, OUTPUT);

  pinMode(pinMotorMinus_ch5, OUTPUT); pinMode(pinMotorPlus_ch5, OUTPUT);
  pinMode(pinEncoderA_ch5, INPUT);    pinMode(pinEncoderB_ch5, INPUT);
  pinMode(pinEn_ch5, OUTPUT);

  pinMode(pinMotorMinus_ch6, OUTPUT); pinMode(pinMotorPlus_ch6, OUTPUT);
  pinMode(pinEncoderA_ch6, INPUT);    pinMode(pinEncoderB_ch6, INPUT);
  pinMode(pinEn_ch6, OUTPUT);

  // Initial Parameters to channel 1:
  encoderValue = &encoderValue_ch1;
  runningstatus = false;

  // Use the new helper function to set up Channel 1
  SelectMotorChannel(1);

  // Initialize PID Controll Parameters:
  float initialP = 20 * 0.001;
  Kp_Pos = initialP;
  Kp_Neg = initialP;
  Kd_Pos = 0.0;
  Kd_Neg = 0.0;
  
  pwmSpeedMax = 255; // 8-bit max
  minPWM = 30;       // Stiction compensation
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
