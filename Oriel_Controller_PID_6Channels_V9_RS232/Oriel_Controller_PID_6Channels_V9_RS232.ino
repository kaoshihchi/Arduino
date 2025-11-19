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
  if (Serial.available() >= COMMANDLENGTH) {
    for (int i = 0; i < COMMANDLENGTH; i++) {
      Command[i] = Serial.read();
    }
    if (Command[0] == 0xFF) {
      if (Command[1] == 0x00 || Command[1] == name_due){ 
        switch (Command[2]) {
  
          // 0x00 MoveStage [FF][Controller Name][00][Traveling Time][Direction][OutputPWMPower], Applying voltage to [Direction] (0, 1) channel within [Traveling Time] (0 ~ 255 ms)
          case 0x00:
            // Deciding the motor direction
            if (Command[4] == 1) {
              HbridgeHigh = pinMotorMinus_running;
              HbridgeLow = pinMotorPlus_running;
            }
            else if (Command[4] == 0) {
              HbridgeHigh = pinMotorPlus_running;
              HbridgeLow = pinMotorMinus_running;
            }
  
            // Enable the motor
            digitalWrite(pinEn_running, HIGH);
  
            // Run the motor during delay Command[3]
            digitalWrite(HbridgeLow, LOW);
            //Serial1.print("Run");

            //analogWrite(HbridgeHigh, Command[5]);
            digitalWrite(HbridgeHigh, HIGH);
            delay(Command[3]);
            digitalWrite(HbridgeHigh, LOW);
  
            //Disable the motor
            digitalWrite(pinEn_running, LOW);
            break;
  
          // 0x01, [Controller Name] Send back current position (steps)
          case 0x01:
            //Serial1.print("Encoder Value = ");
            //[head 255][channel][runningstatus][Position direction (1: >0)][position (U8_a)][position (U8_b)][position (U8_c)][position (U8_d)]
            Respond[1] = channel_num; 
            Respond[2] = runningstatus; 
            if (*encoderValue >= 0){
              Respond[3] = 1; 
            }
            else{
              Respond[3] = 0; 
            }
            U32toU8(abs(*encoderValue)); 
            Respond[4] = U8_a; 
            Respond[5] = U8_b; 
            Respond[6] = U8_c; 
            Respond[7] = U8_d; 
            
            for (int i = 0; i < 8; i++){
             Serial.write(Respond[i]);  
            }
            
            break;
  
          // 0x02 Target Position [Controller Name]{Command[2], Command[3], Command[4], Command[5]}, Direction Command[6], Moving to target position (steps)
          case 0x02:
            if (Command[7] == 1)
              targetValue = U8toU32(Command[3], Command[4], Command[5], Command[6]);
            else
              targetValue = (-1) * U8toU32(Command[3], Command[4], Command[5], Command[6]);
  
            // Enable the motor and change running status
            digitalWrite(pinEn_running, HIGH);
            runningstatus = true; 
            
            //Serial1.print(0xff); Serial1.print(0xff);
            //Serial1.print(targetValue);
  
            // MotorRun(targetValue);
            break;
  
          // 0x03 Changing PID parameters [FF][Controller Name][03][3. pNumber][4. Speed_lowest][5. slowarea_num][6. time_loop]
          case 0x03:
            pNumber = Command[3] * 0.001;
            Speed_lowest = Command[4];      // ms
            slowarea_num = Command[5] * 10; // steps
            time_loop = Command[6];         // ms
            break;
  
          // 0x04: Changing running motor to Command[3] channel [Controller Name]
          case 0x04:
            detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch1));
            detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch2));
            detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch3));
            detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch4));
            detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch5));
            detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch6));
            
            detachInterrupt(digitalPinToInterrupt(pinEncoderB_ch1));
            detachInterrupt(digitalPinToInterrupt(pinEncoderB_ch2));
            detachInterrupt(digitalPinToInterrupt(pinEncoderB_ch3));
            detachInterrupt(digitalPinToInterrupt(pinEncoderB_ch4));
            detachInterrupt(digitalPinToInterrupt(pinEncoderB_ch5));
            detachInterrupt(digitalPinToInterrupt(pinEncoderB_ch6));

            switch (Command[3]) {
              case 0x01:
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
                break;
  
              case 0x02:
                channel_num = 2; 
                pinEncoderA_running = pinEncoderA_ch2;
                pinEncoderB_running = pinEncoderB_ch2;
                pinMotorMinus_running = pinMotorMinus_ch2;
                pinMotorPlus_running = pinMotorPlus_ch2;
                pinEn_running = pinEn_ch2;
  
                encoderValue = &encoderValue_ch2;

                InitialEncoderState();
                attachInterrupt(digitalPinToInterrupt(pinEncoderA_ch2), ReadEncoderState, CHANGE);
                attachInterrupt(digitalPinToInterrupt(pinEncoderB_ch2), ReadEncoderState, CHANGE);
                break;
  
              case 0x03:
                channel_num = 3; 
                pinEncoderA_running = pinEncoderA_ch3;
                pinEncoderB_running = pinEncoderB_ch3;
                pinMotorMinus_running = pinMotorMinus_ch3;
                pinMotorPlus_running = pinMotorPlus_ch3;
                pinEn_running = pinEn_ch3;
  
                encoderValue = &encoderValue_ch3;

                InitialEncoderState();
                attachInterrupt(digitalPinToInterrupt(pinEncoderA_ch3), ReadEncoderState, CHANGE);
                attachInterrupt(digitalPinToInterrupt(pinEncoderB_ch3), ReadEncoderState, CHANGE);
                break;
  
              case 0x04:
                channel_num = 4; 
                pinEncoderA_running = pinEncoderA_ch4;
                pinEncoderB_running = pinEncoderB_ch4;
                pinMotorMinus_running = pinMotorMinus_ch4;
                pinMotorPlus_running = pinMotorPlus_ch4;
                pinEn_running = pinEn_ch4;
  
                encoderValue = &encoderValue_ch4;

                InitialEncoderState();
                attachInterrupt(digitalPinToInterrupt(pinEncoderA_ch4), ReadEncoderState, CHANGE);
                attachInterrupt(digitalPinToInterrupt(pinEncoderB_ch4), ReadEncoderState, CHANGE);
                break;
  
              case 0x05:
                channel_num = 5; 
                pinEncoderA_running = pinEncoderA_ch5;
                pinEncoderB_running = pinEncoderB_ch5;
                pinMotorMinus_running = pinMotorMinus_ch5;
                pinMotorPlus_running = pinMotorPlus_ch5;
                pinEn_running = pinEn_ch5;

                encoderValue = &encoderValue_ch5;
                
                InitialEncoderState();
                attachInterrupt(digitalPinToInterrupt(pinEncoderA_ch5), ReadEncoderState, CHANGE);
                attachInterrupt(digitalPinToInterrupt(pinEncoderB_ch5), ReadEncoderState, CHANGE);
                break;
  
              case 0x06:
                channel_num = 6; 
                pinEncoderA_running = pinEncoderA_ch6;
                pinEncoderB_running = pinEncoderB_ch6;
                pinMotorMinus_running = pinMotorMinus_ch6;
                pinMotorPlus_running = pinMotorPlus_ch6;
                pinEn_running = pinEn_ch6;
  
                encoderValue = &encoderValue_ch6;

                InitialEncoderState();
                attachInterrupt(digitalPinToInterrupt(pinEncoderA_ch6), ReadEncoderState, CHANGE);
                attachInterrupt(digitalPinToInterrupt(pinEncoderB_ch6), ReadEncoderState, CHANGE);
                break;
            }
            break;
  
          // 0x05: Write position (step) [Controller Name]{Command[3]}
          case 0x05:
            if (Command[7] == 1)
              *encoderValue = U8toU32(Command[3], Command[4], Command[5], Command[6]);
            else
              *encoderValue = (-1) * U8toU32(Command[3], Command[4], Command[5], Command[6]);
            //Serial1.print("Encoder Value = ");
            //Serial1.print(*encoderValue);
            break;
  
          // 0x06: Immergency stop [Controller Name]
          case 0x06: 
            runningstatus = false; 
            break; 

          // 0x07: Assign connected controller name [Controller Name], save name to address 0. 
          case 0x07: 
            name_due = Command[3]; 
            dueFlashStorage.write(0, name_due); 
            break; 

          // 0x08: Changing Motor Limit [FF][Controller Name][08][3. threshold]
          case 0x08:
            threshold = Command[3];
            break; 

          // 0x09: Interrupts eable/disable [FF][Controller Name][09][3. Interrupt eable/disable]
          case 0x09:
            if (Command[3] == 0){
//              detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch1));
//              detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch2));
//              detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch3));
//              detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch4));
//              detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch5));
//              detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch6));
            }
            else if (Command[3] == 1){
//              attachInterrupt(digitalPinToInterrupt(pinEncoderA_ch1), count, RISING);
//              attachInterrupt(digitalPinToInterrupt(pinEncoderA_ch2), count, RISING);
//              attachInterrupt(digitalPinToInterrupt(pinEncoderA_ch3), count, RISING);
//              attachInterrupt(digitalPinToInterrupt(pinEncoderA_ch4), count, RISING);
//              attachInterrupt(digitalPinToInterrupt(pinEncoderA_ch5), count, RISING);
//              attachInterrupt(digitalPinToInterrupt(pinEncoderA_ch6), count, RISING);
            }
            break;
        }
      }
    }
  }
  // Serial1.print(runningstatus); 
  MotorRun(); 
}
