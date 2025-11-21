//Sub program---------------------------------------------------------------------------------------------------------------------------
// When interupt, it count encoder steps into pointer value
//void count_A()
//{
//  stateEncoderB_running = digitalRead(pinEncoderB_running);
//    if (stateEncoderB_running == LOW)
//      ++*encoderValue;
//    if (stateEncoderB_running == HIGH)
//      --*encoderValue;
//}
//
//// When interupt, it count encoder steps into pointer value
//void count_B()
//{
//  stateEncoderA_running = digitalRead(pinEncoderA_running);
//    if (stateEncoderA_running == LOW)
//      ++*encoderValue;
//    if (stateEncoderA_running == HIGH)
//      --*encoderValue;
//}

void processSerialCommands(Stream& serialPort) {
  // Read the command from the specific serial port
  for (int i = 0; i < COMMANDLENGTH; i++) {
    Command[i] = serialPort.read();
  }

  // Check header (0xFF) and controller name (0x00 or name_due)
  if (Command[0] == 0xFF) {
    if (Command[1] == 0x00 || Command[1] == name_due){ 
      switch (Command[2]) {

        // 0x00 MoveStage: Applying voltage for [Traveling Time]
        case 0x00:
          if (Command[4] == 1) {
            HbridgeHigh = pinMotorMinus_running;
            HbridgeLow = pinMotorPlus_running;
          }
          else if (Command[4] == 0) {
            HbridgeHigh = pinMotorPlus_running;
            HbridgeLow = pinMotorMinus_running;
          }
          
          digitalWrite(pinEn_running, HIGH);
          digitalWrite(HbridgeLow, LOW);

//          digitalWrite(HbridgeHigh, HIGH);
//          delay(Command[3]);
//          digitalWrite(HbridgeHigh, LOW);
          analogWrite(HbridgeHigh, byte(pwmNumber));
          

          digitalWrite(pinEn_running, LOW);
          break;

        // 0x01 Send back current position (steps)
        case 0x01:
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
          
          // Send response to the port that sent the command!
          for (int i = 0; i < 8; i++){
           serialPort.write(Respond[i]);
          }
          
          break;

        // 0x02 Target Position: Moving to target position
        // 0x02 Target Position: Moving to target position
        case 0x02:
          // Calculate final targetValue from command bytes
          if (Command[7] == 1)
            targetValue = U8toU32(Command[3], Command[4], Command[5], Command[6]);
          else
            targetValue = (-1) * U8toU32(Command[3], Command[4], Command[5], Command[6]);
            
          // --- MODIFICATION FOR UNIDIRECTIONAL APPROACH ---
          // 1. Calculate the Overshoot Target (100 steps behind the final target)
          // This forces the final move to always be in the positive direction (increasing steps).
          tempTarget = targetValue - BACKLASH_OVERSHOOT_STEPS;
          
          // 2. Set the *first* target to the overshoot point
          // The MotorRun loop will initially target tempTarget.
          
          digitalWrite(pinEn_running, HIGH);
          runningstatus = true;
          break;
  
        // 0x03 Changing PID parameters
        case 0x03:
          pNumber = Command[3] * 0.001;
          Speed_lowest = Command[4];      // ms
          slowarea_num = Command[5] * 10; // steps
          time_loop = Command[6];         // ms
          break;

        // 0x04 Changing running motor channel
        case 0x04:
          // Detach all interrupts
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

          // Switch channel parameters and re-attach interrupts
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

        // 0x05 Write position (step)
        case 0x05:
          if (Command[7] == 1)
            *encoderValue = U8toU32(Command[3], Command[4], Command[5], Command[6]);
          else
            *encoderValue = (-1) * U8toU32(Command[3], Command[4], Command[5], Command[6]);
          break;

        // 0x06 Immergency stop
        case 0x06: 
          runningstatus = false;
          break; 

        // 0x07 Assign connected controller name
        case 0x07: 
          name_due = Command[3];
          dueFlashStorage.write(0, name_due);
          break; 

        // 0x08 Changing Motor Limit
        case 0x08:
          threshold = Command[3];
          break; 

        // 0x09 Interrupts eable/disable (Original code is commented out)
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

//---------------------------------------------------------------

void InitialEncoderState()
{
  stateEncoderA_running = digitalRead(pinEncoderA_running);
  stateEncoderB_running = digitalRead(pinEncoderB_running);
  if (stateEncoderA_running == LOW){
    if (stateEncoderB_running == LOW)
      valueEncoder_previous = 1; //0 0
    else
      valueEncoder_previous = 4; // 0 1
  }
  else{
    if (stateEncoderB_running == LOW)
      valueEncoder_previous = 2; //1 0
    else
      valueEncoder_previous = 3; //1 1
  }
}

void ReadEncoderState()
{
  stateEncoderA_running = digitalRead(pinEncoderA_running);
  stateEncoderB_running = digitalRead(pinEncoderB_running);
  
  if (stateEncoderA_running == LOW){
    if (stateEncoderB_running == LOW)
      valueEncoder_present = 1; //0 0
    else
      valueEncoder_present = 4; // 0 1
  }
  else{
    if (stateEncoderB_running == LOW)
      valueEncoder_present = 2; //1 0
    else
      valueEncoder_present = 3; //1 1
  }

  if (valueEncoder_present > valueEncoder_previous){
    diffEncoder = valueEncoder_present - valueEncoder_previous; 
    if (diffEncoder == 1)
      ++*encoderValue;  //+1
    else if (diffEncoder == 3)
      --*encoderValue;  //+3
    else
      runningstatus = false; //Missing steps
  }
  if (valueEncoder_present < valueEncoder_previous){
    diffEncoder = valueEncoder_previous - valueEncoder_present; 
    if (diffEncoder == 1)
      --*encoderValue;  //-1
    else if (diffEncoder == 3)
      ++*encoderValue;  //-3
    else
      runningstatus = false; //Missing steps
  }

  valueEncoder_previous = valueEncoder_present; 
}

// When runningstatus is true
// When runningstatus is true
// When runningstatus is true
void MotorRun() 
{
  // If runningstatus is true, it drives the motor until it gets to the targetValue.
  // If runningstatus is false, it stops output. 
  
  // --- MODIFICATION: Determine the Active Target for PID Loop ---
  int activeTarget = tempTarget;
  
  // Check if we have reached the overshoot target (tempTarget).
  // Check if: 1. Within threshold of tempTarget, AND 2. tempTarget is not already the final target.
  if (abs(*encoderValue - tempTarget) <= thresholdValue && tempTarget != targetValue) {
      // Stage has reached the overshoot target. Switch to the final target.
      tempTarget = targetValue;
      activeTarget = targetValue;
  } else if (tempTarget == targetValue) {
      // Stage is already performing the final approach.
      activeTarget = targetValue;
  }
  // The logic ensures the final move is from (targetValue - Overshoot) to targetValue (positive direction).

  errorNumber1 = abs(activeTarget - *encoderValue);
  
  // Deciding the direction
  if ((activeTarget - *encoderValue) > 0){
    errorDirection = true; // Positive direction (encoder value increasing)
  }
  else{
    errorDirection = false; // Negative direction (encoder value decreasing)
  }
  
  // Decide the direction of motor output based on errorDirection
  if (runningstatus == true){
    // Judge the timming of shutting down the motor
    if (errorNumber1 > thresholdValue){ 
      // Deciding the direction
      if (errorDirection == true) { // Needs to move positive (encoder increasing)
        HbridgeHigh = pinMotorMinus_running;
        HbridgeLow = pinMotorPlus_running;
      }
      else { // Needs to move negative (encoder decreasing)
        HbridgeHigh = pinMotorPlus_running;
        HbridgeLow = pinMotorMinus_running;
      }
      digitalWrite(HbridgeLow, LOW);
      
      //Serial1.println(pNumber * float(errorNumber1) + dNumber * float(errorNumber1 - errorNumber2));
//Correct the problem of motor speed slow
//      if (errorNumber1 == errorNumber2)
//        pwmSpeedMin++;
//      else
//        pwmSpeedMin--;
// set motor end-limit
      if (pwmNumber >= pwmSpeedMax && errorNumber1 == errorNumber2){
        fullpowercount++;
        if (fullpowercount > threshold){
          runningstatus = false; 
          fullpowercount = 0;
        }
      }
//      else
//        fullpowercount = 0;
// calculating pwm by PID
      pwmSpeedMin = constrain(pwmSpeedMin, pwmSpeedValue, 100);
      pwmNumber = constrain(pNumber * float(errorNumber1) + dNumber * float(errorNumber1 - errorNumber2), pwmSpeedMin, pwmSpeedMax);
      // Serial1.print(*encoderValue);
// Use the slowest speed to achieve the final position
      if (errorNumber1 < slowarea_num){ //500
        pwmNumber = Speed_lowest;
//10
      }
  
      // Output motor power
      digitalWrite(HbridgeHigh, HIGH);
      delay(byte(pwmNumber)); 
      digitalWrite(HbridgeHigh, LOW); 
      //analogWrite(HbridgeHigh, byte(pwmNumber)); 
      errorNumber2 = errorNumber1;
      if (errorNumber1 < slowarea_num){
        delay(time_loop);
      }
    }
    // --- MODIFICATION: End of Movement Check ---
    // If the stage is within the thresholdValue of the *final* target, stop.
    else if (errorNumber1 <= thresholdValue && activeTarget == targetValue) {
       runningstatus = false; 
    }
  }  
  else if (runningstatus == false){
    // Stop output
    analogWrite(HbridgeHigh, 0);
// Disable the motor
    digitalWrite(pinEn_running, LOW);
  
    // Serial1.print(0xff); Serial1.print(0x00);
    pwmSpeedMin = pwmSpeedValue;
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------
// Sub function

unsigned long U8toU16(int v1, int v2) {
  unsigned long y1, y2, result;
  y1 = (unsigned long)(v1) << 8;
  y2 = (unsigned long)(v2);

  result = y1 + y2;
  return result;
}

unsigned long U8toU32(int v1, int v2, int v3, int v4) {
  unsigned long y1, y2, y3, y4, y5;
  y1 = (unsigned long)(v1) << 24;
  y2 = (unsigned long)(v2) << 16;
  y3 = (unsigned long)(v3) << 8;
  y4 = (unsigned long)(v4);

  y5 = y1 + y2 + y3 + y4;
  return y5;
}

void U32toU8(unsigned long value){
  U8_a = value >>24; 
  U8_b = value >>16; 
  U8_c = value >>8; 
  U8_d = value;  
}
