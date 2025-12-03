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

// ================================================================
// REVISED MOTOR CONTROL LOOP (Non-Blocking / Directional)
// ================================================================
void MotorRun() {
  // 1. Safety & Idle Check
  if (!runningstatus) {
    analogWrite(HbridgeHigh, 0);   // Stop PWM
    analogWrite(HbridgeLow, 0);    // Safety
    digitalWrite(pinEn_running, LOW); // Disable Driver
    return; // Exit function
  }

  // 2. Sample Rate Control (Ensure PID runs at fixed frequency)
  if (millis() - lastPIDTime < loopTimeMS) {
    return; // Wait for next sample tick
  }
  lastPIDTime = millis();

  // 3. Logic: Overshoot Approach (Your original Backlash strategy)
  // If we are aiming for tempTarget and reached it, switch to final targetValue
  if (tempTarget != targetValue) {
    if (abs(*encoderValue - tempTarget) <= thresholdValue) {
      tempTarget = targetValue; // Switch to final approach
    }
  }
  
  long activeTarget = tempTarget;
  long currentError = activeTarget - *encoderValue;
  
  // 4. Check if Target Reached (Exit Condition)
  // Only stop if we are targeting the FINAL target and are within threshold
  if (activeTarget == targetValue && abs(currentError) <= thresholdValue) {
    runningstatus = false;
    analogWrite(HbridgeHigh, 0);
    digitalWrite(pinEn_running, LOW);
    return;
  }

  // 5. Direction & Gain Scheduling
  float currentKp, currentKd;
  
  if (currentError > 0) {
    // POSITIVE DIRECTION
    HbridgeHigh = pinMotorMinus_running; // Adjust pin mapping if reversed
    HbridgeLow = pinMotorPlus_running; 
    currentKp = Kp_Pos;
    currentKd = Kd_Pos;
  } else {
    // NEGATIVE DIRECTION
    HbridgeHigh = pinMotorPlus_running;
    HbridgeLow = pinMotorMinus_running;
    currentKp = Kp_Neg;
    currentKd = Kd_Neg;
  }

  // 6. PID Calculation
  long errorDelta = currentError - errorNumber2; // errorNumber2 is "prevError"
  float pidTerm = (currentKp * abs(currentError)) + (currentKd * abs(errorDelta));
  
  // 7. Calculate Final PWM
  int outputPWM = 0;
  
  // If we are in the "Slow Area" (Close to target), cap the speed
  if (abs(currentError) < slowarea_num) {
    // In slow area, use calculated PID but cap it at Speed_lowest if it gets too high? 
    // Or strictly force Speed_lowest? Your original code forced it.
    // Ideally: allow PID to work, but cap max speed.
    outputPWM = constrain(pidTerm + minPWM, minPWM, Speed_lowest);
  } else {
    // Normal operation
    outputPWM = (int)(pidTerm + minPWM);
  }

  // Constrain to 8-bit PWM limits
  outputPWM = constrain(outputPWM, 0, 255);

  // 8. Drive Motor (Hardware PWM)
  digitalWrite(pinEn_running, HIGH); // Enable Driver
  digitalWrite(HbridgeLow, LOW);     // Ensure Low side is 0V
  analogWrite(HbridgeHigh, outputPWM); // PWM the High side

  // 9. Store error for next derivative calc
  errorNumber2 = currentError;
}

// ================================================================
// CLEANED COMMAND PROCESSOR
// ================================================================
void processSerialCommands(Stream& serialPort) {
  // Read Command
  if (serialPort.available() < COMMANDLENGTH) return;
  
  for (int i = 0; i < COMMANDLENGTH; i++) {
    Command[i] = serialPort.read();
  }

  // Header Check
  if (Command[0] != 0xFF) return;
  if (Command[1] != 0x00 && Command[1] != name_due) return;

  // Execute Command
  switch (Command[2]) {
    
    // 0x00: Open Loop Move (Manual PWM)
    case 0x00: {
      int manualPWM = Command[3]; // Use "Traveling Time" byte as PWM power for now
      int dir = Command[4];
      
      if (dir == 1) {
        HbridgeHigh = pinMotorMinus_running;
        HbridgeLow = pinMotorPlus_running;
      } else {
        HbridgeHigh = pinMotorPlus_running;
        HbridgeLow = pinMotorMinus_running;
      }
      digitalWrite(pinEn_running, HIGH);
      digitalWrite(HbridgeLow, LOW);
      analogWrite(HbridgeHigh, manualPWM); 
      // Note: This needs a separate timer to stop if you want it timed. 
      // Currently, it just sets speed.
      break;
    }

    // 0x01: Report Status
    case 0x01: {
      Respond[1] = channel_num;
      Respond[2] = runningstatus;
      Respond[3] = (*encoderValue >= 0) ? 1 : 0;
      U32toU8(abs(*encoderValue));
      Respond[4] = U8_a; Respond[5] = U8_b; Respond[6] = U8_c; Respond[7] = U8_d;
      
      for (int i = 0; i < 8; i++) serialPort.write(Respond[i]);
      break;
    }

    // 0x02: Go To Target
    case 0x02: {
      long rawTarget = U8toU32(Command[3], Command[4], Command[5], Command[6]);
      if (Command[7] == 0) rawTarget *= -1; // Handle sign
      
      targetValue = rawTarget;
      
      // Set Overshoot target for backlash compensation
      // Note: BACKLASH_OVERSHOOT_STEPS must be defined in variables.h
      tempTarget = targetValue - 100; 
      
      runningstatus = true;
      break;
    }

    // 0x03: Set Parameters (Updated for Directional PID)
    case 0x03: {
      // You can repurpose bytes to set Pos/Neg gains separately later.
      // For now, setting both to the incoming value.
      float newP = Command[3] * 0.001;
      Kp_Pos = newP;
      Kp_Neg = newP; // Set both initially
      
      Speed_lowest = Command[4];
      slowarea_num = Command[5] * 10;
      break;
    }

    // 0x04: Change Channel
    case 0x04: {
      SelectMotorChannel(Command[3]);
      break;
    }

    // 0x05: Set Current Position
    case 0x05: {
      long newVal = U8toU32(Command[3], Command[4], Command[5], Command[6]);
      if (Command[7] == 0) newVal *= -1;
      *encoderValue = newVal;
      break;
    }

    // 0x06: E-Stop
    case 0x06: {
      runningstatus = false;
      analogWrite(HbridgeHigh, 0); // Cut power immediately
      break;
    }
    
    // 0x07: Set Name
    case 0x07: {
      name_due = Command[3];
      dueFlashStorage.write(0, name_due);
      break;
    }
  }
}

// ================================================================
// HELPER: CHANNEL SWITCHING
// ================================================================
void SelectMotorChannel(int ch) {
  // 1. Detach all interrupts first
  detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch1));
  detachInterrupt(digitalPinToInterrupt(pinEncoderB_ch1));
  detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch2));
  detachInterrupt(digitalPinToInterrupt(pinEncoderB_ch2));
  detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch3));
  detachInterrupt(digitalPinToInterrupt(pinEncoderB_ch3));
  detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch4));
  detachInterrupt(digitalPinToInterrupt(pinEncoderB_ch4));
  detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch5));
  detachInterrupt(digitalPinToInterrupt(pinEncoderB_ch5));
  detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch6));
  detachInterrupt(digitalPinToInterrupt(pinEncoderB_ch6));

  // 2. Set Active Pins based on channel
  channel_num = ch;
  switch (ch) {
    case 1:
      pinEncoderA_running = pinEncoderA_ch1; pinEncoderB_running = pinEncoderB_ch1;
      pinMotorMinus_running = pinMotorMinus_ch1; pinMotorPlus_running = pinMotorPlus_ch1;
      pinEn_running = pinEn_ch1; encoderValue = &encoderValue_ch1;
      break;
    case 2:
      pinEncoderA_running = pinEncoderA_ch2; pinEncoderB_running = pinEncoderB_ch2;
      pinMotorMinus_running = pinMotorMinus_ch2; pinMotorPlus_running = pinMotorPlus_ch2;
      pinEn_running = pinEn_ch2; encoderValue = &encoderValue_ch2;
      break;
    case 3:
      pinEncoderA_running = pinEncoderA_ch3; pinEncoderB_running = pinEncoderB_ch3;
      pinMotorMinus_running = pinMotorMinus_ch3; pinMotorPlus_running = pinMotorPlus_ch3;
      pinEn_running = pinEn_ch3; encoderValue = &encoderValue_ch3;
      break;
    case 4:
      pinEncoderA_running = pinEncoderA_ch4; pinEncoderB_running = pinEncoderB_ch4;
      pinMotorMinus_running = pinMotorMinus_ch4; pinMotorPlus_running = pinMotorPlus_ch4;
      pinEn_running = pinEn_ch4; encoderValue = &encoderValue_ch4;
      break;
    case 5:
      pinEncoderA_running = pinEncoderA_ch5; pinEncoderB_running = pinEncoderB_ch5;
      pinMotorMinus_running = pinMotorMinus_ch5; pinMotorPlus_running = pinMotorPlus_ch5;
      pinEn_running = pinEn_ch5; encoderValue = &encoderValue_ch5;
      break;
    case 6:
      pinEncoderA_running = pinEncoderA_ch6; pinEncoderB_running = pinEncoderB_ch6;
      pinMotorMinus_running = pinMotorMinus_ch6; pinMotorPlus_running = pinMotorPlus_ch6;
      pinEn_running = pinEn_ch6; encoderValue = &encoderValue_ch6;
      break;
  }

  // 3. Re-initialize and Re-attach interrupts for selected channel
  InitialEncoderState();
  attachInterrupt(digitalPinToInterrupt(pinEncoderA_running), ReadEncoderState, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncoderB_running), ReadEncoderState, CHANGE);
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
