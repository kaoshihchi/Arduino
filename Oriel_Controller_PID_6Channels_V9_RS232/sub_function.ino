// ================================================================
// ENCODER FUNCTIONS
// ================================================================

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
    // else runningstatus = false; // Optional: Stop on missed steps
  }
  if (valueEncoder_present < valueEncoder_previous){
    diffEncoder = valueEncoder_previous - valueEncoder_present; 
    if (diffEncoder == 1)
      --*encoderValue;  //-1
    else if (diffEncoder == 3)
      ++*encoderValue;  //-3
    // else runningstatus = false; // Optional: Stop on missed steps
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

  // 3. Logic: Overshoot Approach (Backlash strategy)
  // If we are aiming for tempTarget (Overshoot) and reached it...
  if (tempTarget != targetValue) {
    // Check if we reached the overshoot target
    if (abs(*encoderValue - tempTarget) <= thresholdValue) {
      tempTarget = targetValue; // Switch target to final destination
      // Slight delay to let mechanics settle could be added here if needed
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

  // 5. Direction Setup & Gain Scheduling
  float currentKp, currentKd;
  
  if (currentError > 0) {
    // POSITIVE DIRECTION
    HbridgeHigh = pinMotorMinus_running; // Adjust pin mapping based on your wiring
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
  long errorDelta = currentError - prevError; 
  // Standard PD formula:
  float pidTerm = (currentKp * abs(currentError)) + (currentKd * abs(errorDelta));
  
  // 7. Calculate Final PWM
  int outputPWM = 0;
  
  // "Slow Area" Logic: If close to target, cap the max speed to prevent overshooting
  if (abs(currentError) < slowarea_num) {
    // Allow PID to work, but clamp the maximum result to Speed_lowest
    outputPWM = constrain((int)(pidTerm + minPWM), minPWM, Speed_lowest);
  } else {
    // Normal operation
    outputPWM = (int)(pidTerm + minPWM);
  }

  // Constrain to 8-bit PWM limits (0-255)
  outputPWM = constrain(outputPWM, 0, 255);

  // 8. Drive Motor (Hardware PWM)
  digitalWrite(pinEn_running, HIGH);   // Enable Driver
  digitalWrite(HbridgeLow, LOW);       // Ensure Low side is 0V
  analogWrite(HbridgeHigh, outputPWM); // PWM the High side

  // 9. Store error for next derivative calc
  prevError = currentError;
}

// ================================================================
// COMMAND PROCESSOR
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
      int manualPWM = Command[3]; // Use "Traveling Time" byte as PWM power
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
      
      // Note: This needs an external stop command or a non-blocking timer to handle duration.
      // Currently runs until stopped or new command.
      break;
    }

    // 0x01: Report Status
    case 0x01: {
      Respond[1] = channel_num;
      Respond[2] = runningstatus;
      Respond[3] = (*encoderValue >= 0) ? 1 : 0; // Sign
      U32toU8(abs(*encoderValue));
      Respond[4] = U8_a; Respond[5] = U8_b; Respond[6] = U8_c; Respond[7] = U8_d;
      
      for (int i = 0; i < 8; i++) serialPort.write(Respond[i]);
      break;
    }

    // 0x02: Go To Target (With Backlash Comp)
    case 0x02: {
      long rawTarget = U8toU32(Command[3], Command[4], Command[5], Command[6]);
      if (Command[7] == 0) rawTarget *= -1; // Handle sign
      
      targetValue = rawTarget;
      
      // Set Overshoot target: Aim past the target first to clear backlash
      // If we are moving Positive, aim (Target - Backlash) then go to Target?
      // Or simply aim (Target - Backlash) regardless?
      // Standard approach: Always approach final target from same direction.
      // Here we set tempTarget = target - backlash.
      tempTarget = targetValue - BACKLASH_OVERSHOOT_STEPS; 
      
      runningstatus = true;
      break;
    }

    // 0x03: Set PID Parameters
    case 0x03: {
      // Command[3] is mapped to P-gain. 
      // You can extend protocol to set Pos/Neg separately if needed.
      float newP = Command[3] * 0.001;
      Kp_Pos = newP;
      Kp_Neg = newP; // Set both for now
      
      Speed_lowest = Command[4];
      slowarea_num = Command[5] * 10;
      break;
    }

    // 0x04: Change Channel
    case 0x04: {
      SelectMotorChannel(Command[3]);
      break;
    }

    // 0x05: Set Current Position (Zeroing)
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
      digitalWrite(pinEn_running, LOW);
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
  // 1. Detach all interrupts first to prevent ghost counts
  detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch1)); detachInterrupt(digitalPinToInterrupt(pinEncoderB_ch1));
  detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch2)); detachInterrupt(digitalPinToInterrupt(pinEncoderB_ch2));
  detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch3)); detachInterrupt(digitalPinToInterrupt(pinEncoderB_ch3));
  detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch4)); detachInterrupt(digitalPinToInterrupt(pinEncoderB_ch4));
  detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch5)); detachInterrupt(digitalPinToInterrupt(pinEncoderB_ch5));
  detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch6)); detachInterrupt(digitalPinToInterrupt(pinEncoderB_ch6));

  // 2. Set Active Pins based on channel
  channel_num = ch;
  
  // Default pointers (will be overwritten by switch)
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
// Utility Functions

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
  U8_a = value >> 24; 
  U8_b = value >> 16; 
  U8_c = value >> 8; 
  U8_d = value;  
}
