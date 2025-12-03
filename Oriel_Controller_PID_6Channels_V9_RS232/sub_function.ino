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
  }
  if (valueEncoder_present < valueEncoder_previous){
    diffEncoder = valueEncoder_previous - valueEncoder_present; 
    if (diffEncoder == 1)
      --*encoderValue;  //-1
    else if (diffEncoder == 3)
      ++*encoderValue;  //-3
  }

  valueEncoder_previous = valueEncoder_present; 
}

// ================================================================
// REVISED MOTOR CONTROL LOOP (With Motion Profiling - No Slow Area Clamp)
// ================================================================
void MotorRun() {
  // 1. Safety & Idle Check
  if (!runningstatus) {
    analogWrite(HbridgeHigh, 0);   
    analogWrite(HbridgeLow, 0);    
    digitalWrite(pinEn_running, LOW); 
    return; 
  }

  // 2. Sample Rate Control (5ms loop)
  if (millis() - lastPIDTime < loopTimeMS) {
    return; 
  }
  lastPIDTime = millis();

  // --- RAMP GENERATOR (Motion Profiling) ---
  long distToFinal = finalTarget - tempTarget;

  if (distToFinal != 0) {
    // If we are far away, move tempTarget by rampStep (Velocity Limit)
    if (abs(distToFinal) > rampStep) {
      if (distToFinal > 0) tempTarget += rampStep;
      else                 tempTarget -= rampStep;
    } else {
      // If we are close (less than one step size), snap to final
      tempTarget = finalTarget;
    }
  }
  
  // --- BACKLASH HANDLING & COMPLETION ---
  long currentPos = *encoderValue;
  
  // Check if we reached the current ramp goal (tempTarget)
  if (tempTarget == finalTarget && abs(currentPos - tempTarget) <= thresholdValue) {
      
      // If this goal was the Overshoot target, now switch to Real Target
      if (finalTarget != targetValue) {
          finalTarget = targetValue; 
      } 
      // If this goal WAS the Real Target, we are done.
      else {
          runningstatus = false;
          analogWrite(HbridgeHigh, 0);
          digitalWrite(pinEn_running, LOW);
          return;
      }
  }

  // --- PID CONTROL ---
  long currentError = tempTarget - currentPos;
  
  // Direction Setup
  float currentKp, currentKd;
  if (currentError > 0) {
    HbridgeHigh = pinMotorMinus_running; 
    HbridgeLow = pinMotorPlus_running; 
    currentKp = Kp_Pos;
    currentKd = Kd_Pos;
  } else {
    HbridgeHigh = pinMotorPlus_running;
    HbridgeLow = pinMotorMinus_running;
    currentKp = Kp_Neg;
    currentKd = Kd_Neg;
  }

  // PID Math
  long errorDelta = currentError - prevError; 
  float pidTerm = (currentKp * abs(currentError)) + (currentKd * abs(errorDelta));
  
  // Output Generation
  int outputPWM = 0;
  
  // --- UPDATE: Removed Slow Area Constraint ---
  // We rely on PID + MinPWM to drive the motor into position.
  // The Ramp Generator already handles velocity profiling.
  outputPWM = (int)(pidTerm + minPWM);
  
  // Hard Limits (User defined Max)
  outputPWM = constrain(outputPWM, 0, pwmSpeedMax);

  // Drive
  digitalWrite(pinEn_running, HIGH);
  digitalWrite(HbridgeLow, LOW);
  analogWrite(HbridgeHigh, outputPWM);

  prevError = currentError;
}

// ================================================================
// COMMAND PROCESSOR
// ================================================================
void processSerialCommands(Stream& serialPort) {
  if (serialPort.available() < COMMANDLENGTH) return;
  
  for (int i = 0; i < COMMANDLENGTH; i++) {
    Command[i] = serialPort.read();
  }

  if (Command[0] != 0xFF) return;
  if (Command[1] != 0x00 && Command[1] != name_due) return;

  switch (Command[2]) {
    
    case 0x00: {
      int manualPWM = Command[3]; 
      int dir = Command[4];
      int duration = Command[5]; // Read duration from 6th byte (index 5)

      // Set Direction Pins based on Command[4]
      if (dir == 1) {
        HbridgeHigh = pinMotorMinus_running;
        HbridgeLow = pinMotorPlus_running;
      } else {
        HbridgeHigh = pinMotorPlus_running;
        HbridgeLow = pinMotorMinus_running;
      }
      
      // Start Motor Movement
      digitalWrite(pinEn_running, HIGH);   // Enable driver
      digitalWrite(HbridgeLow, LOW);       // Set Low side
      analogWrite(HbridgeHigh, manualPWM); // Set PWM speed
      
      // Handle Duration
      if (duration > 0) {
        delay(duration); // Wait for the specified duration (blocking)
        
        // Stop Motor immediately after delay
        analogWrite(HbridgeHigh, 0); 
        digitalWrite(pinEn_running, LOW);
      }
      // Note: If duration is 0, the motor continues running until a stop command is sent.
      
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

    // 0x02: Go To Target (With Backlash Comp + Ramping)
    case 0x02: {
      long rawTarget = U8toU32(Command[3], Command[4], Command[5], Command[6]);
      if (Command[7] == 0) rawTarget *= -1; 
      
      targetValue = rawTarget;
      
      // Initialize the Ramp
      tempTarget = *encoderValue; 
      finalTarget = targetValue - BACKLASH_OVERSHOOT_STEPS; 
      
      runningstatus = true;
      break;
    }

    // 0x03: Set Parameters (Added Ramp Speed config)
    case 0x03: {
      float newP = Command[3] * 0.001;
      Kp_Pos = newP;
      Kp_Neg = newP;
      Speed_lowest = Command[4]; 
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
      analogWrite(HbridgeHigh, 0); 
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

void SelectMotorChannel(int ch) {
  detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch1)); detachInterrupt(digitalPinToInterrupt(pinEncoderB_ch1));
  detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch2)); detachInterrupt(digitalPinToInterrupt(pinEncoderB_ch2));
  detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch3)); detachInterrupt(digitalPinToInterrupt(pinEncoderB_ch3));
  detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch4)); detachInterrupt(digitalPinToInterrupt(pinEncoderB_ch4));
  detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch5)); detachInterrupt(digitalPinToInterrupt(pinEncoderB_ch5));
  detachInterrupt(digitalPinToInterrupt(pinEncoderA_ch6)); detachInterrupt(digitalPinToInterrupt(pinEncoderB_ch6));

  channel_num = ch;
  switch (ch) {
    case 1: pinEncoderA_running = pinEncoderA_ch1; pinEncoderB_running = pinEncoderB_ch1; pinMotorMinus_running = pinMotorMinus_ch1; pinMotorPlus_running = pinMotorPlus_ch1; pinEn_running = pinEn_ch1; encoderValue = &encoderValue_ch1; break;
    case 2: pinEncoderA_running = pinEncoderA_ch2; pinEncoderB_running = pinEncoderB_ch2; pinMotorMinus_running = pinMotorMinus_ch2; pinMotorPlus_running = pinMotorPlus_ch2; pinEn_running = pinEn_ch2; encoderValue = &encoderValue_ch2; break;
    case 3: pinEncoderA_running = pinEncoderA_ch3; pinEncoderB_running = pinEncoderB_ch3; pinMotorMinus_running = pinMotorMinus_ch3; pinMotorPlus_running = pinMotorPlus_ch3; pinEn_running = pinEn_ch3; encoderValue = &encoderValue_ch3; break;
    case 4: pinEncoderA_running = pinEncoderA_ch4; pinEncoderB_running = pinEncoderB_ch4; pinMotorMinus_running = pinMotorMinus_ch4; pinMotorPlus_running = pinMotorPlus_ch4; pinEn_running = pinEn_ch4; encoderValue = &encoderValue_ch4; break;
    case 5: pinEncoderA_running = pinEncoderA_ch5; pinEncoderB_running = pinEncoderB_ch5; pinMotorMinus_running = pinMotorMinus_ch5; pinMotorPlus_running = pinMotorPlus_ch5; pinEn_running = pinEn_ch5; encoderValue = &encoderValue_ch5; break;
    case 6: pinEncoderA_running = pinEncoderA_ch6; pinEncoderB_running = pinEncoderB_ch6; pinMotorMinus_running = pinMotorMinus_ch6; pinMotorPlus_running = pinMotorPlus_ch6; pinEn_running = pinEn_ch6; encoderValue = &encoderValue_ch6; break;
  }

  InitialEncoderState();
  attachInterrupt(digitalPinToInterrupt(pinEncoderA_running), ReadEncoderState, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncoderB_running), ReadEncoderState, CHANGE);
}

unsigned long U8toU16(int v1, int v2) {
  unsigned long y1, y2, result;
  y1 = (unsigned long)(v1) << 8; y2 = (unsigned long)(v2);
  result = y1 + y2;
  return result;
}

unsigned long U8toU32(int v1, int v2, int v3, int v4) {
  unsigned long y1, y2, y3, y4, y5;
  y1 = (unsigned long)(v1) << 24; y2 = (unsigned long)(v2) << 16;
  y3 = (unsigned long)(v3) << 8;  y4 = (unsigned long)(v4);
  y5 = y1 + y2 + y3 + y4;
  return y5;
}

void U32toU8(unsigned long value){
  U8_a = value >> 24; U8_b = value >> 16; U8_c = value >> 8; U8_d = value;  
}
