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
void MotorRun() 
{
  // Serial1.print("Run"); 
  // runningstatus = true;
  // motorlimit = false;
  
  // If runningstatus is true, it drives the motor until it gets to the targetValue. If runningstatus is false, it stops output. 
  errorNumber1 = abs(targetValue - *encoderValue);
  if ((targetValue - *encoderValue) > 0){
//    if (errorDirection != true){
//      If the direction changes, it stops the motor. 
//      analogWrite(HbridgeHigh, 0);
//    }
    errorDirection = true; 
  }
  else{
//    if (errorDirection != false){
//      // analogWrite(HbridgeHigh, 0);
//    }
    errorDirection = false; 
  }
  
  // Serial1.print(errorNumber1); 
  
  // Decide the direction
  if (runningstatus == true){
    // Judge the timming of shutting down the motor
    if (errorNumber1 > thresholdValue){
      // Deciding the direction
      if (errorDirection == true) {
        HbridgeHigh = pinMotorMinus_running;
        HbridgeLow = pinMotorPlus_running;
      }
      else {
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
        pwmNumber = Speed_lowest;  //10
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
     else
       runningstatus = false; 
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
