#include <stdio.h>
#include <Wire.h> //Required for communications with MCP23008
#include <FastLED.h> //Required for communications with WS2812b LEDs


//**************DIGITAL PORTS***************
//BTS7960B (IS_2) DC Motor Controllers
#define MotorDrivers_Enabled 4 // define pin 4 for both motors R_EN and L_EN
#define RPWM_R 5 // define pin 5 for RPWM pin right motor(output)
#define LPWM_R 6 // define pin 6 for LPWM pin right motor(output)
#define LPWM_L 9 // define pin 10 for LPWM pin left motor(output)
#define RPWM_L 10 // define pin 11 for RPWM pin left motor(output)
//I2C Pins
#define I2C_Interrupt_Pin 2 //Interrupt pin from MCP23008
//LED Pins
#define LED_PIN 8 //WS2812b LEDs

//**************ANALONG PORTS***************
//Gas Pedal
#define gasPedalPin A3 // define pin A0 for gas pedal pot pin (input)
//Parental Controls
#define throttleResponsePin A1 // define pin A1 for throttle response pot pin (% of max lag) (input)
#define maxPowerPin A2 // define pin A2 for max power pot (input)
//I2C Communication
//Arduino pin A4 for System Data Communication (to one or many MCP23008) - purple
//Arduino pin A5 for Clock Cycle (to one or many MCP23008) - yellow
//#define IS_L A4 // define pin for current sense pin left motor(input)
//#define IS_R A5 // define pin for current sense pin right motor(output)


//MCP23008 Values (Extra ports for Shifter & Shifter LEDs)
byte I2C_Address = 0x20;
byte I2C_Register_GPIO = 0x09; //Input and output value register
byte I2C_Register_IODIR = 0x00; //Configures pins as inputs or outputs
byte I2C_Register_INTEN = 0x02; //Interrupt enabled per pin
byte I2C_Register_INTCAP = 0x08;//pin values captured since last interrupt value read
byte I2C_Register_IOCON = 0x05; //
//GP0: D3 LED Output
//GP1: D2 LED Output
//GP2: D1 LED Output
//GP3: N LED Output
//GP4: R LED Output
//GP5: P LED Output
//GP6: Shifter FWD Input
//GP7: Shifter REV Input

//Enums
enum motorDirection {
  FORWARD,
  BACKWARD
};
enum driveMode {
  PARK,
  REVERSE,
  NEUTRAL,
  DRIVE1,
  DRIVE2,
  DRIVE3
};
enum LSDMode {
  OFF,
  SENSING,
  LSDBOOSTUP,
  LSDBOOSTDOWN
};
enum ShifterState {
  FWD,
  MID,
  BACK
};
enum LEDMode {
  NORMAL,
  FOURWAY,
  POLICE,
  FOG
};
enum LEDState {
  BLUEON,
  BLUEOFF,
  REDON,
  REDOFF,
  YELLOWON,
  YELLOWOFF
};


//Constants
int const throttleResponseMaxLag = 10000; //10 seconds max allowable delay from zero to max power
int const motorVoltageAdjustmentInterval = 100; //100 mS between motor voltage adjustments

//*************Runtime Variables**************
int currentMotorVoltagePercentage = 0;
enum motorDirection currentMotorDirection = FORWARD;
unsigned long lastThrottleAdjustmentApplied = 0; // the time the delay started
unsigned long lastMotorDirectionChanged = 0; // the time the delay started
unsigned long lastShift = 0; // the time the delay started
int rightWheelPercentBias = 0; // + = right, - = left
enum driveMode currentDriveMode = PARK;
enum LSDMode currentLSDMode = OFF;
int currentThrottlePercentage = 0;
float averageWheelCurrentDifference = 0;
int averageWheelCurrentDifferenceSampleSize = 0;
volatile bool ShifterEvent = false;
volatile int ShifterEventIncrement = 1;
//LED Settings
#define NUM_LEDS 20 //number of WS2812B LEDs
CRGB leds[NUM_LEDS];
enum LEDMode CurrentLEDMode = NORMAL;
enum LEDState CurrentLEDState = YELLOWON;
unsigned long lastLEDBlink = 0; // the time the delay started
unsigned long LEDBlinkCount = 0; // the time the delay started
unsigned long lastTimeLessThanFullThrottle = 0;

void setup() {
  // Gas Pedal Setup
  pinMode(gasPedalPin, INPUT);

  // Performance Controls Setup
  pinMode(maxPowerPin, INPUT);
  pinMode(throttleResponsePin, INPUT);

  //Motor Driver Setup
  pinMode(RPWM_R, OUTPUT);
  pinMode(LPWM_R, OUTPUT);
  pinMode(RPWM_L, OUTPUT);
  pinMode(LPWM_L, OUTPUT);
  pinMode(MotorDrivers_Enabled, OUTPUT);
  //Motor Drive Current Sensing
  //pinMode(IS_L, INPUT);
  //pinMode(IS_R, INPUT);

  //Connect to MCP23008 and..
  WriteToI2C(I2C_Address, I2C_Register_IODIR, B00000011); //set I/O first 6 pins to outputs and last 2 to inputs
  WriteToI2C(I2C_Address, I2C_Register_INTEN, B00000011); // enable interrupts for last 2 pins
  WriteToI2C(I2C_Address, I2C_Register_IOCON, B00110010); // active high
  WriteToI2C(I2C_Address, I2C_Register_GPIO, B00000000);// All LEDs off
  pinMode(I2C_Interrupt_Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(I2C_Interrupt_Pin), SetShifterEvent, RISING);


  Serial.begin(9600);
  lastThrottleAdjustmentApplied = millis();
  lastMotorDirectionChanged = millis();
  lastShift = millis();
  currentThrottlePercentage = 0;
  averageWheelCurrentDifference = 0;
  averageWheelCurrentDifferenceSampleSize = 0;

  SetDriveMode(PARK);

  //Configure LEDs
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
}

void loop() {

  //Check for Shifter event and process it if required
  ProcessShifterEvent();

  //Monitor Throttle and apply changes to motor voltage
  AdjustMotorVoltage();

  //Check for LED Control state and adjust LED pattern if required
  UpdateLEDs();

}

void AdjustMotorVoltage() {
  //Get Throttle control State
  int gasPedalValue = analogRead(gasPedalPin);
  Log("Gas Pedal:" + (String)gasPedalValue);
  //Serial.print("Gas Pedal:");
  //Serial.println(gasPedalValue);
  int currentGasPedalPercentage = map(gasPedalValue, 0, 1023, 0, 100);
  if(currentGasPedalPercentage < 90){
    lastTimeLessThanFullThrottle = millis();
  }

  //Determine Target Motor Voltage Percentage and adjust as needed
  int TargetMotorVoltagePercentage = (float)currentGasPedalPercentage * ((float)currentThrottlePercentage / 100);

  if ((millis() - lastMotorDirectionChanged) <= 2000) {
    //Decelrate to zero first.
    Log("Decelerate to zero first.");
    UpdateMotorVoltagePercentage(0);
  } else if (TargetMotorVoltagePercentage > currentMotorVoltagePercentage) {
    Log("Motor Voltage lower than desired..");
    //Motor Voltage Increase neccessary.  Apply slow Acceleration logic
    if ((millis() - lastThrottleAdjustmentApplied) >= motorVoltageAdjustmentInterval) {
      //Its time to apply a voltage increase
      Accelerate(TargetMotorVoltagePercentage);
    }
  } else if (TargetMotorVoltagePercentage < currentMotorVoltagePercentage) {
    Log("Motor Voltage higher than desired..");
    //Motor Voltage Decrease neccessary.  Apply slow Deceleration logic
    if ((millis() - lastThrottleAdjustmentApplied) >= motorVoltageAdjustmentInterval) {
      //Its time to apply a voltage increase
      UpdateMotorVoltagePercentage(TargetMotorVoltagePercentage);
    }
  } else {
    //Rerun code anyways to ensure accomodation for wheel spin etc
    UpdateMotorVoltagePercentage(TargetMotorVoltagePercentage);
  }

}

void ProcessShifterEvent() {
  if (ShifterEvent == true) {
    Log("shifter event!");
    ShifterEvent = false;

    //Get ShifterState (called on interrupt from I2C)
    byte I2CValues = ReadFromI2C(I2C_Address, I2C_Register_INTCAP);
    //Log("GetShifterState I2C Pins" + (String)I2CValues);
    int bit_FORWARD = bitRead(I2CValues, 0);
    int bit_BACK = bitRead(I2CValues, 1);
    enum ShifterState CurrentShifterState = MID;
    if (bit_FORWARD == 1) {
      CurrentShifterState = FWD;
    } else if (bit_BACK == 1) {
      CurrentShifterState = BACK;
    }
    if ((CurrentShifterState != MID) && ((millis() - lastShift) > 100)) { //Ignore Interrupt for shifter returning to center or too quick of a shift

      //Log("Shifter Event!!" + (String)ShifterEventIncrement);
      lastShift = millis();

      //Adjust Throttle Percentage based on Drive Mode
      //Log("last drive mode:" + (String)currentDriveMode);
      switch (currentDriveMode) {
        case DRIVE1:
          if (CurrentShifterState == FWD) {
            SetDriveMode(DRIVE2);
          } else if (CurrentShifterState == BACK) {
            SetDriveMode(NEUTRAL);
            lastMotorDirectionChanged = millis();
          }
          break;
        case DRIVE2:
          if (CurrentShifterState == FWD) {
            SetDriveMode(DRIVE3);
          } else if (CurrentShifterState == BACK) {
            SetDriveMode(DRIVE1);
          }
          break;
        case DRIVE3:
          if (CurrentShifterState == BACK) {
            SetDriveMode(DRIVE2);
          }
          break;
        case NEUTRAL:
          if (CurrentShifterState == FWD) {
            SetDriveMode(DRIVE1);
          } else if (CurrentShifterState == BACK) {
            SetDriveMode(REVERSE);
          }
          break;
        case REVERSE:
          if (CurrentShifterState == FWD) {
            SetDriveMode(NEUTRAL);
          } else if (CurrentShifterState == BACK) {
            SetDriveMode(PARK);
          }
          break;
        case PARK:
          if (CurrentShifterState == FWD) {
            SetDriveMode(REVERSE);
          }
          break;
      }
    }
  }
}


void UpdateLEDs() {
  //Check if we need to change modes
  Log("check:" + (String)(millis() - lastTimeLessThanFullThrottle));
  if((millis() - lastTimeLessThanFullThrottle > 5000)&&(currentDriveMode == PARK)){
    lastTimeLessThanFullThrottle = millis();
    switch (CurrentLEDMode) {
      case NORMAL:
        CurrentLEDMode = POLICE;
        CurrentLEDState = BLUEON;
        break;
      case POLICE:
        CurrentLEDMode = FOURWAY;
        CurrentLEDState = YELLOWON;
        break;
      case FOURWAY:
        CurrentLEDMode = FOG;
        break;
      case FOG:
        CurrentLEDMode = NORMAL;
        CurrentLEDState = YELLOWON;
        break;
    }
  }



  
  //LED Logic
  switch (CurrentLEDMode) {
    case NORMAL:
      SetAllLEDs(CRGB::Yellow);
    case POLICE:
      RunLEDPolicePattern();
      break;
    case FOURWAY:
      RunLED4WAYPattern();
      break;
    case FOG:
      SetAllLEDs(CRGB::White);
      break;
  }
}

void RunLED4WAYPattern() {
  if ((millis() - lastLEDBlink) > 500) {
    lastLEDBlink = millis();
    switch (CurrentLEDState) {
      case YELLOWON:
        CurrentLEDState = YELLOWOFF;
        SetAllLEDs(CRGB::Black);
        break;
      case YELLOWOFF:
        CurrentLEDState = YELLOWON;
        SetAllLEDs(CRGB::Yellow);
        break;
    }
  }
}

void RunLEDPolicePattern() {
  if ((millis() - lastLEDBlink) > 50) {
    //Toggle blink
    lastLEDBlink = millis();
    switch (CurrentLEDState) {
      case BLUEON:
        CurrentLEDState = BLUEOFF;
        SetAllLEDs(CRGB::Black);
        break;
      case BLUEOFF:
        if (LEDBlinkCount > 3) {
          LEDBlinkCount = 1;
          CurrentLEDState = REDON;
          SetLEDs(0,10,CRGB::Red);
        } else {
          LEDBlinkCount++;
          CurrentLEDState = BLUEON;
          SetLEDs(10,20,CRGB::Blue);
        }
        break;
      case REDOFF:
        if (LEDBlinkCount > 3) {
          LEDBlinkCount = 1;
          CurrentLEDState = BLUEON;
          SetLEDs(10,20,CRGB::Blue);
        } else {
          LEDBlinkCount++;
          CurrentLEDState = REDON;
          SetLEDs(0,10,CRGB::Red);
        }
        break;
      case REDON:
        CurrentLEDState = REDOFF;
        SetAllLEDs(CRGB::Black);
        break;
    }
  }
}

void SetAllLEDs(CRGB NewColor){
  SetLEDs(0,NUM_LEDS,NewColor);
}

void SetLEDs(int first, int last, CRGB NewColor){
  for(int l = first; l < last; l++){
    leds[l] = NewColor;
  }
  FastLED.show();
}


void SetDriveMode(driveMode NewDriveMode) {
  currentDriveMode = NewDriveMode;
  switch (NewDriveMode) {
    case DRIVE1:
      //33%
      currentThrottlePercentage = 33;
      currentMotorDirection = FORWARD;
      //Connect to MCP23008 and set I/O first 6 pins to outputs and last 2 to inputs
      WriteToI2C(I2C_Address, I2C_Register_GPIO, B00010000);// Drive 1 LED ON
      digitalWrite(MotorDrivers_Enabled, HIGH); //Motors Enabled
      break;
    case DRIVE2:
      //66%
      currentThrottlePercentage = 66;
      currentMotorDirection = FORWARD;
      WriteToI2C(I2C_Address, I2C_Register_GPIO, B00001000); // Drive 2 LED ON
      digitalWrite(MotorDrivers_Enabled, HIGH); //Motors Enabled
      break;
    case DRIVE3:
      //100%
      currentThrottlePercentage = 100;
      currentMotorDirection = FORWARD;
      WriteToI2C(I2C_Address, I2C_Register_GPIO, B00000100); //Drive 3 LED ON
      digitalWrite(MotorDrivers_Enabled, HIGH); //Motors Enabled
      break;
    case NEUTRAL:
      currentThrottlePercentage = 0;
      currentMotorDirection = FORWARD;
      WriteToI2C(I2C_Address, I2C_Register_GPIO, B00100000); //Neutral LED ON
      digitalWrite(MotorDrivers_Enabled, LOW); //Drivers Disabled (Free-wheel)
      break;
    case REVERSE:
      //33%
      currentThrottlePercentage = 33;
      currentMotorDirection = BACKWARD;
      WriteToI2C(I2C_Address, I2C_Register_GPIO, B01000000); //Reverse LED ON
      digitalWrite(MotorDrivers_Enabled, HIGH); //Motors Enabled
      break;
    case PARK:
      currentThrottlePercentage = 0;
      currentMotorDirection = FORWARD;
      WriteToI2C(I2C_Address, I2C_Register_GPIO, B10000000); //Park LED ON
      digitalWrite(MotorDrivers_Enabled, HIGH); //Motors Enabled
      break;
  }
  //Log("new drive mode:" + (String)currentDriveMode);
}



void SetShifterEvent() {
  ShifterEvent = true;
  ShifterEventIncrement = ShifterEventIncrement + 1;
}

byte ReadFromI2C(int TargetAddress, byte TargetRegister) {
  Wire.beginTransmission(TargetAddress); //starts talking to slave device
  Wire.write(TargetRegister); //selects the GPIO pins
  Wire.endTransmission(); //stops talking to device
  Wire.requestFrom(TargetAddress, 1); // requests one byte of data from I2C
  return Wire.read(); // store the incoming byte into inputs
}

void WriteToI2C(byte TargetAddress, byte TargetRegister, byte writeValue) {
  //Log("Write to I2C!" + (String)writeValue);
  Wire.begin(); //creates a Wire object
  Wire.beginTransmission(TargetAddress); //begins talking to the slave device
  Wire.write(TargetRegister); //selects the IODIRA register
  Wire.write(writeValue); // Set first 6 ports to outputs and last two to inputs
  Wire.endTransmission(); //stops talking to device
}

void Accelerate(int targetThrottlePercentage) {
  //Log("Accelerating..");
  //Determine and Set New Motor Voltage Percentage
  int revisedMotorVoltagePercentage = targetThrottlePercentage;

  //Get slow start control voltage increment per millisecond
  int throttleResponsePotVal = 1; //analogRead(throttleResponsePin);
  //Log("throttleResponsePotVal:" + throttleResponsePotVal);
  int throttleResponseLagMillis = map(throttleResponsePotVal, 0, 1023, 0, throttleResponseMaxLag);
  //Log("  throttleResponseLagMillis:" + throttleResponseLagMillis);
  if (throttleResponseLagMillis > 0) {
    //Throttle Lag Enabled
    float throttlePercentIncreasePerMillisecond = (float)100 / (float)throttleResponseLagMillis;

    //Get time passed since last update
    int millisSinceLastUpdate = millis() - lastThrottleAdjustmentApplied;
    if (millisSinceLastUpdate > 0) {
      //Check for Error in getting time,. this seems to happen sometimes.

      //Get Throttle Percentage Increment
      int motorVoltagePercentIncrement = millisSinceLastUpdate * throttlePercentIncreasePerMillisecond;
      //Log("motorVoltagePercentIncrement:" + motorVoltagePercentIncrement);
      int revisedMotorVoltagePercentage = currentMotorVoltagePercentage + motorVoltagePercentIncrement;


      if (revisedMotorVoltagePercentage > targetThrottlePercentage) {
        UpdateMotorVoltagePercentage(targetThrottlePercentage);
      } else {
        UpdateMotorVoltagePercentage(revisedMotorVoltagePercentage);
      }
    }
  } else {
    UpdateMotorVoltagePercentage(targetThrottlePercentage);
  }
}

void UpdateMotorVoltagePercentage(int targetMotorVoltagePercentage) {
  //Log("target motor %: " + targetMotorVoltagePercentage);

  currentMotorVoltagePercentage = targetMotorVoltagePercentage;

  //Adjust for Max Power Setting
  //int maxPowerValue = analogRead(maxPowerPin);
  //Serial.print("max Power Pot: ");
  //Serial.print(maxPowerValue);
  int maxPowerPercentage = 100; //map(maxPowerValue, 0, 1023, 100, 1);
  //Log("max Power %: " + maxPowerPercentage);
  float maxPowerPercentageFloat = (float)maxPowerPercentage / (float)100;
  //Log("max Power %float: " + (String)maxPowerPercentageFloat);

  float adjustedMotorVoltagePercentage = (float)currentMotorVoltagePercentage * maxPowerPercentageFloat;
  //Log("adjusted motor %: " + (String)adjustedMotorVoltagePercentage);




  float RightWheelMotorVoltagePercentage = adjustedMotorVoltagePercentage;
  float LeftWheelMotorVoltagePercentage = adjustedMotorVoltagePercentage;

  //Limited Slip Mode
  /*
    int CurrentSenseVal_L = analogRead(IS_L); //(0-1023) ~ Max is around 300
    int CurrentSenseVal_R = analogRead(IS_R); //(0-1023)
    int AcceptableDifference = 10;
    int CurrentDifference = CurrentSenseVal_R - CurrentSenseVal_L;


    switch(currentDriveMode){
    case NORMAL:
      //Calculate Average Difference
      if(currentThrottlePercentage >50){
        averageWheelCurrentDifference = (averageWheelCurrentDifference*((float)averageWheelCurrentDifferenceSampleSize) + (float)CurrentDifference)/(float)(averageWheelCurrentDifferenceSampleSize+1);
        averageWheelCurrentDifferenceSampleSize = averageWheelCurrentDifferenceSampleSize + 1;
      }

      //Check if LSD Boost Required
      if((currentThrottlePercentage > 90)&&(currentMotorVoltagePercentage >= currentThrottlePercentage)&&(averageWheelCurrentDifferenceSampleSize>10)) {
        if(abs(CurrentDifference-averageWheelCurrentDifference) > AcceptableDifference){
          currentDriveMode = LSDBOOSTUP;
          if(CurrentDifference > 0){
            //Right Wheel Has Traction
            rightWheelPercentBias = 20;
          } else {
            //Left Wheel Has Traction
            rightWheelPercentBias = -20;
          }
        }
      }
      break;
    case LSDBOOSTUP:
      //LSD Boost Mode increase in effect
      if(abs(rightWheelPercentBias) >= 100){
        currentDriveMode = LSDBOOSTDOWN;
      } else if(rightWheelPercentBias > 0){
        //Right Wheel Has Traction
        rightWheelPercentBias = rightWheelPercentBias + 20;
      } else {
        //Left Wheel Has Traction
        rightWheelPercentBias = rightWheelPercentBias - 20;
      }
      break;
    case LSDBOOSTDOWN:
      //LSD Boost Mode increase in effect
      if(rightWheelPercentBias > 0){
        //Right Wheel Has Traction
        rightWheelPercentBias = rightWheelPercentBias - 20;
      } else if (rightWheelPercentBias < 0) {
        //Left Wheel Has Traction
        rightWheelPercentBias = rightWheelPercentBias + 20;
      } else {
        currentDriveMode = NORMAL;
      }
      break;
    }
    Serial.print("averageWheelCurrentDifferenceSampleSize:");
    Serial.print(averageWheelCurrentDifferenceSampleSize);
    Serial.print("  averageWheelCurrentDifference:");
    Serial.print(averageWheelCurrentDifference);
    Serial.print("  Current Sense  L:");
    Serial.print(CurrentSenseVal_L);
    Serial.print("  R:");
    Serial.print(CurrentSenseVal_R);
    Serial.print("  rightWheelPercentBias:");
    Serial.println(rightWheelPercentBias);


    //Apply LSD Factoring to wheels

    if(rightWheelPercentBias > 0){
    //Right Wheel Has Traction
    RightWheelMotorVoltagePercentage = adjustedMotorVoltagePercentage * (((float)100+(float)rightWheelPercentBias)/(float)100);
    if(RightWheelMotorVoltagePercentage>100){
      RightWheelMotorVoltagePercentage=100;
    }
    LeftWheelMotorVoltagePercentage = adjustedMotorVoltagePercentage * (((float)100-(float)rightWheelPercentBias)/(float)100);
    } else if (rightWheelPercentBias < 0) {
    //Left Wheel Has Traction
    LeftWheelMotorVoltagePercentage = adjustedMotorVoltagePercentage * (((float)100+(float)abs(rightWheelPercentBias))/(float)100);
    if(LeftWheelMotorVoltagePercentage>100){
      LeftWheelMotorVoltagePercentage=100;
    }
    RightWheelMotorVoltagePercentage = RightWheelMotorVoltagePercentage * (((float)100+(float)rightWheelPercentBias)/(float)100);
    }
  */


  //Apply Speed Change To Wheels
  int currentMotorVoltageRight = map(RightWheelMotorVoltagePercentage, 0, 100, 0, 255);
  int currentMotorVoltageLeft = map(LeftWheelMotorVoltagePercentage, 0, 100, 0, 255);
  if (currentMotorDirection == FORWARD) {
    analogWrite(LPWM_R, 0);
    analogWrite(RPWM_R, currentMotorVoltageRight);
    analogWrite(LPWM_L, 0);
    analogWrite(RPWM_L, currentMotorVoltageLeft);
  } else {
    analogWrite(LPWM_R, currentMotorVoltageRight);
    analogWrite(RPWM_R, 0);
    analogWrite(LPWM_L, currentMotorVoltageLeft);
    analogWrite(RPWM_L, 0);

  }

  //report
  lastThrottleAdjustmentApplied = millis();
  //Log(", motor power changed to " + currentMotorVoltagePercentage);
  //Log("% LEFT(" + currentMotorVoltageLeft);
  //Log("/255V) % RIGHT(");
  //Log(currentMotorVoltageRight + "/255V)");


}

void Log(String Message) {
  Serial.println(Message);
}
