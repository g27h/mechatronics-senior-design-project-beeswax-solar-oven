/*Solar Wax Melter
 * Mechatronics 2 Spring Semester Project
 * Gus Hurtado, Brian Godino, Charles Aardema Jr
 * 
 * 
*/










static volatile bool sunTrackingMode; //Mode for purely tracking the sun
static volatile bool tempControllingMode; //Mode for adding the additional functionality of the temperature controlling
static volatile bool returnToHome; //Mode for returning to home when either the day ends, or the bed is full, this can act on a timer or on the magnitude of the voltages
                                    //coming into the sensors
static volatile bool homeSwitch;            //indicates if the melter is in home position (ie is the homeSwitch engaged -true==switch engaged, melter in home position)
static volatile bool ParkSwitch;            //indicates if the melter is in park position (ie is the parkSwitch engaged -true==switch engaged, melter in park)
static volatile bool goToPark; //Moves the oven to the end state for wax removal
static volatile bool park;                      // boolean to indicate park status (true=in park position)
static volatile bool weightSensorSwitch;
static volatile bool thermostatSwitch;    //thermostat to control internal temp
static volatile bool nightSwitch;
static volatile bool weightSwitchToggle;


static volatile uint16_t returnToHomeButtonPin;
static volatile uint16_t homeSwitchPin;
static volatile uint16_t goToParkButtonPin;
static volatile uint16_t ParkSwitchPin;
static volatile uint16_t weightSensorSwitchPin;
static volatile uint16_t thermostatSwitchPin;
static volatile uint16_t thermistorPin;
float Temp;
//Motor Variables
static volatile uint16_t PWM;
static volatile uint16_t PWMthermo;
static volatile bool driveDirection;
static volatile int16_t enA;
static volatile int16_t in1;
static volatile int16_t in2;
static volatile int16_t enB;
static volatile int16_t in3;
static volatile int16_t in4;
static volatile int16_t photoCell1;
static volatile int16_t photoCell2;
static volatile float photoVoltage1;
static volatile float photoVoltage2;
static volatile float k_p;
static volatile float k_i;
static volatile float P;
static volatile float I;
static volatile float Integrator_value;
static volatile float V_setpoint;
static volatile float V_diff;
static volatile float V_error;

void setup() {
  sunTrackingMode = true;
  tempControllingMode = false;
  returnToHome = false;
  homeSwitch = false;
  ParkSwitch = false;
  goToPark = false;
  weightSensorSwitch = false;
  thermostatSwitch = false;
  nightSwitch = false;
  weightSwitchToggle = false;
  park = false;
  
  // Motor variables
  // The motor is controlled via a HIGH to IN1 and a LOW to IN2 to turn in one dir
  // or a LOW to IN1 and a HIGH to IN2. In the cases of sending the high signal
  // we instead use PWM
  enA = 10; //PWM/analog pin 10, connected to module pin 7 (remove the jumper first)
  in1 = 8; //digital pin 8, connected to IN1
  in2 = 6; //digital pin 6, connected to IN2
  enB = 9;     //PWM/analog pin 9, connected to module pin enB (remove the jumper first)
  in3 = 5;      //digital pin 5, connected to IN3
  in4 = 4;      //digital pin 4, connected to IN4
  photoCell1 = A1; //analong pin A0, connected to the photocell positive lead
  photoCell2 = A0; //analog pin A1, connected to the photocell positive lead
  thermistorPin = A2;  //analog pin A2 connected to the thermistor junction
  returnToHomeButtonPin = 2;
  homeSwitchPin = 3;
  goToParkButtonPin = 18; //the button in which the user interacts
  ParkSwitchPin = 19; //the switch in which the oven interacts
  weightSensorSwitchPin = 20;
  k_p = 35;
  k_i = 10;
  V_setpoint = 0;
  
  //Temperature control
  thermostatSwitchPin = 21;

  //Setting pins 10, 9 and 8 as outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(photoCell1, INPUT);
  pinMode(photoCell2, INPUT);
  pinMode(thermistorPin, INPUT);
  pinMode(returnToHomeButtonPin, INPUT_PULLUP);
  pinMode(homeSwitchPin, INPUT_PULLUP);
  pinMode(goToParkButtonPin, INPUT_PULLUP);
  pinMode(ParkSwitchPin, INPUT_PULLUP);
  pinMode(weightSensorSwitchPin, INPUT_PULLUP);
  pinMode(thermostatSwitchPin, INPUT_PULLUP);
  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(2), pin2_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), pin3_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), pin18_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(19), pin19_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(20), pin20_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(21), pin21FanOn_ISR, CHANGE);
}

void loop() {
    photoVoltage1 = (analogRead(photoCell1)); //converted to PWM; left side
    photoVoltage2 = (analogRead(photoCell2)+5); //converted to PWM; right side
    Serial.print("returnToHome = ");  Serial.print(returnToHome);
    Serial.print(";   homeSwitch = ");  Serial.print(homeSwitch);
    Serial.print(";  SunTracking = ");Serial.print(sunTrackingMode);Serial.print(";  goToPark = ");
    Serial.print(goToPark);Serial.print(";  ParkSwitch = ");Serial.print(ParkSwitch);
    Serial.print(";  weightSwitch = ");  Serial.println(weightSensorSwitch);
    Serial.print(";  thermostatSwitch = ");  Serial.print(thermostatSwitch);
    Serial.print(";  nightSwitch = ");  Serial.print(nightSwitch);
    Serial.print(";  weightSwitchToggle = ");  Serial.println(weightSwitchToggle);
    Serial.print("photoVoltage1 =  "); Serial.print(photoVoltage1);
    Serial.print("photoVoltage2 = ");Serial.print(photoVoltage2);
    Serial.print("Temperature: "); Serial.println(Temp);//  Serial.print("               park: "); Serial.println(park);
    Serial.print("           V_diff = ");    Serial.println(V_diff);
    Serial.print("           PWM = ");    Serial.println(PWM);
   
    StatusFunction();                             //  Status  check  on  position
    thermistorFunct();                            //  call for thermomter function for temp value
    overheatFunct();                              //  call for internal temp to detect too high temperature, then call fan function (inside overheatFunct)
    
    V_diff = (photoVoltage2 - photoVoltage1); //left - right cells

    V_error = V_diff - V_setpoint;
    Integrator_value = 0.5*(Integrator_value+V_error);


    //daylight commands
    if(photoVoltage1 >= 600 && photoVoltage2 >= 600){//voltage sensor reaches a threshold for daylight
      nightSwitch = false;
      if (goToPark == true && ParkSwitch == false){
        goToParkFunct();
        sunTrackingMode = false;
        }
      else if (returnToHome == true && ParkSwitch == true){
        returnToHome = false;
        PWM = 255;
        analogWrite(enA, PWM);
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        delay(2000);
        PWM = 0;
        analogWrite(enA, PWM);
        sunTrackingMode = true;
        goToPark = false;
        }
      else if (ParkSwitch == true){
        sunTrackingMode = false;
        tempControllingMode = false;
        returnToHome = false;
        homeSwitch = false;
        goToPark = false;
        weightSwitchToggle = true;
        park = true;
        PWM = 0;
        analogWrite(enA, PWM);
        if (Temp > 40){                                           
          PWMthermo = 255;
          fanFunct();
        }
        else if (Temp < 30){
          PWMthermo = 0;
          fanFunct();
        }
      }
      else if (returnToHome == true && homeSwitch == false){
        returnToHome = false;
        sunTrackingMode = true;
      }
      else if(homeSwitch == true){
        returnToHome = false;
        PWM = 255;
        analogWrite(enA, PWM);
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        delay(2000);
        PWM = 0;
        analogWrite(enA, PWM);
        if(photoVoltage1 >= 600 || photoVoltage2 >= 600){
          sunTrackingMode = true;
        }
      }
      else if (weightSensorSwitch == true && weightSwitchToggle == false){
        goToPark = true;
        PWMthermo = 255;
        fanFunct();
      }
      else if (sunTrackingMode == true ){
        {
          sunTrackingModeFunct();
        }
      }
      else {sunTrackingMode = true;}
      
    }
    
    //dark commands
    else if (photoVoltage1 <= 300 || photoVoltage2 <= 300){
      sunTrackingMode = false;
      if (goToPark == true && ParkSwitch == false){
        goToParkFunct();
        sunTrackingMode = false;
      }
//      else if (returnToHome == true && ParkSwitch == true){
//        returnToHome = true;
//        returnToHomeFunct();
//        goToPark = false;
//      }
      else if (returnToHome == true && homeSwitch == false){
          ParkSwitch = false;
          if (nightSwitch == false) {                                                                                 // redundant with loop????  keeps calling returnToHomeFunct with each loop?
            returnToHomeFunct();
          }
      }
      else if (ParkSwitch == true){
        sunTrackingMode = false;
        tempControllingMode = false;
        returnToHome = false;
        homeSwitch = false;
        goToPark = false;
        weightSwitchToggle = true;
        PWM = 0;
        analogWrite(enA, PWM);
        if (Temp > 40){                                           
          PWMthermo = 255;
          fanFunct();
        }
        else if (Temp < 30){
          PWMthermo = 0;
          fanFunct();
        }
      }
      else if(homeSwitch == true){
        returnToHome = false;
        PWM = 255;
        analogWrite(enA, PWM);
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        delay(2000);
        PWM = 0;
        analogWrite(enA, PWM);
        nightSwitch = true;
      }
      else if (weightSensorSwitch == true && weightSwitchToggle == false){
        goToPark = true;
        PWMthermo = 255;
        fanFunct();
      }
      else if (nightSwitch == false && ParkSwitch == false){
        returnToHome = true;
      }
    }
}
void sunTrackingModeFunct(){                                      /////////////////////////////////////////////////////////////sun tracking function     ///////////////////////////////////

        P = k_p*abs(V_error);
//                      I = k_i*Integrator_value;                         // not used???  gus's original code to PID the system
    PWM = P;
    if(PWM > 255){ //the range on an arduino is 0~255
    PWM = 255;
    }
    if (PWM < 160){
      PWM = 0;
    }
    if (Temp > 55 && Temp <= 88){                                     //  Above 85 deg C  turntable advances 50 photocell units to turn away from the sun.  Less direct solar radiation to help cool down
        PWM = 255;
        if(V_diff < 35){ //more sunlight on the left cell; forcing it to require more light on the right cell
          //drive the motor to move the oven CW
          //driveMotor(false, PWM);
          analogWrite(enA, PWM);
          digitalWrite(in1, HIGH);
          digitalWrite(in2, LOW);
        }
        else {
          PWM = 0;
        analogWrite(enA, PWM);
        }
    }
    else if (Temp <= 55){                                                  //  Set point for normal sun tracking    85 deg C  below this sun tracking normal
      if (V_diff > 15 || V_diff < -15){
      if(V_diff < 0){ //more sunlight on the left cell
        //drive the motor to move the oven CW
        //driveMotor(false, PWM);
        analogWrite(enA, PWM);
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);

      }
      else { //more sunlight on the right cell
        //drive the motor to move the oven CCW
        //driveMotor(false, PWM);
        analogWrite(enA, PWM);
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
      }}
      else {
          PWM = 0;
        analogWrite(enA, PWM);
        }
    }






    
//    else if (Temp > 88){                                              //  Above 88 deg C  turntable advances 10 photocell units.     FIELD TEST TO VARIFY THESE VALUES  BOTH TEMP AND PHOTOCELL UNITS
//            if (Temp < 85){
//        if(V_diff < 30){ //more sunlight on the left cell
//          //drive the motor to move the oven CW
//          //driveMotor(false, PWM);
//          analogWrite(enA, PWM);
//          digitalWrite(in1, HIGH);
//          digitalWrite(in2, LOW);
//        }
//      }
//    }
}                                                                ///////////////////////////////////////////////////    end sun tracking funct       /////////////////////////////////////
//void driveMotor(bool driveDirection, int16_t PWM){
//  if(driveDirection = true){
//    //drive the motor CW using a PWM pin, also a motor control direction pin
//    //TODO: guessing here on the drive direction for the in1 and in2..
//    analogWrite(enA, PWM);
//    digitalWrite(in1, HIGH);
//    digitalWrite(in2, LOW);
//  }
//  else{ //direction = false
//    //drive the motor CCW using a PWM pin, also a motor control direction pin
//    //TODO: guessing here on the drive direction for the in1 and in2..
//    analogWrite(enA, PWM);
//    digitalWrite(in1, LOW);
//    digitalWrite(in2, HIGH);
//  }
//}

void pin2_ISR(){
  returnToHome = true;
  sunTrackingMode = false;
  tempControllingMode = false;
}

void pin3_ISR(){
  if (digitalRead(homeSwitchPin) == 1){
    homeSwitch = false;
  }
  else {
    homeSwitch = true;
  }
}

void pin18_ISR(){
  goToPark = true;
  sunTrackingMode = false;
  tempControllingMode = false;
}

void pin19_ISR(){
  if (digitalRead(ParkSwitchPin) == 1){
    ParkSwitch = false;
  }
  else {
    ParkSwitch = true;
  }
}

void pin20_ISR(){
  if (digitalRead(weightSensorSwitchPin) == 1){ //switch not pressed
    weightSensorSwitch = false;
  }
  else { //switch pressed
    weightSensorSwitch = true;
    weightSwitchToggle = false; //on weight sensor press only
  }
}

void pin21FanOn_ISR(){                                // Not in Use due to overheatFunct
  if (digitalRead(21) == true){                       ////////////////////////////////////////////////////////////////////////////////////////////////////
    PWMthermo = 0;
    analogWrite(enB, PWMthermo);
    }
  else {
    PWMthermo = 255;
    analogWrite(enB, PWMthermo);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    } 
}

void overheatFunct(){
  if (Temp > 85){
    PWMthermo = 255;
    fanFunct();
  }
  else if (Temp < 80 && park == false){
    PWMthermo = 0;
    fanFunct();
    }
}

void returnToHomeFunct(){
  PWM = 200;
  analogWrite(enA, PWM);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}
void goToParkFunct(){
  PWMthermo = 255;
  fanFunct();
  PWM = 255;
  analogWrite(enA, PWM);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}


void StatusFunction(){
  if (digitalRead(ParkSwitchPin) == 0){
    ParkSwitch = true;
    }
  if (digitalRead(ParkSwitchPin) == 1){
    park = false;
    }    
  if (digitalRead(homeSwitchPin) == 0){
    homeSwitch = true;
   }
  }
void fanFunct(){
  analogWrite(enB, PWMthermo);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  }
void thermistorFunct(){
  float  R1 = 9700.00;
  float  RT0 = 10000.00, Rtherm, B = 3977, VRT, VR, VCC=5, ln, T, T0 = 298.15;

  VRT = analogRead(thermistorPin);
  VRT = (5.00/1023.00)* VRT;
  VR = VCC - VRT;
  Rtherm = VRT/(VR/R1);
  ln = log(Rtherm/RT0);
  T = (1.0 / ((ln/B)+(1/T0)));
    Temp = T - 273.15;
//       Serial.print("        T=  "); Serial.print(T);       Serial.print("          VRT "); Serial.println (VRT);
//       Serial.print("        VR=  "); Serial.print(VR);       Serial.print("          VRT "); Serial.println (VRT);
  }
