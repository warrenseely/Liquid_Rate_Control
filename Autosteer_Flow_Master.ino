#include <Wire.h>

  #define LED_PIN 13 
  
  #define   DIR_PIN    12  //PB4
  #define   PWM_PIN    11  //PB3
  
  #define WORKSW_PIN 4  //PD4
  #define STEERSW_PIN 10 //PB2
  
  #define RELAY1_PIN 5  //PD5
  #define RELAY2_PIN 6  //PD6
  #define RELAY3_PIN 7  //PD7
  #define RELAY4_PIN 8  //PB0
  #define RELAY5_PIN 9  //PB1
  #define RELAY6_PIN 3  //PD3
  #define RELAY7_PIN A4  //PC4
  #define RELAY8_PIN A5  //PC5
  //***********************************************************************
  #define valvedirection 2 //direction to drive flow control valve
  #define valvePin A3 //turn valve on/off
  #define flowPin A2 //bound to digital pin 2 and interrupt
  
  #define target_Rate 20 //hard coded to 20gpa
  #define flow_gain 1 //flow rate gain, increase to decrease sensitivity
  #define nozzle_flow 0.2 //flow rate in gpm @ 40psi
  #define nozzle_space 20 //spacing of nozzles in inches
  //***********************************************************************
  #define RAD2GRAD 57.2957795

  //program flow
  bool isDataFound = false, isSettingFound = false;
  int header = 0, tempHeader = 0, temp;

  byte relay = 0, workSwitch = 0, steerSwitch = 1, speeed = 0;
 
  //***********************************************************************
  //sprayer values
  byte sensorInterrupt = 0;  // 0 = digital pin 2
  byte Slaveflow_data[3];  
  // The Raven flow sensor outputs approximately 72 pulses per
  // gal/minute of flow
  float calibrationFactor = 72;
  
  union{
    float litersPerhec;
    byte rawData[4];
  }float_data;
  
  volatile byte pulseCount = 0;  
  
  float actual_Flow = 0;
  //unsigned int flowMilliLitres = 0;
  //unsigned long totalMilliLitres = 0;
  float litersPerhec = 0;
  unsigned long oldTime = 0;
  //***********************************************************************

  float distanceFromLine = 0; // not used
  

  //steering variables
  float steerAngleActual = 0;
  int steerPrevSign = 0, steerCurrentSign = 0; // the steering wheels angle currently and previous one
  float steerAngleSetPoint = 0; //the desired angle from AgOpen
  int steeringPosition = 0, steeringPositionZero = 512; //from steering sensor
  float steerAngleError = 0; //setpoint - actual
  float steerSensorCounts = 40;

  //inclinometer variables
  int tilt = 0, roll = 0;

  //pwm variables
  int pwmDrive = 0, drive = 0, pwmDisplay = 0;
  float pValue = 0, iValue = 0, dValue = 0;
  byte minPWMValue = 10;
 
  //PID variables
  float Ko = 0.0f;  //overall gain
  float Kp = 0.0f;  //proportional gain
  float Ki = 0.0f;//integral gain
  float Kd = 0.0f;  //derivative gain 

  //integral values - **** change as required *****
  int   maxIntErr = 200; //anti windup max
  int maxIntegralValue = 20; //max PWM value for integral PID component 

  //error values
  float lastError = 0, lastLastError = 0, integrated_error = 0, dError = 0;


void setup()
{
  //analogReference(EXTERNAL);
  pinMode(LED_PIN, OUTPUT); //configure LED for output
  pinMode(RELAY1_PIN, OUTPUT); //configure RELAY1 for output //Pin 5
  pinMode(RELAY2_PIN, OUTPUT); //configure RELAY2 for output //Pin 6
  pinMode(RELAY3_PIN, OUTPUT); //configure RELAY3 for output //Pin 7
  pinMode(RELAY4_PIN, OUTPUT); //configure RELAY4 for output //Pin 8
  pinMode(RELAY5_PIN, OUTPUT); //configure RELAY5 for output //Pin 9
  pinMode(RELAY6_PIN, OUTPUT); //configure RELAY6 for output //Pin 3
  pinMode(RELAY7_PIN, OUTPUT); //configure RELAY7 for output //Pin A4
  pinMode(RELAY8_PIN, OUTPUT); //configure RELAY8 for output //Pin A5
  
  pinMode(DIR_PIN, OUTPUT); //D11 PB3 direction pin of PWM Board

  //keep pulled high and drag low to activate, noise free safe    
  pinMode(WORKSW_PIN, INPUT_PULLUP);   //Pin D4 PD4
  pinMode(STEERSW_PIN, INPUT_PULLUP);  //Pin 10 PB2
  //***********************************************************************
  //sprayer
  pinMode(valvePin, OUTPUT);
  pinMode(valvedirection, OUTPUT);
  pinMode(flowPin, INPUT);
  digitalWrite(flowPin, HIGH);
  //***********************************************************************
  Serial.begin(19200);  //open serial port
  Wire.begin(); // join i2c bus (address optional for master)
  //PWM rate settings Adjust to desired PWM Rate
  //TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
  TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
  //TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz

  // The Hall-effect sensor is connected to pin 2 which uses interrupt 0.
  // Configured to trigger on a FALLING state change (transition from HIGH
  // state to LOW state)
  //attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
} 

void loop()
{
  // header high/low, relay byte, speed byte, high distance, low distance, high delta, low delta, PID Settings byte
  if (Serial.available() > 0 && !isDataFound && !isSettingFound) //find the header, 127H + 254L = 32766
  {
    int temp = Serial.read();
    header = tempHeader << 8 | temp;               //high,low bytes to make int
    tempHeader = temp;                             //save for next time
    if (header == 32766) isDataFound = true;     //Do we have a match? 
    if (header == 32764) isSettingFound = true;     //Do we have a match? 
  }
  
  //Header has been found, presumably the next 6 bytes are the data
  if (Serial.available()> 5 && isDataFound)
  {  
    isDataFound = false;    
    Slaveflow_data[1] = relay = Serial.read();   // read relay control from AgOpenGPS

    //actual speed times 4
    Slaveflow_data[0] = speeed = Serial.read()>>2;  //single byte

    //distance from the guidance line in mm
    distanceFromLine = (float)(Serial.read() << 8 | Serial.read());   //high,low bytes     

    //set point steer angle * 10 is sent
    steerAngleSetPoint = (float)(Serial.read() << 8 | Serial.read()); //high low bytes 
    steerAngleSetPoint /=10.0;

    //Slaveflow_data[2] = (float)(Serial.read() << 8 | Serial.read()); //high low bytes //get the target rate from AOG eventually 
    
    workSwitch = digitalRead(WORKSW_PIN);  // read work switch
    steerSwitch = digitalRead(STEERSW_PIN); //read auto steer enable switch open = 0n closed = Off
    SetRelays();

    Wire.beginTransmission(1); // transmit speeed and relay to device #1
    Wire.write(Slaveflow_data, 2);              // sends two bytes, speeed and relay. Add in target_rate
    Wire.endTransmission();    // stop transmitting
 
    
    //analogReference(EXTERNAL);
      
    //steering position and steer angle
    analogRead(A0); //discard initial reading
    steeringPosition = analogRead(A0);
    delay(1);
    steeringPosition += analogRead(A0);
    delay(1);
    steeringPosition += analogRead(A0);
    delay(1);
    steeringPosition += analogRead(A0);
    delay(1);
    steeringPosition = steeringPosition >> 2; //divide by 4    
    steeringPosition = ( steeringPosition - steeringPositionZero);   //read the steering position sensor

    //convert position to steer angle. 4 counts per degree of steer pot position in my case
    //  ***** make sure that negative steer angle makes a left turn and positive value is a right turn *****
    // remove or add the minus for steerSensorCounts to do that.
    steerAngleActual = (float)(steeringPosition)/-steerSensorCounts;    

    //inclinometer
    delay(1);
    analogRead(A1); //discard
    delay(1);
    tilt = analogRead(A1);
   delay(1);
    tilt += analogRead(A1);
   delay(1);
    tilt += analogRead(A1);
   delay(1);
    tilt += analogRead(A1);

    tilt = tilt >> 2; //divide by 4

    //inclinometer goes from -25 to 25 from 0 volts to 5 volts
    tilt = map(tilt,0,1023,-500,500);

    //the steering part - if not off or invalid
    if (distanceFromLine == 32020 | distanceFromLine == 32000             //auto Steer is off if 32020, invalid if 32000
          | speeed < 1 | steerSwitch == 0 )                               //Speed is too slow, motor or footswitch out of place or pulling pin low
    {
      pwmDrive = 0; //turn off steering motor
      motorDrive();

      Wire.requestFrom(1, 4); //request the flow rate from slave #1

      // Receive the 'float' from I2C and print it:-
      if (Wire.available() >= 4)                  // Make sure there are two bytes.
      {
        for (int i = 0; i < 4; i++)             // Receive and rebuild the 'float'.
          float_data.rawData[i] = Wire.read(); //receive the raw float data
      }
      
      //send to AgOpenGPS - autosteer is off
      Serial.print(steerAngleActual);
      Serial.print(",");    
      Serial.print(Kp);
      Serial.print(",");    
      Serial.print(Ki,2);
      Serial.print(",");    
      Serial.print(Kd);
      Serial.print(",");    
      Serial.print(Ko);
      Serial.print(",");    
      Serial.println(tilt); //tilt
      //Serial.print(",");
      //Serial.print(litersPerhec); //flow rate

      
    }
     
    else          //valid spot to turn on autosteer
    {
      bitSet(PINB, 5);              //turn LED on showing in headingDelta loop      

      //calculate the steering error
      steerAngleError = steerAngleActual - steerAngleSetPoint;                    

      //do the pid
      calcSteeringPID();                                                                                                            
      motorDrive();    //out to motors the pwm value

      Wire.requestFrom(1, 4); //request the flow rate from slave #1
      
      // Receive the 'float' from I2C and print it:-
      if (Wire.available() >= 4)                  // Make sure there are two bytes.
      {
        for (int i = 0; i < 4; i++)             // Receive and rebuild the 'float'.
          float_data.rawData[i] = Wire.read(); //receive the raw float data
      }
      
      //send to agopenGPS
      Serial.print(steerAngleActual);
      Serial.print(",");    
      Serial.print((int)pValue);
      Serial.print(",");    
      Serial.print((int)iValue);
      Serial.print(",");    
      Serial.print((int)dValue);
      Serial.print(",");    
      Serial.print(pwmDisplay);
      Serial.print(",");
      Serial.println(tilt);   //tilt
      //Serial.print(",");
      //Serial.print(litersPerhec); //flow rate
       
    } 
  }  

  //Header has been found, presumably the next 8 bytes are the settings
  if (Serial.available() > 7 && isSettingFound)
  {  
    isSettingFound = false;

    //change the factors as required for your own PID values
    Kp = (float)Serial.read() * 1.0;   // read Kp from AgOpenGPS
    Ki = (float)Serial.read() * 0.1;   // read Ki from AgOpenGPS
    Kd = (float)Serial.read() * 1.0;   // read Kd from AgOpenGPS
    Ko = (float)Serial.read() * 0.1;   // read Ko from AgOpenGPS
    steeringPositionZero = 412 + Serial.read();  //read steering zero offset
    minPWMValue = Serial.read(); //read the minimum amount of PWM for instant on
    maxIntegralValue = Serial.read(); //
    steerSensorCounts = Serial.read()*0.1; //sent as 10 times the setting displayed in AOG
  }

  
}



