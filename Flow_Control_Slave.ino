#include <Wire.h>
#include <ppmreader.h>

  
  #define valvedirection 5 //direction to drive flow control valve
  #define valvePin 6 //turn valve on/off
  #define flowinPin A2 //bound to digital pin 2 and interrupt 0
  
  #define target_Rate 75.708 //hard coded to 75.708L       ~20gpa
  #define nozzle_flow 0.75708 //flow rate in Lpm @ 40psi
  #define nozzle_space 50.8 //spacing of nozzles in cm
  //the I2C pins are A4 and A5
  
  //sprayer values
  byte sensorInterrupt = 0;  // 0 = digital pin 2
  
  // The Raven flow sensor outputs approximately 72 pulses per gal of flow
  //AOG does everything in metric so convert to pulses per liter. 1 gal = 3.7854118 L---> 72/3.7854118
  //this gives 19.0203877 pulses per liter so our calibration factor will be x2 = 38.0407754
  //since we are counting on pulse CHANGE
  
  float calibrationFactor = 38.0407754, adjust_time = 0; 
  float litersPerhec = 0;
  
  unsigned int litersPerMinute = 0, GPA = 0;
  unsigned long oldTime = 0, oldtime1 = 0;

 // double flowRate = 0;
  int valve_direction = -1;
  byte relay = 0, speeed = 0;

  PPMReader PPMReader; //create the PPMReader class

void setup() 
{
   //sprayer
  pinMode(valvePin, OUTPUT);
  pinMode(valvedirection, OUTPUT);
  pinMode(flowinPin, INPUT);
  digitalWrite(flowinPin, HIGH);

  Wire.begin(1);                // join i2c bus with slave address #1
  Wire.onRequest(requestEvent); // fucntion to run when asking for data
  Wire.onReceive(receiveEvent); // what to do when receiving data

  // The flow sensor is connected to pin 2 which uses interrupt 0.
  // Configured to trigger on a CHANGE state change (transition from HIGH
  // state to LOW state or LOW to HIGH)
  attachInterrupt(sensorInterrupt, pulseCounter, CHANGE);
}

void loop() 
{  
  unsigned long currentMillis = millis(); //check to see if it is time to check meter

  if((relay & 255) && (currentMillis - oldTime >= 10)) //there is at least one relay on and 10 ms passed so check and change rate
  {
    oldTime = currentMillis; //save last time we checked stuff
    litersPerMinute = PPMReader.get_ppm() / calibrationFactor; //get the liters per minute since last checked by dividing ppm by the pulses per liter

    int number_nozzles = 0; //local variable reset every time enter calculations
    
    //find sections on and calculate # nozzles on
    //#nozzles on each section of our sprayers
    if(bitRead(relay,0)) number_nozzles += 3;
    else if(bitRead(relay,1)) number_nozzles += 9;
    else if(bitRead(relay,2)) number_nozzles += 9;
    else if(bitRead(relay,3)) number_nozzles += 9;
    else if(bitRead(relay,4)) number_nozzles += 9;
    else if(bitRead(relay,5)) number_nozzles += 9;
    else if(bitRead(relay,6)) number_nozzles += 9;
    else if(bitRead(relay,7)) number_nozzles += 3;
      
    litersPerhec = (litersPerMinute * 60) / (speeed * nozzle_space * number_nozzles); //measured L/min  divided by kph * width gives measured L/hec;
    //GPA = (litersPerhec * 0.264172) / 2.47105; //(L/hec * L/gal) * hec/ac gives Gal/ac 

    adjust_time = 1000 * abs(target_Rate - litersPerhec); //time to adjust valve is 1000ms * difference in target rate vs applied rate

    digitalWrite(valvePin, 0); //make sure valve is shut off while we change directions to avoid damage to H-bridge
    
    if(litersPerhec > target_Rate) //putting on too much product
    {
      digitalWrite(valvedirection, 1); //set valve direction to close
      digitalWrite(valvePin, 1); //start the valve adjusting
    }
    else if (litersPerhec < target_Rate)
    {
      digitalWrite(valvedirection, 0); //set valve direction to open
      digitalWrite(valvePin, 1); //start the valve adjusting
    }
    else
    {
      digitalWrite(valvePin, 0); //rate is correct, do nothing, stop valve
    }
    
    oldtime1 = millis(); //capture current time 
    
    //update the displayed rate
    Serial.print("Flow Rate in LPH: ");
    Serial.print(litersPerhec);
    
  }

  if((millis() - oldtime1)  >= adjust_time)
  {
    digitalWrite(valvePin, 0); //shut the valve off 
  }

}

// function: what to do when asked for data
void requestEvent() 
{
  Wire.write((byte*)&litersPerhec, 4); //transmit the flow rate one byte at a time
}

// what to do when receiving data from master
//master only sends speed and relay
void receiveEvent(int howMany)
{
  speeed = Wire.read(); //get the current speed
  relay = Wire.read(); //get the current relay state
}


/*
Insterrupt Service Routine
 */
void pulseCounter()
{
  // Increment the pulse counter
  PPMReader.on_trigger();
}
