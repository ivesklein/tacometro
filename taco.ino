// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <CAN.h> //0.3.1 - the OBD2 library depends on the CAN library
#include <OBD2.h> //0.0.1
#include <Servo.h> //1.2.1

// required to log in SD
//#include <SPI.h>
//#include <SD.h>
//const int chipSelect = SDCARD_SS_PIN;

// gear led indicators
int gear1 = 7;
int gear2 = 6;
int gear3 = 5;
int gear4 = 21;
int gear5 = 4;

// acceleration led indicators
int up = 0;
int down = 1;

// clock needle servo
int servopin = 2;

Servo myservo;

// runtime vars
float rpm = 0;
float speed = 0;
int gear = 0;
float acc = 0.0;

bool turnOff = false;

// memory to calc averages
float ratio[13] = {
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0
};

// memory to calc averages
float rpms[12] = {
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0
};

// memory to calc averages
unsigned long times[12] = {
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0
};

// led indicators timers and status memory
unsigned long timerGear = 0;
unsigned long timerAcc = 0;
bool gl = false;
bool al = false;

// array of PID's to print values of
//const int PIDS[] = {
  //CALCULATED_ENGINE_LOAD,
  //ENGINE_COOLANT_TEMPERATURE,
  //ENGINE_RPM,
  //VEHICLE_SPEED,
  //AIR_INTAKE_TEMPERATURE,
  //MAF_AIR_FLOW_RATE,
  //THROTTLE_POSITION,
  //RUN_TIME_SINCE_ENGINE_START,
  //FUEL_TANK_LEVEL_INPUT,
  //ABSOLULTE_BAROMETRIC_PRESSURE,
  //ABSOLUTE_LOAD_VALUE,
  //RELATIVE_THROTTLE_POSITION
//};


// custom parameters for log function
// converts rpm acceleration to led blinking waiting time
//    5 rpm/s -> 800 ms
// 1000 rpm/s -> 100 ms
const float mx1 = 5.0;
const float my1 = 800.0;
const float mx2 = 1000.0;
const float my2 = 100.0;

// is to convert acc to led blinking time. chatgpt thing. UI needs more feeling calibration, linear interpolation would feel the same.
float logMap(float x) {
  // Check for edge cases to avoid division by zero
  if (x <= 0.0) return 0.0;
  if (x <= mx1) return my1;
  if (x >= mx2) return my2;

  // Calculate the logarithmic mapping
  float a = (log10(my2) - log10(my1)) / (mx2 - mx1);
  float b = log10(my1) - a * mx1;
  float result = pow(10, (a * x) + b);
  
  return result;
}

void setup() {
  // set pin modes
  pinMode(gear1, OUTPUT);
  pinMode(gear2, OUTPUT);
  pinMode(gear3, OUTPUT);
  pinMode(gear4, OUTPUT);
  pinMode(gear5, OUTPUT);
  pinMode(up, OUTPUT);
  pinMode(down, OUTPUT);

  // turn off everything
  digitalWrite(gear1, false);
  digitalWrite(gear2, false);
  digitalWrite(gear3, false);
  digitalWrite(gear4, false);
  digitalWrite(gear5, false);
  digitalWrite(up, false);
  digitalWrite(down, false);

  // start init animation
  myservo.attach(servopin);
  myservo.write(3);
  delay(1000);
  // gear 1
  digitalWrite(gear1, true);
  digitalWrite(gear2, false);
  digitalWrite(gear3, false);
  digitalWrite(gear4, false);
  digitalWrite(gear5, false);
  myservo.write(32);
  delay(1000);
  // gear 2
  digitalWrite(gear2, true);
  digitalWrite(gear1, false);
  digitalWrite(gear3, false);
  digitalWrite(gear4, false);
  digitalWrite(gear5, false);
  myservo.write(69);
  // gear 3
  delay(1000);
  digitalWrite(gear3, true);
  digitalWrite(gear2, false);
  digitalWrite(gear1, false);
  digitalWrite(gear4, false);
  digitalWrite(gear5, false);
  myservo.write(102);
  // gear 4
  delay(1000);
  digitalWrite(gear4, true);
  digitalWrite(gear2, false);
  digitalWrite(gear3, false);
  digitalWrite(gear1, false);
  digitalWrite(gear5, false);
  myservo.write(137);
  // gear 5
  delay(1000);
  digitalWrite(gear5, true);
  digitalWrite(gear2, false);
  digitalWrite(gear3, false);
  digitalWrite(gear4, false);
  digitalWrite(gear1, false);
  myservo.write(168);
  // turn off
  delay(1000);
  digitalWrite(gear1, false);
  digitalWrite(gear2, false);
  digitalWrite(gear3, false);
  digitalWrite(gear4, false);
  digitalWrite(gear5, false);
  myservo.write(3);
  delay(1000);
  // turn off servo
  myservo.detach();

  // see if the card is present and can be initialized:
  //if (!SD.begin(chipSelect)) {
    //digitalWrite(down, true);
    // don't do anything more:
    //while (1);
  //}

  // connect OBD2
  while (true) {
    if (!OBD2.begin()) {
      // failed
      delay(1000);
    } else {
      // success
      break;
    }
  }

}

void loop() {

  readValues();

  if(rpm>0){
    myservo.attach(servopin);
    detectGear();
    calcAcc();
    moveClock();
    updateGear();
    updateAcc();

    // turn off procedure must run when off detected
    turnOff = true;
    delay(50);
  }else{
    if(turnOff){
      moveClock();
      delay(200);
      myservo.detach();
      alloff();
      turnOff = false;
    }
  }

}

void alloff(){
  digitalWrite(gear1, false);
  digitalWrite(gear2, false);
  digitalWrite(gear3, false);
  digitalWrite(gear4, false);
  digitalWrite(gear5, false);
  digitalWrite(up, false);
  digitalWrite(down, false);
}

void readValues(){
  rpm = OBD2.pidRead(ENGINE_RPM);
  if (isnan(rpm)) {
    rpm = 0.0;
  }
  speed = OBD2.pidRead(VEHICLE_SPEED);
  if (isnan(speed)) {
    speed = 0.0;
  }
}

void moveClock(){
  int pos = linearInterpolation(rpm);
  myservo.write(pos);
}

/*void saveValues(){
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {

    String dataString = "";

    dataString += String(millis());
    dataString += ",";
    dataString += String(rpm);
    dataString += ",";
    dataString += String(speed);

    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    //Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    //Serial.println("error opening datalog.txt");
  }

}*/

void detectGear(){

  // remove first, add new value last
  ratio[0] = ratio[1];
  ratio[1] = ratio[2];
  ratio[2] = ratio[3];
  ratio[3] = ratio[4];
  ratio[4] = ratio[5];
  ratio[5] = ratio[6];
  ratio[6] = ratio[7];
  ratio[7] = ratio[8];
  ratio[8] = ratio[9];
  ratio[9] = ratio[10];
  ratio[10] = ratio[11];
  ratio[11] = ratio[12];
  ratio[12] = rpm/speed; // new value is the speed to rpm ratio

  float average = (ratio[0]+ratio[1]+ratio[2]+ratio[3]+ratio[4]+ratio[5]+ratio[6]+ratio[7]+ratio[8]+ratio[9]+ratio[10]+ratio[11]+ratio[12])/13;

  // ranges to detect gear must be adquired by data analysis or car specifications
  if(average>=135&&average<=150){
    gear = 1;
  }else if(average>=77&&average<=85){
    gear = 2;
  }else if(average>=46&&average<=49){
    gear = 3;
  }else if(average>=36&&average<=38){
    gear = 4;
  }else if(average>=28&&average<=31){
    gear = 5;
  }else{
    gear = 0;
  }
}

// recomended gear is based on arbitrary values(more analysis needed)
int recGear(){
  if(speed<=27){
    return 1;
  }else if(speed<=50){
    return 2;
  }else if(speed<=83){
    return 3;
  }else if(speed<=108){
    return 4;
  }else{
    return 5;
  }
}

void calcAcc(){

  rpms[0] = rpms[1];
  rpms[1] = rpms[2];
  rpms[2] = rpms[3];
  rpms[3] = rpms[4];
  rpms[4] = rpms[5];
  rpms[5] = rpms[6];
  rpms[6] = rpms[7];
  rpms[7] = rpms[8];
  rpms[8] = rpms[9];
  rpms[9] = rpms[10];
  rpms[10] = rpms[11];
  rpms[11] = rpm;

  times[0] = times[1];
  times[1] = times[2];
  times[2] = times[3];
  times[3] = times[4];
  times[4] = times[5];
  times[5] = times[6];
  times[6] = times[7];
  times[7] = times[8];
  times[8] = times[9];
  times[9] = times[10];
  times[10] = times[11];
  times[11] = millis();

  float dxdy[11] = {
    (rpms[1]-rpms[0])/((times[1]-times[0])/1000.0),
    (rpms[2]-rpms[1])/((times[2]-times[1])/1000.0),
    (rpms[3]-rpms[2])/((times[3]-times[2])/1000.0),
    (rpms[4]-rpms[3])/((times[4]-times[3])/1000.0),
    (rpms[5]-rpms[4])/((times[5]-times[4])/1000.0),
    (rpms[6]-rpms[5])/((times[6]-times[5])/1000.0),
    (rpms[7]-rpms[6])/((times[7]-times[6])/1000.0),
    (rpms[8]-rpms[7])/((times[8]-times[7])/1000.0),
    (rpms[9]-rpms[8])/((times[9]-times[8])/1000.0),
    (rpms[10]-rpms[9])/((times[10]-times[9])/1000.0),
    (rpms[11]-rpms[10])/((times[11]-times[10])/1000.0),
  };

  acc = (dxdy[0]+dxdy[1]+dxdy[2]+dxdy[3]+dxdy[4]+dxdy[5]+dxdy[6]+dxdy[7]+dxdy[8]+dxdy[9]+dxdy[10])/11;
}

// iteration job to update the gear leds based on values and timers
void updateGear(){
  digitalWrite(gear1, false);
  digitalWrite(gear2, false);
  digitalWrite(gear3, false);
  digitalWrite(gear4, false);
  digitalWrite(gear5, false);

  switch (gear) {
    case 1:
      digitalWrite(gear1, true);
      break;
    case 2:
      digitalWrite(gear2, true);
      break;
    case 3:
      digitalWrite(gear3, true);
      break;
    case 4:
      digitalWrite(gear4, true);
      break;
    case 5:
      digitalWrite(gear5, true);
      break;
    default:
      break;
  }

  int recgear = recGear();
  int recgearPin = -1;
  
  switch (recgear) {
    case 1:
      recgearPin = gear1;
      break;
    case 2:
      recgearPin = gear2;
      break;
    case 3:
      recgearPin = gear3;
      break;
    case 4:
      recgearPin = gear4;
      break;
    case 5:
      recgearPin = gear5;
      break;
    default:
      break;
  }

  if(!(recgear==gear)){
    if(millis()-timerGear>(gl?50:400)){
      timerGear = millis();
      if(gl){
        gl = false;
        digitalWrite(recgearPin, false);
      }else{
        gl = true;
        digitalWrite(recgearPin, true);
      }
    }else{
      if(gl){
        digitalWrite(recgearPin, true);
      }
    }
  }

}

// iteration job to update the acceleration leds based on values and timers
void updateAcc(){
  digitalWrite(up, false);
  digitalWrite(down, false);

  Serial.print(acc);

  if(abs(acc)>20){
    int direction = (acc>0)?up:down;

    float dur = logMap(abs(acc));
    Serial.print(" ");
    Serial.print(dur);

    if(millis()-timerAcc>(gl?50:dur)){
      timerAcc = millis();
      if(al){
        al = false;
        digitalWrite(direction, false);
      }else{
        al = true;
        digitalWrite(direction, true);
      }
    }else{
      if(al){
        digitalWrite(direction, true);
      }
    }
  }
  Serial.println("");
}

// Define the data points for linear regression
// values based on servo positions and physical print positions
const float dataPoints[][2] = {
  {0.0, 3.0}, // 0 rpm
  {1000.0, 32.0}, // 1000 rpm
  {2000.0, 69.0}, // 2000 rpm
  {3000.0, 102.0}, // 3000 rpm
  {4000.0, 137.0}, // 4000 rpm
  {5000.0, 168.0} // 5000 rpm
};

// chatgpt thing (i already write something similar in older projects and is a pain, i thing some libs have this functionality too)
int linearInterpolation(float x) {
  int n = sizeof(dataPoints) / sizeof(dataPoints[0]);
  
  // Check if x is outside the range of provided data
  if (x <= dataPoints[0][0]) {
    return int(dataPoints[0][1]);
  } else if (x >= dataPoints[n - 1][0]) {
    return int(dataPoints[n - 1][1]);
  }
  
  // Perform linear interpolation
  for (int i = 0; i < n - 1; i++) {
    if (x >= dataPoints[i][0] && x <= dataPoints[i + 1][0]) {
      // Calculate the interpolated value
      float slope = (dataPoints[i + 1][1] - dataPoints[i][1]) / (dataPoints[i + 1][0] - dataPoints[i][0]);
      return int(dataPoints[i][1] + slope * (x - dataPoints[i][0]));
    }
  }
  
  // Default case (shouldn't reach here)
  return 0;
}
