
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include <stdlib.h>
//#include <avr.h>

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "5d8a3150-8994-4b11-8ea4-d06dfd5e65bd"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

//////////// IMU //////////////////

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "math.h"

//#define USE_SPI       // Uncomment this to use SPI

#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif

//////////// IMU //////////////////


//////////// TEMP ADC /////////////
// #define RESOLUTION_BITS (16)      // choose resolution (explained in depth below)

// #ifdef ADCPIN
// #define EXTERNAL_ADC_PIN ADCPIN   // ADCPIN is the lowest analog capable pin exposed on the variant
// #endif
//////////// TEMP ADC /////////////


//////////// TOF //////////////////

#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 4 // formerly 3

//Uncomment the following line to use the optional shutdown and interrupt pins.
SFEVL53L1X distanceSensor1(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

SFEVL53L1X distanceSensor2(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

///////////  TOF //////////////////



//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

// for driving
const int LeftPin1 = 0;
const int LeftPin2 = 15;
const int RightPin1 = 3;
const int RightPin2 = 16;

// for PID
float kp = 1.0;
//////////// Global Variables ////////////


void driveForward(int speed){
  digitalWrite(RightPin2, 0);
  digitalWrite(LeftPin2, 0);

  analogWrite(RightPin1, speed);
  analogWrite(LeftPin1, speed);
}

void driveBackward(int speed){
  digitalWrite(RightPin1, 0);
  digitalWrite(LeftPin1, 0);

  analogWrite(RightPin2, speed);
  analogWrite(LeftPin2, speed);
}

void stopDriving(){
  analogWrite(RightPin1, 0);
  analogWrite(RightPin2, 0);
  analogWrite(LeftPin1, 0);
  analogWrite(LeftPin2, 0);
}


///////////// Testing SINGLE TOF ///////////////

void
read_data()
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}

void sendReading(int distance, int time, int sensorN){

  char dst_arr[MAX_MSG_SIZE];
  sprintf(dst_arr, "%d", distance);

  char time_arr[MAX_MSG_SIZE];
  sprintf(time_arr, "%d", time);

  char sensorN_arr[MAX_MSG_SIZE];
  sprintf(sensorN_arr, "%d", sensorN);

  tx_estring_value.clear();
  tx_estring_value.append(dst_arr);
  tx_estring_value.append("|");
  tx_estring_value.append(time_arr);
  tx_estring_value.append("|");
  tx_estring_value.append(sensorN_arr);
  tx_characteristic_string.writeValue(tx_estring_value.c_str());

  Serial.print("Sent back: ");
  Serial.println(tx_estring_value.c_str());

}

void newReading(){

    int start_time = millis();
    distanceSensor1.startRanging(); //Write configuration bytes to initiate measurement
    while (!distanceSensor1.checkForDataReady())
    {
      delay(1);
    }
    int distance = distanceSensor1.getDistance(); //Get the result of the measurement from the sensor
    distanceSensor1.clearInterrupt();
    distanceSensor1.stopRanging();

    int ranging_time = millis() - start_time;
    Serial.print("TOF 1 Ranging Time (mS): ");
    Serial.print(ranging_time);

    Serial.print(" TOF 1 Distance(mm): ");
    Serial.print(distance);

    Serial.println();

    sendReading(distance, ranging_time, 1);

    start_time = millis();
    distanceSensor2.startRanging(); //Write configuration bytes to initiate measurement
    while (!distanceSensor2.checkForDataReady())
    {
      delay(1);
    }
    distance = distanceSensor2.getDistance(); //Get the result of the measurement from the sensor
    distanceSensor2.clearInterrupt();
    distanceSensor2.stopRanging();

    ranging_time = millis() - start_time;
    Serial.print("TOF 2 Ranging Time (mS): ");
    Serial.print(ranging_time);

    Serial.print(" TOF 2 Distance(mm): ");
    Serial.print(distance);

    Serial.println();

    sendReading(distance, ranging_time, 2);
}

void readAndSend50TOF(){
    for (int location = 0; location < 14; location++){
    
      for (int sample = 0; sample < 50; sample++){
        newReading();
      }

      Serial.println(" ");
      Serial.println("...");
      Serial.println("Next test in 10 seconds");
      Serial.println("...");
      Serial.println(" ");
      delay(7000);
      Serial.println("...");
      Serial.println("Next mm test in 3 seconds");
      Serial.println("...");
      delay(3000);
      Serial.println(" ");
    
    }

    Serial.println(" ");
    Serial.println("Complete");
    Serial.println(" ");
}

//////////////// Testing TOF /////////////////

//////////////// Timed TOF ///////////////////

void continuous5SRead(){
  distanceSensor1.startRanging();
  distanceSensor2.startRanging();
  int start_time = millis();

  while(millis() - start_time < 5000){
    if (distanceSensor1.checkForDataReady())
    {
      int ONEdistance = distanceSensor1.getDistance(); 
      distanceSensor1.clearInterrupt();
      distanceSensor1.stopRanging();
      Serial.print("At ");
      Serial.print(millis());
      Serial.print(" TOF ONE reads:");
      Serial.println(ONEdistance);
      sendReading(ONEdistance, millis(), 1);
      distanceSensor1.startRanging();
    }

    if (distanceSensor2.checkForDataReady())
    {
      int TWOdistance = distanceSensor2.getDistance(); 
      distanceSensor2.clearInterrupt();
      distanceSensor2.stopRanging();
      Serial.print("At ");
      Serial.print(millis());
      Serial.print(" TOF TWO reads:");
      Serial.println(TWOdistance);
      sendReading(TWOdistance, millis(), 2);
      distanceSensor2.startRanging();
    }
}

  }
  
/////////////// Timed TOF ////////////////////

void readTimedTOF(){

  Serial.print("Current time: ");
  Serial.println(millis());

    if (distanceSensor1.checkForDataReady())
    {
      int ONEdistance = distanceSensor1.getDistance(); 
      distanceSensor1.clearInterrupt();
      distanceSensor1.stopRanging();
      Serial.print("At ");
      Serial.print(millis());
      Serial.print(" TOF ONE reads:");
      Serial.println(ONEdistance);
      distanceSensor1.startRanging();
    }

    if (distanceSensor2.checkForDataReady())
    {
      int TWOdistance = distanceSensor2.getDistance(); 
      distanceSensor2.clearInterrupt();
      distanceSensor2.stopRanging();
      Serial.print("At ");
      Serial.print(millis());
      Serial.print(" TOF TWO reads:");
      Serial.println(TWOdistance);
      distanceSensor2.startRanging();
    }

}

////////////////// TIMED_FFT (ACCEL and GYRO) //////////////////////////

float getPitch(ICM_20948_I2C *sensor){
  return atan2(sensor->accX(),sensor->accZ()) * 180/M_PI; 
} 

float getRoll(ICM_20948_I2C *sensor){
  return -atan2(sensor->accY(), sensor->accZ()) * 180/M_PI;
}

float getPitchGyr(ICM_20948_I2C *sensor, float pitch, float dt){
  pitch = pitch - sensor->gyrY()*dt/1000.;
  if (pitch < -180){
    pitch = 180 - (abs(pitch) - 180);
  }
  if (pitch > 180){
    pitch = -180 + (pitch - 180);
  }
  return pitch;
}

float getRollGyr(ICM_20948_I2C *sensor, float roll, float dt){
  roll = roll - sensor->gyrX()*dt/1000.;
    if (roll < -180){
    roll = 180 - (abs(roll) - 180);
  }
  if (roll > 180){
    roll = -180 + (roll - 180);
  }
  return roll;
}

float getYawGyr(ICM_20948_I2C *sensor, float yaw, float dt){
  yaw = yaw - sensor->gyrZ()*dt/1000.;
    if (yaw < -180){
    yaw = 180 - (abs(yaw) - 180);
  }
  if (yaw > 180){
    yaw = -180 + (yaw - 180);
  }
  return yaw;
}

void sendFFTData(int time, float pitchA, float rollA, float pitchG, float rollG, float yawG){

  // char time_arr[MAX_MSG_SIZE];
  // sprintf(time_arr, "%d", millis());

  tx_estring_value.clear();
  tx_estring_value.append(time);
  tx_estring_value.append("|");
  tx_estring_value.append(pitchA);
  tx_estring_value.append("|");
  tx_estring_value.append(rollA);
  tx_estring_value.append("|");
  tx_estring_value.append(pitchG);
  tx_estring_value.append("|");
  tx_estring_value.append(rollG);
  tx_estring_value.append("|");
  tx_estring_value.append(yawG);
  tx_characteristic_string.writeValue(tx_estring_value.c_str());

  //Serial.print("Sent back: ");
  //Serial.println(tx_estring_value.c_str());
}

void sendComplement(int time, float cP, float cR){
  tx_estring_value.clear();
  tx_estring_value.append(time);
  tx_estring_value.append("|");
  tx_estring_value.append(cP);
  tx_estring_value.append("|");
  tx_estring_value.append(cR);
  tx_characteristic_string.writeValue(tx_estring_value.c_str());
}

void sendIMUVals(int time, float cP, float cR, float yaw){
  tx_estring_value.clear();
  tx_estring_value.append(time);
  tx_estring_value.append("|");
  tx_estring_value.append(cP);
  tx_estring_value.append("|");
  tx_estring_value.append(cR);
  tx_estring_value.append("|");
  tx_estring_value.append(yaw);
  tx_characteristic_string.writeValue(tx_estring_value.c_str());  
}

void sendAllData(int time, float cP, float cR, float yaw, int ONEDist, int TWODist){
  tx_estring_value.clear();
  tx_estring_value.append(time);
  tx_estring_value.append("|");
  tx_estring_value.append(cP);
  tx_estring_value.append("|");
  tx_estring_value.append(cR);
  tx_estring_value.append("|");
  tx_estring_value.append(yaw);
  tx_estring_value.append("|");
  tx_estring_value.append(ONEDist);
  tx_estring_value.append("|");
  tx_estring_value.append(TWODist);
  tx_characteristic_string.writeValue(tx_estring_value.c_str());  
  
}

void sendPIDDebug(int time, float cP, float cR, float yaw, int ONEDist, int TWODist, int leftPWM, int rightPWM){
  tx_estring_value.clear();
  tx_estring_value.append(time);
  tx_estring_value.append("|");
  tx_estring_value.append(cP);
  tx_estring_value.append("|");
  tx_estring_value.append(cR);
  tx_estring_value.append("|");
  tx_estring_value.append(yaw);
  tx_estring_value.append("|");
  tx_estring_value.append(ONEDist);
  tx_estring_value.append("|");
  tx_estring_value.append(TWODist);
  tx_estring_value.append("|");
  tx_estring_value.append(leftPWM);
  tx_estring_value.append("|");
  tx_estring_value.append(rightPWM);
  tx_characteristic_string.writeValue(tx_estring_value.c_str());  
  
}  

int entries = 3;


void sendDebugArr(int time[], int cP[], int cR[], int yaw[], int ONEDist[], int TWODist[], int leftPWM[], int rightPWM[]){
  tx_estring_value.clear();
  for (int count = 0; count < entries; count++){
    tx_estring_value.append(time[count]);
    tx_estring_value.append("|");
    tx_estring_value.append(cP[count]);
    tx_estring_value.append("|");
    tx_estring_value.append(cR[count]);
    tx_estring_value.append("|");
    tx_estring_value.append(yaw[count]);
    tx_estring_value.append("|");
    tx_estring_value.append(ONEDist[count]);
    tx_estring_value.append("|");
    tx_estring_value.append(TWODist[count]);
    tx_estring_value.append("|");
    tx_estring_value.append(leftPWM[count]);
    tx_estring_value.append("|");
    tx_estring_value.append(rightPWM[count]);
    if (count != entries - 1){
      tx_estring_value.append("|");
    }
  }
  tx_characteristic_string.writeValue(tx_estring_value.c_str());  
  
}  

void sendPIDDebugArr(int time[], int cP[], int cR[], int yaw[], int ONEDist[], int errorArr[], int speedArr[]){
  tx_estring_value.clear();
  for (int count = 0; count < entries; count++){
    tx_estring_value.append(time[count]);
    tx_estring_value.append("|");
    tx_estring_value.append(cP[count]);
    tx_estring_value.append("|");
    tx_estring_value.append(cR[count]);
    tx_estring_value.append("|");
    tx_estring_value.append(yaw[count]);
    tx_estring_value.append("|");
    tx_estring_value.append(ONEDist[count]);
    tx_estring_value.append("|");
    tx_estring_value.append(errorArr[count]);
    tx_estring_value.append("|");
    tx_estring_value.append(speedArr[count]);
    if (count != entries - 1){
      tx_estring_value.append("|");
    }
  }
  tx_characteristic_string.writeValue(tx_estring_value.c_str());  
  
}  

////////////////// TIMED_FFT //////////////////////////

int ONEDistance = 0;
int TWODistance = 0;

void IMUAndTOF(){

  int start_time = millis();
  int last_time = start_time;
  int instances = 0;

  float curPitchG = getPitch(&myICM);
  float curRollG = getRoll(&myICM);
  float curYawG = 0.0;

  float complementPitch = getPitch(&myICM);
  float complementRoll = getRoll(&myICM);



  while (millis() - start_time < 5000){
     if (myICM.dataReady()){
        myICM.getAGMT();
        
        // Sending IMU data 
        curPitchG = getPitchGyr(&myICM, curPitchG, millis() - last_time);
        curRollG = getRollGyr(&myICM, curRollG, millis() - last_time);
        curYawG = getYawGyr(&myICM,curYawG,  millis() - last_time);
        last_time = millis();

        //sendFFTData(millis(), getPitch(&myICM), getRoll(&myICM), curPitchG, curRollG, curYawG);

        complementPitch = curPitchG*0.9 + getPitch(&myICM)*0.1;
        complementRoll = curRollG*0.9 + getRoll(&myICM)*0.1;

      }  
            
    //sendComplement(millis(), complementPitch, complementRoll);  

    // Sending TOF Data
    if (distanceSensor1.checkForDataReady()){
        ONEDistance = distanceSensor1.getDistance(); 
        distanceSensor1.clearInterrupt();
        distanceSensor1.stopRanging();
        distanceSensor1.startRanging();
    }

    if (distanceSensor2.checkForDataReady()){
        TWODistance = distanceSensor2.getDistance(); 
        distanceSensor2.clearInterrupt();
        distanceSensor2.stopRanging();
        distanceSensor2.startRanging();
    }

    sendAllData(millis(), complementPitch, complementRoll, curYawG, ONEDistance, TWODistance);   

  }
}


////////////////// TIMED_FFT //////////////////////////

////////////////// TIMED_PID_DATA //////////////////////////

// Collect TOF, IMU, and PWM for 5 seconds.

void collectPIDDebug(){

  int start_time = millis();
  int last_time = start_time;
  int instances = 0;

  int ONEDistance = 0;
  int TWODistance = 0;

  int leftPWM = 0;
  int rightPWM = 0;  

  float curPitchG = getPitch(&myICM);
  float curRollG = getRoll(&myICM);
  float curYawG = 0.0;

  float complementPitch = getPitch(&myICM);
  float complementRoll = getRoll(&myICM);

  int timeArr[5];
  int pitchArr[5];
  int rollArr[5];
  int yawArr[5];
  int ONEDistArr[5];
  int TWODistArr[5];
  int leftPWMArr[5];
  int rightPWMArr[5];

  int logCount = 0;

  int entries = 3;


  distanceSensor1.startRanging();
  distanceSensor2.startRanging();

  while (millis() - start_time < 5000){
     if (myICM.dataReady()){
        myICM.getAGMT();
        
        // Sending IMU data 
        curPitchG = getPitchGyr(&myICM, curPitchG, millis() - last_time);
        curRollG = getRollGyr(&myICM, curRollG, millis() - last_time);
        curYawG = getYawGyr(&myICM,curYawG,  millis() - last_time);
        last_time = millis();

        //sendFFTData(millis(), getPitch(&myICM), getRoll(&myICM), curPitchG, curRollG, curYawG);

        complementPitch = curPitchG*0.9 + getPitch(&myICM)*0.1;
        complementRoll = curRollG*0.9 + getRoll(&myICM)*0.1;

      }  
            
    //sendComplement(millis(), complementPitch, complementRoll);  

    // Sending TOF Data
    if (distanceSensor1.checkForDataReady()){
        ONEDistance = distanceSensor1.getDistance(); 
        distanceSensor1.clearInterrupt();
        distanceSensor1.stopRanging();
        distanceSensor1.startRanging();
    }

    if (distanceSensor2.checkForDataReady()){
        TWODistance = distanceSensor2.getDistance(); 
        distanceSensor2.clearInterrupt();
        distanceSensor2.stopRanging();
        distanceSensor2.startRanging();
    }

    // to send the most recent reading on every iteration
    //sendPIDDebug(millis(), complementPitch, complementRoll, curYawG, ONEDistance, TWODistance, leftPWM, rightPWM);

    // log up to 10000 readings for each stream
    if (logCount < entries){
      timeArr[logCount] = millis();
      pitchArr[logCount] = complementPitch;
      rollArr[logCount] = complementRoll;
      yawArr[logCount] = curYawG;
      ONEDistArr[logCount] = ONEDistance;
      TWODistArr[logCount] = TWODistance;
      leftPWMArr[logCount] = rightPWM;
      rightPWMArr[logCount] = leftPWM;
      logCount += 1;
    } 
    // if reached conservative number of logs during sampling(to ensure not running out of memory)
    else{

      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      sendDebugArr(timeArr, pitchArr, rollArr, yawArr, ONEDistArr, TWODistArr, leftPWMArr, rightPWMArr);

      // reset all logs at once
      logCount = 0;
    }

  }
}

////////////////// TIMED_PID_DATA //////////////////////////

//////////////// UPDATE_KP ////////////////////////

void updateKP(float newKp){

  kp = newKp;

  Serial.print(kp);
  Serial.println(" is the new kp value");
}

//////////////// UPDATE_KP ////////////////////////


//////// TOF_PID ///////////////

void TOFPID(){

  int targetDistance;
  robot_cmd.get_next_value(targetDistance);

  int start_time = millis();
  int last_time = start_time;
  int instances = 0;

  int ONEDistance = 0;
  int TWODistance = 0;

  int leftPWM = 0;
  int rightPWM = 0;  

  float curPitchG = getPitch(&myICM);
  float curRollG = getRoll(&myICM);
  float curYawG = 0.0;

  float complementPitch = getPitch(&myICM);
  float complementRoll = getRoll(&myICM);

  int timeArr[5];
  int pitchArr[5];
  int rollArr[5];
  int yawArr[5];
  int ONEDistArr[5];
  int TWODistArr[5];
  int leftPWMArr[5];
  int rightPWMArr[5];
  int errorArr[5];
  int speedArr[5];

  int logCount = 0;

  int entries = 3;

  int timeWindow = 5000;

  int error = 0;

  int speed = 0;


  distanceSensor1.startRanging();
  distanceSensor2.startRanging();


  while (millis() - start_time < timeWindow){

    int error = ONEDistance - targetDistance;

    int speed = (int) (kp * error);

    if (speed < -255) speed = -255;
    else if (speed > 255) speed = 255;

    if (speed > 0){
      driveForward(speed);
    }
    else{
      driveForward(-speed);
    }



     if (myICM.dataReady()){
        myICM.getAGMT();
        
        // Sending IMU data 
        curPitchG = getPitchGyr(&myICM, curPitchG, millis() - last_time);
        curRollG = getRollGyr(&myICM, curRollG, millis() - last_time);
        curYawG = getYawGyr(&myICM,curYawG,  millis() - last_time);
        last_time = millis();

        //sendFFTData(millis(), getPitch(&myICM), getRoll(&myICM), curPitchG, curRollG, curYawG);

        complementPitch = curPitchG*0.9 + getPitch(&myICM)*0.1;
        complementRoll = curRollG*0.9 + getRoll(&myICM)*0.1;

      }  
            
    //sendComplement(millis(), complementPitch, complementRoll);  

    // Sending TOF Data
    if (distanceSensor1.checkForDataReady()){
        ONEDistance = distanceSensor1.getDistance(); 
        distanceSensor1.clearInterrupt();
        distanceSensor1.stopRanging();
        distanceSensor1.startRanging();
    }

    if (distanceSensor2.checkForDataReady()){
        TWODistance = distanceSensor2.getDistance(); 
        distanceSensor2.clearInterrupt();
        distanceSensor2.stopRanging();
        distanceSensor2.startRanging();
    }

    // to send the most recent reading on every iteration
    //sendPIDDebug(millis(), complementPitch, complementRoll, curYawG, ONEDistance, TWODistance, leftPWM, rightPWM);

    // log up to 10000 readings for each stream
    if (logCount < entries){
      timeArr[logCount] = millis();
      pitchArr[logCount] = complementPitch;
      rollArr[logCount] = complementRoll;
      yawArr[logCount] = curYawG;
      ONEDistArr[logCount] = ONEDistance;
      errorArr[logCount] = error;
      speedArr[logCount] = speed;
      logCount += 1;
    } 
    // if reached conservative number of logs during sampling(to ensure not running out of memory)
    else{

      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      sendPIDDebugArr(timeArr, pitchArr, rollArr, yawArr, ONEDistArr, errorArr, speedArr);

      // reset all logs at once
      logCount = 0;
    }

  }

  stopDriving();
}


//////// TOF_PID ///////////////


enum CommandTypes
{
    PING,
    SEND_TWO_INTS,
    SEND_THREE_FLOATS,
    ECHO,
    DANCE,
    SET_VEL,
    GET_TIME_MILLIS,
    GET_TEMP_5s,
    GET_TEMP_5s_RAPID,
    TEST_TOF,
    TIMED_TOF,
    TIMED_FFT,
    TIMED_PID_DATA,
    TOF_PID,
    UPDATE_KP
    
};

void
handle_command()
{   
    // Set the command string from the characteristic value
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;

    // Get robot command type (an integer)
    /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
     * since it uses strtok internally (refer RobotCommand.h and 
     * https://www.cplusplus.com/reference/cstring/strtok/)
     */
    success = robot_cmd.get_command_type(cmd_type);

    // Check if the last tokenization was successful and return if failed
    if (!success) {
        return;
    }

    // Handle the command type accordingly
    switch (cmd_type) {
        /*
         * Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
         */
        case PING:
            tx_estring_value.clear();
            tx_estring_value.append("PONG");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        /*
         * Extract two integers from the command string
         */
        case SEND_TWO_INTS:
            int int_a, int_b;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_a);
            if (!success)
                return;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_b);
            if (!success)
                return;

            Serial.print("Two Integers: ");
            Serial.print(int_a);
            Serial.print(", ");
            Serial.println(int_b);
            
            break;
        /*
         * Extract three floats from the command string
         */
        case SEND_THREE_FLOATS:
            /*
             * Your code goes here.
             */

            break;
        /*
         * Add a prefix and postfix to the string value extracted from the command string
         */
        case ECHO:

            char char_arr[MAX_MSG_SIZE];

            // Extract the next value from the command string as a character array
            success = robot_cmd.get_next_value(char_arr);
            if (!success)
                return;

            tx_estring_value.clear();
            tx_estring_value.append(char_arr);

            char src[MAX_MSG_SIZE], dest[MAX_MSG_SIZE];
            strcpy(src, tx_estring_value.c_str()); 
            strcpy(dest,"Artemis said: " );
            strcat(dest, src);
            
            tx_characteristic_string.writeValue(dest);

            Serial.print("Sent back: ");
            Serial.println(dest);
            
            break;

        /*
         * DANCE
         */
        case DANCE:
            Serial.println("Look Ma, I'm Dancin'!");

            break;
        
        /*
         * SET_VEL
         */
        case SET_VEL:

            break;

        case GET_TIME_MILLIS: {

            char time_arr[MAX_MSG_SIZE];
            strcpy(time_arr, "T:");
            char buffer[MAX_MSG_SIZE];
            sprintf(buffer, "%d", millis());
            strcat(time_arr, buffer);

            tx_estring_value.clear();
            tx_estring_value.append(time_arr);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        }

        case GET_TEMP_5s:{
            for (int instance = 0; instance < 5; instance++){

                char time_arr[MAX_MSG_SIZE];
                strcpy(time_arr, "T:");
                char buffer[MAX_MSG_SIZE];
                sprintf(buffer, "%d", millis());
                strcat(time_arr, buffer);

                tx_estring_value.clear();            
                tx_estring_value.append(time_arr);
                tx_estring_value.append("|");
                tx_estring_value.append("C:");
                tx_estring_value.append(getTempDegC());
                tx_characteristic_string.writeValue(tx_estring_value.c_str());

                delay(1000);

            }

            break;
        }

        case GET_TEMP_5s_RAPID: {
            int start_times = millis();

            while (millis() - start_times < 5000){
              
                char time_arr[MAX_MSG_SIZE];
                strcpy(time_arr, "T:");
                char buffer[MAX_MSG_SIZE];
                sprintf(buffer, "%d", millis());
                strcat(time_arr, buffer);

                tx_estring_value.clear();            
                tx_estring_value.append(time_arr);
                tx_estring_value.append("|");
                tx_estring_value.append("C:");
                tx_estring_value.append(getTempDegC());
                tx_characteristic_string.writeValue(tx_estring_value.c_str());

            }


            break;
          }

        case TEST_TOF:{
          readAndSend50TOF();
          break;
        }

        case TIMED_TOF:{
          continuous5SRead();
          break;          
        }

        case TIMED_FFT:{
          IMUAndTOF();
          break;
        }

        case TIMED_PID_DATA:{
          collectPIDDebug();
          break;
        }

        case TOF_PID:{
          TOFPID();
          break;
        }

        case UPDATE_KP:{

          float newKp;
          robot_cmd.get_next_value(newKp);
          
          updateKP(newKp);
          break;
        }



        /* 
         * The default case may not capture all types of invalid commands.
         * It is safer to validate the command string on the central device (in python)
         * before writing to the characteristic.
         */
        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
    }
}



void
setup()
{

    // startup sequence
    pinMode(LED_BUILTIN, OUTPUT);
  
    for (int blink = 0; blink < 3; blink++){
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);    
    }

    Serial.begin(115200);

    pinMode(RightPin1, OUTPUT);
    pinMode(RightPin2, OUTPUT);
    pinMode(LeftPin1, OUTPUT);
    pinMode(LeftPin2, OUTPUT);

    analogWrite(RightPin1, 0);
    analogWrite(RightPin2, 0);
    analogWrite(LeftPin1, 0);
    analogWrite(LeftPin2, 0);


    // init IMU readings
    #ifdef USE_SPI
      SPI_PORT.begin();
    #else
      WIRE_PORT.begin();
      WIRE_PORT.setClock(400000);
    #endif

    bool initialized = false;
    while (!initialized)
    {

    #ifdef USE_SPI
        myICM.begin(CS_PIN, SPI_PORT);
    #else
        myICM.begin(WIRE_PORT, AD0_VAL);
    #endif

      SERIAL_PORT.print(F("Initialization of the sensor returned: "));
      SERIAL_PORT.println(myICM.statusString());
      if (myICM.status != ICM_20948_Stat_Ok)
      {
        SERIAL_PORT.println("Trying again...");
        delay(500);
      }
      else
      {
        initialized = true;
      }
    }


    // init TOFs
    
    Wire.begin();

    pinMode(2, OUTPUT);
    digitalWrite(2, LOW);
    distanceSensor1.setI2CAddress(0x32);
    digitalWrite(2, HIGH);

    Serial.println("VL53L1X Qwiic Test");
    if (distanceSensor1.begin() != 0) //Begin returns 0 on a good init
    {
      Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
      while (1)
        ;
    }
    Serial.println("Sensor 1 online!");

   if (distanceSensor2.begin() != 0) //Begin returns 0 on a good init
    {
      Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
      while (1)
        ;
    }
    Serial.println("Sensor 2 online!");

    // analogReadResolution(RESOLUTION_BITS);    // set the resolution of analogRead results
    //                                         //  - maximum: 16 bits (padded with trailing zeroes)
    //                                         //  - ADC:     14 bits (maximum ADC resolution)
    //                                         //  - default: 10 bits (standard Arduino setting)
    //                                         //  - minimum:  1 bit

    // analogWriteResolution(RESOLUTION_BITS);   // match resolution for analogWrite

    // set TOF to short distance
    distanceSensor1.setDistanceModeShort();

    distanceSensor2.setDistanceModeShort();
    


  // setup BLE
    BLE.begin();

    // Set advertised local name and service
    BLE.setDeviceName("Artemis BLE");
    BLE.setLocalName("Artemis BLE");
    BLE.setAdvertisedService(testService);

    // Add BLE characteristics
    testService.addCharacteristic(tx_characteristic_float);
    testService.addCharacteristic(tx_characteristic_string);
    testService.addCharacteristic(rx_characteristic_string);

    // Add BLE service
    BLE.addService(testService);

    // Initial values for characteristics
    // Set initial values to prevent errors when reading for the first time on central devices
    tx_characteristic_float.writeValue(0.0);

    /*
     * An example using the EString
     */
    // Clear the contents of the EString before using it
    tx_estring_value.clear();

    // Append the string literal "[->"
    tx_estring_value.append("[->");

    // Append the float value
    tx_estring_value.append(9.0);

    // Append the string literal "<-]"
    tx_estring_value.append("<-]");

    // Write the value to the characteristic
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    // Output MAC Address
    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());

    BLE.advertise();

    
}

void
write_data()
{
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {

        tx_float_value = tx_float_value + 0.5;
        tx_characteristic_float.writeValue(tx_float_value);

        if (tx_float_value > 10000) {
            tx_float_value = 0;
            
        }

        previousMillis = currentMillis;
    }
}

void
loop()
{
    // Listen for connections
    BLEDevice central = BLE.central();

    // If a central is connected to the peripheral
    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());

      // create an array for storing a string concatenation of distance and ranging time
      // distanceSensor1.startRanging();
      // distanceSensor2.startRanging();

        // While central is connected
        while (central.connected()) {

            //Send data
            write_data();

            //Read data
            read_data();

        }

        Serial.println("Disconnected");
    }

}












// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




// #include "BLECStringCharacteristic.h"
// #include "EString.h"
// #include "RobotCommand.h"
// #include <ArduinoBLE.h>
// #include <stdlib.h>
// //#include <avr.h>

// //////////// BLE UUIDs ////////////
// #define BLE_UUID_TEST_SERVICE "5d8a3150-8994-4b11-8ea4-d06dfd5e65bd"

// #define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

// #define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
// #define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
// //////////// BLE UUIDs ////////////

// //////////// IMU //////////////////

// #include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
// #include "math.h"

// //#define USE_SPI       // Uncomment this to use SPI

// #define SERIAL_PORT Serial

// #define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
// #define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

// #define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// // The value of the last bit of the I2C address.
// // On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
// #define AD0_VAL 1

// #ifdef USE_SPI
// ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
// #else
// ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
// #endif

// //////////// IMU //////////////////


// //////////// TEMP ADC /////////////
// // #define RESOLUTION_BITS (16)      // choose resolution (explained in depth below)

// // #ifdef ADCPIN
// // #define EXTERNAL_ADC_PIN ADCPIN   // ADCPIN is the lowest analog capable pin exposed on the variant
// // #endif
// //////////// TEMP ADC /////////////


// //////////// TOF //////////////////

// #include <Wire.h>
// #include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

// //Optional interrupt and shutdown pins.
// #define SHUTDOWN_PIN 2
// //#define INTERRUPT_PIN 4

// //Uncomment the following line to use the optional shutdown and interrupt pins.
// SFEVL53L1X distanceSensor1(Wire, SHUTDOWN_PIN);

// SFEVL53L1X distanceSensor2(Wire, SHUTDOWN_PIN);

// ///////////  TOF //////////////////



// //////////// Global Variables ////////////
// BLEService testService(BLE_UUID_TEST_SERVICE);

// BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

// BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
// BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// // RX
// RobotCommand robot_cmd(":|");

// // TX
// EString tx_estring_value;
// float tx_float_value = 0.0;

// long interval = 500;
// static long previousMillis = 0;
// unsigned long currentMillis = 0;
// //////////// Global Variables ////////////

// const int LeftPin1 = 0;
// const int LeftPin2 = 15;
// const int RightPin1 = 3;
// const int RightPin2 = 16;

// ///////////// Testing SINGLE TOF ///////////////

// void
// read_data()
// {
//     // Query if the characteristic value has been written by another BLE device
//     if (rx_characteristic_string.written()) {
//         handle_command();
//     }
// }

// void sendReading(int distance, int time, int sensorN){

//   char dst_arr[MAX_MSG_SIZE];
//   sprintf(dst_arr, "%d", distance);

//   char time_arr[MAX_MSG_SIZE];
//   sprintf(time_arr, "%d", time);

//   char sensorN_arr[MAX_MSG_SIZE];
//   sprintf(sensorN_arr, "%d", sensorN);

//   tx_estring_value.clear();
//   tx_estring_value.append(dst_arr);
//   tx_estring_value.append("|");
//   tx_estring_value.append(time_arr);
//   tx_estring_value.append("|");
//   tx_estring_value.append(sensorN_arr);
//   tx_characteristic_string.writeValue(tx_estring_value.c_str());

//   Serial.print("Sent back: ");
//   Serial.println(tx_estring_value.c_str());

// }

// void newReading(){

//     int start_time = millis();
//     distanceSensor1.startRanging(); //Write configuration bytes to initiate measurement
//     while (!distanceSensor1.checkForDataReady())
//     {
//       delay(1);
//     }
//     int distance = distanceSensor1.getDistance(); //Get the result of the measurement from the sensor
//     distanceSensor1.clearInterrupt();
//     distanceSensor1.stopRanging();

//     int ranging_time = millis() - start_time;
//     Serial.print("TOF 1 Ranging Time (mS): ");
//     Serial.print(ranging_time);

//     Serial.print(" TOF 1 Distance(mm): ");
//     Serial.print(distance);

//     Serial.println();

//     sendReading(distance, ranging_time, 1);

//     start_time = millis();
//     distanceSensor2.startRanging(); //Write configuration bytes to initiate measurement
//     while (!distanceSensor2.checkForDataReady())
//     {
//       delay(1);
//     }
//     distance = distanceSensor2.getDistance(); //Get the result of the measurement from the sensor
//     distanceSensor2.clearInterrupt();
//     distanceSensor2.stopRanging();

//     ranging_time = millis() - start_time;
//     Serial.print("TOF 2 Ranging Time (mS): ");
//     Serial.print(ranging_time);

//     Serial.print(" TOF 2 Distance(mm): ");
//     Serial.print(distance);

//     Serial.println();

//     sendReading(distance, ranging_time, 2);
// }

// void readAndSend50TOF(){
//     for (int location = 0; location < 14; location++){
    
//       for (int sample = 0; sample < 50; sample++){
//         newReading();
//       }

//       Serial.println(" ");
//       Serial.println("...");
//       Serial.println("Next test in 10 seconds");
//       Serial.println("...");
//       Serial.println(" ");
//       delay(7000);
//       Serial.println("...");
//       Serial.println("Next mm test in 3 seconds");
//       Serial.println("...");
//       delay(3000);
//       Serial.println(" ");
    
//     }

//     Serial.println(" ");
//     Serial.println("Complete");
//     Serial.println(" ");
// }

// //////////////// Testing TOF /////////////////

// //////////////// Timed TOF ///////////////////

// void continuous5SRead(){
//   distanceSensor1.startRanging();
//   distanceSensor2.startRanging();
//   int start_time = millis();

//   while(millis() - start_time < 5000){
//     if (distanceSensor1.checkForDataReady())
//     {
//       int ONEdistance = distanceSensor1.getDistance(); 
//       distanceSensor1.clearInterrupt();
//       distanceSensor1.stopRanging();
//       Serial.print("At ");
//       Serial.print(millis());
//       Serial.print(" TOF ONE reads:");
//       Serial.println(ONEdistance);
//       sendReading(ONEdistance, millis(), 1);
//       distanceSensor1.startRanging();
//     }

//     if (distanceSensor2.checkForDataReady())
//     {
//       int TWOdistance = distanceSensor2.getDistance(); 
//       distanceSensor2.clearInterrupt();
//       distanceSensor2.stopRanging();
//       Serial.print("At ");
//       Serial.print(millis());
//       Serial.print(" TOF TWO reads:");
//       Serial.println(TWOdistance);
//       sendReading(TWOdistance, millis(), 2);
//       distanceSensor2.startRanging();
//     }
// }

//   }
  
// /////////////// Timed TOF ////////////////////

// void readTimedTOF(){

//   Serial.print("Current time: ");
//   Serial.println(millis());

//     if (distanceSensor1.checkForDataReady())
//     {
//       int ONEdistance = distanceSensor1.getDistance(); 
//       distanceSensor1.clearInterrupt();
//       distanceSensor1.stopRanging();
//       Serial.print("At ");
//       Serial.print(millis());
//       Serial.print(" TOF ONE reads:");
//       Serial.println(ONEdistance);
//       distanceSensor1.startRanging();
//     }

//     if (distanceSensor2.checkForDataReady())
//     {
//       int TWOdistance = distanceSensor2.getDistance(); 
//       distanceSensor2.clearInterrupt();
//       distanceSensor2.stopRanging();
//       Serial.print("At ");
//       Serial.print(millis());
//       Serial.print(" TOF TWO reads:");
//       Serial.println(TWOdistance);
//       distanceSensor2.startRanging();
//     }

// }

// ////////////////// TIMED_FFT (ACCEL and GYRO) //////////////////////////

// float getPitch(ICM_20948_I2C *sensor){
//   return atan2(sensor->accX(),sensor->accZ()) * 180/M_PI; 
// } 

// float getRoll(ICM_20948_I2C *sensor){
//   return -atan2(sensor->accY(), sensor->accZ()) * 180/M_PI;
// }

// float getPitchGyr(ICM_20948_I2C *sensor, float pitch, float dt){
//   pitch = pitch - sensor->gyrY()*dt/1000.;
//   if (pitch < -180){
//     pitch = 180 - (abs(pitch) - 180);
//   }
//   if (pitch > 180){
//     pitch = -180 + (pitch - 180);
//   }
//   return pitch;
// }

// float getRollGyr(ICM_20948_I2C *sensor, float roll, float dt){
//   roll = roll - sensor->gyrX()*dt/1000.;
//     if (roll < -180){
//     roll = 180 - (abs(roll) - 180);
//   }
//   if (roll > 180){
//     roll = -180 + (roll - 180);
//   }
//   return roll;
// }

// float getYawGyr(ICM_20948_I2C *sensor, float yaw, float dt){
//   yaw = yaw - sensor->gyrZ()*dt/1000.;
//     if (yaw < -180){
//     yaw = 180 - (abs(yaw) - 180);
//   }
//   if (yaw > 180){
//     yaw = -180 + (yaw - 180);
//   }
//   return yaw;
// }

// void sendFFTData(int time, float pitchA, float rollA, float pitchG, float rollG, float yawG){

//   // char time_arr[MAX_MSG_SIZE];
//   // sprintf(time_arr, "%d", millis());

//   tx_estring_value.clear();
//   tx_estring_value.append(time);
//   tx_estring_value.append("|");
//   tx_estring_value.append(pitchA);
//   tx_estring_value.append("|");
//   tx_estring_value.append(rollA);
//   tx_estring_value.append("|");
//   tx_estring_value.append(pitchG);
//   tx_estring_value.append("|");
//   tx_estring_value.append(rollG);
//   tx_estring_value.append("|");
//   tx_estring_value.append(yawG);
//   tx_characteristic_string.writeValue(tx_estring_value.c_str());

//   //Serial.print("Sent back: ");
//   //Serial.println(tx_estring_value.c_str());
// }

// void sendComplement(int time, float cP, float cR){
//   tx_estring_value.clear();
//   tx_estring_value.append(time);
//   tx_estring_value.append("|");
//   tx_estring_value.append(cP);
//   tx_estring_value.append("|");
//   tx_estring_value.append(cR);
//   tx_characteristic_string.writeValue(tx_estring_value.c_str());
// }

// void sendIMUVals(int time, float cP, float cR, float yaw){
//   tx_estring_value.clear();
//   tx_estring_value.append(time);
//   tx_estring_value.append("|");
//   tx_estring_value.append(cP);
//   tx_estring_value.append("|");
//   tx_estring_value.append(cR);
//   tx_estring_value.append("|");
//   tx_estring_value.append(yaw);
//   tx_characteristic_string.writeValue(tx_estring_value.c_str());  
// }

// void sendAllData(int time, float cP, float cR, float yaw, int ONEDist, int TWODist){
//   tx_estring_value.clear();
//   tx_estring_value.append(time);
//   tx_estring_value.append("|");
//   tx_estring_value.append(cP);
//   tx_estring_value.append("|");
//   tx_estring_value.append(cR);
//   tx_estring_value.append("|");
//   tx_estring_value.append(yaw);
//   tx_estring_value.append("|");
//   tx_estring_value.append(ONEDist);
//   tx_estring_value.append("|");
//   tx_estring_value.append(TWODist);
//   tx_characteristic_string.writeValue(tx_estring_value.c_str());  
  
// }  

// int ONEDistance = 0;
// int TWODistance = 0;

// void IMUAndTOF(){

//   int start_time = millis();
//   int last_time = start_time;
//   int instances = 0;

//   float curPitchG = getPitch(&myICM);
//   float curRollG = getRoll(&myICM);
//   float curYawG = 0.0;

//   float complementPitch = getPitch(&myICM);
//   float complementRoll = getRoll(&myICM);



//   while (millis() - start_time < 5000){
//      if (myICM.dataReady()){
//         myICM.getAGMT();
        
//         // Sending IMU data 
//         curPitchG = getPitchGyr(&myICM, curPitchG, millis() - last_time);
//         curRollG = getRollGyr(&myICM, curRollG, millis() - last_time);
//         curYawG = getYawGyr(&myICM,curYawG,  millis() - last_time);
//         last_time = millis();

//         //sendFFTData(millis(), getPitch(&myICM), getRoll(&myICM), curPitchG, curRollG, curYawG);

//         complementPitch = curPitchG*0.9 + getPitch(&myICM)*0.1;
//         complementRoll = curRollG*0.9 + getRoll(&myICM)*0.1;

//       }  
            
//     //sendComplement(millis(), complementPitch, complementRoll);  

//     // Sending TOF Data
//     if (distanceSensor1.checkForDataReady()){
//         ONEDistance = distanceSensor1.getDistance(); 
//         distanceSensor1.clearInterrupt();
//         distanceSensor1.stopRanging();
//         distanceSensor1.startRanging();
//     }

//     if (distanceSensor2.checkForDataReady()){
//         TWODistance = distanceSensor2.getDistance(); 
//         distanceSensor2.clearInterrupt();
//         distanceSensor2.stopRanging();
//         distanceSensor2.startRanging();
//     }

//     sendAllData(millis(), complementPitch, complementRoll, curYawG, ONEDistance, TWODistance);   

//   }
// }


// ////////////////// TIMED_FFT //////////////////////////


// enum CommandTypes
// {
//     PING,
//     SEND_TWO_INTS,
//     SEND_THREE_FLOATS,
//     ECHO,
//     DANCE,
//     SET_VEL,
//     GET_TIME_MILLIS,
//     GET_TEMP_5s,
//     GET_TEMP_5s_RAPID,
//     TEST_TOF,
//     TIMED_TOF,
//     TIMED_FFT,
// };

// void
// handle_command()
// {   
//     // Set the command string from the characteristic value
//     robot_cmd.set_cmd_string(rx_characteristic_string.value(),
//                              rx_characteristic_string.valueLength());

//     bool success;
//     int cmd_type = -1;

//     // Get robot command type (an integer)
//     /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
//      * since it uses strtok internally (refer RobotCommand.h and 
//      * https://www.cplusplus.com/reference/cstring/strtok/)
//      */
//     success = robot_cmd.get_command_type(cmd_type);

//     // Check if the last tokenization was successful and return if failed
//     if (!success) {
//         return;
//     }

//     // Handle the command type accordingly
//     switch (cmd_type) {
//         /*
//          * Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
//          */
//         case PING:
//             tx_estring_value.clear();
//             tx_estring_value.append("PONG");
//             tx_characteristic_string.writeValue(tx_estring_value.c_str());

//             Serial.print("Sent back: ");
//             Serial.println(tx_estring_value.c_str());

//             break;
//         /*
//          * Extract two integers from the command string
//          */
//         case SEND_TWO_INTS:
//             int int_a, int_b;

//             // Extract the next value from the command string as an integer
//             success = robot_cmd.get_next_value(int_a);
//             if (!success)
//                 return;

//             // Extract the next value from the command string as an integer
//             success = robot_cmd.get_next_value(int_b);
//             if (!success)
//                 return;

//             Serial.print("Two Integers: ");
//             Serial.print(int_a);
//             Serial.print(", ");
//             Serial.println(int_b);
            
//             break;
//         /*
//          * Extract three floats from the command string
//          */
//         case SEND_THREE_FLOATS:
//             /*
//              * Your code goes here.
//              */

//             break;
//         /*
//          * Add a prefix and postfix to the string value extracted from the command string
//          */
//         case ECHO:

//             char char_arr[MAX_MSG_SIZE];

//             // Extract the next value from the command string as a character array
//             success = robot_cmd.get_next_value(char_arr);
//             if (!success)
//                 return;

//             tx_estring_value.clear();
//             tx_estring_value.append(char_arr);

//             char src[MAX_MSG_SIZE], dest[MAX_MSG_SIZE];
//             strcpy(src, tx_estring_value.c_str()); 
//             strcpy(dest,"Artemis said: " );
//             strcat(dest, src);
            
//             tx_characteristic_string.writeValue(dest);

//             Serial.print("Sent back: ");
//             Serial.println(dest);
            
//             break;

//         /*
//          * DANCE
//          */
//         case DANCE:
//             Serial.println("Look Ma, I'm Dancin'!");

//             break;
        
//         /*
//          * SET_VEL
//          */
//         case SET_VEL:

//             break;

//         case GET_TIME_MILLIS: {

//             char time_arr[MAX_MSG_SIZE];
//             strcpy(time_arr, "T:");
//             char buffer[MAX_MSG_SIZE];
//             sprintf(buffer, "%d", millis());
//             strcat(time_arr, buffer);

//             tx_estring_value.clear();
//             tx_estring_value.append(time_arr);
//             tx_characteristic_string.writeValue(tx_estring_value.c_str());

//             Serial.print("Sent back: ");
//             Serial.println(tx_estring_value.c_str());

//             break;
//         }

//         case GET_TEMP_5s:{
//             for (int instance = 0; instance < 5; instance++){

//                 char time_arr[MAX_MSG_SIZE];
//                 strcpy(time_arr, "T:");
//                 char buffer[MAX_MSG_SIZE];
//                 sprintf(buffer, "%d", millis());
//                 strcat(time_arr, buffer);

//                 tx_estring_value.clear();            
//                 tx_estring_value.append(time_arr);
//                 tx_estring_value.append("|");
//                 tx_estring_value.append("C:");
//                 tx_estring_value.append(getTempDegC());
//                 tx_characteristic_string.writeValue(tx_estring_value.c_str());

//                 delay(1000);

//             }

//             break;
//         }

//         case GET_TEMP_5s_RAPID: {
//             int start_times = millis();

//             while (millis() - start_times < 5000){
              
//                 char time_arr[MAX_MSG_SIZE];
//                 strcpy(time_arr, "T:");
//                 char buffer[MAX_MSG_SIZE];
//                 sprintf(buffer, "%d", millis());
//                 strcat(time_arr, buffer);

//                 tx_estring_value.clear();            
//                 tx_estring_value.append(time_arr);
//                 tx_estring_value.append("|");
//                 tx_estring_value.append("C:");
//                 tx_estring_value.append(getTempDegC());
//                 tx_characteristic_string.writeValue(tx_estring_value.c_str());

//             }


//             break;
//           }

//         case TEST_TOF:{
//           readAndSend50TOF();
//           break;
//         }

//         case TIMED_TOF:{
//           continuous5SRead();
//           break;          
//         }

//         case TIMED_FFT:{
//           IMUAndTOF();
//           break;
//         }


//         /* 
//          * The default case may not capture all types of invalid commands.
//          * It is safer to validate the command string on the central device (in python)
//          * before writing to the characteristic.
//          */
//         default:
//             Serial.print("Invalid Command Type: ");
//             Serial.println(cmd_type);
//             break;
//     }
// }

// void driveForward(){

//   analogWrite(LeftPin1, 100);
//   analogWrite(RightPin1, 100);

//   delay(1000);

//   analogWrite(LeftPin1, 0);
//   analogWrite(RightPin1, 0);
// }

// void driveBackward(){

//   analogWrite(LeftPin2, 100);
//   analogWrite(RightPin2, 100);

//   delay(1000);

//   analogWrite(LeftPin2, 0);
//   analogWrite(RightPin2, 0);
// }

// void
// setup()
// {

//     // startup sequence
//     pinMode(LED_BUILTIN, OUTPUT);
  
//     for (int blink = 0; blink < 3; blink++){
//       digitalWrite(LED_BUILTIN, HIGH);
//       delay(500);
//       digitalWrite(LED_BUILTIN, LOW);
//       delay(500);    
//     }

//     Serial.begin(115200);

//     pinMode(RightPin1, OUTPUT);
//     pinMode(RightPin2, OUTPUT);
//     pinMode(LeftPin1, OUTPUT);
//     pinMode(LeftPin2, OUTPUT);

//     analogWrite(RightPin1, 1000);
//     analogWrite(RightPin2, 0);
//     analogWrite(LeftPin1, 100);
//     analogWrite(LeftPin2, 0);


//     // init IMU readings
//     #ifdef USE_SPI
//       SPI_PORT.begin();
//     #else
//       WIRE_PORT.begin();
//       WIRE_PORT.setClock(400000);
//     #endif

//     bool initialized = false;
//     while (!initialized)
//     {

//     #ifdef USE_SPI
//         myICM.begin(CS_PIN, SPI_PORT);
//     #else
//         myICM.begin(WIRE_PORT, AD0_VAL);
//     #endif

//       SERIAL_PORT.print(F("Initialization of the sensor returned: "));
//       SERIAL_PORT.println(myICM.statusString());
//       if (myICM.status != ICM_20948_Stat_Ok)
//       {
//         SERIAL_PORT.println("Trying again...");
//         delay(500);
//       }
//       else
//       {
//         initialized = true;
//       }
//     }


//     // init TOFs
    
//     Wire.begin();

//     pinMode(2, OUTPUT);
//     digitalWrite(2, LOW);
//     distanceSensor1.setI2CAddress(0x32);
//     digitalWrite(2, HIGH);

//     Serial.println("VL53L1X Qwiic Test");
//     if (distanceSensor1.begin() != 0) //Begin returns 0 on a good init
//     {
//       Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
//       while (1)
//         ;
//     }
//     Serial.println("Sensor 1 online!");

//    if (distanceSensor2.begin() != 0) //Begin returns 0 on a good init
//     {
//       Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
//       while (1)
//         ;
//     }
//     Serial.println("Sensor 2 online!");

//     // analogReadResolution(10);    // set the resolution of analogRead results
//     //                                         //  - maximum: 16 bits (padded with trailing zeroes)
//     //                                         //  - ADC:     14 bits (maximum ADC resolution)
//     //                                         //  - default: 10 bits (standard Arduino setting)
//     //                                         //  - minimum:  1 bit

//     // analogWriteResolution(10);   // match resolution for analogWrite

//     // set TOF to short distance
//     distanceSensor1.setDistanceModeShort();

//     distanceSensor2.setDistanceModeShort();
    


//   // setup BLE
//     BLE.begin();

//     // Set advertised local name and service
//     BLE.setDeviceName("Artemis BLE");
//     BLE.setLocalName("Artemis BLE");
//     BLE.setAdvertisedService(testService);

//     // Add BLE characteristics
//     testService.addCharacteristic(tx_characteristic_float);
//     testService.addCharacteristic(tx_characteristic_string);
//     testService.addCharacteristic(rx_characteristic_string);

//     // Add BLE service
//     BLE.addService(testService);

//     // Initial values for characteristics
//     // Set initial values to prevent errors when reading for the first time on central devices
//     tx_characteristic_float.writeValue(0.0);

//     /*
//      * An example using the EString
//      */
//     // Clear the contents of the EString before using it
//     tx_estring_value.clear();

//     // Append the string literal "[->"
//     tx_estring_value.append("[->");

//     // Append the float value
//     tx_estring_value.append(9.0);

//     // Append the string literal "<-]"
//     tx_estring_value.append("<-]");

//     // Write the value to the characteristic
//     tx_characteristic_string.writeValue(tx_estring_value.c_str());

//     driveForward();

//     driveBackward();

//     // Output MAC Address
//     Serial.print("Advertising BLE with MAC: ");
//     Serial.println(BLE.address());

//     BLE.advertise();

    
// }

// void
// write_data()
// {
//     currentMillis = millis();
//     if (currentMillis - previousMillis > interval) {

//         tx_float_value = tx_float_value + 0.5;
//         tx_characteristic_float.writeValue(tx_float_value);

//         if (tx_float_value > 10000) {
//             tx_float_value = 0;
            
//         }

//         previousMillis = currentMillis;
//     }
// }

// void
// loop()
// {
//     // Listen for connections
//     BLEDevice central = BLE.central();

//     // If a central is connected to the peripheral
//     if (central) {
//         Serial.print("Connected to: ");
//         Serial.println(central.address());

//       //delay(2000);

//       // create an array for storing a string concatenation of distance and ranging time
//       distanceSensor1.startRanging();
//       distanceSensor2.startRanging();

//         // While central is connected
//         while (central.connected()) {

//           //continuous5SRead();
//           // distanceSensor1.startRanging(); 
//           // distanceSensor2.startRanging(); 
//             //readAndSendTimed();

//             //Send data
//             write_data();

//             //Read data
//             read_data();

//             //readTimedTOF();
  
//             // read 50 TOF values
//             //readAndSend50TOF();

//               // Serial.println(millis() + " is the current time");
//               // if (ONEdistance != -1 and TWOdistance != -1){
//               //   Serial.println(ONEdistance + " is TOF ONE distance (mm)");
//               //   Serial.println(TWOdistance + " is TOF TWO distance (mm)");
//               //   ONEdistance = -1;
//               //   TWOdistance = -1;
//               //   distanceSensor1.startRanging();
//               //   distanceSensor2.startRanging();
//               // }
              
//               // if (distanceSensor1.checkForDataReady())
//               // {
//               //   ONEdistance = distanceSensor1.getDistance(); 
//               //   distanceSensor1.clearInterrupt();
//               //   distanceSensor1.stopRanging();
//               // }

//               // if (distanceSensor2.checkForDataReady())
//               // {
//               //   TWOdistance = distanceSensor2.getDistance(); 
//               //   distanceSensor2.clearInterrupt();
//               //   distanceSensor2.stopRanging();
//               // }

//         }

//         Serial.println("Disconnected");
//     }

// }







// // #include "BLECStringCharacteristic.h"
// // #include "EString.h"
// // #include "RobotCommand.h"
// // #include <ArduinoBLE.h>

// // //////////// BLE UUIDs ////////////
// // #define BLE_UUID_TEST_SERVICE "5d8a3150-8994-4b11-8ea4-d06dfd5e65bd"

// // #define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

// // #define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
// // #define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
// // //////////// BLE UUIDs ////////////

// // //////////// Global Variables ////////////
// // BLEService testService(BLE_UUID_TEST_SERVICE);

// // BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

// // BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
// // BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// // // RX
// // RobotCommand robot_cmd(":|");

// // // TX
// // EString tx_estring_value;
// // float tx_float_value = 0.0;

// // long interval = 500;
// // static long previousMillis = 0;
// // unsigned long currentMillis = 0;
// // //////////// Global Variables ////////////

// // enum CommandTypes
// // {
// //     PING,
// //     SEND_TWO_INTS,
// //     SEND_THREE_FLOATS,
// //     ECHO,
// //     DANCE,
// //     SET_VEL,
// // };

// // void
// // handle_command()
// // {   
// //     // Set the command string from the characteristic value
// //     robot_cmd.set_cmd_string(rx_characteristic_string.value(),
// //                              rx_characteristic_string.valueLength());

// //     bool success;
// //     int cmd_type = -1;

// //     // Get robot command type (an integer)
// //     /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
// //      * since it uses strtok internally (refer RobotCommand.h and 
// //      * https://www.cplusplus.com/reference/cstring/strtok/)
// //      */
// //     success = robot_cmd.get_command_type(cmd_type);

// //     // Check if the last tokenization was successful and return if failed
// //     if (!success) {
// //         return;
// //     }

// //     // Handle the command type accordingly
// //     switch (cmd_type) {
// //         /*
// //          * Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
// //          */
// //         case PING:
// //             tx_estring_value.clear();
// //             tx_estring_value.append("PONG");
// //             tx_characteristic_string.writeValue(tx_estring_value.c_str());

// //             Serial.print("Sent back: ");
// //             Serial.println(tx_estring_value.c_str());

// //             break;
// //         /*
// //          * Extract two integers from the command string
// //          */
// //         case SEND_TWO_INTS:
// //             int int_a, int_b;

// //             // Extract the next value from the command string as an integer
// //             success = robot_cmd.get_next_value(int_a);
// //             if (!success)
// //                 return;

// //             // Extract the next value from the command string as an integer
// //             success = robot_cmd.get_next_value(int_b);
// //             if (!success)
// //                 return;

// //             Serial.print("Two Integers: ");
// //             Serial.print(int_a);
// //             Serial.print(", ");
// //             Serial.println(int_b);
            
// //             break;
// //         /*
// //          * Extract three floats from the command string
// //          */
// //         case SEND_THREE_FLOATS:
// //             /*
// //              * Your code goes here.
// //              */

// //             break;
// //         /*
// //          * Add a prefix and postfix to the string value extracted from the command string
// //          */
// //         case ECHO:

// //             char char_arr[MAX_MSG_SIZE];

// //             // Extract the next value from the command string as a character array
// //             success = robot_cmd.get_next_value(char_arr);
// //             if (!success)
// //                 return;

// //             /*
// //              * Your code goes here.
// //              */
            
// //             break;
// //         /*
// //          * DANCE
// //          */
// //         case DANCE:
// //             Serial.println("Look Ma, I'm Dancin'!");

// //             break;
        
// //         /*
// //          * SET_VEL
// //          */
// //         case SET_VEL:

// //             break;
        
// //         /* 
// //          * The default case may not capture all types of invalid commands.
// //          * It is safer to validate the command string on the central device (in python)
// //          * before writing to the characteristic.
// //          */
// //         default:
// //             Serial.print("Invalid Command Type: ");
// //             Serial.println(cmd_type);
// //             break;
// //     }
// // }

// // void
// // setup()
// // {
// //     Serial.begin(115200);

// //     BLE.begin();

// //     // Set advertised local name and service
// //     BLE.setDeviceName("Artemis BLE");
// //     BLE.setLocalName("Artemis BLE");
// //     BLE.setAdvertisedService(testService);

// //     // Add BLE characteristics
// //     testService.addCharacteristic(tx_characteristic_float);
// //     testService.addCharacteristic(tx_characteristic_string);
// //     testService.addCharacteristic(rx_characteristic_string);

// //     // Add BLE service
// //     BLE.addService(testService);

// //     // Initial values for characteristics
// //     // Set initial values to prevent errors when reading for the first time on central devices
// //     tx_characteristic_float.writeValue(0.0);

// //     /*
// //      * An example using the EString
// //      */
// //     // Clear the contents of the EString before using it
// //     tx_estring_value.clear();

// //     // Append the string literal "[->"
// //     tx_estring_value.append("[->");

// //     // Append the float value
// //     tx_estring_value.append(9.0);

// //     // Append the string literal "<-]"
// //     tx_estring_value.append("<-]");

// //     // Write the value to the characteristic
// //     tx_characteristic_string.writeValue(tx_estring_value.c_str());

// //     // Output MAC Address
// //     Serial.print("Advertising BLE with MAC: ");
// //     Serial.println(BLE.address());

// //     BLE.advertise();
// // }

// // void
// // write_data()
// // {
// //     currentMillis = millis();
// //     if (currentMillis - previousMillis > interval) {

// //         tx_float_value = tx_float_value + 0.5;
// //         tx_characteristic_float.writeValue(tx_float_value);

// //         if (tx_float_value > 10000) {
// //             tx_float_value = 0;
            
// //         }

// //         previousMillis = currentMillis;
// //     }
// // }

// // void
// // read_data()
// // {
// //     // Query if the characteristic value has been written by another BLE device
// //     if (rx_characteristic_string.written()) {
// //         handle_command();
// //     }
// // }

// // void
// // loop()
// // {
// //     // Listen for connections
// //     BLEDevice central = BLE.central();

// //     // If a central is connected to the peripheral
// //     if (central) {
// //         Serial.print("Connected to: ");
// //         Serial.println(central.address());

// //         // While central is connected
// //         while (central.connected()) {
// //             // Send data
// //             write_data();

// //             // Read data
// //             read_data();
// //         }

// //         Serial.println("Disconnected");
// //     }
// // }

