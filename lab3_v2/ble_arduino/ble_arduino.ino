
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "5d8a3150-8994-4b11-8ea4-d06dfd5e65bd"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////


//////////// TEMP ADC /////////////
#define RESOLUTION_BITS (16)      // choose resolution (explained in depth below)

#ifdef ADCPIN
#define EXTERNAL_ADC_PIN ADCPIN   // ADCPIN is the lowest analog capable pin exposed on the variant
#endif
//////////// TEMP ADC /////////////


//////////// TOF //////////////////

#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensor;
//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

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
//////////// Global Variables ////////////



///////////// Testing TOF ///////////////

void
read_data()
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}

void sendReading(int distance, int time){

  char dst_arr[MAX_MSG_SIZE];
  sprintf(dst_arr, "%d", distance);

  char time_arr[MAX_MSG_SIZE];
  sprintf(time_arr, "%d", time);

  tx_estring_value.clear();
  tx_estring_value.append(dst_arr);
  tx_estring_value.append("|");
  tx_estring_value.append(time_arr);
  tx_characteristic_string.writeValue(tx_estring_value.c_str());

  Serial.print("Sent back: ");
  Serial.println(tx_estring_value.c_str());

}

void newReading(){

    int start_time = millis();
    distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
    while (!distanceSensor.checkForDataReady())
    {
      delay(1);
    }
    int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
    distanceSensor.clearInterrupt();
    distanceSensor.stopRanging();

    int ranging_time = millis() - start_time;
    Serial.print("Ranging Time (mS): ");
    Serial.print(ranging_time);

    Serial.print(" Distance(mm): ");
    Serial.print(distance);

    Serial.println();

    sendReading(distance, ranging_time);
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
    TEST_TOF
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
    Wire.begin();

    Serial.begin(115200);
    Serial.println("VL53L1X Qwiic Test");

    if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
    {
      Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
      while (1)
        ;
    }
    Serial.println("Sensor online!");

    analogReadResolution(RESOLUTION_BITS);    // set the resolution of analogRead results
                                            //  - maximum: 16 bits (padded with trailing zeroes)
                                            //  - ADC:     14 bits (maximum ADC resolution)
                                            //  - default: 10 bits (standard Arduino setting)
                                            //  - minimum:  1 bit

    analogWriteResolution(RESOLUTION_BITS);   // match resolution for analogWrite

    // set TOF to short distance
    distanceSensor.setDistanceModeShort();

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

    //delay(2000);

    // create an array for storing a string concatenation of distance and ranging time
     

        // While central is connected
        while (central.connected()) {
            //Send data
            write_data();

            //Read data
            read_data();

            // read 50 TOF values
            //readAndSend50TOF();
        }

        Serial.println("Disconnected");
    }

}







// #include "BLECStringCharacteristic.h"
// #include "EString.h"
// #include "RobotCommand.h"
// #include <ArduinoBLE.h>

// //////////// BLE UUIDs ////////////
// #define BLE_UUID_TEST_SERVICE "5d8a3150-8994-4b11-8ea4-d06dfd5e65bd"

// #define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

// #define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
// #define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
// //////////// BLE UUIDs ////////////

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

// enum CommandTypes
// {
//     PING,
//     SEND_TWO_INTS,
//     SEND_THREE_FLOATS,
//     ECHO,
//     DANCE,
//     SET_VEL,
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

//             /*
//              * Your code goes here.
//              */
            
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

// void
// setup()
// {
//     Serial.begin(115200);

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
// read_data()
// {
//     // Query if the characteristic value has been written by another BLE device
//     if (rx_characteristic_string.written()) {
//         handle_command();
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

//         // While central is connected
//         while (central.connected()) {
//             // Send data
//             write_data();

//             // Read data
//             read_data();
//         }

//         Serial.println("Disconnected");
//     }
// }
