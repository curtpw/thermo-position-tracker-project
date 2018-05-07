

/********************************************************************************************************/
/************************ INCLUDES **********************************************************************/
/********************************************************************************************************/
#define NRF52

#include <SPI.h>
#include <BLEPeripheral.h>    //bluetooth
#include <BLEUtil.h>
#include <Wire.h>
#include "KX126_SPI.h"        //accelerometer
#include <VL53L0X.h>          //lidar distance sensor
#include <APDS9960.h>         //4X IR photodiode gesture sensor


/********************************************************************************************************/
/************************ CONSTANTS / SYSTEM VAR ********************************************************/
/********************************************************************************************************/
bool    debug = true;                 //turn serial on/off to get data or turn up sample rate
bool    debug_time = true;           //turn loop component time debug on/off

//start on sleep mode for battery charging
bool    SLEEP_MODE = true;

//native max loop speed is about 35 ms or 28Hz
float   speedLowpower  = 1000 / 6;    //2Hz default power saving speed
float   speedBluetooth = 1000 / 16;   //16Hz while connected to 
float   speedBallpark  = 1000 / 8;    //8Hz when NN approach target

float   speedMs = speedBluetooth;

float   detect_objT_lowpass =    80;
float   detect_objT_highpass =   102;

/********************************************************************************************************/
/************************ DEFINITIONS *******************************************************************/
/********************************************************************************************************/
//SCL = 12  SDA = 11   RX = 14   TX = 13

#define GREEN_LED_PIN               16

//share with RX
#define BUTTON_PIN                  14 

//#define HEART_RATE_LED_PIN          4
//#define HEART_RATE_DETECTOR_PIN   29
//#define TOUCH_BUTTON_PIN          30
#define VIBRATE_PIN                 8

#define BATTERY_PIN                 28
// OR P5? #define BATTERY_PIN       5

//Lidar Pins
//#define VL53L0X_SELECT1           30
#define VL53L0X_SELECT2             29
#define VL53L0X_SELECT3             15

//IR LED gesture sensor
#define APDS9960_INT                4 

//Accelerometer Pins
#define CS_PIN                      24

#define KX022_SDI 19
#define KX022_SDO 20
#define KX022_SCL 18
#define KX022_INT 23

#define PIN_SPI_MISO         (KX022_SDO)
#define PIN_SPI_MOSI         (KX022_SDI)
#define PIN_SPI_SCK          (KX022_SCL)


//Thermopile Addresses
#define MLX90615_I2CADDR          0x00
#define MLX90615_I2CADDR1         0x2A
#define MLX90615_I2CADDR2         0x2B 
#define MLX90615_I2CADDR3         0x2C
#define MLX90615_I2CADDR4         0x2D
#define MLX90615_I2CADDR5         0x2E
#define MLX90615_I2CADDR6         0x3B     //0x2F not working, 30 & 31 taken by lidar
#define MLX90615_I2CADDR7         0x32
#define MLX90615_I2CADDR8         0x33
#define MLX90615_I2CADDR9         0x34
#define MLX90615_I2CADDR10        0x35
#define MLX90615_I2CADDR11        0x36
#define MLX90615_I2CADDR12        0x37
#define MLX90615_I2CADDR13        0x38
#define MLX90615_I2CADDR14        0x3A
#define MLX90615_I2CADDR15        0x3C

#define MLX90615_I2CADDR16        0x3D
#define MLX90615_I2CADDR17        0x3E
#define MLX90615_I2CADDR18        0x40
#define MLX90615_I2CADDR19        0x41
#define MLX90615_I2CADDR20        0x42
#define MLX90615_I2CADDR21        0x43
#define MLX90615_I2CADDR22        0x44


// RAM
#define MLX90615_RAWIR1           0x04
#define MLX90615_RAWIR2           0x05
#define MLX90615_TA               0x26
#define MLX90615_TOBJ1            0x27
#define MLX90615_TOBJ2            0x28
// EEPROM
#define MLX90615_TOMAX            0x20
#define MLX90615_TOMIN            0x21
#define MLX90615_PWMCTRL          0x22
#define MLX90615_TARANGE          0x23
#define MLX90615_EMISS            0x24
#define MLX90615_CONFIG           0x25
#define MLX90615_ADDR             0x0E

//dummy LED pin for BLE
#define LED_PIN   3


/********************************************************************************************************/
/************************ VARIABLES *********************************************************************/
/********************************************************************************************************/

  //LED
    float   greenLED_timer = 0;
    int     LED_counter = 10;
    bool    greenLED_status = false;

  //Battery MGMT  
    int     batteryValue = 100;
    float   buttonBeginPressTime = 0;
    
  //Button
    int     buttonState = 0;         // variable for reading the pushbutton

  //Detection
    int     vibrate_counter = 0;
    bool    vibrate_status = false;


  //Timestamp
    float   clocktime = 0;
    
  //Bluetooth
    unsigned long microsPerReading, microsPrevious;
    float   accelScal;
    int     received_command = 99; //controlls how device and app talk to each other

  //Bondstore flash storage
    const unsigned char*  bondStoreWriteData = 0;       //reference var for bondstore flash storage
    unsigned char*        bondStoreReadData = 0;        //reference var for bondstore flash storage
    unsigned int          bondStoreOffset = 1;          //default memory location for nn target configuration flash storage
    unsigned int          bondStoreLength = 1;          //default data length for nn target configuration flash storage


  //System
    int     varState =    0; //variable state controlled in app and appended to data stream
    bool sleepLight =     false;
    bool sleepDeep =      false;

  //MLX90615 Thermopiles
    float   TObj[22] =    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    float   TAmb[22] =    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    float   TAmbAv;

  //vl53l0x Distance
    float   distance[3] = {0,0,0};

 
  //KX126 Accelerometer
    // pins used for the connection with the sensor
    // the other you need are controlled by the SPI library):
    const int dataReadyPin =  6;
    const int chipSelectPin = 7;

    float     acc[3];
    double    pitch;
    double    roll;


/********************************************************************************************************/
/************************ DECLARATIONS ******************************************************************/
/********************************************************************************************************/

//Time of Flight LIDAR distance sensors
VL53L0X lidar1;
VL53L0X lidar2;
VL53L0X lidar3;

//KX022 Accelerometer
KX126_SPI kx126(CS_PIN);

//IR LED gesture sensor APDS9960
APDS9960 apds = APDS9960();
int isr_flag = 0;

//Bluetooth
// create peripheral instance, see pinouts above
BLEPeripheral blePeripheral = BLEPeripheral();

// create service
BLEService customService =    BLEService("a000");

// create command i/o characteristics
BLECharCharacteristic    ReadOnlyArrayGattCharacteristic  = BLECharCharacteristic("a001", BLERead);
BLECharCharacteristic    WriteOnlyArrayGattCharacteristic = BLECharCharacteristic("a002", BLEWrite);

//create streaming data characteristic
BLECharacteristic        DataCharacteristic("a003", BLERead | BLENotify, 20);  //@param data - an Uint8Array.

//Permanent storage for neural network target configuration
//BLEBondStore bleBondStore;


/********************************************************************************************************/
/************************ UTILITY FUNCTIONS *************************************************/
/********************************************************************************************************/
float differenceBetweenAngles(float firstAngle, float secondAngle)
  {
        double difference = secondAngle - firstAngle;
        while (difference < -180) difference += 360;
        while (difference > 180) difference -= 360;
        return difference;
 }


/********************************************************************************************************/
/************************ MLX90615 THERMOPILE FUNCTIONS *************************************************/
/********************************************************************************************************/
uint16_t read16(uint8_t a, int sensorNum) {
  uint8_t _addr = MLX90615_I2CADDR;

  if(sensorNum == 0) _addr = MLX90615_I2CADDR;   //custom addresses
  else if(sensorNum == 1)  _addr = MLX90615_I2CADDR1;   
  else if(sensorNum == 2)  _addr = MLX90615_I2CADDR2;
  else if(sensorNum == 3)  _addr = MLX90615_I2CADDR3;
  else if(sensorNum == 4)  _addr = MLX90615_I2CADDR4;
  else if(sensorNum == 5)  _addr = MLX90615_I2CADDR5;
  else if(sensorNum == 6)  _addr = MLX90615_I2CADDR6;
  else if(sensorNum == 7)  _addr = MLX90615_I2CADDR7;
  else if(sensorNum == 8)  _addr = MLX90615_I2CADDR8;
  else if(sensorNum == 9)  _addr = MLX90615_I2CADDR9;
  else if(sensorNum == 10) _addr = MLX90615_I2CADDR10;
  else if(sensorNum == 11) _addr = MLX90615_I2CADDR11;
  else if(sensorNum == 12) _addr = MLX90615_I2CADDR12;
  else if(sensorNum == 13) _addr = MLX90615_I2CADDR13;
  else if(sensorNum == 14) _addr = MLX90615_I2CADDR14;
  else if(sensorNum == 15) _addr = MLX90615_I2CADDR15;

  else if(sensorNum == 16) _addr = MLX90615_I2CADDR16;
  else if(sensorNum == 17) _addr = MLX90615_I2CADDR17;
  else if(sensorNum == 18) _addr = MLX90615_I2CADDR18;
  else if(sensorNum == 19) _addr = MLX90615_I2CADDR19;
  else if(sensorNum == 20) _addr = MLX90615_I2CADDR20;
  else if(sensorNum == 21) _addr = MLX90615_I2CADDR21;
  else if(sensorNum == 22) _addr = MLX90615_I2CADDR22;

  
  uint16_t ret;
  Wire.beginTransmission(_addr);                  // start transmission to device 
  Wire.write(a); delayMicroseconds(100);                        // sends register address to read from
  Wire.endTransmission(false);                    // end transmission
  Wire.requestFrom(_addr, (uint8_t)3); delayMicroseconds(100);  // send data n-bytes read
  ret = Wire.read();// delay(1);                    // receive DATA
  ret |= Wire.read() << 8;// delay(1);              // receive DATA
  uint8_t pec = Wire.read(); delayMicroseconds(100);
  return ret;
}

float readTemp(uint8_t reg, int sensorNum) {
  float temp;
  temp = read16(reg, sensorNum);
  temp *= .02;
  temp  -= 273.15;
  return temp;
}

double readObjectTempF(int sensorNum) {
  return (readTemp(MLX90615_TOBJ1, sensorNum) * 9 / 5) + 32;
}

double readAmbientTempF(int sensorNum) {
  return (readTemp(MLX90615_TA, sensorNum) * 9 / 5) + 32;
}

double readObjectTempC(int sensorNum) {
  return readTemp(MLX90615_TOBJ1, sensorNum);
}

double readAmbientTempC(int sensorNum) {
  return readTemp(MLX90615_TA, sensorNum);
}


/********************************************************************************************************/
/************************ BLUETOOTH BLE FUNCTIONS *************************************************/
/********************************************************************************************************/
void blePeripheralConnectHandler(BLECentral& central) {
  // central connected event handler

/*  if(debug){
    Serial.print(F("Connected event, central: "));
    Serial.println(central.address());
  } */
  delay(5);
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  // central disconnected event handler
  
 /* if(debug){
    Serial.print(F("Disconnected event, central: "));
    Serial.println(central.address());
  } */
  delay(5);
}

void blePeripheralServicesDiscoveredHandler(BLECentral& central) {
  // central  services discovered event handler
/*  if(debug){
    Serial.print(F(" services discovered event, central: "));
    Serial.println(central.address());
  } */

  delay(5);
}

void bleCharacteristicValueUpdatedHandle(BLECentral& central, BLECharacteristic& characteristic) {
  
  const unsigned char* the_buffer = characteristic.value();
  unsigned char the_length = characteristic.valueLength();
  
  String bleRawVal = "";
  for (byte i = 0; i < the_length; i++){ 
    bleRawVal += String(the_buffer[i], HEX); 
  }

  char *char_buf = const_cast<char*>(bleRawVal.c_str());
  
  received_command = (int)strtol(char_buf, NULL, 16);
  if(debug){ Serial.print("RECEIVED COMMAND: "); Serial.println( received_command ); }

  if(debug){ BLEUtil::printBuffer(characteristic.value(), characteristic.valueLength()); }

  delay(5);
}

/*
void switchCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  // central wrote new value to characteristic, update command val
  Serial.print(F("Characteristic event, written: "));

  if (ReadOnlyArrayGattCharacteristic.value()) {
    if(debug) Serial.println(F("Command on"));
  } else {
    if(debug) Serial.println(F("Command off"));
  }
}
*/

void setupBluetooth(){
  /************ INIT BLUETOOTH BLE instantiate BLE peripheral *********/
   // set advertised local name and service UUID
    blePeripheral.setLocalName("ThermoV2");
    blePeripheral.setDeviceName("ThermoV2");
    blePeripheral.setAdvertisedServiceUuid(customService.uuid());
    blePeripheral.setAppearance(0xFFFF);
  
    // add attributes (services, characteristics, descriptors) to peripheral
    blePeripheral.addAttribute(customService);
    
    blePeripheral.addAttribute(ReadOnlyArrayGattCharacteristic);
    blePeripheral.addAttribute(WriteOnlyArrayGattCharacteristic);
    
    blePeripheral.addAttribute(DataCharacteristic); //streaming data for app graph
    
    // assign event handlers for connected, disconnected to peripheral
    blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
    blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

    // assign event handlers for characteristic
    ReadOnlyArrayGattCharacteristic.setEventHandler(BLEWritten /*BLEValueUpdated*/, bleCharacteristicValueUpdatedHandle);
    WriteOnlyArrayGattCharacteristic.setEventHandler(BLEWritten /*BLEValueUpdated*/, bleCharacteristicValueUpdatedHandle);

    // assign initial values
    char readValue[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    ReadOnlyArrayGattCharacteristic.setValue(0);
    char writeValue[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    WriteOnlyArrayGattCharacteristic.setValue(0);

    // begin initialization
    blePeripheral.begin();
  
    if(debug) Serial.println("BLE MOBILE APP PERIPHERAL");
}

/********************************************************************************************************/
/************************ SETUP *************************************************************************/
/********************************************************************************************************/

void setup() 
{
    /************ SENSOR I/O, BUTTON, LED, HAPTIC FEEDBACK *********************/
    // Configure lidar device selection pins (XSHUT)
    pinMode(VL53L0X_SELECT2, OUTPUT); 
    pinMode(VL53L0X_SELECT3, OUTPUT); 

    // Set APDS9960 interrupt pin as input
    pinMode(APDS9960_INT, INPUT);

    // Configure button pin
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Configure LED pin
    pinMode(GREEN_LED_PIN, OUTPUT);
    digitalWrite(GREEN_LED_PIN, 1);  // turn LED OFF

    // Configure HR LED pin DISABLE
  //  pinMode(HEART_RATE_LED_PIN, OUTPUT);
 //   digitalWrite(HEART_RATE_LED_PIN, 1);  // turn LED OFF

    // Configure haptic feedback pin
    pinMode(VIBRATE_PIN, OUTPUT);  
    digitalWrite(VIBRATE_PIN, 0);

     
    Serial.begin(115200);
    if(debug) Serial.print("STARTING\t");
    delay(50);

    // start the I2C library:
    Wire.begin();
 //   Wire.setClock(250000); //increase from default 100KHz
    delay(50);

    /************ CONFIGURE & START BLUETOOTH *********************/
    setupBluetooth();

  /************ INIT VL53L0X DISTANCE SENSOR *****************************/
    //Reset lidar devices
    digitalWrite(VL53L0X_SELECT2, 0); digitalWrite(VL53L0X_SELECT3, 0);    
    delay(100);
 
    Serial.println("VL53L0X LIDAR1 INIT");
    lidar1.init();
    lidar1.setAddress(0x45);
    lidar1.setTimeout(500);

    Serial.println("VL53L0X LIDAR2 INIT");
    //wake up lidar #2
    digitalWrite(VL53L0X_SELECT2, 1);
    delay(100);
    lidar2.init();
    lidar2.setAddress(0x46);
    lidar2.setTimeout(500);

    Serial.println("VL53L0X LIDAR3 INIT");
    //wake up lidar #3
    digitalWrite(VL53L0X_SELECT3, 1);
    delay(100);
    lidar3.init();
    lidar3.setAddress(0x47); //default address is 0x29
    lidar3.setTimeout(500);

    delay(100);

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
    lidar1.startContinuous(65);
    delay(80);
    lidar2.startContinuous(65);
    delay(80);
    lidar3.startContinuous(65);


  /************ INIT APDS9960 IR LED GESTURE SENSOR ******************/
    // Initialize interrupt service routine
    attachInterrupt(APDS9960_INT, interruptRoutine, FALLING);
  
    // Initialize APDS-9960 (configure I2C and initial values)
    if ( apds.init() ) {
      Serial.println(F("APDS-9960 initialization complete"));
    } else {
      Serial.println(F("Something went wrong during APDS-9960 init!"));
    }
    
    // Start running the APDS-9960 gesture sensor engine
    if ( apds.enableGestureSensor(true) ) {
      Serial.println(F("Gesture sensor is now running"));
    } else {
      Serial.println(F("Something went wrong during gesture sensor init!"));
    }


  /************ INIT KX126 ACCELEROMETER *****************************/
    Serial.print("KX126 INIT RESPONSE WAS ");
    Serial.println(kx126.init());
    delay(200);


  /************ BONDSTORE PERMANENT NN CONFIG ************************/
 /*   if( bleBondStore.hasData() ){
      bleBondStore.getData(bondStoreReadData, bondStoreOffset, bondStoreLength);
      selectNN = (int)bondStoreReadData;
    } else {
      bleBondStore.putData(bondStoreWriteData, bondStoreOffset, bondStoreLength);
    }
  */
    
 /************ LOW POWER SLEEP SETTINGS ******************************/
 lidar1.stopContinuous();
 lidar2.stopContinuous();
 lidar3.stopContinuous();

 apds.disablePower();

  
  delay(500);  
}

/********************************************************************************************************/
/************************ LOOP **************************************************************************/
/********************************************************************************************************/

void loop()
{     
//BOOT ON SLEEP FOR CHARGING
if(SLEEP_MODE){
  if(debug){ Serial.println( "sleep mode"); }

  //LEAVE SLEEP ON BUTTON PRESS
  if(digitalRead(BUTTON_PIN) == 0){
    //leave sleep mode
    SLEEP_MODE = false;

    //startup lidar in continuous mode
    if(debug){ Serial.println(F("Power up lidar")); }
    lidar1.startContinuous(65);
    delay(80);
    lidar2.startContinuous(65);
    delay(80);
    lidar3.startContinuous(65);
    delay(80);

    //startup IR gesture
    apds.enablePower();
        // Start running the APDS-9960 gesture sensor engine
    if ( apds.enableGestureSensor(true) ) {
      if(debug){ Serial.println(F("Gesture sensor is now running")); }
    } else {
      if(debug){ Serial.println(F("Something went wrong during gesture sensor init!")); }
    }
    delay(50);

    
  }
    /******************* Low Power Sleep ********************/
  //https://github.com/sandeepmistry/arduino-nRF5/issues/190
/*  uint32_t timeoutTest = 1000;
   __delay(timeoutTest);
  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  sd_app_evt_wait(); 
  */
}
  /*************** LOOP SPEED CONTROL **********************************/
if(clocktime + speedMs < millis() && SLEEP_MODE != true){
  
  /*************** Timestamp *******************************************/
    clocktime = millis();
    if(debug){ Serial.println(" "); Serial.print("TIME: "); Serial.print( clocktime/1000 ); Serial.println(" s"); }
    if(debug_time){ Serial.print("Time after init speed limit check: "); Serial.println(millis() - clocktime); }
    

  /*************** Bluetooth App Integration ***************************/
    blePeripheral.poll(); 
    if(debug_time){ Serial.print("Time after BLE poll: "); Serial.println( (millis() - clocktime))/1000; }

  /*************** BUTTON MGMT *****************************************/
 //  buttonMGMT();
 //test button and LED
   if(digitalRead(BUTTON_PIN)){
      //button off
      digitalWrite(GREEN_LED_PIN, 1);  // turn LED OFF
   } else {
      //button on
      digitalWrite(GREEN_LED_PIN, 0);  // turn LED ON
   }

  /*************** LED MGMT ********************************************/
  // ledMGMT();
   if(debug_time){ Serial.print("Time after button & LED: "); Serial.println( (millis() - clocktime))/1000; }

  /*************** VIBRATION MOTOR MGMT ********************************/
  if(vibrate_counter > 0){
      vibrate_counter--;
      if(vibrate_status == false){
          digitalWrite(VIBRATE_PIN, 1);
          vibrate_status == true;
      }
  } else if(vibrate_status == true || vibrate_counter <= 0) {
    vibrate_status == false;
    digitalWrite(VIBRATE_PIN, 0);
  }


   /************** READ KX126 ACCELEROMETER *****************************/
   sampleAngularPosition();
   if(debug_time){ Serial.print("Time after accelerometer read: "); Serial.println( (millis() - clocktime))/1000; }


   /************** READ VL6180X LIDAR DISTANCE **************************/ 
   sampleLIDAR();   
   if(debug_time){ Serial.print("Time after distance read: "); Serial.println( (millis() - clocktime))/1000; }


   /************** READ MLX90615 THERMOPILES ****************************/
   sampleThermopiles();
   if(debug_time){ Serial.print("Time after thermo read: "); Serial.println( (millis() - clocktime))/1000; }

   /************** READ APDS9960 IR LED GESTURE SENSOR ******************/
   //don't run if top of device is against an object or surface 
   /* DISABLE FOR NOW
  if( isr_flag == 1 && distance[0] > 500 && distance[1] > 500 && distance[2] > 500) {
    detachInterrupt(APDS9960_INT);
    handleGesture();
    isr_flag = 0;
    attachInterrupt(APDS9960_INT, interruptRoutine, FALLING);
  }
  */


   /************** TRANSMIT SENSOR DATA OVER BLUETOOTH ******************/ 
   transmitSensorData();
   if(debug_time){ Serial.print("Time after Bluetooth serial send: "); Serial.println( (millis() - clocktime))/1000; }


   /************** PRINT SENSOR DATA TO CONSOLE *************************/ 
   if(debug){ printSensorData(); }
  
   if(debug_time){ Serial.print("TIME LOOP: "); Serial.println(millis() - clocktime); }

//end timed loop
 }

  /*********** Power MGMT ******************/
  //https://github.com/sandeepmistry/arduino-nRF5/issues/190
//  uint32_t timeoutTest = 1000;
//   __delay(timeoutTest);
//  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
//  sd_app_evt_wait();
  
//end infinate loop
} 



/********************************************************************************************************/
/************************ FUNCTIONS *********************************************************************/
/********************************************************************************************************/

/*********************************************************************
*************** READ KX126 ACCELEROMETER *****************************
*********************************************************************/
void sampleAngularPosition(){
    //KX022 ACCELEROMETER I2C
    acc[0] = (float)(kx126.getAccel(0) * 10 );
    acc[1] = (float)(kx126.getAccel(1) * 10 );
    acc[2] = (float)(kx126.getAccel(2) * 10 );
    float eulerX, eulerY, eulerZ;
    
    eulerX = acc[0]; eulerY = acc[1]; eulerZ = acc[2]; 
    pitch = (180/3.141592) * ( atan2( eulerX, sqrt( eulerY * eulerY + eulerZ * eulerZ)) );
    roll = (180/3.141592) * ( atan2(-eulerY, -eulerZ) );

    //adjust for device upward = 180 to prevent crossover 
    pitch = pitch + 180;
    if(roll < -90 ){
      roll = 450 + roll;
      if(roll > 360){roll = roll - 360;}
    } else { roll = roll + 90; }
}

/*********************************************************************
*************** READ MLX90615 THERMOPILES ****************************
*********************************************************************/
void sampleThermopiles(){
    for(int j = 0; j < 22; j++){
        TAmb[j] = readAmbientTempF(j+1); 
        TObj[j] = readObjectTempF(j+1);
    }

    TAmbAv = (TAmb[0] + TAmb[1] + TAmb[2] + TAmb[3]) / 4;
}

/*********************************************************************
*************** READ VL6180X LIDAR DISTANCE **************************
*********************************************************************/
void sampleLIDAR(){

    distance[0] = (float)lidar1.readRangeContinuousMillimeters();
    if(distance[0] < 0.1){ distance[0] = 0; } // edge case
    if (lidar1.timeoutOccurred() && debug) { Serial.print("LIDAR1 TIMEOUT"); } 

    distance[1] = (float)lidar2.readRangeContinuousMillimeters();
    if(distance[1] < 0.1){ distance[1] = 0; } // edge case
    if (lidar2.timeoutOccurred() && debug) { Serial.print("LIDAR2 TIMEOUT"); } 

    distance[2] = (float)lidar3.readRangeContinuousMillimeters();
    if(distance[2] < 0.1){ distance[2] = 0; } // edge case
    if (lidar3.timeoutOccurred() && debug) { Serial.print("LIDAR3 TIMEOUT"); } 


}

/*********************************************************************
*************** TRANSMIT SENSOR DATA OVER BLUETOOTH ****************** 
*********************************************************************/
void transmitSensorData(){
    BLECentral central = blePeripheral.central();
    
    if(central){ // if a central is connected to peripheral

              /*
               * reduce temperature readings to a range between 0 and 31, then multiply to use up the 256 max decimal value of an 8 bit integer
               * Object temperature floor: 70F
               * Object temperature ceiling: 101F
               */
              float TObj_compressed[22];
              for(int q=0; q < 22; q++){
                  TObj_compressed[q] = TObj[q];
                  if(TObj_compressed[q] < 70){ TObj_compressed[q] = 70;  }
                  else if(TObj_compressed[q] > 101){ TObj_compressed[q] = 101;  }
                  TObj_compressed[q] = (TObj_compressed[q] - 70)*8;
              }
              
              float TAmbAv_compressed;
              TAmbAv_compressed = TAmbAv;
              if(TAmbAv < 70){ TAmbAv_compressed = 70;  }
              else if(TAmbAv > 101){ TAmbAv_compressed = 101;  }
              TAmbAv_compressed = (TAmbAv_compressed - 70)*8;

              int command = 0; //placeholder
              
              //get battery charge value
              batteryValue = ( (float)analogRead(BATTERY_PIN) / 1022) * 100; //max A read is 1023

              //OLD BACKWARDS COMPATABLE
              const unsigned char imuCharArray[20] = {
                  (uint8_t)(roll),  
                  (uint8_t)(pitch),
                  (uint8_t)(distance[0]),
                  (uint8_t)(TObj_compressed[0]),  
                  (uint8_t)(TObj_compressed[1]),
                  (uint8_t)(TObj_compressed[2]),
                  (uint8_t)(TObj_compressed[3]),
                  (uint8_t)(TAmbAv_compressed),
                  (uint8_t)(batteryValue),
                  (uint8_t)(command),
                  (uint8_t)( (acc[0] + 1.00) * 100.00),               
                  (uint8_t)( (acc[1] + 1.00) * 100.00),
                  (uint8_t)( (acc[2] + 1.00) * 100.00),  
                  (uint8_t)0,
                  (uint8_t)0,
                  (uint8_t)0,
                  (uint8_t)0,
                  (uint8_t)0,
                  (uint8_t)0,  
                  (uint8_t)0                   //empty
              }; 
              //send data over bluetooth
              DataCharacteristic.setValue(imuCharArray,20);
              //time to send
              delay(5);
          }
   if(debug_time){ Serial.print("Time after bluetooth send: "); Serial.println( (millis() - clocktime))/1000; }
}

/*********************************************************************
*************** PRINT SENSOR DATA TO CONSOLE *************************
*********************************************************************/
void printSensorData(){

    batteryValue = ( (float)analogRead(BATTERY_PIN) / 1022) * 100; //max A read is 1023
  
    Serial.print("T1: "); Serial.print( TObj[0] ); Serial.print(" "); 
    Serial.print("T2: "); Serial.print( TObj[1] ); Serial.print(" ");
    Serial.print("T3: "); Serial.print( TObj[2] ); Serial.print(" ");
    Serial.print("T4: "); Serial.print( TObj[3] ); Serial.print(" ");
    Serial.print("T5: "); Serial.print( TObj[4] ); Serial.print(" ");
    Serial.print("T6: "); Serial.print( TObj[5] ); Serial.print(" ");
    Serial.print("T7: "); Serial.print( TObj[6] ); Serial.print(" ");
    Serial.print("T8: "); Serial.print( TObj[7] ); Serial.print(" ");
    Serial.print("T9: "); Serial.print( TObj[8] ); Serial.print(" ");
    Serial.print("T10: "); Serial.print( TObj[9] ); Serial.print(" ");
    Serial.print("T11: "); Serial.print( TObj[10] ); Serial.print(" ");
    Serial.print("T12: "); Serial.print( TObj[11] ); Serial.print(" ");
    Serial.print("T13: "); Serial.print( TObj[12] ); Serial.print(" ");
    Serial.print("T14: "); Serial.print( TObj[13] ); Serial.print(" ");
    Serial.print("T15: "); Serial.print( TObj[14] ); Serial.print(" ");

    Serial.print("T16: "); Serial.print( TObj[15] ); Serial.print(" ");
    Serial.print("T17: "); Serial.print( TObj[16] ); Serial.print(" ");
    Serial.print("T18: "); Serial.print( TObj[17] ); Serial.print(" ");
    Serial.print("T19: "); Serial.print( TObj[18] ); Serial.print(" ");
    Serial.print("T20: "); Serial.print( TObj[19] ); Serial.print(" ");
    Serial.print("T21: "); Serial.print( TObj[20] ); Serial.print(" ");
    Serial.print("T22: "); Serial.print( TObj[21] ); Serial.print(" ");
    
    Serial.print("DTav: "); Serial.print( TAmbAv ); Serial.println("");

    Serial.print("pitch: "); Serial.print( pitch ); Serial.print("\t"); 
    Serial.print("roll: "); Serial.print( roll ); Serial.print("\t"); 
    Serial.print("accX: "); Serial.print( acc[0] ); Serial.print("\t"); 
    Serial.print("accY: "); Serial.print( acc[1] ); Serial.print("\t"); 
    Serial.print("accZ: "); Serial.print( acc[2] ); Serial.println(""); 
    
    Serial.print("Distance (mm): "); Serial.print(distance[0]); Serial.print("\t"); Serial.print(distance[1]); Serial.print("\t"); Serial.println(distance[2]);

    Serial.print("CMD: "); Serial.println(received_command);
    Serial.print("  Battery: "); Serial.println(batteryValue);
}

/*********************************************************************
**************** BUTTON MGMT *****************************************
*********************************************************************/
void buttonMGMT(){
    
    int lastButtonState = buttonState;
    float pressTime = 0;
    
    // read the state of the pushbutton value:
    buttonState = digitalRead(BUTTON_PIN);
    
    if (buttonState == 1) {
      
        if(buttonBeginPressTime != 0){ buttonBeginPressTime = millis(); }
        pressTime = (buttonBeginPressTime - millis() ) / 1000;
      
      // turn LED on:
     // LED_counter = 80;
        digitalWrite(GREEN_LED_PIN, 1);
        delay(500);
        digitalWrite(GREEN_LED_PIN, 0);
    //  greenLED_status = true;
    } else {
      
        if(buttonBeginPressTime != 0){ 
            pressTime = (buttonBeginPressTime - millis() ) / 1000;
            buttonBeginPressTime = 0; 
        }
        
        if(pressTime > 3){ 
            if(!sleepLight){ sleepLight = true; }
            if(sleepLight){ sleepLight = false; }
            LED_counter = 3;
            Serial.println("****LIGHT SLEEP****");
        }
    }
    if (debug) { Serial.print("BUTTON: "); Serial.print(buttonState); Serial.print("  Press time: "); Serial.print(pressTime); Serial.println("s"); } 
    
    if(pressTime > 8){ sleepDeep = true; if(debug){ Serial.println("DEEP SLEEP"); } delay(2000); }
}

/*********************************************************************
**************** LED MGMT ********************************************
*********************************************************************/
void ledMGMT(){
   //example blink program
   if(LED_counter > 0 && greenLED_status == false ){
      LED_counter--;
      if(LED_counter <= 0){
          LED_counter = 2;
          digitalWrite(GREEN_LED_PIN, 1);
          greenLED_status = true;
      }
   }
   else if(LED_counter > 0 && greenLED_status == true){
      LED_counter--;
      if(LED_counter <= 0){
          LED_counter = 30;
          digitalWrite(GREEN_LED_PIN, 0);
          greenLED_status = false;
      }
   }  
}



/*********************************************************************
*************** ETC **************************************************
*********************************************************************/

void __delay(uint32_t timeout)
{
  uint32_t start;
  start = millis();

 do
 {
   __WFE();
 }
   while ((millis() - start) >= timeout);
}

int hex_to_int(char c){
  int first;
  int second;
  int value;
  
  if (c >= 97) {
    c -= 32;
  }
  first = c / 16 - 3;
  second = c % 16;
  value = first * 10 + second;
  if (value > 9) {
    value--;
  }
  return value;
}

int hex_to_ascii(char c, char d){
  int high = hex_to_int(c) * 16;
  int low = hex_to_int(d);
  return high+low;
}

void interruptRoutine() {
  isr_flag = 1;
}

void handleGesture() {
    if ( apds.isGestureAvailable() ) {
      delayMicroseconds(1000);   
      Serial.println("********************************************************************************************");
    switch ( apds.readGesture() ) {
      case DIR_UP:
        Serial.println("UP");
        break;
      case DIR_DOWN:
        Serial.println("DOWN");
        break;
      case DIR_LEFT:
        Serial.println("LEFT");
        break;
      case DIR_RIGHT:
        Serial.println("RIGHT");
        break;
      case DIR_NEAR:
        Serial.println("NEAR");
        break;
      case DIR_FAR:
        Serial.println("FAR");
        break;
      default:
        Serial.println("NONE");
    }
    Serial.println("********************************************************************************************");
  }
}
