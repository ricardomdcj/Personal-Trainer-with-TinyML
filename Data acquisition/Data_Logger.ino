#include <ArduinoBLE.h> // Arduino BLE library
#include "LSM6DS3.h"
#include "Wire.h"

//Create an instance of class LSM6DS3
LSM6DS3 xIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

/* Online GUID / UUID Generator:
https://www.guidgenerator.com/online-guid-generator.aspx
64cf715d-f89e-4ec0-b5c5-d10ad9b53bf2
*/

// UUid for Service
const char* UUID_serv = "64cf715d-f89e-4ec0-b5c5-d10ad9b53bf2";

// UUids for accelerometer characteristics
const char* UUID_ax   = "9c84f5f1-43be-4656-ace8-e3dfe7e5e9cb";
const char* UUID_ay   = "7c6ae352-91f9-4fb7-bf54-328eeeb408db";
const char* UUID_az   = "05554b91-c93f-4493-920f-69f38835f9a3";

// UUids for gyroscope characteristics
const char* UUID_gx   = "3038ca30-1e2f-4737-a5c6-1730b6c54360";
const char* UUID_gy   = "42279244-dc93-401d-88aa-4c1ae82adcb8";
const char* UUID_gz   = "e64a0295-bf1d-4714-a0c0-7bbddf82f6f4";

// BLE Service
BLEService myService(UUID_serv); 

// BLE accelerometer Characteristics
BLEFloatCharacteristic  chAX(UUID_ax,  BLERead|BLENotify);
BLEFloatCharacteristic  chAY(UUID_ay,  BLERead|BLENotify);
BLEFloatCharacteristic  chAZ(UUID_az,  BLERead|BLENotify);

// BLE gyroscope Characteristics
BLEFloatCharacteristic  chGX(UUID_gx,  BLERead|BLENotify);
BLEFloatCharacteristic  chGY(UUID_gy,  BLERead|BLENotify);
BLEFloatCharacteristic  chGZ(UUID_gz,  BLERead|BLENotify);

void setup() 
{
  Serial.begin(115200);
  //while (!Serial);
  Serial.println("Seeed XIAO BLE Sense IMU-Acc Data Logger");
  
  bool err=false;

  pinMode(LEDR, OUTPUT); // onboard led red 
  pinMode(LEDB, OUTPUT); // onboard led blue 
  pinMode(LEDG, OUTPUT); // onboard led green 
  digitalWrite(LEDR, HIGH); // led red off
  digitalWrite(LEDB, HIGH); // led blue off
  digitalWrite(LEDG, HIGH);
  //digitalWrite(LEDG, LOW); // led green on to indicate power on

  // init IMU
  if (xIMU.begin() != 0) {
    Serial.println("Device error");
    err = true;
  } else {
    Serial.println("Device OK!");
  }

  // init BLE
  if (!BLE.begin()) 
  {
    Serial.println("BLE: failed");
    err=true;
  }
  Serial.println("BLE: ok");

  // error: flash led forever
  if (err)
  {
    Serial.println("Init error. System halted");
    while(1)
    {
      digitalWrite(LEDG, HIGH);
      digitalWrite(LEDR, LOW);
      delay(500); 
      digitalWrite(LEDR, HIGH); // red led blinking to indicate error
      delay(500);
 
    } 
  }

  // Choose and set the BLE name
  //Caixa 1
  BLE.setLocalName("IMU-Acc DataLogger");
  BLE.setDeviceName("XIAO-BLE-Sense - IMU DataLogger"); 

  //Caixa 2
//  BLE.setLocalName("IMU-Acc DataLogger - Caixa 2");
//  BLE.setDeviceName("XIAO-BLE-Sense - IMU DataLogger - Caixa 2"); 
  
  // Set advertised Service
  BLE.setAdvertisedService(myService);
  
  // Add accelerometer characteristics to the Service
  myService.addCharacteristic(chAX);
  myService.addCharacteristic(chAY);
  myService.addCharacteristic(chAZ);

  // Add gyroscope characteristics to the Service
  myService.addCharacteristic(chGX);
  myService.addCharacteristic(chGY);
  myService.addCharacteristic(chGZ);
  
  // add service to BLE
  BLE.addService(myService);
  
  // characteristics initial values
  // Accelerometer
  chAX.writeValue(0);  chAY.writeValue(0);  chAZ.writeValue(0);
  // Gyroscope
  chGX.writeValue(0);  chGY.writeValue(0);  chGZ.writeValue(0);
 
  // start advertising
  BLE.advertise();
  Serial.println("Advertising started");
  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() 
{
  static long preMillis = 0;
  
  // listen for BLE centrals devices
  BLEDevice central = BLE.central();

  // central device connected?
  if (central) 
  {
    digitalWrite(LEDG, HIGH); // turn off the green led
    digitalWrite(LEDB, LOW); // turn on the blue led
    Serial.print("Connected to central: ");
    Serial.println(central.address()); // central device MAC address
    
    // while the central is still connected to peripheral:
    while (central.connected()) 
    {     
      long curMillis = millis();
      if (preMillis>curMillis) preMillis=0; // millis() rollover?
      if (curMillis - preMillis >= 10) // check values every 10mS
      {
        preMillis = curMillis;
        updateValues(); // call function for updating value to send to central
      }
    } // still here while central connected

    // central disconnected:
    digitalWrite(LEDB, HIGH);
    //digitalWrite(LEDG, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  } // no central
}

void updateValues() 
{
  uint8_t averages=10; // average on this values count (accelerometer)
  static long PreMillis = 0;
  long CurMillis = millis();
  
  // accelerometer averaged values/actual values
  static float ax=0;
  static float ay=0;
  static float az=0;
  float ax1, ay1, az1;

  // gyroscope averaged values/actual values
  static float gx=0;
  static float gy=0;
  static float gz=0;
  float gx1, gy1, gz1;
  
  static uint8_t i_a=0; // accelerometer readings counter
  
// read accelerometer values
  i_a++;
  ax1 = xIMU.readFloatAccelX();
  ay1 = xIMU.readFloatAccelY();
  az1 = xIMU.readFloatAccelZ();

  ax+=ax1;
  ay+=ay1;
  az+=az1;

// read gyroscope values
  gx1 = xIMU.readFloatGyroX();
  gy1 = xIMU.readFloatGyroY();
  gz1 = xIMU.readFloatGyroZ();

  gx+=gx1;
  gy+=gy1;
  gz+=gz1;
  
  if (i_a==averages) // send average over BLE
  {
    if (PreMillis>CurMillis) PreMillis=0; // millis() rollover?
    if (CurMillis - PreMillis >= 50) // check values every 50mS
    {
      ax/=averages;    ay/=averages;    az/=averages;
      gx/=averages;    gy/=averages;    gz/=averages;
      Serial.println("Accelerometer: "+String(ax)+","+String(ay)+","+String(az));
      Serial.println("Gyroscope: "+String(gx)+","+String(gy)+","+String(gz));
      chAX.writeValue(ax);    chAY.writeValue(ay);    chAZ.writeValue(az); 
      chGX.writeValue(gx);    chGY.writeValue(gy);    chGZ.writeValue(gz); 
      ax=0;    ay=0;    az=0;
      gx=0;    gy=0;    gz=0;
      i_a=0;
    }
    Serial.println(CurMillis);
  }
}
