/* Includes ---------------------------------------------------------------- */
//#include <CNPq_-_Personal_Trainer_-_AIO_-_Novos_Dados_inferencing.h>
#include <CNPq_-_UNIFEI_-_Personal_Trainer__with_TinyML_inferencing.h>
#include <ArduinoBLE.h>
#include "LSM6DS3.h"
#include "Wire.h"

/*-------------------------------------------------------------------------- */
// Create an instance of class LSM6DS3
LSM6DS3 xIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

/*-------------------------------------------------------------------------- */
// DeepSleep defines as variables
#define int1Pin PIN_LSM6DS3TR_C_INT1

uint8_t interruptCount = 0;      // Amount of received interrupts
uint8_t prevInterruptCount = 0;  // Interrupt Counter from last loop

/*-------------------------------------------------------------------------- */
//  Private Variables
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
long  sleepPreMillis = 0, sleepCurMillis = 0;

/*-------------------------------------------------------------------------- */
//  Function instances
void displayResults(int pred_index);
void turnOffLeds();
void classifier();
void setupDoubleTapInterrupt();
void inits();
void goToPowerOff();
void int1ISR();
void batteryMeter();
float eiGetSign(float number);

/*-------------------------------------------------------------------------- */
//BLE configuration

/* Online GUID / UUID Generator:
https://www.uuidgenerator.net/
ba50c021-3295-468e-a99c-6771810eea3f
*/

// UUid for Service
const char* UUID_serv = "2e9ee18e-4066-4d72-b603-873b6dfab9d7";

// UUids to send the inference result
const char* UUID_bp   = "2e9ee18f-4066-4d72-b603-873b6dfab9d7"; //bench press
const char* UUID_bc   = "2e9ee190-4066-4d72-b603-873b6dfab9d7"; //biceps curl
const char* UUID_lr   = "2e9ee191-4066-4d72-b603-873b6dfab9d7"; //lateral raise
const char* UUID_we   = "2e9ee192-4066-4d72-b603-873b6dfab9d7"; //wrong exercise
const char* UUID_id   = "2e9ee193-4066-4d72-b603-873b6dfab9d7"; //idle

// UUid to send the battery voltage
const char* UUID_bt   = "2e9ee194-4066-4d72-b603-873b6dfab9d7";

// BLE Service
BLEService myService(UUID_serv); 

// BLE accelerometer characteristics
BLEIntCharacteristic  chBp(UUID_bp,  BLERead|BLENotify);
BLEIntCharacteristic  chBc(UUID_bc,  BLERead|BLENotify);
BLEIntCharacteristic  chLr(UUID_lr,  BLERead|BLENotify);
BLEIntCharacteristic  chWe(UUID_we,  BLERead|BLENotify);
BLEIntCharacteristic  chId(UUID_id,  BLERead|BLENotify);

// BLE battery characteristic
BLEIntCharacteristic  chBt(UUID_bt,  BLERead|BLENotify);

void setup()
{
    Serial.begin(115200);
    // comment out the below line to cancel the wait for USB connection (needed for native USB)
    //while (!Serial);
    Serial.println("CNPq classifier + Sleep");

    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);

    // Battery voltage setup
    pinMode(P0_31, INPUT);            //Battery Voltage monitoring pin
    pinMode(P0_13, OUTPUT);           //Charge Current setting pin
    pinMode(P0_14, OUTPUT);           //Enable Battery Voltage monitoring pin
    digitalWrite(P0_13, LOW);         //Charge Current 100mA   
    digitalWrite(P0_14, LOW);         //Enable
    analogReference(AR_INTERNAL2V4);  //Vref=2.4V
    analogReadResolution(12);         //12bits

    // Interrupt pin setup
    pinMode(int1Pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(int1Pin), int1ISR, RISING);
    sleepPreMillis = millis(); 

    turnOffLeds();
    inits();

    // Choose and set the BLE name
    BLE.setLocalName("IMU - classifier");
    BLE.setDeviceName("XIAO-BLE-Sense - IMU classifier"); 

    // Set advertised Service
    BLE.setAdvertisedService(myService);

    myService.addCharacteristic(chBp);
    myService.addCharacteristic(chBc);
    myService.addCharacteristic(chLr);
    myService.addCharacteristic(chWe);
    myService.addCharacteristic(chId);

    myService.addCharacteristic(chBt);
    
    // add service to BLE
    BLE.addService(myService);

    chBp.writeValue(0);
    chBc.writeValue(0);
    chLr.writeValue(0);
    chWe.writeValue(0);
    chId.writeValue(0);

    chBt.writeValue(100);

    BLE.advertise();
    Serial.println("Advertising started");
    Serial.println("Bluetooth device active, waiting for connections...");

    if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 6) {
        ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 6 (the 3 sensor axes of acc + 3 sensor axes of gyro)\n");
        return;
    }
}

/**
 * @brief Return the sign of the number
 * 
 * @param number 
 * @return int 1 if positive (or 0) -1 if negative
 */
float eiGetSign(float number) {
    return (number >= 0.0) ? 1.0 : -1.0;
}

void loop()
{  
  BLEDevice central = BLE.central();
  long preMillis = millis();
  digitalWrite(LEDG, LOW);
  
  if (central) 
  {
    Serial.print("Connected to central: ");
    Serial.println(central.address()); // central device MAC address
      
    while (central.connected()) 
    {
      long curMillis = millis();
      //if (preMillis>curMillis) preMillis=0; // millis() rollover?
      if (curMillis - preMillis > 3000)
      {      
        classifier();
        // batteryMeter();
      }
    }
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
    turnOffLeds();
    sleepPreMillis = millis();    
  }

  // Waits 2 minutes and enter sleep mode
  sleepCurMillis = millis();
  if (sleepCurMillis - sleepPreMillis >= 60000)
  {
//    sleepPreMillis = sleepCurMillis;
    Serial.println(sleepCurMillis - sleepPreMillis);
    goToPowerOff();
  }
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_FUSION
#error "Invalid model for current sensor"
#endif

void classifier()
{
    ei_printf("\nStarting inferencing in 4 seconds...\n");
    
    delay(2500);
    
    turnOffLeds();    
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDR, LOW);
//    digitalWrite(LEDB, LOW);
    delay(1500);
    turnOffLeds();

    ei_printf("Sampling...\n");

    // Allocate a buffer here for the values we'll read from the IMU
    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 6) {
        // Determine the next tick (and then sleep later)
        uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

        buffer[ix]     = xIMU.readFloatAccelX();
        buffer[ix + 1] = xIMU.readFloatAccelY();
        buffer[ix + 2] = xIMU.readFloatAccelZ();
        buffer[ix + 3] = xIMU.readFloatGyroX();
        buffer[ix + 4] = xIMU.readFloatGyroY();
        buffer[ix + 5] = xIMU.readFloatGyroZ();

        delayMicroseconds(next_tick - micros());
    }

    // Turn the raw buffer in a signal which we can the classify
    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("Failed to create signal from buffer (%d)\n", err);
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    // print the predictions
    ei_printf("Predictions ");
    ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(": \n");

    int pred_index = 0;
    float pred_value = result.classification[0].value;   
    
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
        if (result.classification[ix].value > pred_value){
          pred_index = ix;
          pred_value = result.classification[ix].value;
        }
    }
    ei_printf("  Prediction: %s with probability %.2f\n", result.classification[pred_index].label, pred_value);
    displayResults (pred_index);
    
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif
}

void inits()
{
    bool err=false;

    // init IMU
    if (xIMU.begin() != 0) {
      Serial.println("IMU error");
      err = true;
    } else {
      Serial.println("IMU OK!");
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
}

void turnOffLeds(){
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
}

void displayResults(int pred_index) {
  switch (pred_index)
  {
    case 0:     // Idle:[0] ==> White ON
      turnOffLeds();         
      digitalWrite(LEDG, LOW);
      digitalWrite(LEDR, LOW);
      digitalWrite(LEDB, LOW);
      chId.writeValue(1);
      break;

    case 1:      // BenchPress:[1] ==> Blue ON
      turnOffLeds();
      digitalWrite(LEDB, LOW);
      chBp.writeValue(1);
      break;

    case 2:     // BicepsCurl:[2] ==> Purple ON
      turnOffLeds();         
      digitalWrite(LEDR, LOW);
      digitalWrite(LEDB, LOW);
      chBc.writeValue(1);
      break;
      
    case 3:     // LateralRaise:[3] ==> Green ON
      turnOffLeds();         
      digitalWrite(LEDG, LOW);
      chLr.writeValue(1);
      break;

    case 4:     // WrongExercises:[4] ==> Red ON
      turnOffLeds();         
      digitalWrite(LEDR, LOW);
      chWe.writeValue(1);
      break;    
  }
}

void batteryMeter()
{
  int VbattPctg = 0, Vadc = 0;;

  for (int i=0; i<5; i++){
    Vadc = Vadc + analogRead(P0_31);
  }

  Vadc /= 5;
  float Vbatt = ((510e3 + 1000e3) / 510e3) * 2.4 * Vadc / 4096;
  VbattPctg = int(round(((Vbatt - 2.9)/1.21)*100));
  Serial.print("Battery Level: ");
  Serial.print(Vbatt, 3);
  Serial.print(", ");
  Serial.print(VbattPctg);
  Serial.println("%");
  chBt.writeValue(VbattPctg);
}

void goToPowerOff() 
{
  Serial.println("Going to System OFF");
  turnOffLeds();
  setupDoubleTapInterrupt();
  delay(1000);  // delay seems important to apply settings, before going to System OFF
  //Ensure interrupt pin from IMU is set to wake up device
  nrf_gpio_cfg_sense_input(digitalPinToInterrupt(int1Pin), NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
  delay(2000);  // Trigger System OFF
  NRF_POWER->SYSTEMOFF = 1;
}

void int1ISR() 
{
  interruptCount++;
}

void setupDoubleTapInterrupt() 
{
  xIMU.settings.gyroEnabled = 0;  // Gyro currently not used, disabled to save power
  uint8_t error = 0;
  uint8_t dataToWrite = 0;

  // Double Tap Config
//  xIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x30); // Acc = 52Hz
//  xIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x40); // Acc = 104Hz
  xIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x50); // Acc = 208Hz
//  xIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x60);  //* Acc = 416Hz (High-Performance mode)// Turn on the accelerometer
  // ODR_XL = 416 Hz, FS_XL = 2g
  xIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x8E);     // INTERRUPTS_ENABLE, SLOPE_FDS// Enable interrupts and tap detection on X, Y, Z-axis
  xIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D, 0x85);   // Set tap threshold 8C
  xIMU.writeRegister(LSM6DS3_ACC_GYRO_INT_DUR2, 0x7F);     // Set Duration, Quiet and Shock time windows 7F
  xIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x80);  // Single & double-tap enabled (SINGLE_DOUBLE_TAP = 1)
  xIMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x08);      // Double-tap interrupt driven to INT1 pin
}