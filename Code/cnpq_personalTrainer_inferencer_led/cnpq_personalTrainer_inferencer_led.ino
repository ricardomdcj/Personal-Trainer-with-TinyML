
/* Includes ---------------------------------------------------------------- */
#include <CNPq_-_Personal_Trainer_-_AIO_-_Novos_Dados_inferencing.h>
#include "LSM6DS3.h"
#include "Wire.h"

//Create an instance of class LSM6DS3
LSM6DS3 xIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

void turn_on_leds(int pred_index);
void turn_off_leds();
void inferencer();

void setup()
{
    Serial.begin(115200);
    // comment out the below line to cancel the wait for USB connection (needed for native USB)
    //while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo");

    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);

    turn_off_leds();
        
    bool err=false;
  
    // init IMU
    if (xIMU.begin() != 0) {
      Serial.println("Device error");
      err = true;
    } else {
      Serial.println("Device OK!");
    }

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
    
    if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 6) {
        ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 6 (the 3 sensor axes of the accelerometer + 3 sensor axes of the gyroscope!)\n");
        return;
    }
}

float ei_get_sign(float number) {
    return (number >= 0.0) ? 1.0 : -1.0;
}

void loop()
{
  inferencer();
}

void inferencer()
{
    ei_printf("\nStarting inferencing in 3.5 seconds...\n");

    delay(3500);

    turn_off_leds();    
    digitalWrite(LEDG, LOW);  // turn the RGB led on yellow for 1,5 seconds 
    digitalWrite(LEDR, LOW);  // to indicate that the sampling is initializing
    delay(1500);
    turn_off_leds();          // turn off the RGB led to show that the sampling has started
    
    ei_printf("Sampling...\n");

    // Allocate a buffer here for the values we'll read from the IMU
    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 6) {
        // Determine the next tick (and then sleep later)
        uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

        //xIMU.readFloatAccelX() reads the accelerometer value of the X axis
        //xIMU.readFloatGyroX()  reads the gyroscope value of the X axis
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
    
    turn_on_leds (pred_index);    // calls the function to display the result on the RGB led.
    
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif
}

//---------------------------- Functions below -----------------------------

void turn_off_leds(){
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
}

void turn_on_leds(int pred_index) {
  switch (pred_index)
  {
    case 0:     // Idle:[0] ==> White ON
      turn_off_leds();         
      digitalWrite(LEDG, LOW);
      digitalWrite(LEDR, LOW);
      digitalWrite(LEDB, LOW);
      break;
      
    case 1:     // Lateral Raise:[1] ==> Green ON
      turn_off_leds();         
      digitalWrite(LEDG, LOW);
      break;

    case 2:     // Wrong Exercise:[2] ==> Red ON
      turn_off_leds();         
      digitalWrite(LEDR, LOW);
      break;
    
    case 3:     // Biceps Curl:[3] ==> Purple ON
      turn_off_leds();         
      digitalWrite(LEDR, LOW);
      digitalWrite(LEDB, LOW);
      break;

    case 4:     // Bench Press:[4] ==> Blue ON
      turn_off_leds();
      digitalWrite(LEDB, LOW);
      break;
  }
}
