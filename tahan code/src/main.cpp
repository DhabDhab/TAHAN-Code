#include <Arduino.h>
#include <ArduinoBLE.h>
#include <model.h>
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

void updatesdata();

float spo2_values[5] = {0,0,0,0,0}; // Store previous SpO2 values

#define MAX_BRIGHTNESS 255

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)

uint16_t irBuffer[100];   //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100];   //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength;  //data length
int32_t spo2;          //SPO2 value

int8_t validSPO2;       //indicator to show if the SPO2 calculation is valid
int32_t heartRate;      //heart rate value
int8_t validHeartRate;  //indicator to show if the heart rate calculation is valid

int spo2reset = 0;
float oldTemp = 0;
float oldTempF = 0;

byte pulseLED = 11;
byte readLED = 10;

// Bluetooth® Low Energy
BLEService genhealthService("1840");

// Bluetooth® Low Energy Characteristics

BLEIntCharacteristic   spo2Char("2A5F", BLERead | BLENotify);
BLEIntCharacteristic   CtempChar("1809", BLERead | BLENotify);
BLEIntCharacteristic   FtempChar("2A6E", BLERead | BLENotify);

void setup() {
  Serial.begin(115200);  // initialize serial communication
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))  //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  while (Serial.available() == 1);  //activation of the sensor
  Serial.read();

  byte ledBrightness = 60;  //Options: 0=Off to 255=50mA
  byte sampleAverage = 4;   //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;         //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100;    //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;     //Options: 69, 118, 215, 411
  int adcRange = 4096;      //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);
  particleSensor.enableDIETEMPRDY();

  // For Bluetooth Setup without the serial monitor
  while (Serial);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  /* Set a local name for the Bluetooth® Low Energy device
     This name will appear in advertising packets
     and can be used by remote devices to identify this Bluetooth® Low Energy device
     The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  BLE.setLocalName("TAHAN");
  BLE.setAdvertisedService(genhealthService);
  genhealthService.addCharacteristic(spo2Char);
  genhealthService.addCharacteristic(CtempChar);
  genhealthService.addCharacteristic(FtempChar);

  BLE.addService(genhealthService);

  spo2Char.writeValue(spo2reset);
  CtempChar.writeValue(oldTemp);
  FtempChar.writeValue(oldTempF);

  BLE.advertise();

  Serial.println("TAHAN: SPo2_Temp Sensor");
  Serial.println("Bluetooth® device active, waiting for connections...");
}

void loop() 
{
  bufferLength = 20; //DAT

  // for a Bluetooth® Low Energy central
  BLEDevice central = BLE.central();
  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());

    //read the first 20 samples and determine the signal range
    for (byte i = 0; i < bufferLength; i++) {
      while (particleSensor.available() == false)  //do we have new data?
      particleSensor.check();                    //Check the sensor for new data
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample();
    }

    //calculate heart rate and SpO2 after first 20 samples (first 4 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    // while the central is connected:
    while (central.connected()) 
    {

      //dumping the first 5 sets of samples in the memory and shift the last 15 sets of samples to the top
      for (byte i = 5; i < 20; i++) {
        redBuffer[i -5] = redBuffer[i];
        irBuffer[i - 5] = irBuffer[i];
      }

      //take 15 sets of samples before calculating the heart rate.
      for (byte i = 15; i < 20; i++) 
      {
        while (particleSensor.available() == false)  //do we have new data?
        particleSensor.check();                    //Check the sensor for new data
        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample();

        updatesdata();
      }
        maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    }
    // central disconnects
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

void updatesdata() 
{
  if (validSPO2 == 1) {
  float temperature = particleSensor.readTemperature();
  float temperatureF = particleSensor.readTemperatureF();

  // Predict SpO2
  int final_spo2 = intercept + (coefficients[0] * spo2_values[4]);

  // Print the result
  Serial.print(" SPO2= ");
  Serial.print(final_spo2);
  spo2Char.writeValue(final_spo2);
  Serial.print(" SPO2Valid= ");
  Serial.println(validSPO2, DEC);
  Serial.print("Predicted SpO2: ");
  Serial.println(spo2);
  CtempChar.writeValue(temperature);
  
  delay(500); 

  FtempChar.writeValue(temperatureF);
  for (int i = 0; i < 4; i++) {
    spo2_values[i] = spo2_values[i + 1]; // Shift older values
  }
  spo2_values[4] = spo2;  // Add the new SpO2 value to the array
  }
}