#include <string.h>
#include <Wire.h>
#include <SPI.h>
#include <XBee.h>
#include <SoftwareSerial.h>
#include <Digital_Light_TSL2561.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS (10)

Adafruit_BMP280 bmp;

const int BAT_PIN = 38;
uint8_t ssRX = 0;
uint8_t ssTX = 1;
int ledPin = 13;
float batData = 0.00;

void blinkLED() {
  Serial.println("Starting up...");
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
}

void setup() {
  Wire.begin();
  TSL2561.init();
  bmp.begin();

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  pinMode(ledPin, OUTPUT);
  pinMode(BAT_PIN, INPUT);
  Serial.begin(115200);

  blinkLED();
  Serial.println("Starting up...");
}

void loop() {
  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(2000)); /* Adjusted to local forecast! */
  Serial.println(" m");

  Serial.print("The Light intensity is: ");
  Serial.println(TSL2561.readVisibleLux());

  batData = analogRead(BAT_PIN);
  batData = (-0.0000004 * pow(batData, 3) + 0.0007 * pow(batData, 2) - 0.2017 * batData - 0.1125) + 0.11;
  if (isnan(batData)) batData = 0;
  
  Serial.print("Battery Percentage: ");
  Serial.print(batData);
  Serial.println("%");

  delay(5000);

  Serial.println();
}
