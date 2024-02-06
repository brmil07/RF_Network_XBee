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
int batAnalog = 0;
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

double polynomial(double x) {
  double a = -0.00003481427;
  double b = 0.09863429;
  double c = -92.25270;
  double d = 28521.622;

  return a * pow(x, 3) + b * pow(x, 2) + c * x + d;
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
  Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
  Serial.println(" m");

  Serial.print("The Light intensity is: ");
  Serial.println(TSL2561.readVisibleLux());

  batAnalog = analogRead(BAT_PIN);
  
  if (batAnalog > 1022){
    batData = 100.00;
  }
  else if (batAnalog < 887) {
    batData = 0.00;
  } else {
    batData = polynomial(batAnalog);
  }
  
  Serial.print("Analog Value: ");
  Serial.print(batAnalog);
  Serial.print("; Battery Percentage: ");
  Serial.print(batData);
  Serial.println("%");

  delay(1000);

  Serial.println();
}
