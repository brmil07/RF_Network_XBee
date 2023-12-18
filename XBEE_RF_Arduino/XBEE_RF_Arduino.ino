#include <string.h>
#include <cmath>
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

//Initialize Variables
int Bat_Pin = 38;
int sleep_trig = 33;

float Light_Data = 0;
float Temp_Data = 0;
float Alti_Data = 0;
float Pres_Data = 0;
float Bat_Val = 0;
String sensor_data;
String hexArray;
size_t payloadSize;
bool State = false;
bool sendtx_flag = false;
uint8_t ssRX = 0;
uint8_t ssTX = 1;

const int onDuration = 10000;
const int offDuration = 5000;
unsigned long previousTime;
unsigned long previousTime2;

unsigned long currentTime;
unsigned long currentTime2;
unsigned long currentTime3;

//Ambient Sensor
Adafruit_BMP280 bmp;

SoftwareSerial SoftSerial(ssRX, ssTX);
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();
XBeeAddress64 addr64 = XBeeAddress64(0x0013A200, 0x41BDFF8D);
Tx64Request tx;
TxStatusResponse txStatus = TxStatusResponse();

void sendtx(uint8_t* payload, size_t payloadSize) {
  Tx64Request tx = Tx64Request(addr64, payload, payloadSize);
  xbee.send(tx);
  xbee.readPacket();

  // after sending a tx request, we expect a status response
  // wait up for the status response
  if (xbee.readPacket(1000)) {
    // got a response!
    Serial.println("got a response");
    // should be a znet tx status
    if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
      xbee.getResponse().getTxStatusResponse(txStatus);
      Serial.println("response");
      // get the delivery status, the fifth byte
      if (txStatus.getStatus() == SUCCESS) {
        // success.  time to celebrate
        Serial.println("Success");
      } else {
        // the remote XBee did not receive our packet. is it powered on?
        Serial.println("Failed");
      }
    }
    Serial.println();
  }
}

String ConvertToHexArray(String& data) {
  String hexArray = "";

  for (size_t i = 0; i < data.length(); ++i) {
    // Convert character to hexadecimal
    char hexStr[3];
    sprintf(hexStr, "%02X", data[i]);

    // Append the two hexadecimal characters to the result
    hexArray += hexStr[0];
    hexArray += hexStr[1];
    hexArray += ' ';
  }

  return hexArray;
}

String get_sensor_data() {
  Light_Data = TSL2561.readVisibleLux();
  Temp_Data = bmp.readTemperature();
  Alti_Data = bmp.readAltitude(1013.25);  /* Adjusted to local forecast! */
  Pres_Data = bmp.readPressure();
  Bat_Val = analogRead(Bat_Pin);
  Bat_Val = (-0.0000004 * pow(Bat_Val, 3) + 0.0007 * pow(Bat_Val, 2) - 0.2017 * Bat_Val - 0.1125) + 0.11;

  if (isnan(Light_Data)) {
    Light_Data = 0;
  }

  if (isnan(Temp_Data)) {
    Temp_Data = 0;
  }

  if (isnan(Alti_Data)) {
    Alti_Data = 0;
  }

  if (isnan(Pres_Data)) {
    Pres_Data = 0;
  }

  if (isnan(Bat_Val)) {
    Bat_Val = 0;
  }

  String data = "Light:" + String(Light_Data)
                + ",Temperature:" + String(Temp_Data)
                + ",Altitude:" + String(Alti_Data)
                + ",Pressure:" + String(Pres_Data)
                + ",Battery:" + String(Bat_Val);

  return data;
}

String print_sensor_data(String data) {
  currentTime2 = millis();

  size_t startIdx = 0;
  int endIdx;

  while (startIdx < data.length()) {
    endIdx = data.indexOf(':', startIdx);
    String param = data.substring(startIdx, endIdx);

    startIdx = endIdx + 1;
    endIdx = data.indexOf(',', startIdx);
    if (endIdx == -1) {
      endIdx = data.length();
    }

    String value = data.substring(startIdx, endIdx);

    if (param.equals("Light")) {
      Light_Data = value.toFloat();
    } else if (param.equals("Temperature")) {
      Temp_Data = value.toFloat();
    } else if (param.equals("Altitude")) {
      Alti_Data = value.toFloat();
    } else if (param.equals("Pressure")) {
      Pres_Data = value.toFloat();
    } else if (param.equals("Battery")) {
      Bat_Val = value.toFloat();
    }

    startIdx = endIdx + 1;
  }

  if (currentTime2 - previousTime2 >= 5000) {

    Serial.print("Battery Percentage: ");
    Serial.print(Bat_Val);
    Serial.println("%");

    Serial.print(F("Temperature = "));
    Serial.print(Temp_Data);
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(Pres_Data);
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(Alti_Data);
    Serial.println(" m");

    Serial.print("The Light intensity is: ");
    Serial.print(Light_Data);
    Serial.println(" Lux");
    Serial.println();

    previousTime2 = currentTime2;
  }

  return "";
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(sleep_trig, OUTPUT);

  xbee.setSerial(SoftSerial);
  Wire.begin();
  bmp.begin();
  Serial.begin(115200);
  SoftSerial.begin(115200);
  TSL2561.init();

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

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
}

void loop() {
  //////////////////////////////////////////////////////////////////////////////////////
  sensor_data = get_sensor_data();
  hexArray = ConvertToHexArray(sensor_data);

  // Convert hexArray to uint8_t array
  uint8_t payload[hexArray.length() / 3];  // 3 characters in hexArray represent one byte
  for (size_t i = 0, j = 0; i < hexArray.length(); i += 3, ++j) {
    payload[j] = strtol(hexArray.substring(i, i + 2).c_str(), NULL, 16);
  }
  payloadSize = sizeof(payload) / sizeof(payload[0]);
  ///////////////////////////////////////////////////////////////////////////////////////

  currentTime = millis();
  if (State == false && currentTime - previousTime >= offDuration) {
    // Turn on the state if it's currently off and the off duration has passed
    Serial.println("LED IS ON");

    State = true;
    sendtx_flag = false;
    digitalWrite(sleep_trig, State);
    previousTime = currentTime;
  } else if (State == true && currentTime - previousTime >= onDuration) {
    // Turn off the state if it's currently on and the on duration has passed
    Serial.println("LED IS OFF");

    State = false;
    digitalWrite(sleep_trig, State);
    previousTime = currentTime;

    unsigned long sendtxperiod = currentTime;
    while (currentTime - sendtxperiod < 5000) {
      // Execute sendtx() only if it hasn't been executed during the off period
      delay(1000);
      if (!sendtx_flag) {
        Serial.println("Sending Packet!");
        sendtx(payload, payloadSize);
        sendtx_flag = true;  // Set the flag to true once sendtx() is executed
      }
      currentTime = millis();
    }
  }

  print_sensor_data(sensor_data);
}