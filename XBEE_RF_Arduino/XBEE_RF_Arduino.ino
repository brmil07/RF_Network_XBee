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

// Initialize Constant Variables
const int BAT_PIN = 38;
const int SLEEP_TRIG_PIN = 33;
const int ON_DURATION = 10000;
const int OFF_DURATION = 5000;
const int SEND_TX_PERIOD = 5000;
const int PRINT_DURATION = 5000;

// Initialize Global Variables
float lightData = 0;
float tempData = 0;
float altiData = 0;
float presData = 0;
float batData = 0;

String sensor_data;
String hexArray;
size_t payloadSize;
uint8_t* payload;

uint8_t ssRX = 0;
uint8_t ssTX = 1;

bool state = false;
bool sendTxFlag = false;

unsigned long previousTime;
unsigned long previousTime2;
unsigned long currentTime;
unsigned long currentTime2;

// Instantiating objects for various classes
Adafruit_BMP280 bmp;
SoftwareSerial SoftSerial(ssRX, ssTX);
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();
Tx64Request tx;
TxStatusResponse txStatus = TxStatusResponse();

// Set the node destination address here
XBeeAddress64 addr64 = XBeeAddress64(0x0013A200, 0x41BDFF8D);

// Set the AT Command
uint8_t idCmd[] = { 'F', 'R' };  // AT Command 'FR' is for software reset
AtCommandRequest atRequest = AtCommandRequest(idCmd);
AtCommandResponse atResponse = AtCommandResponse();

void sendAtCommand() {
  Serial.println("Sending command to the XBee");

  // Send the AT Command
  xbee.send(atRequest);

  // Wait up to 5 seconds for the status response
  if (xbee.readPacket(5000)) {
    // Got a response!

    // Should be an AT command response
    if (xbee.getResponse().getApiId() == AT_COMMAND_RESPONSE) {
      xbee.getResponse().getAtCommandResponse(atResponse);

      if (atResponse.isOk()) {
        Serial.print("Command [");
        Serial.print(atResponse.getCommand()[0]);
        Serial.print(atResponse.getCommand()[1]);
        Serial.println("] was successful!");

        if (atResponse.getValueLength() > 0) {
          Serial.print("Command value length is ");
          Serial.println(atResponse.getValueLength(), DEC);

          Serial.print("Command value: ");

          for (int i = 0; i < atResponse.getValueLength(); i++) {
            Serial.print(atResponse.getValue()[i], HEX);
            Serial.print(" ");
          }

          Serial.println(" ");
        }
      } else {
        Serial.print("Command return error code: ");
        Serial.println(atResponse.getStatus(), HEX);
      }
    } else {
      Serial.print("Expected AT response but got ");
      Serial.println(xbee.getResponse().getApiId(), HEX);
    }
  } else {
    // AT Command failed
    if (xbee.getResponse().isError()) {
      Serial.print("Error reading packet.  Error code: ");
      Serial.println(xbee.getResponse().getErrorCode());
    } else {
      Serial.print("No response from radio");
    }
  }
}

void sendTx(uint8_t* payload, size_t payloadSize) {
  Tx64Request tx = Tx64Request(addr64, payload, payloadSize);
  xbee.send(tx);
  xbee.readPacket();

  // After sending a tx request, we expect a status response
  // Wait up for the status response
  if (xbee.readPacket(1000)) {
    // Got a response!
    Serial.println("got a response");
    // Should be a znet tx status
    if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
      xbee.getResponse().getTxStatusResponse(txStatus);
      Serial.println("response");
      // Get the delivery status, the fifth byte
      if (txStatus.getStatus() == SUCCESS) {
        // Success.  time to celebrate
        Serial.println("Success");
      } else {
        // The remote XBee did not receive our packet. is it powered on?
        Serial.println("Failed");
      }
    }
    Serial.println();
  }
}

String convertToHexArray(String& data) {
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

String getSensorData() {
  lightData = TSL2561.readVisibleLux();
  tempData = bmp.readTemperature();
  altiData = bmp.readAltitude(1013.25);  // Adjust to local forecast!
  presData = bmp.readPressure();
  batData = analogRead(BAT_PIN);
  batData = (-0.0000004 * pow(batData, 3) + 0.0007 * pow(batData, 2) - 0.2017 * batData - 0.1125) + 0.11;

  if (isnan(lightData)) lightData = 0;
  if (isnan(tempData)) tempData = 0;
  if (isnan(altiData)) altiData = 0;
  if (isnan(presData)) presData = 0;
  if (isnan(batData)) batData = 0;

  String data = "Light:" + String(lightData)
                + ",Temperature:" + String(tempData)
                + ",Altitude:" + String(altiData)
                + ",Pressure:" + String(presData)
                + ",Battery:" + String(batData);

  return data;
}

String printSensorData(String data) {
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
      lightData = value.toFloat();
    } else if (param.equals("Temperature")) {
      tempData = value.toFloat();
    } else if (param.equals("Altitude")) {
      altiData = value.toFloat();
    } else if (param.equals("Pressure")) {
      presData = value.toFloat();
    } else if (param.equals("Battery")) {
      batData = value.toFloat();
    }

    startIdx = endIdx + 1;
  }

  if (currentTime2 - previousTime2 >= PRINT_DURATION) {
    Serial.print("Battery Percentage: ");
    Serial.print(batData);
    Serial.println("%");

    Serial.print(F("Temperature = "));
    Serial.print(tempData);
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(presData);
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(altiData);
    Serial.println(" m");

    Serial.print("The Light intensity is: ");
    Serial.print(lightData);
    Serial.println(" Lux");
    Serial.println();

    previousTime2 = currentTime2;
  }

  return "";
}

void handleSleepState(uint8_t* payload, size_t payloadSize) {
  currentTime = millis();
  if (!state && currentTime - previousTime >= OFF_DURATION) {
    state = true;
    sendTxFlag = false;
    digitalWrite(SLEEP_TRIG_PIN, state);
    previousTime = currentTime;
  } else if (state && currentTime - previousTime >= ON_DURATION) {
    state = false;
    digitalWrite(SLEEP_TRIG_PIN, state);
    previousTime = currentTime;

    unsigned long sendTxPeriod = currentTime;

    while (currentTime - sendTxPeriod < SEND_TX_PERIOD) {
      // Execute sendtx() only if it hasn't been executed during the off period
      delay(1000);
      if (!sendTxFlag) {
        Serial.println("Sending Packet!");
        sendTx(payload, payloadSize);
        sendTxFlag = true;  // Set the flag to true once sendtx() is executed
      }
      currentTime = millis();
    }
  }
}

void convertSensorDataToPayload() {
  sensor_data = getSensorData();

  // Convert sensor_data to Hex format
  hexArray = convertToHexArray(sensor_data);

  // Convert hexArray to uint8_t array
  payloadSize = hexArray.length() / 3; // 3 characters in hexArray represent one byte
  payload = new uint8_t[payloadSize];

  for (size_t i = 0, j = 0; i < hexArray.length(); i += 3, ++j) {
    payload[j] = strtol(hexArray.substring(i, i + 2).c_str(), NULL, 16);
  }
}

void allocatePayload() {
  hexArray = convertToHexArray(sensor_data);
  payloadSize = hexArray.length() / 3; // 3 characters in hexArray represent one byte
  payload = new uint8_t[payloadSize];

  for (size_t i = 0, j = 0; i < hexArray.length(); i += 3, ++j) {
    payload[j] = strtol(hexArray.substring(i, i + 2).c_str(), NULL, 16);
  }
}

void deallocatePayload() {
  delete[] payload;
}

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
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SLEEP_TRIG_PIN, OUTPUT);
  pinMode(BAT_PIN, INPUT);

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

  blinkLED();

  // Send the AT Command to reset the XBee device during the initialization
  sendAtCommand();
  delay(3000);
  digitalWrite(LED_BUILTIN, HIGH);
  
  // Allocate payload memory once during setup
  allocatePayload();
}

void loop() {
  // Continuously getting the sensor data and convert it into Hex format
  convertSensorDataToPayload();
  handleSleepState(payload, payloadSize);
  printSensorData(sensor_data);

  // Deallocate payload memory
  deallocatePayload();
}