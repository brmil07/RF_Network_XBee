/**
 * Copyright (c) 2009 Andrew Rapp. All rights reserved.
 *
 * This file is part of XBee-Arduino.
 *
 * XBee-Arduino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * XBee-Arduino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with XBee-Arduino.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <XBee.h>
#include <SoftwareSerial.h>

uint8_t ssRX = 0;
uint8_t ssTX = 1;

SoftwareSerial nss(ssRX, ssTX);

XBee xbee = XBee();

// Network ID
uint8_t idCmd[] = { 'I', 'D' };

// Command Value
uint8_t valCmd[] = { 0x20, 0x15 };

AtCommandRequest atRequest1 = AtCommandRequest(idCmd);
AtCommandRequest atRequest2 = AtCommandRequest(idCmd, valCmd, sizeof(valCmd));

AtCommandResponse atResponse = AtCommandResponse();

void setup() {
  Serial.begin(9600);
  xbee.begin(nss);
  // start soft serial
  nss.begin(9600);

  // Startup delay to wait for XBee radio to initialize.
  // you may need to increase this value if you are not getting a response
  Serial.println("Starting up...");
  delay(3000);
}

void loop() {

  // get SH
  //  sendAtCommand();

  // set command to SL
  // atRequest1.setCommand(idCmd);

  sendAtCommand();

  // we're done.  Hit the Arduino reset button to start the sketch over
  while (1) {};
}

void sendAtCommand() {
  Serial.println("Sending command to the XBee");

  // send the command
  xbee.send(atRequest1);

  // wait up to 5 seconds for the status response
  if (xbee.readPacket(5000)) {
    // got a response!

    // should be an AT command response
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
    // at command failed
    if (xbee.getResponse().isError()) {
      Serial.print("Error reading packet.  Error code: ");
      Serial.println(xbee.getResponse().getErrorCode());
    } else {
      Serial.print("No response from radio");
    }
  }
}