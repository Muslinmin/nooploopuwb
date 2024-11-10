#define START_BYTE 0x55
#define FUNCTION_MARK_BYTE 0x04
#define BUFFER_SIZE 256

#include "MPU9250.h"
#include <Wire.h>
#include <Servo.h>




unsigned long lastSerialActivity = 0; // For tracking serial freeze
const unsigned long timeout = 5000;   // 5-second timeout
static float robot_yaw = 0;
// uint32_t prev_ms;

// MPU9250 mpu;


// Data structure to hold anchor node information
struct AnchorNode {
  uint8_t role;
  uint8_t id;
  float distance;   // Distance in meters
  float fpRssi;     // FP RSSI in dB
  float rxRssi;     // RX RSSI in dB
};

struct Position {
  float x;
  float y;
};

// Data structure to hold the parsed values
struct NLinkData {
  uint8_t frameHeader;
  uint8_t functionMark;
  uint16_t frameLength;
  uint8_t role;
  uint8_t id;
  uint32_t systemTime;
  float eop_x, eop_y, eop_z;
  float pos_x, pos_y, pos_z; // the estimated position
  float vel_x, vel_y, vel_z;
  float gyro_x, gyro_y, gyro_z;
  float acc_x, acc_y, acc_z;
  float angle_x, angle_y, angle_z;
  float q0, q1, q2, q3;
  uint32_t localTime;
  float voltage;
  uint8_t validNodeQuantity;
  AnchorNode anchors[4]; // Array of anchor nodes (maximum of 4)
};

NLinkData parsedData;


void setup() {
    Wire.begin();
    Wire.setClock(400000);  // Set the I2C clock to 400 kHz
    delay(2000);
    
    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    // Calibration code

    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();


    delay(5000);
    mpu.calibrateMag();

    mpu.verbose(false);

    while (!Serial);
    Serial.end();
    delay(1000);
    Serial.begin(9600);
    Serial1.begin(115200);
    lastSerialActivity = millis(); // Initialize last activity timestamp
}



void parsePacket(const uint8_t *buffer) {
  int offset = 0;
  
  parsedData.frameHeader = buffer[offset++];
  parsedData.functionMark = buffer[offset++];
  parsedData.frameLength = buffer[offset] | (buffer[offset + 1] << 8);
  offset += 2;

  parsedData.role = buffer[offset++];
  parsedData.id = buffer[offset++];

  parsedData.systemTime = buffer[offset] | (buffer[offset + 1] << 8) | (buffer[offset + 2] << 16) | (buffer[offset + 3] << 24);
  offset += 4;

  parsedData.eop_x = buffer[offset++] / 100.0f;
  parsedData.eop_y = buffer[offset++] / 100.0f;
  parsedData.eop_z = buffer[offset++] / 100.0f;

  parsedData.pos_x = ((int32_t)(buffer[offset] | (buffer[offset + 1] << 8) | (buffer[offset + 2] << 16))) / 1000.0f;
  offset += 3;
  parsedData.pos_y = ((int32_t)(buffer[offset] | (buffer[offset + 1] << 8) | (buffer[offset + 2] << 16))) / 1000.0f;
  offset += 3;
  parsedData.pos_z = ((int32_t)(buffer[offset] | (buffer[offset + 1] << 8) | (buffer[offset + 2] << 16))) / 1000.0f;
  offset += 3;

  parsedData.vel_x = ((int32_t)(buffer[offset] | (buffer[offset + 1] << 8) | (buffer[offset + 2] << 16))) / 10000.0f;
  offset += 3;
  parsedData.vel_y = ((int32_t)(buffer[offset] | (buffer[offset + 1] << 8) | (buffer[offset + 2] << 16))) / 10000.0f;
  offset += 3;
  parsedData.vel_z = ((int32_t)(buffer[offset] | (buffer[offset + 1] << 8) | (buffer[offset + 2] << 16))) / 10000.0f;
  offset += 3;

  offset += 9; // reserved

  parsedData.gyro_x = *((float*)&buffer[offset]);
  offset += 4;
  parsedData.gyro_y = *((float*)&buffer[offset]);
  offset += 4;
  parsedData.gyro_z = *((float*)&buffer[offset]);
  offset += 4;

  parsedData.acc_x = *((float*)&buffer[offset]);
  offset += 4;
  parsedData.acc_y = *((float*)&buffer[offset]);
  offset += 4;
  parsedData.acc_z = *((float*)&buffer[offset]);
  offset += 4;

  offset += 12; //reserved

  parsedData.angle_x = ((int16_t)(buffer[offset] | (buffer[offset + 1] << 8))) / 100.0f;
  offset += 2;
  parsedData.angle_y = ((int16_t)(buffer[offset] | (buffer[offset + 1] << 8))) / 100.0f;
  offset += 2;
  parsedData.angle_z = ((int16_t)(buffer[offset] | (buffer[offset + 1] << 8))) / 100.0f;
  offset += 2;
  
  parsedData.q0 = *((float*)&buffer[offset]);
  offset += 4;
  parsedData.q1 = *((float*)&buffer[offset]);
  offset += 4;
  parsedData.q2 = *((float*)&buffer[offset]);
  offset += 4;
  parsedData.q3 = *((float*)&buffer[offset]);
  offset += 4;

  offset += 4; // reserved

  parsedData.localTime = buffer[offset] | (buffer[offset + 1] << 8) | (buffer[offset + 2] << 16) | (buffer[offset + 3] << 24);
  offset += 4;

  offset += 10; // reserved

  parsedData.voltage = ((int16_t)(buffer[offset] | (buffer[offset + 1] << 8))) / 1000.0f;
  offset += 2;

  parsedData.validNodeQuantity = buffer[offset++];

  // Parse each anchor node based on validNodeQuantity
  for (int i = 0; i < parsedData.validNodeQuantity && i < 4; i++) {
    parsedData.anchors[i].role = buffer[offset++];
    parsedData.anchors[i].id = buffer[offset++];

    parsedData.anchors[i].distance = ((int32_t)(buffer[offset] | (buffer[offset + 1] << 8) | (buffer[offset + 2] << 16))) / 1000.0f;
    offset += 3;

    parsedData.anchors[i].fpRssi = -buffer[offset++] / 2.0f;
    parsedData.anchors[i].rxRssi = -buffer[offset++] / 2.0f;

    offset += 6; // Skip reserved bytes for each anchor node
  }
}





void sendExtendedData() {
  // Send the tag's position
  Serial.print(parsedData.pos_x);
  Serial.print(',');
  Serial.print(parsedData.pos_y);
  Serial.print(';');

  // Send each anchor's data: distance, fpRssi, and rxRssi
  for (int i = 0; i < parsedData.validNodeQuantity && i < 4; i++) {
    // Convert each anchor's data to int (scaled by 10 to keep one decimal place)
    int anchor_fpRssi = static_cast<int>(parsedData.anchors[i].fpRssi);
    int anchor_rxRssi = static_cast<int>(parsedData.anchors[i].rxRssi);
    Serial.print(parsedData.anchors[i].distance);
    Serial.print(',');
    Serial.print(parsedData.anchors[i].fpRssi);
    Serial.print(',');
    Serial.print(parsedData.anchors[i].rxRssi);

    // Add a semicolon after each anchor data except the last one
    if (i < parsedData.validNodeQuantity - 1) {
      Serial.print(';');
    }
  }
  Serial.print(';');
  Serial.print(robot_yaw);
  // End with a newline character
  Serial.println();
}



void loop() {
  static uint8_t buffer[BUFFER_SIZE];
  static int bufferIndex = 0;
  static bool packetStarted = false;
  static uint16_t frameLength = 0;


  // resetSerialIfNeeded(); // Call function to check for freezes

  while (Serial1.available()) {
    // lastSerialActivity = millis(); // Update last activity timestamp
    uint8_t incomingByte = Serial1.read();

    if (!packetStarted) {
      if (incomingByte == START_BYTE) {
        packetStarted = true;
        bufferIndex = 0;
        frameLength = 0;
        buffer[bufferIndex++] = incomingByte;
      }
    } else {
      buffer[bufferIndex++] = incomingByte;

      if (bufferIndex == 2 && incomingByte != FUNCTION_MARK_BYTE) {
        packetStarted = false;
        bufferIndex = 0;
      }

      if (bufferIndex == 4) {
        frameLength = buffer[2] | (buffer[3] << 8);
        if (frameLength > BUFFER_SIZE) {
          Serial.println("Invalid frame length");
          packetStarted = false;
          bufferIndex = 0;
        }
      }

      if (frameLength > 0 && bufferIndex >= frameLength) {
        uint8_t calculatedChecksum = 0;
        for (int i = 0; i < frameLength - 1; i++) {
          calculatedChecksum += buffer[i];
        }
        if (calculatedChecksum == buffer[frameLength - 1]) {
          parsePacket(buffer);
          if(parsedData.validNodeQuantity >= 4){
              sendExtendedData();
          }
        } else {
          Serial.println("Checksum failed.");
        }
        packetStarted = false;
        bufferIndex = 0;
      }
    }
  }
      if (mpu.update()) {
          static uint32_t prev_ms = millis();
          if (millis() > prev_ms + 500) {
            robot_yaw = mpu.getEulerZ();  // Update yaw if needed
            prev_ms = millis();
          }
         
      }

}
