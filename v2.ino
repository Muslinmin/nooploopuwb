#define START_BYTE 0x55
#define FUNCTION_MARK_BYTE 0x04
#define BUFFER_SIZE 256

// Data structure to hold anchor node information
struct AnchorNode {
  uint8_t role;
  uint8_t id;
  float distance;   // Distance in meters
  float fpRssi;     // FP RSSI in dB
  float rxRssi;     // RX RSSI in dB
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
  Serial.begin(115200);
  while (!Serial); // Wait for the serial monitor to open
  Serial.println("Starting...");

  Serial1.begin(115200); // Ensure this matches the external device's baud rate
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

void printParsedData() {
  Serial.println("\n--- Parsed Data ---");
  Serial.print("Frame Header: "); Serial.println(parsedData.frameHeader, HEX);
  Serial.print("Function Mark: "); Serial.println(parsedData.functionMark, HEX);
  Serial.print("Frame Length: "); Serial.println(parsedData.frameLength);
  Serial.print("Role: "); Serial.println(parsedData.role);
  Serial.print("ID: "); Serial.println(parsedData.id);
  Serial.print("System Time: "); Serial.println(parsedData.systemTime);

  Serial.println("\n--- Position Error (EOP) ---");
  Serial.print("EOP X: "); Serial.println(parsedData.eop_x);
  Serial.print("EOP Y: "); Serial.println(parsedData.eop_y);
  Serial.print("EOP Z: "); Serial.println(parsedData.eop_z);

  Serial.println("\n--- Position ---");
  Serial.print("Pos X: "); Serial.println(parsedData.pos_x);
  Serial.print("Pos Y: "); Serial.println(parsedData.pos_y);
  Serial.print("Pos Z: "); Serial.println(parsedData.pos_z);

  Serial.print("Voltage: "); Serial.println(parsedData.voltage);
  Serial.print("Valid Node Quantity: "); Serial.println(parsedData.validNodeQuantity);

  // Print anchor node data
  Serial.println("\n--- Anchor Nodes ---");
  for (int i = 0; i < parsedData.validNodeQuantity && i < 4; i++) {
    Serial.print("Anchor ");
    Serial.print(i + 1);
    Serial.println(":");

    Serial.print("  Role: ");
    Serial.println(parsedData.anchors[i].role);

    Serial.print("  ID: ");
    Serial.println(parsedData.anchors[i].id);

    Serial.print("  Distance (m): ");
    Serial.println(parsedData.anchors[i].distance);

    Serial.print("  FP RSSI (dB): ");
    Serial.println(parsedData.anchors[i].fpRssi);

    Serial.print("  RX RSSI (dB): ");
    Serial.println(parsedData.anchors[i].rxRssi);
  }
  Serial.println("--------------------");
}

void loop() {
  static uint8_t buffer[BUFFER_SIZE];
  static int bufferIndex = 0;
  static bool packetStarted = false;
  static uint16_t frameLength = 0;

  while (Serial1.available()) {
    uint8_t incomingByte = Serial1.read();

    if (!packetStarted) {
      if (incomingByte == START_BYTE) {
        packetStarted = true;
        bufferIndex = 0;
        frameLength = 0;
        buffer[bufferIndex++] = incomingByte; // Store START_BYTE
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
          printParsedData();
        } else {
          Serial.println("Checksum failed.");
        }
        packetStarted = false;
        bufferIndex = 0;
      }
    }
  }
}
