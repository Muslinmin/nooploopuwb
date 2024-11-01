#define START_BYTE 0x55
#define FUNCTION_MARK_BYTE 0x04
#define BUFFER_SIZE 256

// Data structure to hold the parsed values
struct NLinkData {
  uint8_t frameHeader;
  uint8_t functionMark;
  uint16_t frameLength;
  uint8_t role;
  uint8_t id;
  uint32_t systemTime;
  float eop_x, eop_y, eop_z;
  float pos_x, pos_y, pos_z;
  float vel_x, vel_y, vel_z;
  float gyro_x, gyro_y, gyro_z;
  float acc_x, acc_y, acc_z;
  float angle_x, angle_y, angle_z;
  float q0, q1, q2, q3;
  uint32_t localTime;
  float voltage;
  uint8_t validNodeQuantity;
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

  parsedData.localTime = buffer[offset] | (buffer[offset + 1] << 8) | (buffer[offset + 2] << 16) | (buffer[offset + 3] << 24);
  offset += 4;

  parsedData.voltage = ((int16_t)(buffer[offset] | (buffer[offset + 1] << 8))) / 1000.0f;
  offset += 2;

  parsedData.validNodeQuantity = buffer[offset++];
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

  // Serial.println("\n--- Velocity ---");
  // Serial.print("Vel X: "); Serial.println(parsedData.vel_x);
  // Serial.print("Vel Y: "); Serial.println(parsedData.vel_y);
  // Serial.print("Vel Z: "); Serial.println(parsedData.vel_z);

  // Serial.println("\n--- Gyroscope ---");
  // Serial.print("Gyro X: "); Serial.println(parsedData.gyro_x);
  // Serial.print("Gyro Y: "); Serial.println(parsedData.gyro_y);
  // Serial.print("Gyro Z: "); Serial.println(parsedData.gyro_z);

  // Serial.println("\n--- Acceleration ---");
  // Serial.print("Acc X: "); Serial.println(parsedData.acc_x);
  // Serial.print("Acc Y: "); Serial.println(parsedData.acc_y);
  // Serial.print("Acc Z: "); Serial.println(parsedData.acc_z);

  // Serial.println("\n--- Angle ---");
  // Serial.print("Angle X: "); Serial.println(parsedData.angle_x);
  // Serial.print("Angle Y: "); Serial.println(parsedData.angle_y);
  // Serial.print("Angle Z: "); Serial.println(parsedData.angle_z);

  // Serial.println("\n--- Quaternion ---");
  // Serial.print("Q0: "); Serial.println(parsedData.q0);
  // Serial.print("Q1: "); Serial.println(parsedData.q1);
  // Serial.print("Q2: "); Serial.println(parsedData.q2);
  // Serial.print("Q3: "); Serial.println(parsedData.q3);

  // Serial.print("Local Time: "); Serial.println(parsedData.localTime);
  Serial.print("Voltage: "); Serial.println(parsedData.voltage);
  Serial.print("Valid Node Quantity: "); Serial.println(parsedData.validNodeQuantity);
  Serial.println("--------------------");
}

void loop() {
  static uint8_t buffer[BUFFER_SIZE];
  static int bufferIndex = 0;
  static bool packetStarted = false;
  static uint16_t frameLength = 0;

  // Check if data is available on Serial1 (from RX)
  while (Serial1.available()) {
    uint8_t incomingByte = Serial1.read();

    // Look for the start of the packet
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
