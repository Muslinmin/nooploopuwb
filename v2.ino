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

void sendCoordinates() {
  // Convert the tag's coordinates to int (scaled by 10 to keep one decimal place)
  int tag_x = static_cast<int>(parsedData.pos_x);
  int tag_y = static_cast<int>(parsedData.pos_y);

  // Send the tag's coordinates
  Serial.print(tag_x);
  Serial.print(',');
  Serial.print(tag_y);
  Serial.print(';');
 //A0
  Serial.print(0);
  Serial.print(',');
  Serial.print(0);
  Serial.print(';');
//A1
  Serial.print(0);
  Serial.print(',');
  Serial.print(11);
  Serial.print(';');
//A2
    Serial.print(5);
  Serial.print(',');
  Serial.print(11);
  Serial.print(';');
//A3
    Serial.print(6);
  Serial.print(',');
  Serial.print(0);
  Serial.print(';');

  // End with a newline character
  Serial.println();
}


Position weightedLeastSquares() {
  float anchorRanges[4];
  for (int i = 0; i < 4; i++) {
    anchorRanges[i] = parsedData.anchors[i].distance;
  }
  // Fixed anchor positions
  float anchorPositions[4][2] = {
    {0, 0},         // Anchor 0 at (0, 0)
    {0.485, 11.19}, // Anchor 1 at (0.485, 11.19)
    {5.478, 11.254},// Anchor 2 at (5.478, 11.254)
    {6.866, 0}      // Anchor 3 at (6.866, 0)
  };

  // Anchor distances (ranges)
  float r0 = anchorRanges[0];
  float r1 = anchorRanges[1];
  float r2 = anchorRanges[2];
  float r3 = anchorRanges[3];

  // Fixed weights for each anchor (adjust these if variance-based weights are available)
  float w1 = 0.001777;
  float w2 = 0.001704728;
  float w3 = 0.001193;

  // Set up the system of equations
  float A[3][2] = {
    {2 * (anchorPositions[1][0] - anchorPositions[0][0]), 2 * (anchorPositions[1][1] - anchorPositions[0][1])},
    {2 * (anchorPositions[2][0] - anchorPositions[0][0]), 2 * (anchorPositions[2][1] - anchorPositions[0][1])},
    {2 * (anchorPositions[3][0] - anchorPositions[0][0]), 2 * (anchorPositions[3][1] - anchorPositions[0][1])}
  };

  float b[3] = {
    r0 * r0 - r1 * r1 - anchorPositions[0][0] * anchorPositions[0][0] + anchorPositions[1][0] * anchorPositions[1][0]
    - anchorPositions[0][1] * anchorPositions[0][1] + anchorPositions[1][1] * anchorPositions[1][1],
    
    r0 * r0 - r2 * r2 - anchorPositions[0][0] * anchorPositions[0][0] + anchorPositions[2][0] * anchorPositions[2][0]
    - anchorPositions[0][1] * anchorPositions[0][1] + anchorPositions[2][1] * anchorPositions[2][1],
    
    r0 * r0 - r3 * r3 - anchorPositions[0][0] * anchorPositions[0][0] + anchorPositions[3][0] * anchorPositions[3][0]
    - anchorPositions[0][1] * anchorPositions[0][1] + anchorPositions[3][1] * anchorPositions[3][1]
  };

  // Compute the weighted least squares solution
  float W[3][3] = {{w1, 0, 0}, {0, w2, 0}, {0, 0, w3}};
  
  // A' * W
  float ATW[2][3];
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 3; j++) {
      ATW[i][j] = A[0][i] * W[j][0] + A[1][i] * W[j][1] + A[2][i] * W[j][2];
    }
  }

  // A' * W * A
  float ATWA[2][2];
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      ATWA[i][j] = ATW[i][0] * A[0][j] + ATW[i][1] * A[1][j] + ATW[i][2] * A[2][j];
    }
  }

  // A' * W * b
  float ATWb[2] = {ATW[0][0] * b[0] + ATW[0][1] * b[1] + ATW[0][2] * b[2],
                   ATW[1][0] * b[0] + ATW[1][1] * b[1] + ATW[1][2] * b[2]};

  // Invert 2x2 matrix ATWA to solve for x = (A' * W * A)^-1 * (A' * W * b)
  float det = ATWA[0][0] * ATWA[1][1] - ATWA[0][1] * ATWA[1][0];
  if (det == 0) {
    // Handle the case where the determinant is zero (no unique solution)
    return {0, 0}; // Return a default value (or handle error as needed)
  }

  float invATWA[2][2] = {{ ATWA[1][1] / det, -ATWA[0][1] / det},
                         {-ATWA[1][0] / det,  ATWA[0][0] / det}};

  // Final tag position (x, y)
  Position tagPosition;
  tagPosition.x = invATWA[0][0] * ATWb[0] + invATWA[0][1] * ATWb[1];
  tagPosition.y = invATWA[1][0] * ATWb[0] + invATWA[1][1] * ATWb[1];

  return tagPosition;
}



void sendExtendedData() {
  // Convert the tag's coordinates to int (scaled by 10 to keep one decimal place)
  // Position tagPosition = weightedLeastSquares();
  // int tag_x = tagPosition.x;
  // int tag_y = tagPosition.y;
  int tag_x = static_cast<int>(parsedData.pos_x * 10);
  int tag_y = static_cast<int>(parsedData.pos_y * 10);

  // Send the tag's position
  Serial.print(tag_x);
  Serial.print(',');
  Serial.print(tag_y);
  Serial.print(';');

  // Send each anchor's data: distance, fpRssi, and rxRssi
  for (int i = 0; i < parsedData.validNodeQuantity && i < 4; i++) {
    // Convert each anchor's data to int (scaled by 10 to keep one decimal place)
    int anchor_distance = static_cast<int>(parsedData.anchors[i].distance * 10);
    int anchor_fpRssi = static_cast<int>(parsedData.anchors[i].fpRssi * 10);
    int anchor_rxRssi = static_cast<int>(parsedData.anchors[i].rxRssi * 10);

    Serial.print(anchor_distance);
    Serial.print(',');
    Serial.print(anchor_fpRssi);
    Serial.print(',');
    Serial.print(anchor_rxRssi);

    // Add a semicolon after each anchor data except the last one
    if (i < parsedData.validNodeQuantity - 1) {
      Serial.print(';');
    }
  }

  // End with a newline character
  Serial.println();
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
          // printParsedData();
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
}
