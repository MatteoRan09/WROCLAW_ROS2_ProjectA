void printBeaconStatus(){
  Serial2.write(0x32);
  Serial2.write(0x00);
  delay(100);
  // Read and print response
  if (Serial2.available() > 0) {
    Serial.println("Response received:");
    // Should see: 40h 01h 00h 5Ah ...
    while (Serial2.available() > 0) {
      byte response = Serial2.read();
      if (response < 0x10) Serial.print("0");
      Serial.print(response, HEX);
      Serial.print("\h");
      Serial.print(" ");
    }
  } else {
    Serial.print("No response received");
    delay(5000);
  }
}



void ReadLocation(){
  Serial2.write(0x02);
  Serial2.write(0x00);

  // for pos get write 0x02 0x00
  
  // Wait for response
  delay(100);
  
  // Read and print response
  if (Serial2.available() >= 18) {
    byte buffer[18];
    
    // Read the full packet into a buffer
    Serial2.readBytes(buffer, 18);

    // --- PARSE PACKET 1: ERROR CODE ---
    // Byte 0: Type (0x40)
    // Byte 1: Length (0x01)
    // Byte 2: Error Code
    byte errorCode = buffer[2];

    // --- PARSE PACKET 2: POSITION ---
    // Byte 3: Type (0x41)
    // Byte 4: Length (0x0D -> 13)
    if (errorCode == 0){
      // Bytes 5-8: X Coordinate (32-bit Little Endian)
      x_mm = (int32_t)buffer[5] | 
                      ((int32_t)buffer[6] << 8) | 
                      ((int32_t)buffer[7] << 16) | 
                      ((int32_t)buffer[8] << 24);

      // Bytes 9-12: Y Coordinate
      y_mm = (int32_t)buffer[9] | 
                      ((int32_t)buffer[10] << 8) | 
                      ((int32_t)buffer[11] << 16) | 
                      ((int32_t)buffer[12] << 24);

      // Bytes 13-16: Z Coordinate
      z_mm = (int32_t)buffer[13] | 
                      ((int32_t)buffer[14] << 8) | 
                      ((int32_t)buffer[15] << 16) | 
                      ((int32_t)buffer[16] << 24);

      // Byte 17: Quality
      quality = buffer[17];
    }
  }
}