const byte SYNC_BYTE = 0xAA;

void setup() {
  Serial.begin(115200);    // 接 PC 的 Serial
  Serial1.begin(115200);   // 接 Arduino B 的 Serial
}

void loop() {
  if (Serial.available() >= 7) {  // 確保收到一包完整的 7 Bytes
    uint8_t buffer[7];
    Serial.readBytes(buffer, 7);

    // 計算接收到的資料的 checksum
    uint8_t checksum = 0;
    for (int i = 0; i < 6; i++) {
      checksum += buffer[i];
    }
    checksum &= 0xFF;

    // 檢查 checksum 是否正確
    if (checksum == buffer[6]) {
      // 先送出同步字
      Serial1.write(SYNC_BYTE);

      // 再送出包含 checksum 的 buffer
      Serial1.write(buffer, 7);
    } else {
      // 如果 checksum 錯誤，可以選擇忽略或回報錯誤
      Serial.println("Checksum error, packet discarded.");
    }
  }
}
