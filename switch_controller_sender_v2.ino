/*
 * 讀取 PC 送來的 Joy‑Con 封包並轉成 Switch 支援的 HID。
 * 需要安裝 NintendoSwitchController library。
 */
#include "NintendoSwitchController.h"

NintendoSwitchController switchCtrl;

// ---------- 協定常數 ----------
const uint8_t HDR0 = 0xAA;
const uint8_t HDR1 = 0x55;
const uint8_t PLEN = 6;          // 固定長度

// ---------- 解析狀態 ----------
enum State { WAIT_HDR0, WAIT_HDR1, WAIT_LEN, WAIT_PAYLOAD, WAIT_CRC };
State state = WAIT_HDR0;
uint8_t buf[PLEN];
uint8_t idx = 0;
uint8_t crc = 0;

void setup() {
  Serial.begin(115200);        // 與 PC 連線
  switchCtrl.begin();          // 初始化 HID
}

void loop() {
  while (Serial.available()) {
    uint8_t b = Serial.read();
    switch (state) {

      case WAIT_HDR0:
        if (b == HDR0) state = WAIT_HDR1;
        break;

      case WAIT_HDR1:
        state = (b == HDR1) ? WAIT_LEN : WAIT_HDR0;
        break;

      case WAIT_LEN:
        if (b == PLEN) { idx = 0; crc = 0; state = WAIT_PAYLOAD; }
        else state = WAIT_HDR0;    // 長度錯誤重新同步
        break;

      case WAIT_PAYLOAD:
        buf[idx++] = b;
        crc += b;
        if (idx >= PLEN) state = WAIT_CRC;
        break;

      case WAIT_CRC:
        if (crc == b) handlePacket();   // 驗證通過
        state = WAIT_HDR0;              // 無論對錯都重新同步
        break;
    }
  }
  switchCtrl.task();   // 必須在 loop 裡呼叫以服務 USB
}

// ---------- 包解析 ----------
void handlePacket() {
  uint16_t btns = buf[0] | (uint16_t(buf[1]) << 8);

  // 左搖桿
  switchCtrl.setLeftStick(buf[2], buf[3]);
  // 右搖桿
  switchCtrl.setRightStick(buf[4], buf[5]);

  // --- 按鍵對映（對照 JSON bit 編號）---
  switchCtrl.releaseAll();          // 先清空再逐一判斷

  auto pressIf = [&](bool c, SwitchButton s){ if(c) switchCtrl.pressButton(s); };

  pressIf(btns & (1<<0),  ButtonY);
  pressIf(btns & (1<<1),  ButtonB);
  pressIf(btns & (1<<2),  ButtonX);
  pressIf(btns & (1<<3),  ButtonA);
  pressIf(btns & (1<<4),  ButtonL);
  pressIf(btns & (1<<5),  ButtonZL);
  pressIf(btns & (1<<6),  ButtonMinus);
  pressIf(btns & (1<<7),  ButtonSelect);   // d‑pad 按下預留
  pressIf(btns & (1<<8),  ButtonUp);
  pressIf(btns & (1<<9),  ButtonDown);
  pressIf(btns & (1<<10), ButtonLeft);
  pressIf(btns & (1<<11), ButtonRight);
  pressIf(btns & (1<<12), ButtonR);
  pressIf(btns & (1<<13), ButtonZR);

  switchCtrl.sendUpdate();          // 送出 HID
}
