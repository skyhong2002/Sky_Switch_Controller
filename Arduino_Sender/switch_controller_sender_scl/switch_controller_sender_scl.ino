#include <SwitchControlLibrary.h>

/* ——— 封包常數 ——— */
const uint8_t HDR0 = 0xAA, HDR1 = 0x55, PLEN = 6;

/* ——— 解析暫存 ——— */
uint8_t buf[PLEN];
uint8_t idx = 0, crc = 0;
enum State { WAIT0, WAIT1, WAITLEN, WAIT_PAYLOAD, WAIT_CRC } state = WAIT0;

/* ——— 前次狀態 ——— */
uint16_t prevBtn = 0;
uint8_t  prevHat = 0;

/* ======================================================
   這裡是「封包驗證通過」後的解析
   ====================================================== */
void handlePacket() {
  uint16_t btn = buf[0] | (uint16_t(buf[1]) << 8);

  /* ① 取得 SwitchControlLibrary 單例參考 --------------------- */
  auto& ctl = SwitchControlLibrary();   // ←★ 新增

  /* ② 十字鍵 → HatButton bit field -------------------------- */
  uint8_t hatBits = 0;
  if (btn & (1 <<  8)) hatBits |= HatButton::UP;
  if (btn & (1 <<  9)) hatBits |= HatButton::DOWN;
  if (btn & (1 << 10)) hatBits |= HatButton::LEFT;
  if (btn & (1 << 11)) hatBits |= HatButton::RIGHT;

  /* ③ 其他按鍵（差異比較） ----------------------------------- */
  uint16_t diff = btn ^ prevBtn;
  auto btnSync = [&](uint16_t mask, uint16_t code) {
    if (diff & mask) {
      (btn & mask) ? ctl.pressButton(code)
                   : ctl.releaseButton(code);
    }
  };

  btnSync(1 << 0,  Button::Y);
  btnSync(1 << 1,  Button::B);
  btnSync(1 << 2,  Button::X);
  btnSync(1 << 3,  Button::A);
  btnSync(1 << 4,  Button::L);
  btnSync(1 << 5,  Button::ZL);
  btnSync(1 << 6,  Button::MINUS);
  btnSync(1 << 7,  Button::LCLICK);
  btnSync(1 << 12, Button::R);
  btnSync(1 << 13, Button::ZR);

  prevBtn = btn & ~(0x0F << 8);   // 不記錄十字鍵那 4 bit

  /* ④ 十字鍵同步 -------------------------------------------- */
  uint8_t hatDiff = hatBits ^ prevHat;
  auto hatSync = [&](uint8_t m) {
    if (hatDiff & m) {
      (hatBits & m) ? ctl.pressHatButton(m)
                    : ctl.releaseHatButton(m);
    }
  };
  hatSync(HatButton::UP);
  hatSync(HatButton::DOWN);
  hatSync(HatButton::LEFT);
  hatSync(HatButton::RIGHT);
  prevHat = hatBits;

  /* ⑤ 搖桿 & 送出報表 ---------------------------------------- */
  ctl.moveLeftStick (buf[2], buf[3]);
  ctl.moveRightStick(buf[4], buf[5]);
  ctl.sendReport();
}

/* ---------------- 主迴圈：逐位元組解析 ---------------- */
void setup() { Serial.begin(115200); }

void loop() {
  while (Serial.available()) {
    uint8_t b = Serial.read();

    switch (state) {
      case WAIT0:
        state = (b == HDR0) ? WAIT1 : WAIT0;
        break;

      case WAIT1:
        state = (b == HDR1) ? WAITLEN : WAIT0;
        break;

      case WAITLEN:
        if (b == PLEN) { idx = crc = 0; state = WAIT_PAYLOAD; }
        else            state = WAIT0;
        break;

      case WAIT_PAYLOAD:
        buf[idx++] = b; crc += b;
        if (idx >= PLEN) state = WAIT_CRC;
        break;

      case WAIT_CRC:
        if (crc == b) handlePacket();   // 驗證通過才處理
        state = WAIT0;
        break;
    }
  }
}