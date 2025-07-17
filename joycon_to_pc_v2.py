#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
將 Joy‑Con 事件轉封包，透過 USB‑TTL 送到 Arduino。
依賴：
  pip install evdev pyserial
使用方法：
  1. 確認 Joy‑Con 已以 Bluetooth 方式連到 PC。
  2. python3 joycon_to_pc_v2.py
"""

import json, time, serial, serial.tools.list_ports
import evdev
from evdev import InputDevice, categorize, ecodes

# ---------- 協定常數 ----------
HDR     = bytes([0xAA, 0x55])
PAYLOAD = 6                 # 固定長度
BAUD    = 115200

# ---------- 工具函式 ----------
def normalize(val, in_min, in_max):
    """將原始 -32768~32767 座標壓成 0~255，並做飽和保護"""
    val = max(min(val, in_max), in_min)
    return int((val - in_min) * 255 / (in_max - in_min))

def build_packet(btns, lx, ly, rx, ry):
    payload = bytearray([
        btns & 0xFF,
        (btns >> 8) & 0xFF,
        lx, ly, rx, ry
    ])
    checksum = sum(payload) & 0xFF
    return HDR + bytes([PAYLOAD]) + payload + bytes([checksum])

# ---------- 主程式 ----------
def main():
    # 1. 選 Joy‑Con input device
    devices = {d.path: d.name for d in map(evdev.InputDevice, evdev.list_devices())}
    print("\n所有輸入裝置：")
    for p, n in devices.items():
        print(f"  {p} ➜ {n}")
    path = input("\n輸入 Joy‑Con 的 event 代號（例如 27 代表 /dev/input/event27）：")
    joycon = InputDevice(f"/dev/input/event{path}")
    side   = 'L' if 'Left' in joycon.name or 'Joy-Con (L' in joycon.name else 'R'
    mapping = json.load(open(f"key_mapping_{side}.json", encoding='utf-8'))
    print(f"已連線：{joycon.name}（{side}）")

    # 2. 選串列埠
    print("\n可用的 USB 裝置：")
    for p in serial.tools.list_ports.comports():
        print(f"  {p.device} ({p.description})")
    port = input("輸入 Arduino 所在埠 (預設 /dev/ttyACM0)：") or "/dev/ttyACM0"
    ser  = serial.Serial(port, BAUD, timeout=0)

    # 3. 狀態變數
    buttons = 0
    lx = ly = rx = ry = 127

    abs_range = joycon.absinfo(ecodes.ABS_X)  # 同一支 Joy‑Con 各軸 range 相同
    in_min, in_max = abs_range.min, abs_range.max

    last_send = 0
    DEBOUNCE  = 0.01       # 10 ms

    print("\n開始監聽，Ctrl+C 結束。")
    for ev in joycon.read_loop():
        if ev.type == ecodes.EV_KEY:
            key = categorize(ev).keycode
            # evdev 有時回傳 tuple；取第一個 alias
            if isinstance(key, (list, tuple)):
                key = key[0]
            bit = mapping.get(str(key))
            if bit is not None:
                if ev.value: buttons |=  (1 << bit)
                else:        buttons &= ~(1 << bit)

        elif ev.type == ecodes.EV_ABS:
            code = ev.code
            val  = ev.value
            if   code == ecodes.ABS_X : lx = normalize(val,  in_min, in_max)
            elif code == ecodes.ABS_Y : ly = normalize(-val, in_min, in_max)
            elif code == ecodes.ABS_RX: rx = normalize(val,  in_min, in_max)
            elif code == ecodes.ABS_RY: ry = normalize(-val, in_min, in_max)

        # 每 DEBOUNCE 秒或有事件就送一次
        now = time.time()
        if now - last_send >= DEBOUNCE:
            ser.write(build_packet(buttons, lx, ly, rx, ry))
            last_send = now

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n離開程式，串列埠已關閉。")
