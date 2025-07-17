#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Joy‑Con → PC → UART → Arduino 封包轉換器
依賴：
  pip install evdev pyserial
用法（兩種擇一）：
  1. 互動式：python3 joycon_to_pc_v2.py
  2. 參數式：python3 joycon_to_pc_v2.py --event 6 --port /dev/ttyUSB0
"""

import argparse, json, time, sys
import serial, serial.tools.list_ports
import evdev
from evdev import InputDevice, categorize, ecodes

# ---------- 協定常數 ----------
HDR     = bytes((0xAA, 0x55))
PAYLOAD = 6
BAUD    = 115200
DEBOUNCE = 0.01          # 10 ms

# ---------- 通用工具 ----------
def normalize(val: int, in_min: int, in_max: int) -> int:
    """把 val 從 [in_min, in_max] 線性壓到 0 – 255。range=0 則回 127。"""
    rng = in_max - in_min
    if rng == 0:
        return 127
    val = max(min(val, in_max), in_min)
    return int((val - in_min) * 255 / rng)

def axis_range(dev: InputDevice, code: int,
               default=(-32768, 32767)) -> tuple[int, int]:
    """讀取單一軸 min/max；若兩者相等則回傳預設值。"""
    ai = dev.absinfo(code)
    return (ai.min if ai.min != ai.max else default[0],
            ai.max if ai.min != ai.max else default[1])

def build_packet(btns: int, lx: int, ly: int, rx: int, ry: int) -> bytes:
    payload = bytearray((
        btns & 0xFF,
        (btns >> 8) & 0xFF,
        lx, ly, rx, ry
    ))
    checksum = (sum(payload) & 0xFF)
    return HDR + bytes((PAYLOAD,)) + payload + bytes((checksum,))

# ---------- 互動選單 ----------
def choose_event() -> str:
    devs = list(map(evdev.InputDevice, evdev.list_devices()))
    print("\n所有輸入裝置：")
    for d in devs:
        print(f"  {d.path} ➜ {d.name}")
    path = input("\n輸入 Joy‑Con 的 event 代號（例如 27 代表 /dev/input/event27）：")
    return f"/dev/input/event{path.strip()}"

def choose_serial() -> str:
    ports = list(serial.tools.list_ports.comports())
    print("\n可用的 USB 裝置：")
    for p in ports:
        print(f"  {p.device: <15} ({p.description})")
    return input("輸入 Arduino 所在埠 (預設 /dev/ttyACM0)：").strip() or "/dev/ttyACM0"

# ---------- 主流程 ----------
def main():
    # ------- 參數解析 -------
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--event", type=str, help="Joy‑Con event 編號或完整路徑")
    ap.add_argument("--port",  type=str, help="Arduino 串列埠")
    args, _ = ap.parse_known_args()

    event_path = (f"/dev/input/event{args.event}"
                  if args.event and args.event.isdigit()
                  else args.event) if args.event else choose_event()

    try:
        joycon = InputDevice(event_path)
    except FileNotFoundError:
        sys.exit(f"[錯誤] 找不到 {event_path}")

    side = 'L' if 'Left' in joycon.name or '(L' in joycon.name else 'R'
    with open(f"key_mapping_{side}.json", encoding="utf-8") as fp:
        mapping: dict[str, int] = json.load(fp)

    port = args.port or choose_serial()
    try:
        ser = serial.Serial(port, BAUD, timeout=0)
    except serial.SerialException as e:
        sys.exit(f"[錯誤] 開啟 {port} 失敗：{e}")

    print(f"\n✔ Joy‑Con 連線：{joycon.name}（{side}）")
    print(f"✔ UART 已開啟：{port} @ {BAUD}\n按 Ctrl+C 結束。\n")

    # ------- 軸範圍 -------
    lx_min, lx_max = axis_range(joycon, ecodes.ABS_X)
    ly_min, ly_max = axis_range(joycon, ecodes.ABS_Y)
    rx_min, rx_max = axis_range(joycon, ecodes.ABS_RX)
    ry_min, ry_max = axis_range(joycon, ecodes.ABS_RY)

    buttons = 0
    lx = ly = rx = ry = 127
    last_send = 0.0

    for ev in joycon.read_loop():
        if ev.type == ecodes.EV_KEY:
            key = categorize(ev).keycode
            if isinstance(key, (list, tuple)):
                key = key[0]
            bit = mapping.get(str(key))
            if bit is not None:
                if ev.value:
                    buttons |= 1 << bit
                else:
                    buttons &= ~(1 << bit)

        elif ev.type == ecodes.EV_ABS:
            code, val = ev.code, ev.value
            if   code == ecodes.ABS_X : lx = normalize(val,  lx_min, lx_max)
            elif code == ecodes.ABS_Y : ly = normalize(-val, ly_min, ly_max)
            elif code == ecodes.ABS_RX: rx = normalize(val,  rx_min, rx_max)
            elif code == ecodes.ABS_RY: ry = normalize(-val, ry_min, ry_max)

        now = time.time()
        if now - last_send >= DEBOUNCE:
            ser.write(build_packet(buttons, lx, ly, rx, ry))
            last_send = now

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[結束] 已離開程式。")
