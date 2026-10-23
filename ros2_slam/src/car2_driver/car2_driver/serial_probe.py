"""LD 계열 라이다가 꽂힌 시리얼 포트를 찾는다(ld_lidar_node의 port:=auto). rclpy 없이 쓴다.

라이다와 팔의 펌프 ESP32가 둘 다 CP2102 일련번호 '_0001'이라 /dev/serial/by-id 이름이 하나만 생기고,
어느 장치를 가리킬지는 꽂은 순서에 따라 바뀐다(2026-10-22 라이다 노드가 펌프를 잡아 팔이 펌프를 못 엶).
그래서 이름 대신 내용으로 고른다: 라이다는 명령 없이 230400 baud로 47바이트 묶음(0x54 0x2C로 시작)을 계속 보낸다.
다른 프로그램이 쓰는 포트(팔 작업 중인 펌프 등)는 배타적으로 열리지 않으므로 건너뛴다.
"""
import glob
import os
import time

HEADER = b'\x54\x2c'
PACKET = 47


def looks_like_ld(data, need=3):
    """받은 바이트에 47바이트 간격으로 이어지는 라이다 묶음 머리가 need번 이상 있으면 True."""
    hits, i = 0, data.find(HEADER)
    while 0 <= i and i + PACKET + len(HEADER) <= len(data):
        if data[i + PACKET:i + PACKET + len(HEADER)] == HEADER:
            hits += 1
            if hits >= need:
                return True
        i = data.find(HEADER, i + 1)
    return False


def candidates():
    """USB 시리얼(ttyUSB*)의 포트 위치 이름. 같은 장치의 usb-/usbv2- 두 이름은 하나로 센다."""
    seen, ports = set(), []
    for path in sorted(glob.glob('/dev/serial/by-path/*')):
        real = os.path.realpath(path)
        if 'ttyUSB' in real and real not in seen:
            seen.add(real)
            ports.append(path)
    return ports


def find_ld_lidar(baud=230400, listen_s=1.0, ports=None, opener=None):
    """라이다 신호가 나오는 첫 포트의 이름, 없으면 None."""
    if opener is None:
        import serial
        opener = lambda port: serial.Serial(port, baud, timeout=0.2, exclusive=True)   # noqa: E731
    for port in candidates() if ports is None else ports:
        try:
            link = opener(port)
        except Exception:            # 다른 프로그램이 쓰는 중이거나 권한 없음
            continue
        try:
            data, deadline = bytearray(), time.monotonic() + listen_s
            while time.monotonic() < deadline and len(data) < 4000:
                data += link.read(512)
            if looks_like_ld(bytes(data)):
                return port
        finally:
            link.close()
    return None
