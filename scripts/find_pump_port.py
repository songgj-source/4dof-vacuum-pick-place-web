"""꽂힌 CP2102 시리얼 장치 중 어느 것이 펌프 ESP32인지 찾아 USB 포트 위치 이름(/dev/serial/by-path)을 알려 준다.

팀 라이다(LD 계열)와 펌프 ESP32가 둘 다 CP2102 일련번호 '_0001'이라 /dev/serial/by-id 이름이 하나만 생긴다.
그래서 둘을 한 Pi에 꽂으면 포트 위치 이름으로 나눠 지정해야 한다:
  우리 팔   ARM_PUMP_PORT=/dev/serial/by-path/...  (pi/arm-api.env)
  팀 라이다 ros2 launch car2_bringup robot_lidar.launch.py lidar_port:=/dev/serial/by-path/...

  python scripts/find_pump_port.py           # 찾기만
  python scripts/find_pump_port.py --write   # 찾은 펌프 경로를 pi/arm-api.env의 ARM_PUMP_PORT에 적는다

각 장치를 115200으로 열어 식별 명령 'q'를 보내고 'ESP32_VACUUM' 답을 기다린다. 펌프 ESP32는 열 때 재부팅되고
마지막에 정지('s')를 받으므로 펌프는 꺼진 채로 끝난다. 라이다는 받기만 하는 장치라 'q'는 무시된다.
다른 프로그램(라이다 노드, 팔 작업)이 쓰는 중인 포트는 건드리지 않고 '사용 중'으로 표시한다.
꽂은 USB 포트를 바꾸면 by-path 이름이 바뀌므로 다시 실행해야 한다.
"""
import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from hardware.servo_transport import open_exclusive_serial  # noqa: E402

BY_PATH = Path('/dev/serial/by-path')
CP2102 = ('10c4', 'ea60')
ENV_FILE = ROOT / 'pi' / 'arm-api.env'


def usb_id(tty):
    """ttyUSB0 -> (vendor, product) from sysfs, or None."""
    node = Path('/sys/class/tty') / tty / 'device'
    for parent in [node.resolve(), *node.resolve().parents]:
        vendor, product = parent / 'idVendor', parent / 'idProduct'
        if vendor.exists() and product.exists():
            return vendor.read_text().strip(), product.read_text().strip()
    return None


def cp2102_ports():
    if not BY_PATH.exists():
        return []
    ports = {}
    for path in sorted(BY_PATH.iterdir()):            # 같은 장치가 'usb-'와 'usbv2-' 두 이름으로 나온다 -> 장치 하나로 센다
        if usb_id(path.resolve().name) == CP2102 and path.resolve() not in ports:
            ports[path.resolve()] = path
    return list(ports.values())


def is_pump(path, timeout_s=2.5):
    """True = 펌프 ESP32, False = 다른 장치, None = 다른 프로그램이 사용 중."""
    import serial
    try:
        port = open_exclusive_serial(serial, path, 115200)
    except (OSError, serial.SerialException):
        return None
    try:
        deadline, reply, next_query = time.monotonic() + timeout_s, bytearray(), 0.
        while time.monotonic() < deadline:
            if time.monotonic() >= next_query:
                port.write(b'q')
                next_query = time.monotonic() + .1
            if port.in_waiting:
                reply.extend(port.read(min(port.in_waiting, 4096)))
                if b'ESP32_VACUUM' in reply:
                    port.write(b's')                  # 펌프 정지(켜져 있지 않아도 그대로 꺼진 상태)
                    return True
                del reply[:-64]
            time.sleep(.01)
        return False
    finally:
        port.close()


def write_env(path):
    lines = ENV_FILE.read_text().splitlines() if ENV_FILE.exists() else []
    line = f'ARM_PUMP_PORT={path}'
    lines = [line if l.startswith('ARM_PUMP_PORT=') else l for l in lines]
    if line not in lines:
        lines.append(line)
    ENV_FILE.write_text('\n'.join(lines) + '\n')
    ENV_FILE.chmod(0o600)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--write', action='store_true', help=f'찾은 펌프 경로를 {ENV_FILE.relative_to(ROOT)}에 적는다')
    args = parser.parse_args()
    ports = cp2102_ports()
    if not ports:
        print('CP2102 장치가 없습니다(/dev/serial/by-path). 펌프 ESP32가 꽂혀 있는지 확인하세요.')
        return 1
    pumps, others = [], []
    for path in ports:
        found = is_pump(path)
        label = {True: '펌프 ESP32', False: '다른 장치(라이다 등)', None: '사용 중 - 확인 못 함(라이다 노드·팔 작업이 쓰는 중일 수 있음)'}[found]
        print(f'{label:<12}  {path}  -> {path.resolve().name}')
        (pumps if found else others).append((path, found))
    if len(pumps) != 1:
        print('펌프를 하나로 정하지 못했습니다.' + (' 사용 중인 포트가 있으면 그 프로그램을 잠시 끄고 다시 실행하세요.' if any(f is None for _, f in others) else ''))
        return 1
    pump = pumps[0][0]
    print(f'\n우리 팔:   ARM_PUMP_PORT={pump}')
    for path, found in others:
        if found is False:
            print(f'팀 라이다: lidar_port:={path}')
    if args.write:
        write_env(pump)
        print(f'{ENV_FILE} 에 적었습니다. 서버를 다시 켜세요: bash pi/arm_api_ctl.sh restart')
    return 0


if __name__ == '__main__':
    sys.exit(main())
