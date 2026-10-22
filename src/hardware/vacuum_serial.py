"""Exclusive target-local p/d/s vacuum protocol. No I/O until explicit open.

Command transmission is not pressure or successful-grasp confirmation.
"""
from pathlib import Path
import time
import math
from hardware.servo_transport import require_device_available,open_exclusive_serial


class SerialVacuum:
    def __init__(self,device):
        self.device=Path(device);self.serial=None;self.suction_requested=False

    def open(self):
        from hardware import devices
        named=devices.configured('pump_port')      # 직접 지정한 경로면 CP2102_0001 이름 검사를 하지 않는다
        if (not devices.stable_serial(self.device,named)
                or (not named and ('CP2102' not in self.device.name or '_0001-' not in self.device.name))):
            raise ValueError(f'expected stable by-id/by-path pump device required: {self.device} (ARM_PUMP_PORT로 지정 가능)')
        if self.serial is not None:raise RuntimeError('pump already open')
        require_device_available(self.device.resolve())
        import serial
        self.serial=open_exclusive_serial(serial,self.device,115200)
        try:
            self._wait_ready()
            self.stop()
        except BaseException:
            self.serial.close();self.serial=None
            raise
        return self

    def _wait_ready(self,timeout_s=2.5):
        """포트를 열면 ESP32가 재부팅된다. 예전에는 무조건 2초를 기다렸다. 이제 식별 명령(q)에 답이 오면 바로 넘어간다
        (펌웨어 setup에는 지연이 없다). 답이 없으면 예전처럼 시간을 채우고 넘어간다(응답 확인은 원래 하지 않았다)."""
        deadline=time.monotonic()+timeout_s;reply=bytearray();next_query=0.
        while time.monotonic()<deadline:
            if time.monotonic()>=next_query:
                self._send(b'q');next_query=time.monotonic()+.1
            count=self.serial.in_waiting
            if count:
                reply.extend(self.serial.read(min(count,4096)))
                if b'ESP32_VACUUM' in reply:
                    self.serial.reset_input_buffer()
                    return True
                del reply[:-64]
            time.sleep(.01)
        return False

    def _send(self,command):
        if self.serial is None:raise RuntimeError('pump is not open')
        if self.serial.write(command)!=len(command):raise OSError('short pump command write')
        self.serial.flush()

    def suction(self,*,require_ack=False,timeout_s=1.):
        if not math.isfinite(timeout_s) or not 0<timeout_s<=5:
            raise ValueError('bounded positive suction response timeout required')
        if require_ack:
            if self.serial is None:raise RuntimeError('pump is not open')
            self.serial.reset_input_buffer()  # stale startup/previous replies cannot acknowledge this command
        self._send(b'p');self.suction_requested=True
        if not require_ack:return None
        reply=bytearray();deadline=time.monotonic()+timeout_s
        expected='[동작] 물건 흡착 (Pick)'.encode('utf-8')
        try:
            while time.monotonic()<deadline:
                count=self.serial.in_waiting
                if count:
                    reply.extend(self.serial.read(min(count,4096-len(reply))))
                    if expected in reply:
                        return {'controller_acknowledged':True,
                                'reply':bytes(reply).decode('utf-8',errors='replace'),
                                'pressure_verified':False}
                    if len(reply)>=4096:raise OSError('unexpected pump response too long')
                time.sleep(.01)
            raise TimeoutError('pump did not acknowledge suction; lifting refused')
        except BaseException:
            self.stop()
            raise

    def vent(self):
        self._send(b'd');self.suction_requested=False

    def stop(self):
        self._send(b's');self.suction_requested=False

    def close(self):
        if self.serial is not None:
            try:self.stop()
            finally:self.serial.close();self.serial=None

    def __enter__(self):return self.open()
    def __exit__(self,*args):self.close()
