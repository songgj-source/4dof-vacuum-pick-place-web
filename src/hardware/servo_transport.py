"""Target-local, exclusive Feetech transport; no I/O until open()."""
import fcntl
import hashlib
import os
from pathlib import Path
import subprocess
import termios
import time


def require_device_available(device, allow_self=False):
    """Fail before SDK access when any existing process owns this TTY."""
    result = subprocess.run(['fuser',str(Path(device).resolve())],capture_output=True,text=True,timeout=1)
    if result.returncode == 1 and not result.stdout.strip() and not result.stderr.strip():
        return
    if result.returncode != 0:
        raise OSError('cannot establish motor port ownership: '+result.stderr.strip())
    try: owners = {int(x) for x in result.stdout.split()}
    except ValueError as exc:raise OSError('unrecognized motor port ownership report') from exc
    if not owners or owners - ({os.getpid()} if allow_self else set()):
        raise OSError('motor port already owned by another open handle: '+result.stdout.strip())


def claim_kernel_exclusive(fd):
    """Unlike flock, TIOCEXCL also blocks ordinary noncooperative TTY opens."""
    fcntl.ioctl(fd,termios.TIOCEXCL)


def open_exclusive_serial(serial_module, device, baudrate):
    # pyserial3.5 POSIX open calls _reconfigure_port immediately after os.open,
    # before changing termios, modem lines or flushing input. Guard that hook.
    if serial_module.VERSION != '3.5':
        raise OSError('exclusive initialization requires verified pyserial3.5')

    class ExclusiveSerial(serial_module.Serial):
        def open(self):
            self._kernel_owned = False
            return super().open()

        def _reconfigure_port(self, force_update=False):
            if not self._kernel_owned:
                claim_kernel_exclusive(self.fd)
                try:require_device_available(Path(device).resolve(),allow_self=True)
                except BaseException:
                    fcntl.ioctl(self.fd,termios.TIOCNXCL)
                    raise
                self._kernel_owned = True
            return super()._reconfigure_port(force_update=force_update)

    return ExclusiveSerial(str(device),baudrate,bytesize=serial_module.EIGHTBITS,
        timeout=0,exclusive=True)


# Default/hold stays40. In this arm's phase12 fine-speed mode, the old100
# commissioning ceiling was ~8.8deg/s, not the motor's hardware maximum.
MAX_GOAL_SPEED = 1000
MAX_GOAL_ACCELERATION = 32


from motion import guard_mode

# 스트림 시작 때 목표·위치 허용 차이(펄스). 멀리 뻗은 자세는 하중으로 3~6펄스 밀려 서므로 완화 모드(기본)는 8까지 받는다: 첫 목표가 최대 8펄스(약 0.7도) 움직일 뿐이다.
STREAM_START_TOLERANCE = 8 if guard_mode.relaxed() else 2


class ServoTransport:
    def __init__(self, profile):
        self.profile = profile
        self.port = self.packet = self.lock = None
        self.read_retry_count = 0
        self.read_retry_events = []
        self._motion_static = None

    def open(self):
        from hardware import devices
        data = self.profile.data
        device = Path(devices.get('servo_port', data['port']))      # ARM_SERVO_PORT / config/devices.json / 프로필
        named = devices.configured('servo_port')
        if (not devices.stable_serial(device, named)
                or (not named and data['device_identity'] not in device.name)):
            raise ValueError(f'expected stable by-id/by-path servo device is absent: {device} (ARM_SERVO_PORT로 지정 가능)')
        lock_dir = self.profile.root/'tmp/servo_locks'
        lock_dir.mkdir(parents=True, exist_ok=True)
        digest = hashlib.sha256(str(device.resolve()).encode()).hexdigest()[:16]
        self.lock = os.open(lock_dir/(digest+'.lock'), os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
            require_device_available(device.resolve())
            import serial
            from scservo_sdk import PortHandler, PacketHandler
            # The installed SDK's normal setup opens without exclusive=True.
            # Populate its public serial handle using the same framing, exclusively.
            self.port = PortHandler(str(device))
            self.port.baudrate = int(data['baudrate'])
            if self.port.baudrate != 1000000:
                raise ValueError('unexpected bus baudrate')
            self.port.ser = open_exclusive_serial(serial,device,self.port.baudrate)
            self.port.is_open = True
            self.port.tx_time_per_byte = (1000/self.port.baudrate)*10
            self.port.ser.reset_input_buffer()
            self.packet = PacketHandler(0)
        except BaseException:
            self.close()
            raise
        return self

    def _check(self, result, operation):
        if result[-2] != 0 or result[-1] != 0:
            raise OSError(f'{operation}: comm={result[-2]}, servo={result[-1]}')
        return result[0] if len(result) == 3 else None

    def _read_register(self, motor_id, address, size):
        # SDK read1/read2 decode the checksum of a delayed write ACK as data.
        # Use its checksummed raw transaction and validate the reply shape first.
        request = [0,0,motor_id,4,2,address,size,0]
        reply, comm, error = self.packet.txRxPacket(self.port,request)
        # Only a read may be repeated, once, after RX timeout/corruption.
        # Never repeat goal/torque writes or mask a servo alarm/invalid shape.
        if comm in (-6,-7) and error == 0 and getattr(self.port,'ser',None) is not None:
            self.read_retry_count += 1
            self.read_retry_events.append({'id':motor_id,'address':address,'comm':comm})
            self.read_retry_events = self.read_retry_events[-32:]
            time.sleep(.02)
            self.port.ser.reset_input_buffer()
            reply, comm, error = self.packet.txRxPacket(self.port,request)
        self._check((None,comm,error),f'motor{motor_id}/register{address}')
        if (reply is None or len(reply) != size+6
                or reply[:2] != [255,255] or reply[2] != motor_id
                or reply[3] != size+2 or reply[4] != 0):
            raise OSError(f'motor{motor_id}/register{address}: invalid register response')
        return int.from_bytes(bytes(reply[5:5+size]),'little')

    def read(self):
        if self.port is None: raise RuntimeError('transport is not open')
        if self._motion_static is not None:
            return self._read_motion()
        stamp = time.monotonic()
        fields = [('model',3,2),('min_limit',9,2),('max_limit',11,2),('homing',31,2),
                  ('mode',33,1),('torque',40,1),('goal',42,2),('position',56,2),('status',65,1)]
        rows = {}
        for joint in self.profile.joints:
            row = {'id':joint.motor_id}
            for name,address,size in fields:
                row[name] = self._read_register(joint.motor_id,address,size)
            if row['status']:
                raise OSError(f'{joint.name}/status: {row["status"]}')
            rows[joint.name] = row
        return {'timestamp':stamp, 'joints':rows,
                'read_retry_count':self.read_retry_count,
                'read_retry_events':list(self.read_retry_events)}

    def goal(self, name, raw, speed=40, acceleration=2):
        joint = next(j for j in self.profile.joints if j.name == name)
        if type(raw) is not int or not joint.raw_min <= raw <= joint.raw_max:
            raise ValueError('raw goal outside profile range')
        if type(speed) is not int or not 1 <= speed <= MAX_GOAL_SPEED:
            raise ValueError('unsafe goal speed')
        if type(acceleration) is not int or not 1 <= acceleration <= MAX_GOAL_ACCELERATION:
            raise ValueError('unsafe goal acceleration')
        # Official STS SRAM: acceleration41, position42, time44=0, speed46.
        payload = [acceleration,raw&255,(raw>>8)&255,0,0,speed&255,(speed>>8)&255]
        result = self.packet.writeTxRx(self.port,joint.motor_id,41,len(payload),payload)
        self._check(result,f'{name}/goal')

    def _validate_targets(self,targets):
        if set(targets)!={j.name for j in self.profile.joints}:
            raise ValueError('all joint goals required')
        for j in self.profile.joints:
            if type(targets[j.name]) is not int or not j.raw_min<=targets[j.name]<=j.raw_max:
                raise ValueError('raw goal outside profile range')

    def begin_position_stream(self,targets,speed=600,acceleration=20):
        """Configure SRAM once, then stream position-only sync writes.

        Caller must hold the driver lock and its explicit motion gate. Identity,
        limits and homing are checked in a fresh full read before being cached
        for this exclusive TTY lease. Dynamic feedback stays fresh every tick.
        No factory acceleration limit, phase or calibration is written.
        """
        if self._motion_static is not None:raise RuntimeError('position stream already active')
        self._validate_targets(targets)
        if type(speed) is not int or not 1<=speed<=MAX_GOAL_SPEED:
            raise ValueError('unsafe goal speed')
        if type(acceleration) is not int or not 1<=acceleration<=MAX_GOAL_ACCELERATION:
            raise ValueError('unsafe goal acceleration')
        snapshot=self.read()
        for j in self.profile.joints:
            row=snapshot['joints'][j.name]
            if row['mode']!=0 or row['torque']!=1:raise ValueError('position mode and existing torque ON required')
            if (row['model']!=777 or self._read_register(j.motor_id,18,1)!=12
                    or self._read_register(j.motor_id,82,1)!=50):
                raise ValueError('verified STS3215 fine speed units required')
            if abs(row['goal']-row['position'])>STREAM_START_TOLERANCE or abs(targets[j.name]-row['position'])>STREAM_START_TOLERANCE:
                # 진단용: 어느 관절이 어떤 값 때문에 거부됐는지(2026-10-19 복귀가 이 검사에서 간헐적으로 멈췄다). 기준은 그대로다.
                raise ValueError(f"current neutral goals required before streaming: {j.name} goal {row['goal']} position {row['position']} start {targets[j.name]}")
        payload=[]
        for j in self.profile.joints:
            raw=targets[j.name]
            payload.extend([j.motor_id,acceleration,raw&255,raw>>8,0,0,speed&255,speed>>8])
        if self.packet.syncWriteTxOnly(self.port,41,7,payload,len(payload))!=0:
            raise OSError('stream setup transmission failed')
        self._motion_static={n:{k:row[k] for k in ('id','model','min_limit','max_limit','homing')}
                             for n,row in snapshot['joints'].items()}
        try:
            feedback=self.read()
            for n,row in feedback['joints'].items():
                if (row['mode']!=0 or row['torque']!=1 or row['goal']!=targets[n]
                        or row['goal_speed']!=speed or row['acceleration']!=acceleration):
                    raise ValueError('stream setup readback mismatch')
        except BaseException:
            self.end_position_stream();raise

    def sync_goal_positions(self,targets):
        if self._motion_static is None:raise RuntimeError('position stream is not initialized')
        self._validate_targets(targets)
        payload=[]
        for j in self.profile.joints:
            raw=targets[j.name];payload.extend([j.motor_id,raw&255,raw>>8])
        if self.packet.syncWriteTxOnly(self.port,42,2,payload,len(payload))!=0:
            raise OSError('goal position sync transmission failed')

    def _read_motion(self):
        stamp=time.monotonic();rows={}
        # One checked contiguous read/motor instead of36 scalar transactions.
        # Includes mode, torque, goal, encoder, voltage, temperature and alarm.
        for j in self.profile.joints:
            raw=self._read_register(j.motor_id,33,34).to_bytes(34,'little')
            row=dict(self._motion_static[j.name])
            row.update(mode=raw[0],torque=raw[7],acceleration=raw[8],
                       goal=int.from_bytes(raw[9:11],'little'),
                       goal_speed=int.from_bytes(raw[13:15],'little'),
                       position=int.from_bytes(raw[23:25],'little'),
                       voltage_deciV=raw[29],temperature_C=raw[30],status=raw[32],moving=raw[33])
            if row['status']:raise OSError(f'{j.name}/status: {row["status"]}')
            rows[j.name]=row
        return {'timestamp':stamp,'joints':rows,'read_retry_count':self.read_retry_count,
                'read_retry_events':list(self.read_retry_events)}

    def end_position_stream(self):
        # Pure state cleanup. Caller performs a fresh checked hold afterwards.
        self._motion_static=None

    def prime_goals(self, targets, speed=40):
        """Prime all neutral goals in one packet; caller must verify readback.

        Broadcast success confirms transmission only, not servo acceptance.
        This does not enable torque or authorize trajectory execution.
        """
        if self.port is None:
            raise RuntimeError('transport is not open')
        if set(targets) != {j.name for j in self.profile.joints}:
            raise ValueError('all joint goals required for neutral priming')
        if type(speed) is not int or not 1 <= speed <= MAX_GOAL_SPEED:
            raise ValueError('unsafe goal speed')
        payload = []
        for joint in self.profile.joints:
            raw = targets[joint.name]
            if type(raw) is not int or not joint.raw_min <= raw <= joint.raw_max:
                raise ValueError('raw goal outside profile range')
            payload.extend([joint.motor_id,2,raw&255,(raw>>8)&255,0,0,speed&255,(speed>>8)&255])
        result = self.packet.syncWriteTxOnly(self.port,41,7,payload,len(payload))
        if result != 0:
            raise OSError(f'neutral goal batch: comm={result}')

    def torque(self, name, enabled):
        joint = next(j for j in self.profile.joints if j.name == name)
        result = self.packet.write1ByteTxRx(self.port,joint.motor_id,40,int(enabled))
        self._check(result,f'{name}/torque')

    def close(self):
        self.end_position_stream()
        if self.port is not None and self.port.ser is not None:
            self.port.ser.close()
        self.port = self.packet = None
        if self.lock is not None:
            os.close(self.lock)
            self.lock = None

    def __enter__(self): return self.open()
    def __exit__(self, *args): self.close()
