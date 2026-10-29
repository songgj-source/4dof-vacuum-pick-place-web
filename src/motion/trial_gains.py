"""Backed-up temporary pitch gains and optional bounded base gain trial."""
import json
import time
from pathlib import Path

FIELDS=(('p',21,1),('d',22,1),('i',23,1),('startup',24,2),('lock',55,1))
BASELINE=dict(p=16,d=32,i=0,startup=16,lock=1)


class TemporaryPitchGains:
    def __init__(self,bus,output,sleep=time.sleep,*,target_p=None,shoulder_i=0,target_d=None,base_p=None,base_d=None):
        if type(shoulder_i) is not int or shoulder_i not in (0,1):
            raise ValueError('shoulder integral trial must be 0 or 1')
        self.shoulder_i=shoulder_i
        self.target_p=dict({2:64,3:64} if target_p is None else target_p)
        if (set(self.target_p)!={2,3} or any(type(v) is not int for v in self.target_p.values())
                or self.target_p[2] not in (32,48,64,96,128) or self.target_p[3] not in (32,48,64)):
            raise ValueError('bounded shoulder 32-128 and elbow 32-64 trial only')
        if base_p is not None:
            if type(base_p) is not int or base_p not in (16,32):
                raise ValueError('base trial P must be16/32')
            self.target_p[1]=base_p
        self.motors=(1,2,3) if base_p is not None else (2,3)
        # Optional extra damping (D, register 22) on the same two pitch motors; restored on exit.
        self.target_d=None if target_d is None else dict(target_d)
        if self.target_d is not None and (set(self.target_d)!={2,3} or any(type(v) is not int or v not in (32,48,64,96) for v in self.target_d.values())):
            raise ValueError('bounded shoulder/elbow D trial of 32/48/64/96 only')
        if base_d is not None:
            if base_p is None:raise ValueError('base damping needs the base gain trial (base_p)')
            if type(base_d) is not int or base_d not in (32,48,64):raise ValueError('base D trial of 32/48/64 only')
            self.target_d=dict(self.target_d or {});self.target_d[1]=base_d
        self.bus=bus;self.output=Path(output);self.sleep=sleep
        self.backup={};self.dirty=set();self.events=[];self.restoration_verified=False

    def _settings(self,motor):return {n:self.bus._read_register(motor,a,s) for n,a,s in FIELDS}

    def _write(self,motor,address,value):
        # 2026-10-20: 쓰기 한 번에 0.35s(0.15+0.1x2) 기다리던 것을 약 0.07s로 줄임(한 사이클 46회 쓰기 = 약 16s 대기).
        # 두 번 읽어 확인하는 것은 그대로이고, 틀리면 한 번 더 써 본 뒤에도 틀릴 때만 실패.
        for attempt in range(2):
            reply=self.bus.packet.write1ByteTxRx(self.bus.port,motor,address,value)
            self.sleep(.03);readback=[]
            for _ in range(2):
                self.bus.port.ser.reset_input_buffer();self.sleep(.02)
                readback.append(self.bus._read_register(motor,address,1))
            self.events.append(dict(motor=motor,address=address,value=value,reply=reply,readback=readback,attempt=attempt+1))
            if reply[-1]==0 and readback==[value,value]:return
        raise OSError('gain write/alarm/readback failure')

    def __enter__(self):
        self.output.mkdir(parents=True,exist_ok=True)
        self.backup={str(m):self._settings(m) for m in self.motors}
        (self.output/'settings_backup.json').write_text(json.dumps(self.backup,indent=2)+'\n')
        if any(row!=BASELINE for row in self.backup.values()):raise ValueError('gain baseline changed')
        try:
            for motor in self.motors:
                self.dirty.add(motor)
                self._write(motor,55,0);self._write(motor,21,self.target_p[motor])
                if self.target_d is not None and motor in self.target_d:self._write(motor,22,self.target_d[motor])
                if motor==2 and self.shoulder_i:self._write(motor,23,self.shoulder_i)
                self._write(motor,55,1)
        except BaseException as original:
            try:self.restore()
            except BaseException as exc:original.add_note('gain restoration failed: '+str(exc))
            raise
        return self

    def set_shoulder_integral(self,value):
        """Change only the shoulder I gain mid-trial; restore() puts it back."""
        if value not in (0,1):raise ValueError('shoulder integral trial must be 0 or 1')
        self.shoulder_i=1  # makes restore() rewrite the baseline integral gain
        self.dirty.add(2)
        self._write(2,55,0);self._write(2,23,value);self._write(2,55,1)

    def set_pitch_p(self,value):
        """Switch at a stationary phase boundary; keep I/D and original backup."""
        if type(value) is not int or value not in (32,48,64):
            raise ValueError('phase pitch P must be32/48/64')
        if not self.backup or self.restoration_verified:
            raise ValueError('active backed-up gain trial required')
        for motor in (2,3):
            if self.target_p[motor]==value:continue
            self.dirty.add(motor)
            self._write(motor,55,0);self._write(motor,21,value);self._write(motor,55,1)
            self.target_p[motor]=value

    def restore(self):
        restored={};failures=[]
        for motor in sorted(self.dirty):
            for _ in range(3):
                try:
                    row=self.backup[str(motor)]
                    self._write(motor,55,0);self._write(motor,21,row['p'])
                    if self.target_d is not None and motor in self.target_d:self._write(motor,22,row['d'])
                    if motor==2 and self.shoulder_i:self._write(motor,23,row['i'])
                    self._write(motor,55,row['lock'])
                    actual=self._settings(motor)
                    if actual!=row:raise OSError('gain restore mismatch')
                    restored[str(motor)]=actual;break
                except BaseException as exc:failures.append(str(exc))
        self.restoration_verified=len(restored)==len(self.dirty)
        (self.output/'settings_restore.json').write_text(json.dumps(dict(restored=restored,verified=self.restoration_verified,errors=failures,events=self.events),indent=2)+'\n')
        if not self.restoration_verified:raise OSError('original pitch gain restoration unverified')

    def __exit__(self,kind,original,tb):
        try:self.restore()
        except BaseException as exc:
            if original is not None:original.add_note(str(exc))
            else:raise


def restore_baseline_gains(bus,output,*,motors=(1,2,3),execute=False,sleep=time.sleep):
    """임시 게인(P·D·I)이 서보에 남았을 때 기준값(BASELINE)으로 되돌린다. 게인은 서보에 저장되는 값이라, 임시 게인을 쓰던 프로세스가
    복구 없이 끊기면(2026-10-28 return_home이 들어올리기 도중 끊김: 어깨 P48·I1, 팔꿈치 P48) 다음 복귀가 'gain baseline changed'로 계속 막힌다.
    execute=False면 읽기만 한다. 쓸 때는 정상 경로(TemporaryPitchGains)와 같은 순서로 쓰고 읽어서 확인한다(잠금 해제 → P·D·I → 잠금 → 읽기 확인).
    startup·잠금은 기준과 달라도 건드리지 않고 알린다. 돌려주는 것: {모터: dict(current, baseline_ok, changed, verified)}."""
    gains=TemporaryPitchGains(bus,output,sleep=sleep)
    report={}
    for motor in motors:
        current=gains._settings(motor)
        row=dict(current=current,baseline_ok=current==BASELINE,changed=False,verified=current==BASELINE)
        differs=[n for n in ('p','d','i') if current[n]!=BASELINE[n]]
        if execute and differs:
            gains._write(motor,55,0)
            for name,address in (('p',21),('d',22),('i',23)):
                if name in differs:gains._write(motor,address,BASELINE[name])
            gains._write(motor,55,BASELINE['lock'])
            after=gains._settings(motor)
            row.update(changed=True,after=after,verified=all(after[n]==BASELINE[n] for n in ('p','d','i')))
        report[str(motor)]=row
    return report
