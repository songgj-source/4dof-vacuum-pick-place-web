"""Shared manual driving for the robot's web UIs (camera_test.py, map_web.py).

DriveControl publishes geometry_msgs/Twist on /cmd_vel for car2_serial_node.
car2_serial_node keeps executing the last /cmd_vel (until its own 1 s
cmd_vel_timeout), so every command here expires after drive_hold_s: the page
resends while a button / key is held, and when it stops (released, tab hidden,
browser closed, WiFi drop) this node publishes a few zero Twists itself.
Speeds are capped to what the wheels can do (~0.167 m/s, ~0.77 rad/s).

DRIVE_CSS / DRIVE_HTML / DRIVE_JS are the matching page pieces (direction pad,
speed sliders, keyboard: arrows/WASD, Space = stop, +/- = speed). DRIVE_JS
expects a `$ = id => document.getElementById(id)` helper on the page.

HTTP: POST /api/drive {"linear": m/s, "angular": rad/s (+ = left)},
      POST /api/drive/stop, GET /api/drive (status) -- see DriveControl.handle_post().
"""
import json
import threading
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class DriveControl:

    def __init__(self, node):
        node.declare_parameter('max_linear', 0.16)     # m/s  (wheel limit ~0.167)
        node.declare_parameter('max_angular', 0.75)    # rad/s (in-place spin limit ~0.77)
        node.declare_parameter('drive_hold_s', 0.5)    # a command lives this long unless resent
        self.max_linear = float(node.get_parameter('max_linear').value)
        self.max_angular = float(node.get_parameter('max_angular').value)
        self.hold_s = float(node.get_parameter('drive_hold_s').value)
        self.lock = threading.Lock()
        self.cmd = (0.0, 0.0)
        self.deadline = 0.0
        self.zero_left = 0          # zero Twists still to send after a stop
        self.odom = None
        self.pub = node.create_publisher(Twist, 'cmd_vel', 10)
        node.create_subscription(Odometry, 'odom', self._on_odom, 10)
        node.create_timer(0.1, self._tick)

    def _on_odom(self, msg: Odometry):
        self.odom = (msg.twist.twist.linear.x, msg.twist.twist.angular.z, time.time())

    def drive(self, linear: float, angular: float):
        linear = max(-self.max_linear, min(self.max_linear, float(linear)))
        angular = max(-self.max_angular, min(self.max_angular, float(angular)))
        with self.lock:
            self.cmd = (linear, angular)
            self.deadline = time.time() + self.hold_s
        self._publish(linear, angular)   # react now, not at the next tick

    def stop(self):
        with self.lock:
            self.cmd = (0.0, 0.0)
            self.deadline = 0.0
            self.zero_left = 3
        self._publish(0.0, 0.0)

    def _publish(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x, msg.angular.z = linear, angular
        self.pub.publish(msg)

    def _tick(self):
        # Keep /cmd_vel alive while a command is fresh; when it expires, send a
        # few zero Twists (the driver would otherwise keep the last command).
        with self.lock:
            if self.deadline:
                if time.time() < self.deadline:
                    cmd = self.cmd
                else:
                    self.cmd, self.deadline, self.zero_left = (0.0, 0.0), 0.0, 2
                    cmd = (0.0, 0.0)
            elif self.zero_left > 0:
                self.zero_left -= 1
                cmd = (0.0, 0.0)
            else:
                return
        self._publish(*cmd)

    def status(self) -> dict:
        with self.lock:
            linear, angular = self.cmd
            active = time.time() < self.deadline
        odom = self.odom if self.odom and time.time() - self.odom[2] < 2.0 else None
        return {
            'linear': linear if active else 0.0, 'angular': angular if active else 0.0, 'moving': active,
            'max_linear': self.max_linear, 'max_angular': self.max_angular, 'hold_s': self.hold_s,
            # car2_serial_node (or anything else) listening on /cmd_vel
            'driver_connected': self.pub.get_subscription_count() > 0,
            'odom': None if odom is None else {'linear': round(odom[0], 3), 'angular': round(odom[1], 3)},
        }

    def handle_post(self, path: str, body: bytes):
        """(status_code, json_dict) for POST /api/drive and /api/drive/stop, else None."""
        if path == '/api/drive/stop':
            self.stop()
            return 200, self.status()
        if path == '/api/drive':
            try:
                req = json.loads(body or b'{}')
                self.drive(req.get('linear', 0.0), req.get('angular', 0.0))
            except (ValueError, TypeError, AttributeError) as exc:
                self.stop()
                return 400, {'error': f'잘못된 주행 명령: {exc}'}
            return 200, self.status()
        return None


DRIVE_CSS = """
  .drive { margin-top:12px; background:var(--panel); border-radius:8px; padding:12px; display:flex; flex-wrap:wrap;
           gap:16px 24px; align-items:center; }
  .pad { display:grid; grid-template-columns:repeat(3, 64px); grid-template-rows:repeat(3, 64px); gap:6px;
         touch-action:none; user-select:none; -webkit-user-select:none; }
  .pad button { border:0; border-radius:10px; font-size:24px; background:#2e343d; color:var(--text); cursor:pointer; }
  .pad button.on { background:var(--accent); color:#06222f; }
  #stopbtn { background:#5a2a2a; color:#ffcdd2; font-size:20px; }
  .drivectl { flex:1; min-width:260px; display:flex; flex-direction:column; gap:8px; }
  .spd { display:flex; align-items:center; gap:10px; }
  .spd b { min-width:5em; color:var(--muted); font-weight:400; font-size:13px; }
  .spd input { flex:1; }
  .spd span { min-width:6.5em; font-variant-numeric:tabular-nums; }
  .drivestate { font-size:13px; font-variant-numeric:tabular-nums; }
  .drivestate.off { color:#ff8a80; } .drivestate.moving { color:var(--accent); }
"""

DRIVE_HTML = """<div class="drive">
  <div class="pad">
    <span></span><button class="dir" data-l="1" data-a="0" title="전진 (↑ / W)">▲</button><span></span>
    <button class="dir" data-l="0" data-a="1" title="좌회전 (← / A)">◀</button>
    <button id="stopbtn" title="정지 (Space)">■</button>
    <button class="dir" data-l="0" data-a="-1" title="우회전 (→ / D)">▶</button>
    <span></span><button class="dir" data-l="-1" data-a="0" title="후진 (↓ / S)">▼</button><span></span>
  </div>
  <div class="drivectl">
    <div class="spd"><b>직진 속도</b><input id="vlin" type="range" min="0.02" max="0.16" step="0.01" value="0.08">
      <span id="vlinv">0.08 m/s</span></div>
    <div class="spd"><b>회전 속도</b><input id="vang" type="range" min="0.1" max="0.75" step="0.05" value="0.4">
      <span id="vangv">0.40 rad/s</span></div>
    <div id="drivestate" class="drivestate">드라이버 확인 중…</div>
    <div class="hint">누르고 있는 동안만 움직입니다 (떼면 0.5초 안에 정지). 키보드: ↑↓←→ 또는 WASD, 대각선은 두 키 동시,
      Space 정지, +/− 직진 속도. 탭을 바꾸거나 창을 벗어나면 자동 정지합니다.</div>
  </div>
</div>"""

DRIVE_JS = r"""
// ---- driving (drive_control.py) --------------------------------------------
const held = new Set();               // 'up' | 'down' | 'left' | 'right' currently pressed
let driveTimer = null;
const KEY = { ArrowUp: 'up', KeyW: 'up', ArrowDown: 'down', KeyS: 'down',
              ArrowLeft: 'left', KeyA: 'left', ArrowRight: 'right', KeyD: 'right' };
const vlin = () => parseFloat($('vlin').value), vang = () => parseFloat($('vang').value);
function driveCmd() {
  const l = (held.has('up') ? 1 : 0) - (held.has('down') ? 1 : 0);
  const a = (held.has('left') ? 1 : 0) - (held.has('right') ? 1 : 0);
  // reversing with a turn key: steer like a car (left key turns the nose left)
  return { linear: l * vlin(), angular: a * vang() * (l < 0 ? -1 : 1) };
}
function sendDrive() {
  const c = driveCmd();
  if (!c.linear && !c.angular) { stopDrive(); return; }
  fetch('/api/drive', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(c) })
    .catch(() => {});
}
function updateDrive() {
  document.querySelectorAll('.pad .dir').forEach(b => b.classList.toggle('on', held.has(b.dataset.dir)));
  if (held.size) { sendDrive(); if (!driveTimer) driveTimer = setInterval(sendDrive, 100); }
  else stopDrive();
}
function stopDrive() {
  held.clear(); clearInterval(driveTimer); driveTimer = null;
  document.querySelectorAll('.pad .dir').forEach(b => b.classList.remove('on'));
  fetch('/api/drive/stop', { method: 'POST' }).catch(() => {});
}
const dirName = b => b.dataset.l === '1' ? 'up' : b.dataset.l === '-1' ? 'down' : b.dataset.a === '1' ? 'left' : 'right';
document.querySelectorAll('.pad .dir').forEach(b => {
  b.dataset.dir = dirName(b);
  b.addEventListener('pointerdown', e => { e.preventDefault(); b.setPointerCapture(e.pointerId); held.add(b.dataset.dir); updateDrive(); });
  for (const ev of ['pointerup', 'pointercancel', 'lostpointercapture'])
    b.addEventListener(ev, () => { held.delete(b.dataset.dir); updateDrive(); });
});
$('stopbtn').addEventListener('click', stopDrive);
addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT' && e.target.type !== 'range') return;   // typing in a number box
  if (e.code === 'Space') { e.preventDefault(); stopDrive(); return; }
  if (e.key === '+' || e.key === '=') { $('vlin').stepUp(); $('vlin').dispatchEvent(new Event('input')); return; }
  if (e.key === '-' || e.key === '_') { $('vlin').stepDown(); $('vlin').dispatchEvent(new Event('input')); return; }
  const d = KEY[e.code];
  if (!d) return;
  e.preventDefault();
  if (!held.has(d)) { held.add(d); updateDrive(); }
});
addEventListener('keyup', e => { const d = KEY[e.code]; if (d && held.delete(d)) updateDrive(); });
addEventListener('blur', () => held.size && stopDrive());
document.addEventListener('visibilitychange', () => document.hidden && held.size && stopDrive());
for (const [id, unit] of [['vlin', 'm/s'], ['vang', 'rad/s']])
  $(id).addEventListener('input', e => { $(id + 'v').textContent = parseFloat(e.target.value).toFixed(2) + ' ' + unit; });
async function driveStatus() {
  try {
    const d = await (await fetch('/api/drive')).json();
    $('vlin').max = d.max_linear; $('vang').max = d.max_angular;
    const st = $('drivestate');
    const odom = d.odom ? `실제(odom) ${d.odom.linear.toFixed(2)} m/s, ${d.odom.angular.toFixed(2)} rad/s` : 'odom 없음';
    if (!d.driver_connected) {
      st.className = 'drivestate off';
      st.textContent = '차량 드라이버가 /cmd_vel을 구독하지 않습니다 — car2_lidar / robot_rgbd / car2_driver가 꺼져 있습니다';
    } else {
      st.className = 'drivestate' + (d.moving ? ' moving' : '');
      st.textContent = (d.moving ? `명령 ${d.linear.toFixed(2)} m/s, ${d.angular.toFixed(2)} rad/s` : '정지') + ' · ' + odom;
    }
  } catch (e) { $('drivestate').textContent = '상태 확인 실패'; }
}
setInterval(driveStatus, 300); driveStatus();
"""
