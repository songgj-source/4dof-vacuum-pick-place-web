#!/usr/bin/env python3
"""Web viewer for the maps saved in ros2_slam/maps/.

  - 2D maps (map_saver output, e.g. scripts/save_map.sh -> my_map.yaml + .pgm):
    occupancy grid with size / resolution and the map coordinate under the mouse.
  - RTAB-Map databases (rtabmap.db): 2D grid + trajectory, a 3D colored cloud,
    and graph stats (nodes, loop closures, path length).

The 3D cloud is assembled from the local grids rtabmap stored per node (the same
data /rtabmap/cloud_map is built from), placed with the optimized graph poses.
Those grids already went through rtabmap's filters (Grid/DepthRoiRatios from the
camera web UI, range / height limits), so the robot body is not in it -- unlike
re-projecting the raw depth images (rtabmap-export --cloud, RViz MapCloud).

A database can be viewed while rtabmap is still writing it: it is read through
an SQLite backup snapshot, never opened in place. Results are cached in
~/.cache/car2_map_viewer per (file, mtime, size); "새로 읽기" reprocesses.

No ROS nodes are started. RTAB-Map databases need the `rtabmap-export` tool
(ros-jazzy-rtabmap, already installed with rtabmap_ros).

Usage (laptop):
  /usr/bin/python3 map_viewer.py                 # http://localhost:8090
  /usr/bin/python3 map_viewer.py --port 8091 --maps-dir /path/to/maps
"""
import argparse
import json
import math
import os
import shutil
import sqlite3
import struct
import subprocess
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import yaml

ROS_PREFIX = '/opt/ros/jazzy'
CACHE_ROOT = os.path.expanduser('~/.cache/car2_map_viewer')
VOXEL_M = 0.05          # 3D cloud de-duplication (= rtabmap Grid/CellSize)
CACHE_VERSION = 1       # bump when the cached output format changes

LINK_TYPES = {0: 'neighbor', 1: 'global_closure', 2: 'local_space_closure', 3: 'local_time_closure',
              4: 'user_closure', 5: 'virtual_closure', 6: 'neighbor_merged', 7: 'pose_prior',
              8: 'landmark', 9: 'gravity'}


# ---------------------------------------------------------------------------
# 2D occupancy grids (map_saver yaml + pgm)

def load_grid(yaml_path: str) -> dict:
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img_path = meta['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(yaml_path), img_path)
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError(f'cannot read {img_path}')
    if meta.get('negate'):
        img = 255 - img
    h, w = img.shape
    res = float(meta['resolution'])
    origin = [float(v) for v in meta['origin'][:2]]
    # map_saver trinary values: 0 occupied, 254 free, 205 unknown (thresholds for anything else).
    occ_t, free_t = float(meta.get('occupied_thresh', 0.65)), float(meta.get('free_thresh', 0.196))
    p_occ = (255 - img.astype(np.float32)) / 255.0
    occupied = int((p_occ > occ_t).sum())
    free = int((p_occ < free_t).sum())
    return {
        'image': img, 'resolution': res, 'origin': origin, 'width_px': w, 'height_px': h,
        'width_m': round(w * res, 2), 'height_m': round(h * res, 2),
        'occupied_cells': occupied, 'free_cells': free, 'unknown_cells': int(h * w - occupied - free),
        'free_area_m2': round(free * res * res, 1),
    }


def grid_png(img: np.ndarray) -> bytes:
    # Unknown gray / free light / occupied dark, slightly tinted for the dark UI.
    lut = np.zeros((256, 3), np.uint8)
    for v in range(256):
        p_occ = (255 - v) / 255.0
        if p_occ > 0.65:
            lut[v] = (40, 30, 25)          # occupied (BGR)
        elif p_occ < 0.196:
            lut[v] = (240, 236, 232)       # free
        else:
            lut[v] = (110, 104, 98)        # unknown
    ok, buf = cv2.imencode('.png', lut[img])
    return buf.tobytes()


# ---------------------------------------------------------------------------
# RTAB-Map databases

def decode_cells(blob: bytes) -> np.ndarray:
    """rtabmap compressData2 blob (zlib + int32 rows, cols, cv type) -> float32 (N, channels)."""
    rows, cols, cv_type = struct.unpack('<3i', blob[-12:])
    channels = (cv_type >> 3) + 1
    data = np.frombuffer(zlib.decompress(blob[:-12]), np.float32)
    return data.reshape(-1, channels)


def packed_rgb(col: np.ndarray) -> np.ndarray:
    """PCL-style float-packed 0x00RRGGBB -> uint8 (N, 3) RGB."""
    u = col.astype(np.float32).view(np.uint32)
    return np.stack([(u >> 16) & 255, (u >> 8) & 255, u & 255], axis=1).astype(np.uint8)


def pose_matrix(x, y, z, qx, qy, qz, qw) -> np.ndarray:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    R = np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, (x, y, z)
    return T


def rtabmap_env() -> dict:
    env = dict(os.environ)
    env['PATH'] = f'{ROS_PREFIX}/bin:' + env.get('PATH', '')
    env['LD_LIBRARY_PATH'] = f'{ROS_PREFIX}/lib:' + env.get('LD_LIBRARY_PATH', '')
    return env


class DbJob:
    """Background processing of one database into the cache directory."""

    def __init__(self, db_path: str, cache_dir: str):
        self.db_path, self.dir = db_path, cache_dir
        self.state, self.message, self.started = 'running', '스냅샷 생성 중…', time.time()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        tmp = self.dir + '.tmp'
        shutil.rmtree(tmp, ignore_errors=True)
        os.makedirs(tmp)
        snap = os.path.join(tmp, 'snap.db')
        try:
            # Consistent copy even while rtabmap is writing the database.
            src = sqlite3.connect(f'file:{self.db_path}?mode=ro', uri=True)
            dst = sqlite3.connect(snap)
            src.backup(dst)
            dst.close()
            src.close()

            self.message = '그래프 최적화 + 2D 맵 내보내기 (rtabmap-export)…'
            proc = subprocess.run(
                ['rtabmap-export', '--poses', '--map', '--opt', '0',
                 '--output_dir', tmp, '--output', 'map', snap],
                env=rtabmap_env(), capture_output=True, text=True, timeout=600)
            poses_txt = os.path.join(tmp, 'map_poses.txt')
            if not os.path.exists(poses_txt):
                tail = (proc.stdout + proc.stderr).strip().splitlines()[-5:]
                raise RuntimeError('rtabmap-export 실패: ' + ' / '.join(tail))

            self.message = '3D 클라우드 조립 중…'
            meta = self._assemble(snap, poses_txt, tmp)
            os.remove(snap)
            with open(os.path.join(tmp, 'meta.json'), 'w') as f:
                json.dump(meta, f)
            shutil.rmtree(self.dir, ignore_errors=True)
            os.replace(tmp, self.dir)
            self.state, self.message = 'done', ''
        except Exception as exc:  # noqa: BLE001 -- reported to the UI
            shutil.rmtree(tmp, ignore_errors=True)
            self.state, self.message = 'error', str(exc)

    def _assemble(self, snap: str, poses_txt: str, out_dir: str) -> dict:
        poses, stamps = {}, {}
        with open(poses_txt) as f:
            for line in f:
                if line.startswith('#') or not line.strip():
                    continue
                v = line.split()   # stamp x y z qx qy qz qw id
                nid = int(v[8])
                poses[nid] = pose_matrix(*map(float, v[1:8]))
                stamps[nid] = float(v[0])
        ids = sorted(poses)

        db = sqlite3.connect(snap)
        xyz_all, rgb_all, label_all = [], [], []
        for nid, ground, obstacle in db.execute('SELECT id, ground_cells, obstacle_cells FROM Data'):
            if nid not in poses:
                continue   # not in the optimized graph (e.g. robot standing still)
            T = poses[nid]
            for label, blob in ((0, ground), (1, obstacle)):
                if not blob:
                    continue
                cells = decode_cells(blob)
                if len(cells) == 0:
                    continue
                pts = cells[:, :3].astype(np.float64)
                if cells.shape[1] == 2:      # 2D grid (Grid/3D false): z = 0
                    pts = np.column_stack([cells, np.zeros(len(cells))])
                xyz_all.append(pts @ T[:3, :3].T + T[:3, 3])
                if cells.shape[1] >= 4:
                    rgb_all.append(packed_rgb(cells[:, 3]))
                else:
                    rgb_all.append(np.full((len(cells), 3), 160 if label == 0 else 230, np.uint8))
                label_all.append(np.full(len(cells), label, np.uint8))

        node_count = db.execute('SELECT COUNT(*) FROM Node').fetchone()[0]
        links = {LINK_TYPES.get(t, str(t)): n for t, n in db.execute('SELECT type, COUNT(*) FROM Link GROUP BY type')}
        sessions = db.execute('SELECT COUNT(DISTINCT map_id) FROM Node').fetchone()[0]
        db.close()

        if xyz_all:
            xyz = np.concatenate(xyz_all)
            rgb = np.concatenate(rgb_all)
            label = np.concatenate(label_all)
            # One point per voxel (obstacles win over ground in a shared voxel).
            order = np.argsort(-label, kind='stable')
            keys = np.floor(xyz[order] / VOXEL_M).astype(np.int64)
            _, first = np.unique(keys, axis=0, return_index=True)
            keep = order[first]
            xyz, rgb, label = xyz[keep].astype(np.float32), rgb[keep], label[keep]
        else:
            xyz, rgb, label = np.zeros((0, 3), np.float32), np.zeros((0, 3), np.uint8), np.zeros(0, np.uint8)
        with open(os.path.join(out_dir, 'cloud.bin'), 'wb') as f:
            f.write(xyz.tobytes())
            f.write(rgb.tobytes())
            f.write(label.tobytes())

        traj = [[round(float(poses[i][0, 3]), 3), round(float(poses[i][1, 3]), 3),
                 round(float(poses[i][2, 3]), 3)] for i in ids]
        path_len = float(sum(np.linalg.norm(np.subtract(b, a)) for a, b in zip(traj, traj[1:])))
        closures = sum(n for k, n in links.items() if k.endswith('closure'))
        meta = {
            'points': int(len(xyz)), 'obstacle_points': int((label == 1).sum()),
            'ground_points': int((label == 0).sum()),
            'bounds': [xyz.min(0).round(2).tolist(), xyz.max(0).round(2).tolist()] if len(xyz) else None,
            'nodes': node_count, 'graph_nodes': len(ids), 'sessions': sessions,
            'links': links, 'loop_closures': closures,
            'path_length_m': round(path_len, 2),
            'duration_s': round(stamps[ids[-1]] - stamps[ids[0]], 1) if ids else 0,
            'trajectory': traj, 'has_grid': os.path.exists(os.path.join(out_dir, 'map.yaml')),
        }
        return meta


class MapStore:

    def __init__(self, maps_dir: str):
        self.maps_dir = os.path.abspath(maps_dir)
        self.jobs = {}
        self.lock = threading.Lock()

    def list(self):
        out = []
        for name in sorted(os.listdir(self.maps_dir)):
            path = os.path.join(self.maps_dir, name)
            if name.endswith('.yaml'):
                kind = '2d'
            elif name.endswith('.db'):
                kind = 'rtabmap'
            else:
                continue
            st = os.stat(path)
            out.append({'name': name, 'kind': kind, 'size': st.st_size,
                        'mtime': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(st.st_mtime))})
        return out

    def path(self, name: str) -> str:
        if not name or '/' in name or name.startswith('.'):
            raise ValueError('invalid map name')
        path = os.path.join(self.maps_dir, name)
        if not os.path.isfile(path):
            raise FileNotFoundError(name)
        return path

    def cache_dir(self, name: str) -> str:
        st = os.stat(self.path(name))
        return os.path.join(CACHE_ROOT, f'{name}-{int(st.st_mtime)}-{st.st_size}-v{CACHE_VERSION}')

    def db_status(self, name: str, refresh: bool = False) -> dict:
        cdir = self.cache_dir(name)
        with self.lock:
            job = self.jobs.get(name)
            if refresh and (job is None or job.state != 'running'):
                shutil.rmtree(cdir, ignore_errors=True)
                job = None
            if os.path.exists(os.path.join(cdir, 'meta.json')) and not (job and job.state == 'running'):
                with open(os.path.join(cdir, 'meta.json')) as f:
                    return {'state': 'done', 'meta': json.load(f)}
            if job is None or job.dir != cdir or (job.state == 'error' and refresh):
                job = self.jobs[name] = DbJob(self.path(name), cdir)
            return {'state': job.state, 'message': job.message,
                    'elapsed_s': round(time.time() - job.started, 1)}

    def grid_for(self, name: str) -> dict:
        if name.endswith('.yaml'):
            return load_grid(self.path(name))
        return load_grid(os.path.join(self.cache_dir(name), 'map.yaml'))


PAGE = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>car2 map viewer</title>
<style>
  :root { color-scheme: dark; --bg:#16181d; --panel:#21252c; --text:#e6e8eb; --muted:#9aa3ad; --accent:#4cc2ff;
          --line:#ffffff14; }
  * { box-sizing:border-box; }
  [hidden] { display:none !important; }
  body { margin:0; background:var(--bg); color:var(--text); font:14px system-ui, sans-serif; }
  .app { display:grid; grid-template-columns:260px 1fr; min-height:100vh; }
  aside { background:var(--panel); border-right:1px solid var(--line); padding:16px 12px; }
  aside h1 { font-size:16px; margin:0 0 12px; }
  .map { display:block; width:100%; text-align:left; background:none; border:1px solid transparent; color:var(--text);
         border-radius:6px; padding:8px 10px; margin-bottom:4px; cursor:pointer; font:inherit; }
  .map:hover { background:#ffffff0a; } .map.on { border-color:var(--accent); background:#4cc2ff14; }
  .map small { display:block; color:var(--muted); font-size:11px; margin-top:2px; }
  .badge { font-size:10px; padding:1px 5px; border-radius:3px; margin-left:6px; vertical-align:1px; }
  .b2d { background:#3b4a5a; } .bdb { background:#5a3b52; }
  main { padding:16px; min-width:0; }
  .cards { display:grid; grid-template-columns:repeat(auto-fit, minmax(130px, 1fr)); gap:8px; margin-bottom:12px; }
  .card { background:var(--panel); border-radius:8px; padding:8px 12px; }
  .card b { display:block; color:var(--muted); font-weight:400; font-size:12px; }
  .card span { font-size:17px; font-variant-numeric:tabular-nums; }
  .views { display:grid; grid-template-columns:repeat(auto-fit, minmax(380px, 1fr)); gap:12px; }
  .view { background:var(--panel); border-radius:8px; padding:8px; min-width:0; }
  .view h2 { font-size:13px; margin:0 0 6px; color:var(--muted); font-weight:500; display:flex; gap:12px;
             align-items:center; flex-wrap:wrap; }
  .view h2 .sp { flex:1; }
  .stage { position:relative; height:520px; background:#0f1115; border-radius:4px; overflow:hidden; }
  canvas { display:block; width:100%; height:100%; }
  #c2d { cursor:grab; } #c2d.drag { cursor:grabbing; }
  .coord { position:absolute; left:8px; bottom:8px; background:#000b; padding:2px 6px; border-radius:3px;
           font-size:12px; font-variant-numeric:tabular-nums; pointer-events:none; }
  .msg { position:absolute; inset:0; display:flex; align-items:center; justify-content:center; color:var(--muted);
         text-align:center; padding:20px; }
  label { color:var(--muted); font-size:12px; display:inline-flex; align-items:center; gap:5px; }
  select, button.small { background:#14171c; color:var(--text); border:1px solid #ffffff22; border-radius:4px;
                         padding:2px 6px; font:12px system-ui, sans-serif; }
  button.small { cursor:pointer; }
  .hint { color:var(--muted); font-size:12px; margin-top:8px; }
  @media (max-width: 700px) { .app { grid-template-columns:1fr; } aside { border-right:0; } .stage { height:380px; } }
</style>
<script type="importmap">
{ "imports": { "three": "https://unpkg.com/three@0.160.0/build/three.module.js",
               "three/addons/": "https://unpkg.com/three@0.160.0/examples/jsm/" } }
</script>
</head><body>
<div class="app">
  <aside>
    <h1>저장된 맵</h1>
    <div id="list">불러오는 중…</div>
    <p class="hint">maps 폴더: <code id="dir"></code></p>
  </aside>
  <main>
    <div class="cards" id="cards"></div>
    <div class="views">
      <div class="view">
        <h2>2D 점유 격자 <span class="sp"></span>
          <label><input id="traj2d" type="checkbox" checked> 경로</label>
          <button class="small" id="fit2d">맞춤</button></h2>
        <div class="stage"><canvas id="c2d"></canvas><div class="coord" id="coord" hidden></div>
          <div class="msg" id="msg2d">왼쪽에서 맵을 고르세요</div></div>
      </div>
      <div class="view" id="v3d">
        <h2>3D 클라우드 <span class="sp"></span>
          <label>색 <select id="color3d"><option value="rgb">카메라 색</option><option value="height">높이</option></select></label>
          <label><input id="ground3d" type="checkbox" checked> 바닥</label>
          <label>점 <select id="size3d"><option>0.03</option><option selected>0.05</option><option>0.08</option></select></label>
          <button class="small" id="refresh">새로 읽기</button></h2>
        <div class="stage"><canvas id="c3d"></canvas><div class="msg" id="msg3d">RTAB-Map DB(.db)를 고르면 표시됩니다</div></div>
      </div>
    </div>
    <p class="hint">2D: 드래그로 이동, 휠로 확대. 마우스 위치의 map 좌표(m)가 왼쪽 아래에 표시됩니다.
      3D: 드래그 회전, 우클릭 드래그 이동, 휠 확대. 3D 클라우드는 rtabmap이 필터(RoiRatios 등)를 거쳐 저장한 로컬 격자로 만든 것이라 로봇 몸체가 빠져 있습니다.</p>
  </main>
</div>
<script type="module">
const $ = id => document.getElementById(id);
const fmtSize = b => b > 1e9 ? (b / 1e9).toFixed(1) + ' GB' : b > 1e6 ? (b / 1e6).toFixed(1) + ' MB' : b >= 1e3 ? (b / 1e3).toFixed(0) + ' KB' : b + ' B';
let current = null, grid = null, meta = null, pollTimer = null;

// ---------------- map list
async function loadList() {
  const r = await (await fetch('/api/maps')).json();
  $('dir').textContent = r.dir;
  $('list').innerHTML = '';
  if (!r.maps.length) { $('list').textContent = '저장된 맵이 없습니다'; return; }
  for (const m of r.maps) {
    const b = document.createElement('button');
    b.className = 'map' + (current === m.name ? ' on' : '');
    b.innerHTML = `${m.name}<span class="badge ${m.kind === '2d' ? 'b2d' : 'bdb'}">${m.kind === '2d' ? '2D' : 'RTAB-Map'}</span>
                   <small>${fmtSize(m.size)} · ${m.mtime}</small>`;
    b.onclick = () => { location.hash = encodeURIComponent(m.name); select(m.name); };
    $('list').appendChild(b);
  }
}

function card(label, value) { return `<div class="card"><b>${label}</b><span>${value}</span></div>`; }

async function select(name, refresh = false) {
  current = name; meta = null; grid = null; img2d = null; clearTimeout(pollTimer);
  document.querySelectorAll('.map').forEach(b => b.classList.toggle('on', b.textContent.startsWith(name)));
  $('cards').innerHTML = ''; $('msg2d').hidden = false; $('msg2d').textContent = '불러오는 중…';
  setCloud(null, name.endsWith('.db') ? '처리 중…' : '2D 맵에는 3D 정보가 없습니다');
  draw2d();
  const r = await (await fetch(`/api/map?name=${encodeURIComponent(name)}${refresh ? '&refresh=1' : ''}`)).json();
  if (current !== name) return;
  if (r.error) { $('msg2d').textContent = r.error; return; }
  if (r.state && r.state !== 'done') {
    const text = r.state === 'error' ? '처리 실패: ' + r.message : `${r.message} (${r.elapsed_s}s)`;
    $('msg2d').textContent = text; setCloud(null, text);
    if (r.state === 'running') pollTimer = setTimeout(() => select(name), 1000);
    return;
  }
  meta = r.meta || null;
  grid = r.grid || null;
  const cards = [];
  if (grid) {
    cards.push(card('크기', `${grid.width_m} × ${grid.height_m} m`), card('해상도', `${grid.resolution} m/칸`),
               card('빈 공간 넓이', `${grid.free_area_m2} m²`), card('장애물 칸', grid.occupied_cells.toLocaleString()));
  }
  if (meta) {
    cards.push(card('노드 (그래프 / 전체)', `${meta.graph_nodes} / ${meta.nodes}`),
               card('루프 클로저', meta.loop_closures), card('이동 경로', `${meta.path_length_m} m`),
               card('매핑 시간', `${Math.round(meta.duration_s / 60)}분`), card('3D 점', meta.points.toLocaleString()));
  }
  $('cards').innerHTML = cards.join('');
  if (grid) await loadGridImage(name); else { $('msg2d').hidden = false; $('msg2d').textContent = '2D 격자 없음'; }
  if (meta) loadCloud(name); else if (!name.endsWith('.db')) setCloud(null, '2D 맵에는 3D 정보가 없습니다');
}
$('refresh').onclick = () => current && current.endsWith('.db') && select(current, true);

// ---------------- 2D view (canvas, pan/zoom in map meters)
const c2d = $('c2d'), ctx = c2d.getContext('2d');
let img2d = null, view = { scale: 1, ox: 0, oy: 0 };   // screen = (px - ox) * scale
function loadGridImage(name) {
  return new Promise(res => {
    const im = new Image();
    im.onload = () => { img2d = im; $('msg2d').hidden = true; fit2d(); res(); };
    im.src = `/map/grid.png?name=${encodeURIComponent(name)}&t=${Date.now()}`;
  });
}
function resize2d() { const r = c2d.getBoundingClientRect(); c2d.width = r.width * devicePixelRatio; c2d.height = r.height * devicePixelRatio; }
function fit2d() {
  if (!img2d) return; resize2d();
  view.scale = Math.min(c2d.width / img2d.width, c2d.height / img2d.height) * 0.95;
  view.ox = img2d.width / 2 - c2d.width / 2 / view.scale; view.oy = img2d.height / 2 - c2d.height / 2 / view.scale;
  draw2d();
}
// map meters <-> image pixels (row 0 = top = max y)
const toPx = (x, y) => [(x - grid.origin[0]) / grid.resolution, grid.height_px - (y - grid.origin[1]) / grid.resolution];
const toMap = (u, v) => [grid.origin[0] + u * grid.resolution, grid.origin[1] + (grid.height_px - v) * grid.resolution];
function draw2d() {
  resize2d(); ctx.fillStyle = '#0f1115'; ctx.fillRect(0, 0, c2d.width, c2d.height);
  if (!img2d || !grid) return;
  ctx.save(); ctx.scale(view.scale, view.scale); ctx.translate(-view.ox, -view.oy);
  ctx.imageSmoothingEnabled = false; ctx.drawImage(img2d, 0, 0);
  // map origin axes (x red, y green), 1 m long
  const [u0, v0] = toPx(0, 0), m = 1 / grid.resolution, lw = 2 / view.scale;
  ctx.lineWidth = lw;
  ctx.strokeStyle = '#ff5252'; ctx.beginPath(); ctx.moveTo(u0, v0); ctx.lineTo(u0 + m, v0); ctx.stroke();
  ctx.strokeStyle = '#69f0ae'; ctx.beginPath(); ctx.moveTo(u0, v0); ctx.lineTo(u0, v0 - m); ctx.stroke();
  if (meta && meta.trajectory.length && $('traj2d').checked) {
    ctx.strokeStyle = '#4cc2ff'; ctx.lineWidth = 2.5 / view.scale; ctx.beginPath();
    meta.trajectory.forEach(([x, y], i) => { const [u, v] = toPx(x, y); i ? ctx.lineTo(u, v) : ctx.moveTo(u, v); });
    ctx.stroke();
    const dot = (p, color) => { const [u, v] = toPx(p[0], p[1]); ctx.fillStyle = color; ctx.beginPath();
                                ctx.arc(u, v, 5 / view.scale, 0, 7); ctx.fill(); };
    dot(meta.trajectory[0], '#69f0ae'); dot(meta.trajectory.at(-1), '#ffb74c');
  }
  ctx.restore();
  // scale bar
  const meters = [0.5, 1, 2, 5, 10].find(v => v / grid.resolution * view.scale > 60) || 10;
  const len = meters / grid.resolution * view.scale, x0 = c2d.width - len - 16 * devicePixelRatio, y0 = c2d.height - 16 * devicePixelRatio;
  ctx.fillStyle = '#e6e8eb'; ctx.fillRect(x0, y0, len, 3 * devicePixelRatio);
  ctx.font = `${12 * devicePixelRatio}px system-ui`; ctx.fillText(`${meters} m`, x0, y0 - 6 * devicePixelRatio);
}
let drag = null;
c2d.addEventListener('mousedown', e => { drag = [e.clientX, e.clientY]; c2d.classList.add('drag'); });
addEventListener('mouseup', () => { drag = null; c2d.classList.remove('drag'); });
c2d.addEventListener('mousemove', e => {
  if (drag) { view.ox -= (e.clientX - drag[0]) * devicePixelRatio / view.scale; view.oy -= (e.clientY - drag[1]) * devicePixelRatio / view.scale;
              drag = [e.clientX, e.clientY]; draw2d(); }
  if (!grid) return;
  const r = c2d.getBoundingClientRect();
  const u = (e.clientX - r.left) * devicePixelRatio / view.scale + view.ox, v = (e.clientY - r.top) * devicePixelRatio / view.scale + view.oy;
  const [x, y] = toMap(u, v);
  $('coord').hidden = false; $('coord').textContent = `x ${x.toFixed(2)} m, y ${y.toFixed(2)} m`;
});
c2d.addEventListener('mouseleave', () => $('coord').hidden = true);
c2d.addEventListener('wheel', e => {
  e.preventDefault(); const r = c2d.getBoundingClientRect();
  const sx = (e.clientX - r.left) * devicePixelRatio, sy = (e.clientY - r.top) * devicePixelRatio;
  const u = sx / view.scale + view.ox, v = sy / view.scale + view.oy;
  view.scale *= e.deltaY < 0 ? 1.2 : 1 / 1.2; view.ox = u - sx / view.scale; view.oy = v - sy / view.scale; draw2d();
}, { passive: false });
$('fit2d').onclick = fit2d; $('traj2d').onchange = draw2d; addEventListener('resize', () => { draw2d(); resize3d(); });

// ---------------- 3D view (three.js)
let THREE = null, renderer, scene, camera, controls, cloudObj = null, trajObj = null, cloudData = null;
try {
  THREE = await import('three');
  const { OrbitControls } = await import('three/addons/controls/OrbitControls.js');
  renderer = new THREE.WebGLRenderer({ canvas: $('c3d'), antialias: true });
  renderer.setPixelRatio(devicePixelRatio);
  scene = new THREE.Scene(); scene.background = new THREE.Color(0x0f1115);
  camera = new THREE.PerspectiveCamera(60, 1, 0.05, 500); camera.up.set(0, 0, 1); camera.position.set(-4, -4, 5);
  controls = new OrbitControls(camera, renderer.domElement); controls.enableDamping = true;
  const g = new THREE.GridHelper(20, 20, 0x3a4048, 0x262a31); g.rotation.x = Math.PI / 2; scene.add(g);
  scene.add(new THREE.AxesHelper(1));
  resize3d();
  (function loop() { requestAnimationFrame(loop); controls.update(); renderer.render(scene, camera); })();
} catch (e) {
  $('msg3d').textContent = '3D 표시용 three.js를 불러오지 못했습니다 (인터넷 연결 필요): ' + e.message;
}
function resize3d() {
  if (!renderer) return; const r = $('c3d').getBoundingClientRect();
  renderer.setSize(r.width, r.height, false); camera.aspect = r.width / r.height; camera.updateProjectionMatrix();
}
function setCloud(data, message) {
  cloudData = data;
  if (cloudObj) { scene.remove(cloudObj); cloudObj.geometry.dispose(); cloudObj = null; }
  if (trajObj) { scene.remove(trajObj); trajObj = null; }
  $('msg3d').hidden = !!data; if (message) $('msg3d').textContent = message;
  if (data && THREE) buildCloud();
}
async function loadCloud(name) {
  if (!THREE) return;
  const buf = await (await fetch(`/map/cloud.bin?name=${encodeURIComponent(name)}`)).arrayBuffer();
  if (current !== name) return;
  const n = meta.points;
  setCloud({ xyz: new Float32Array(buf, 0, n * 3), rgb: new Uint8Array(buf, n * 12, n * 3), label: new Uint8Array(buf, n * 15, n) });
  if (meta.bounds) {
    const [lo, hi] = meta.bounds, c = lo.map((v, i) => (v + hi[i]) / 2), span = Math.max(hi[0] - lo[0], hi[1] - lo[1], 2);
    controls.target.set(c[0], c[1], 0); camera.position.set(c[0] - span * 0.6, c[1] - span * 0.8, span * 0.9);
  }
}
function buildCloud() {
  if (cloudObj) { scene.remove(cloudObj); cloudObj.geometry.dispose(); }
  const { xyz, rgb, label } = cloudData, showGround = $('ground3d').checked, byHeight = $('color3d').value === 'height';
  const idx = []; for (let i = 0; i < label.length; i++) if (showGround || label[i]) idx.push(i);
  const pos = new Float32Array(idx.length * 3), col = new Float32Array(idx.length * 3);
  let zmin = Infinity, zmax = -Infinity; for (const i of idx) { zmin = Math.min(zmin, xyz[i * 3 + 2]); zmax = Math.max(zmax, xyz[i * 3 + 2]); }
  const c = new THREE.Color();
  idx.forEach((i, k) => {
    pos.set([xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]], k * 3);
    if (byHeight) { c.setHSL(0.66 * (1 - (xyz[i * 3 + 2] - zmin) / Math.max(zmax - zmin, 0.01)), 0.8, 0.55); col.set([c.r, c.g, c.b], k * 3); }
    else col.set([rgb[i * 3] / 255, rgb[i * 3 + 1] / 255, rgb[i * 3 + 2] / 255], k * 3);
  });
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3)); geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
  cloudObj = new THREE.Points(geo, new THREE.PointsMaterial({ size: parseFloat($('size3d').value), vertexColors: true }));
  scene.add(cloudObj);
  if (!trajObj && meta && meta.trajectory.length > 1) {
    const g = new THREE.BufferGeometry().setFromPoints(meta.trajectory.map(p => new THREE.Vector3(p[0], p[1], p[2] + 0.02)));
    trajObj = new THREE.Line(g, new THREE.LineBasicMaterial({ color: 0x4cc2ff })); scene.add(trajObj);
  }
}
for (const id of ['color3d', 'ground3d', 'size3d']) $(id).addEventListener('change', () => cloudData && buildCloud());

await loadList(); setInterval(loadList, 10000);
// Open a map straight from the URL, e.g. http://host:8090/#rtabmap.db
if (location.hash.length > 1) select(decodeURIComponent(location.hash.slice(1)));
addEventListener('hashchange', () => location.hash.length > 1 && select(decodeURIComponent(location.hash.slice(1))));
</script></body></html>"""


def make_handler(store: MapStore):

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.0'

        def log_message(self, fmt, *args):
            pass

        def _send(self, code, body: bytes, ctype: str):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj).encode(), 'application/json')

        def do_GET(self):
            url = urlparse(self.path)
            q = parse_qs(url.query)
            name = q.get('name', [''])[0]
            try:
                if url.path == '/':
                    self._send(200, PAGE.encode(), 'text/html; charset=utf-8')
                elif url.path == '/api/maps':
                    self._json({'dir': store.maps_dir, 'maps': store.list()})
                elif url.path == '/api/map':
                    if name.endswith('.yaml'):
                        g = store.grid_for(name)
                        g.pop('image')
                        self._json({'state': 'done', 'grid': g})
                    else:
                        st = store.db_status(name, refresh='refresh' in q)
                        if st['state'] == 'done' and st['meta']['has_grid']:
                            g = store.grid_for(name)
                            g.pop('image')
                            st['grid'] = g
                        self._json(st)
                elif url.path == '/map/grid.png':
                    self._send(200, grid_png(store.grid_for(name)['image']), 'image/png')
                elif url.path == '/map/cloud.bin':
                    with open(os.path.join(store.cache_dir(name), 'cloud.bin'), 'rb') as f:
                        self._send(200, f.read(), 'application/octet-stream')
                else:
                    self._json({'error': 'not found'}, 404)
            except FileNotFoundError as exc:
                self._json({'error': f'파일 없음: {exc}'}, 404)
            except (ValueError, KeyError, yaml.YAMLError) as exc:
                self._json({'error': str(exc)}, 400)
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description='Web viewer for ros2_slam/maps (2D grids + RTAB-Map databases)')
    ap.add_argument('--maps-dir', default=os.path.join(here, '..', 'maps'))
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--port', type=int, default=8090)
    args = ap.parse_args()
    store = MapStore(args.maps_dir)
    os.makedirs(CACHE_ROOT, exist_ok=True)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(store))
    server.daemon_threads = True
    print(f'map viewer: http://localhost:{args.port}/  (maps: {store.maps_dir}, cache: {CACHE_ROOT})')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
