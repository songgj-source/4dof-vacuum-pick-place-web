# map_web REST API

`map_web.py`가 제공하는 HTTP API. 웹 UI(`/`)도 이 API로 동작한다.
지도·로봇 위치, 수동 주행, 위치 라벨과 Nav2 이동(Go to), 위치 다시 잡기, 도착 오차, 카메라 영상,
깊이 근접 이동, 라이다 근접 이동을 다룬다.

!! 로봇을 움직이는 API(`/api/drive`, `/api/nav/goto`, `/api/approach/start`, `/api/wall/start`)는
주변이 안전한지 확인한 뒤 호출할 것 !!

## 서버 실행
노트북에서 원격으로 실행한다 (Pi의 tmux 세션 `map_web`, 로그 `~/map_web.log`):
```bash
./scripts/remote_map_web.sh start                         # http://192.168.0.115:8081
./scripts/remote_map_web.sh start -p port:=8082 -p max_linear:=0.1
./scripts/remote_map_web.sh status | stop | restart | log | attach
```
Pi에서 직접 실행할 때:
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
/usr/bin/python3 map_web.py --ros-args -p port:=8081
```

### 함께 실행되어야 하는 것
| 기능 | 필요한 것 |
|---|---|
| 위치(odom), 수동 주행 | 로봇 드라이버: `remote_car2_lidar.sh start` 또는 `remote_robot_rgbd.sh start` |
| 라이다 스캔, 라이다 근접 이동 | `remote_car2_lidar.sh start` (robot_rgbd와 함께면 `use_car2_driver:=false`) |
| 카메라 영상, 깊이 근접 이동 | `remote_robot_rgbd.sh start` |
| 지도, map 좌표 위치, 라벨 저장 | 노트북: `run_lidar_mapping.sh` 또는 `run_lidar_navigation.sh` |
| Go to, 위치 다시 잡기, 도착 오차 적용 | 노트북: `run_lidar_navigation.sh` (Nav2 + AMCL) |

### 노드 파라미터 (`-p 이름:=값`)
| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `port` | 8081 | HTTP 포트 |
| `map_frame` / `odom_frame` / `base_frame` | `map` / `odom` / `base_link` | TF 프레임 |
| `robot_radius` | 0.26 | 차체 좌우 반폭 (m). 경로 폭 판정, 지도의 로봇 원 |
| `robot_front` | 0.16 | 로봇 중심 → 차체 앞까지 (m). 근접 이동 정지 거리 기준 |
| `trail_max` | 3000 | 주행 궤적 최대 점 수 |
| `max_linear` / `max_angular` | 0.16 / 0.75 | 수동 주행 속도 상한 (m/s, rad/s) |
| `drive_hold_s` | 0.5 | 주행 명령 유지 시간 (s) |
| `labels_file` | `~/.ros/car2_map_labels.json` | 위치 라벨 저장 파일 |
| `reloc_min_match` | 0.5 | 위치 다시 잡기: 이 일치율 미만이면 AMCL에 보내지 않음 |
| `goal_xy_min` / `goal_xy_max` | 0.03 / 0.5 | 도착 오차(위치) 허용 범위 (m) |
| `goal_yaw_min_deg` / `goal_yaw_max_deg` | 3 / 90 | 도착 오차(방향) 허용 범위 (°) |
| `tolerance_file` | `~/.ros/car2_nav_tolerance.json` | 도착 오차 저장 파일 |
| `controller_node` / `goal_checker` | `/controller_server` / `general_goal_checker` | 도착 오차를 설정할 Nav2 노드/플러그인 |
| `camera_max_fps` / `camera_jpeg_quality` | 10 / 70 | MJPEG 스트림 최대 fps, JPEG 품질 |
| `depth_max_mm` | 4000 | 깊이 컬러맵 기본 최대 거리 |
| `rgb_topic` | `/camera/color/image_raw/compressed` | RGB (JPEG CompressedImage) |
| `depth_topic` | `/camera/depth_raw/image` | 깊이 (16UC1, mm) |
| `approach_file` | `~/.ros/car2_approach.json` | 깊이 근접 이동 설정 파일 |
| `approach_max_m` / `approach_timeout_s` | 1.5 / 60 | 깊이 근접 이동 최대 이동 거리 / 시간 |
| `approach_lidar_stop_m` | 0.12 | 깊이 근접 이동의 라이다 정지 여유 초기값 |
| `wall_file` | `~/.ros/car2_wall_approach.json` | 라이다 근접 이동 설정 파일 |
| `goto_backup_m` / `goto_backup_speed` | 0.30 / 0.08 | Go to 전에 후진할 거리 (m, 0이면 후진 안 함) / 속도 (m/s) |
| `robot_back` | 0.16 | 로봇 중심 → 차체 뒤까지 (m) |
| `goto_backup_clear_m` | 0.12 | 후진 중 차체 뒤에서 이 거리 안에 물체가 있으면 후진을 멈춤 |
| `goal_max_cost` | 50 | Go to 목표 칸의 비용지도 값 상한. 이 값 이상인 라벨은 가장 가까운 상한 미만 칸으로 옮겨 간다 (99면 예전처럼 막힌 라벨만 옮김) |
| `wall_max_m` / `wall_timeout_s` | 3.0 / 90 | 라이다 근접 이동 최대 이동 거리 / 시간 |

## 공통 사항
- Base URL: `http://192.168.0.115:8081`
- 요청/응답은 JSON. 요청 body가 없으면 `{}`로 취급한다. POST에 `Content-Type` 헤더는 필수가 아니다.
- 오류 응답은 `{"error": "메시지"}` 형식이고, 메시지는 한국어로 원인과 해결 방법을 알려 준다.
- 없는 경로는 `404 {"error": "not found"}`.
- 인증과 CORS 헤더는 없다. 같은 네트워크의 누구나 조종할 수 있으니 신뢰할 수 있는 네트워크에서만 사용한다.
- 좌표 단위: 위치 m, 각도 rad (이름이 `_deg`로 끝나면 °). 방향 0 = +X, 반시계 방향이 +.
- 시간 값(`started`, `finished`, `received` 등)은 Unix 시간(초).
- 설정 API로 바꾼 값은 Pi의 `~/.ros/*.json` 파일에 저장되어 재시작 후에도 유지된다.

## 안전 동작 (중요)
- **수동 주행 명령은 0.5초 동안만 유효하다.** 계속 움직이려면 0.5초보다 짧은 간격(권장 0.1초)으로 다시 보낸다.
  명령이 끊기면 서버가 정지(0 속도)를 보낸다. 서버가 죽어도 차량 드라이버의 1초 `/cmd_vel` 타임아웃으로 멈춘다.
- 속도는 `max_linear`(0.16 m/s), `max_angular`(0.75 rad/s)로 자동 제한된다.
- 자동 동작(Go to, 깊이 근접 이동, 라이다 근접 이동)은 **동시에 하나만** 실행된다.
  - `POST /api/drive` 또는 `/api/drive/stop`을 보내면 실행 중인 근접 이동은 모두 멈춘다.
  `/api/drive`는 Go to도 취소하고, `/api/drive/stop`은 Go to의 후진 단계를 취소한다.
  - Go to를 시작하면 근접 이동이 멈춘다.
  - 한 근접 이동을 시작하면 다른 근접 이동이 멈춘다.
- 근접 이동에는 각자 최대 이동 거리, 시간 제한, 센서 끊김, 장애물 정지 조건이 있다 (각 절 참고).
- 서버 종료 시 정지를 보낸다.

## 엔드포인트 요약
| 메서드 | 경로 | 설명 | 로봇 이동 |
|---|---|---|---|
| GET | `/` | 웹 UI 페이지 | |
| GET | `/api/state` | 전체 상태 (위치, 스캔, 지도, 라벨, Nav2, 설정 등) | |
| GET | `/api/map/meta` | 지도 정보 | |
| GET | `/api/get_map_info` | 계획용 요약: 내 위치, 라벨(거리·방향·도달 가능), 지도 범위, Nav2 상태 | |
| GET | `/map.png` | 지도 이미지 | |
| GET | `/stream/rgb.mjpg` | RGB 카메라 MJPEG | |
| GET | `/stream/depth.mjpg` | 깊이 영상 MJPEG (근접 이동 표시 포함) | |
| GET | `/api/drive` | 수동 주행 상태 | |
| POST | `/api/drive` | 수동 주행 (0.5초 유효) | **예** |
| POST | `/api/drive/stop` | 즉시 정지 | |
| POST | `/api/trail/clear` | 주행 궤적 지우기 | |
| GET | `/api/labels` | 위치 라벨 목록 | |
| POST | `/api/labels` | 라벨 추가 (현재 위치 또는 지정 좌표) | |
| POST | `/api/labels/delete` | 라벨 삭제 | |
| POST | `/api/labels/rename` | 라벨 이름 변경 | |
| POST | `/api/labels/tolerance` | 라벨별 도착 오차 설정/해제 | |
| POST | `/api/nav/goto` | 라벨 위치로 Nav2 자율 주행 (목표가 뒤쪽이면 먼저 30 cm 후진) | **예** |
| POST | `/api/nav/goto_wall` | Go to 후 도착하면 라이다 근접 이동 | **예** |
| POST | `/api/nav/cancel` | Nav2 목표 취소 | |
| GET | `/api/nav/tolerance` | 도착 오차 | |
| POST | `/api/nav/tolerance` | 도착 오차 설정 | |
| POST | `/api/relocalize` | 스캔 매칭으로 위치 다시 잡기 (AMCL) | |
| GET | `/api/approach` | 깊이 근접 이동 설정·상태 | |
| POST | `/api/approach/config` | 깊이 근접 이동 설정 | |
| POST | `/api/approach/start` | 깊이 근접 이동 시작 | **예** |
| POST | `/api/approach/stop` | 깊이 근접 이동 정지 | |
| GET | `/api/wall` | 라이다 근접 이동 설정·상태·벽 | |
| POST | `/api/wall/config` | 라이다 근접 이동 설정 | |
| POST | `/api/wall/start` | 라이다 근접 이동 시작 | **예** |
| POST | `/api/wall/stop` | 라이다 근접 이동 정지 | |

---

## 상태 · 지도

### GET `/api/state`
웹 UI가 0.2초마다 읽는 전체 상태다. 서버가 0.2초마다 다시 계산한다.

**응답 예시** (일부 생략)
```json
{
  "frame": "map",
  "pose": {"x": -3.172, "y": 1.243, "yaw": 0.037, "age": 0.04},
  "robot_radius": 0.26,
  "scan": [[-3.346, -0.337], [-3.346, -0.369]],
  "scan_hz": 6.0,
  "trail": [[0.28, 0.004], [0.249, 0.005]],
  "plan": [],
  "amcl": {"sx": 0.066, "sy": 0.061, "syaw": 0.03, "received": 1790930400.1},
  "map": {"version": 3, "frame": "map", "width": 291, "height": 189, "resolution": 0.05,
          "origin": [-8.455, -6.202, 0.0], "received": 1790930274.5, "known_m2": 56.6, "age": 4.3},
  "labels": [{"id": "994b2b49", "name": "시작위치", "x": -3.176, "y": 1.256, "yaw": 0.0307,
              "frame": "map", "created": "2026-10-02 17:42:45", "cost": 0}],
  "robot_cost": 0,
  "nav": {"state": "idle", "server": true},
  "tolerance": {"...": "GET /api/nav/tolerance와 같음"},
  "reloc": {"state": "idle", "amcl": true},
  "approach": {"...": "GET /api/approach와 같음"},
  "wall": {"...": "GET /api/wall와 같음"},
  "camera": {"rgb": {"fps": 29.5, "subscribed": true, "publishers": 1},
             "depth": {"fps": 9.5, "subscribed": true, "publishers": 1}}
}
```

| 필드 | 설명 |
|---|---|
| `frame` | 위치의 기준 좌표계. `map`(SLAM/AMCL 실행 중), 지도가 없으면 `odom`(바퀴 주행거리), TF가 없으면 `null` |
| `pose` | 로봇 위치 `x`, `y` (m), `yaw` (rad), `age` = TF 지연 (s). TF가 없으면 `null` |
| `scan` | 라이다 점 `[x, y]` 목록 (`frame` 좌표). 1초 넘게 오래된 스캔이면 빈 목록 |
| `scan_hz` | 라이다 수신 주기 (Hz) |
| `trail` | 지나온 길 (3 cm마다 한 점). `frame`이 바뀌면 초기화 |
| `plan` | Nav2 경로 (`/plan`, 최대 약 400점). 30초 지나면 빈 목록 |
| `amcl` | AMCL 위치 표준편차 `sx`, `sy` (m), `syaw` (rad). 30초 지나면 `null` |
| `map` | 지도 정보 (`/api/map/meta` + 수신 후 경과 `age` s). 지도가 없으면 `null` |
| `labels[].cost` | 라벨 위치의 Nav2 전역 비용지도 값 (0~100, -1 = 미탐색). **99 이상이면 그 자리에는 갈 수 없다** (Go to는 가장 가까운 갈 수 있는 지점까지 간다). 비용지도가 없으면 `null` |
| `robot_cost` | 로봇 현재 위치의 비용지도 값 (map 좌표일 때만) |
| `nav` | Go to 상태 (아래 `/api/nav/goto` 참고). `server` = Nav2 `navigate_to_pose` 사용 가능 여부 |
| `reloc` | 위치 다시 잡기 상태 (아래 `/api/relocalize` 참고). `amcl` = `/initialpose` 구독자(AMCL) 유무 |
| `camera` | 영상별 수신 `fps`, 구독 중인지(`subscribed`), 토픽 발행자 수(`publishers`, 0이면 robot_rgbd 꺼짐) |

### GET `/api/get_map_info`
LLM 같은 계획기에 넘기려고 만든 작은 요약이다. 스캔, 궤적, 지도 이미지는 넣지 않는다.
`hand_eye/llm_client.py`는 지시를 처리하는 매 단계에 이것을 읽어 프롬프트에 넣는다.
```json
{"frame": "map",
 "pose": {"frame": "map", "x": -3.18, "y": 1.358, "yaw": 0.0357, "yaw_deg": 2.0},
 "robot_cost": 0,
 "robot": {"radius": 0.26, "front": 0.16, "back": 0.16},
 "labels": [{"id": "1b4a6030", "name": "책상", "x": -3.975, "y": 0.822, "yaw": -3.1256, "yaw_deg": -179.1,
             "cost": 99, "reachable": false, "dist_m": 0.96, "bearing_deg": -146.0}],
 "map": {"frame": "map", "resolution": 0.05, "width_m": 14.55, "height_m": 9.45, "origin": [-8.455, -6.202, 0.0],
         "bounds": [-8.2, -5.9, 5.6, 3.0], "known_m2": 56.6, "age_s": 12.3},
 "nav": {"server": true, "state": "idle", "target": null, "message": null},
 "wall_ahead": {"dist_m": 1.938, "err_deg": 51.0},
 "units": "..."}
```
| 필드 | 설명 |
|---|---|
| `pose` | 로봇 위치. 2초 넘게 갱신이 없으면 `null`. `frame`이 `odom`이면 지도가 없다는 뜻 |
| `labels[].reachable` | 비용지도 값이 99 미만이거나 비용지도가 없으면 `true`. `false`여도 Go to는 가장 가까운 지점까지 간다 |
| `labels[].dist_m`, `bearing_deg` | 로봇에서 라벨까지 거리와 방향 (0 = 정면, + = 왼쪽, ±180 = 뒤). map 좌표일 때만 있다. 이때 목록은 가까운 순서다 |
| `map.bounds` | 탐색된(unknown이 아닌) 칸의 범위 `[x_min, y_min, x_max, y_max]` (m) |
| `wall_ahead` | 정면에 맞춘 벽 (2초 안의 결과만, 없으면 `null`) |

### GET `/api/map/meta`
마지막으로 받은 `/map` 정보다. 지도를 아직 받지 못했으면 `null`.
```json
{"version": 3, "frame": "map", "width": 291, "height": 189, "resolution": 0.05,
 "origin": [-8.455, -6.202, 0.0], "received": 1790930274.5, "known_m2": 56.6}
```
- `origin`: 지도 왼쪽 아래 칸의 `[x, y, yaw]`.
- `version`: 새 지도를 받을 때마다 1씩 증가.
- `known_m2`: 탐색된 면적.

### GET `/map.png?v=<version>`
지도 이미지(PNG, 흑백)다.
- 색: 빈 공간 = 밝은 회색(235), 장애물 = 검정, 미탐색 = 회색(128).
- 이미지 맨 위 행이 y 최대값이다(아래가 `origin`).
- `v`는 캐시 구분용이다. 응답은 1시간 캐시되므로 지도가 바뀌면 `v`를 새 `version`으로 바꿔 요청한다.
- 지도가 없으면 `404 {"error": "아직 /map을 받지 못했습니다"}`.

---

## 카메라

### GET `/stream/rgb.mjpg`
### GET `/stream/depth.mjpg?max=4000`
`multipart/x-mixed-replace; boundary=frame` MJPEG 스트림이다. `<img src="...">`로 바로 볼 수 있다.

- 최대 `camera_max_fps`(10) fps로 보낸다.
- RGB는 카메라의 JPEG를 그대로 전달한다.
- 깊이는 0~`max` mm를 JET 컬러맵으로 칠하고(측정 없음 = 검정), 640 폭으로 키운다. 그 위에 깊이 근접 이동 표시를 그린다:
  - 흰 근접선
  - 장애물 픽셀: 근접 영역 안 = 빨강, 밖 = 분홍
  - 하단 제외 영역은 어둡게
  - 근접 영역 장애물 픽셀 수
- 보는 클라이언트가 있을 때만 Pi가 카메라 토픽을 구독한다. 마지막 클라이언트가 떠나고 5초 뒤 구독을 해제한다.
- robot_rgbd가 꺼져 있으면 연결은 유지되지만 프레임이 오지 않는다(`/api/state`의 `camera.*.publishers`가 0).

---

## 수동 주행

### GET `/api/drive`
```json
{"linear": 0.0, "angular": 0.0, "moving": false, "max_linear": 0.16, "max_angular": 0.75,
 "hold_s": 0.5, "driver_connected": true, "odom": {"linear": 0.0, "angular": 0.0}}
```
| 필드 | 설명 |
|---|---|
| `linear` / `angular` | 현재 유효한 명령 (m/s, rad/s). 명령이 없으면 0 |
| `moving` | 명령이 유효 시간 안에 있는지 |
| `driver_connected` | `/cmd_vel` 구독자(차량 드라이버) 유무. `false`면 명령을 보내도 움직이지 않음 |
| `odom` | `/odom` 실제 속도. 2초 넘게 없으면 `null` |

### POST `/api/drive`
| 필드 | 타입 | 필수 | 설명 |
|---|---|---|---|
| `linear` | float | 아니오 (기본 0) | 직진 m/s. `+` 전진, `-` 후진. ±`max_linear`로 자동 제한 |
| `angular` | float | 아니오 (기본 0) | 회전 rad/s. `+` 왼쪽(반시계). ±`max_angular`로 자동 제한 |

```bash
# 0.1초마다 보내는 동안 0.08 m/s로 전진
while true; do curl -s -X POST http://192.168.0.115:8081/api/drive -d '{"linear":0.08}' >/dev/null; sleep 0.1; done
```
- 응답: `GET /api/drive`와 같은 상태.
- 받는 즉시 한 번 발행하고, 0.5초 동안 0.1초마다 다시 발행한다.
- 실행 중인 Go to와 근접 이동을 취소한다.
- 오류: `400 {"error": "잘못된 주행 명령: ..."}` (JSON 형식 오류 등). 이때 정지한다.

### POST `/api/drive/stop`
즉시 정지한다. 0 속도를 바로 보내고, 이어서 3번 더 보낸다. 실행 중인 근접 이동도 멈춘다(Go to는 취소하지 않으니 `/api/nav/cancel` 사용).
응답은 `GET /api/drive`와 같다.

### POST `/api/trail/clear`
지도에 그려지는 주행 궤적을 지운다. → `{"ok": true}`

---

## 위치 라벨 · Go to (Nav2)

라벨은 **map 좌표**의 위치·방향이고 Pi의 `labels_file`에 저장된다.
라벨을 만든 그 지도에서만 의미가 있으니, 내비게이션도 같은 저장 맵으로 실행해야 한다.

### GET `/api/labels`
```json
{"labels": [{"id": "994b2b49", "name": "시작위치", "x": -3.176, "y": 1.256, "yaw": 0.0307,
             "frame": "map", "created": "2026-10-02 17:42:45"}],
 "file": "/home/user/.ros/car2_map_labels.json"}
```
비용지도 값(`cost`)이 붙은 목록은 `/api/state`의 `labels`에 있다.

### POST `/api/labels`
| 필드 | 타입 | 필수 | 설명 |
|---|---|---|---|
| `name` | string | 예 | 라벨 이름 (1~40자, 중복 불가) |
| `x`, `y` | float | 아니오 | 지정하면 이 map 좌표로 저장. 생략하면 **로봇의 현재 map 위치·방향** |
| `yaw` | float | 아니오 | `x`, `y`를 지정할 때의 방향 (rad, 기본 0) |

```bash
curl -X POST http://192.168.0.115:8081/api/labels -d '{"name":"주방"}'
curl -X POST http://192.168.0.115:8081/api/labels -d '{"name":"문 앞","x":1.2,"y":-0.5,"yaw":1.57}'
```
응답: 저장된 라벨 `{"id", "name", "x", "y", "yaw", "frame", "created"}`.

| 상태 | 원인 |
|---|---|
| 400 | 이름 없음 / 40자 초과 / 이미 있는 이름 / 현재 위치를 모름(TF 없음) / map 좌표계 없음(SLAM·내비게이션 꺼짐) |
| 500 | 라벨 파일 저장 실패 |

### POST `/api/labels/delete`
`{"id": "994b2b49"}` → `{"ok": true}`. 없는 id면 `404 {"error": "없는 라벨입니다"}`.

### POST `/api/labels/rename`
`{"id": "1b4a6030", "name": "책상 앞"}` → 바뀐 라벨 객체를 돌려준다. 위치, 방향, id는 그대로다.
웹 UI에서는 라벨을 선택하고 **이름 변경** 버튼을 누르거나, 목록에서 라벨을 더블클릭한다.

| 상태 | 원인 |
|---|---|
| 400 | 빈 이름, 40자 초과, 이미 있는 이름 |
| 404 | 없는 라벨 |
| 500 | 라벨 파일 저장 실패 |

### POST `/api/labels/tolerance`
라벨마다 도착 오차를 따로 정한다. 그 라벨로 Go to(`/api/nav/goto`, `/api/nav/goto_wall`)를 하는 동안에만 Nav2 goal checker에 이 값을 쓴다.
정하지 않은 라벨은 `/api/nav/tolerance`의 기본값을 쓴다.
```bash
curl -X POST http://192.168.0.115:8081/api/labels/tolerance -d '{"id":"1b4a6030","xy":0.05,"yaw_deg":10}'   # 설정
curl -X POST http://192.168.0.115:8081/api/labels/tolerance -d '{"id":"1b4a6030"}'                         # 해제 (기본값 사용)
```
- 응답은 바뀐 라벨이다. 설정한 라벨에는 `"tol": {"xy": 0.05, "yaw_deg": 10.0}`가 붙는다. 이 값은 `/api/labels`, `/api/state`, `/api/get_map_info`의 라벨에도 나온다.
- 범위는 기본값과 같다 (`limits`).
- 범위 밖이거나 `xy`, `yaw_deg` 중 하나만 주면 400, 없는 라벨이면 404.
- **적용 시점:**
  - Go to가 목표를 보낼 때 적용한다. 후진 단계가 있으면 후진이 끝난 뒤다. 약 0.2초 안에 Nav2에 들어간다.
  - 목표가 끝나면(도착, 실패, 취소, 거부) 기본값으로 돌아간다.
  - 적용 중에는 `/api/state`의 `tolerance.override`에 `{"xy", "yaw_deg", "label"}`이 나온다.
- 웹 UI 사용법:
  - 라벨을 선택하고 **도착 오차** 줄에서 값을 맞춘 뒤 **'라벨'에 저장**을 누른다. **라벨 값 지우기**를 누르면 기본값으로 돌아간다.
  - 라벨 목록의 "도착 오차" 열에 값이나 "기본"이 보인다.
  - 값이 있는 라벨을 선택하면 그 값이 입력칸에 채워진다.

### POST `/api/nav/goto`
`{"id": "994b2b49"}` 라벨의 위치·방향으로 Nav2 `navigate_to_pose` 목표를 보낸다(RViz "Nav2 Goal"과 같음).
**목표가 로봇 뒤쪽이면, 즉 로봇 정면에서 본 목표 방향이 ±90°를 넘으면, 먼저 똑바로 `goto_backup_m`(30 cm) 후진한다.**
목표가 앞쪽이거나 map 좌표의 현재 위치를 모르면 후진하지 않는다. 그 이유는 `nav.note`에 남는다.
**로봇이 움직인다.**

**후진 단계**
- 속도 `goto_backup_speed`(0.08 m/s)로 후진하며, 거리는 odom으로 잰다.
- 다음 경우 후진을 멈추고 바로 Nav2 목표를 보낸다. 이유는 `nav.note`에 남는다.
  - 라이다로 차체 뒤(중심에서 `robot_back` + `goto_backup_clear_m` = 0.28 m 안, 로봇 폭 안)에 물체가 보일 때
  - 후진이 15초 안에 끝나지 않을 때
- Nav2가 실행 중이 아니면 후진하지 않고 바로 `unavailable`이 된다.
- 다음은 후진 중에도 즉시 멈추고 Go to를 취소한다:
  - `/api/nav/cancel`
  - `/api/drive`
  - `/api/drive/stop`
- 진행 중인 Go to가 있을 때 새로 요청하면, 기존 목표를 취소하고 다시 후진부터 시작한다.
- **라벨 위치가 막혀 있거나 장애물에 너무 가까우면, 가장 가까운 비용 낮은 지점까지 간다.**
  - 라벨 칸의 비용지도 값이 `goal_max_cost`(50) 이상이면 대신 가장 가까운 칸으로 간다.
    이 칸은 비용이 `goal_max_cost` 미만이고, 로봇이 지금 있는 빈 영역(비용 99 미만)과 이어져 있다.
  - 이유: 비용 99 바로 아래 칸(장애물 옆 팽창 영역)을 목표로 주면, MPPI의 CostCritic이 그 안으로 들어가지 않는다.
    그래서 로봇이 목표 앞에서 멈추고 `Failed to make progress`가 반복됐다.
  - 지금 설정(팽창 반경 0.4 m)에서는 목표 중심이 장애물에서 약 0.4 m 떨어진다. 차체 앞으로는 약 24 cm가 남는다.
    더 붙어야 하면 `/api/nav/goto_wall`로 도착 뒤 벽 근접 이동을 한다.
  - 비용 50~98인 라벨 근처에 비용 낮은 칸이 없으면 라벨 위치 그대로 간다.
  - 라벨에서 1.5 m 안에 있는 칸만 고른다.
  - 방향은 라벨의 방향을 그대로 쓴다.
  - 도착하면 멈춘다.
  - 이때 `nav.target`은 옮긴 좌표다. `orig`에 원래 라벨 좌표, `shift`에 옮긴 거리(m)가 있다.
    `nav.label`에는 원래 라벨이 있다. `nav.note`에는 "라벨 위치가 막혀 있어(cost 99) 18 cm 떨어진 가장 가까운 지점으로"가 남는다.
  - 가는 도중에 장애물이 새로 보여 Nav2가 `aborted`로 끝날 수 있다. 이때 가장 가까운 지점이 5 cm 넘게 바뀌었으면
    그 지점으로 다시 시도한다 (최대 2번, `nav.retries`).

```bash
curl -X POST http://192.168.0.115:8081/api/nav/goto -d '{"id":"994b2b49"}'
```
- 응답: `{"ok": true, "target": {라벨}}`.
  - 이것은 요청이 접수되었다는 뜻일 뿐이다.
  - 진행과 결과는 `/api/state`의 `nav`에서 확인한다.
- 실행 중인 근접 이동을 멈추고 시작한다.

| 상태 | 원인 |
|---|---|
| 404 | 없는 라벨 |
| 400 | 라벨 위치의 비용지도 값이 99 이상이고, 1.5 m 안에 로봇이 지금 위치에서 갈 수 있는 지점도 없음 |

**`nav` 상태** (`/api/state`):
```json
{"state": "active", "target": {라벨}, "started": 1790932078.0, "message": "이동 중",
 "backed": 0.302, "note": "후진 30 cm 완료",
 "distance": 1.42, "eta": 12.0, "recoveries": 0, "server": true}
```
| `state` | 의미 |
|---|---|
| `idle` | 목표 없음 |
| `backing` | Go to 전 후진 중 (`backed` = 후진한 거리 m) |
| `sending` | 목표 전송 중 |
| `active` | 이동 중 (`distance` 남은 거리 m, `eta` 예상 남은 시간 s, `recoveries` 복구 동작 횟수) |
| `canceling` | 취소 요청됨 |
| `succeeded` | 도착 |
| `failed` | 후진 중 odom TF 없음, 또는 Nav2는 도착이라고 했지만 실제로는 목표에서 `max(0.35, 도착오차 + 0.15)` m 넘게 떨어져 있음. 목표가 장애물 영역이거나 TF 지연 오판 |
| `aborted` | 실패 (경로 없음, 막힘) |
| `canceled` | 취소됨 (후진 중 취소 포함) |
| `rejected` | Nav2가 목표를 거부함 |
| `unavailable` | Nav2가 실행 중이 아님 |

### POST `/api/nav/goto_wall`
`/api/nav/goto`와 같다. 다만 Nav2가 도착(`succeeded`)하면 이어서 라이다 근접 이동(`/api/wall/start`)을 시작한다.
라이다 근접 이동은 정면 벽과 수직으로 맞춘 뒤 `gap`까지 전진하는 동작이다.
라벨 방향을 벽(책상) 쪽으로 저장해 두면 "책상 앞까지 가서 붙기"를 한 번에 할 수 있다.

```bash
curl -X POST http://192.168.0.115:8081/api/nav/goto_wall -d '{"id":"1b4a6030"}'
```
- 요청 본문, 응답, 오류는 `/api/nav/goto`와 같다.
- 웹 UI: 라벨을 선택한 뒤 **Go to + 벽 근접** 버튼을 누른다.
- 도착하고 0.5초 기다린 뒤 근접 이동을 시작한다. 멈춘 상태에서 찍힌 스캔을 쓰기 위해서다.
  - 정면에 벽이 안 잡히면 3초 동안 다시 시도한 뒤 포기한다.
  - Go to가 `succeeded`가 아니면(실패, 취소, 도착 오판) 근접 이동은 하지 않는다.
- 진행 상황은 `/api/state`의 `nav`와 `wall`에서 확인한다.
  - `nav.then` = `"wall"`
  - `nav.then_state`: `pending`(도착, 시작 대기) → `started`(근접 이동 시작), `failed`(벽 못 찾음 등), `done`(이미 목표 거리 안)
  - 이후 진행과 결과는 `wall.state`에 나온다.
- 대기 중(0.5~3.5초)에 `/api/nav/cancel`, `/api/drive`, `/api/drive/stop`을 보내면 근접 이동을 시작하지 않는다.
  근접 이동이 시작된 뒤에는 `/api/wall/stop`이나 `/api/drive/stop`으로 멈춘다.

### POST `/api/nav/cancel`
진행 중인 Go to를 취소한다. → `{"ok": true}` (진행 중인 목표가 없어도 같은 응답).

### GET `/api/nav/tolerance`
```json
{"saved": {"xy": 0.05, "yaw_deg": 14.0}, "active": {"xy": 0.05, "yaw_deg": 14.0},
 "message": "Nav2에 적용됨", "limits": {"xy": [0.03, 0.5], "yaw_deg": [3.0, 90.0]}}
```
- `saved`: 이 서버에 저장된 값.
- `active`: 실행 중인 Nav2(`controller_server`)의 실제 값. Nav2가 꺼져 있으면 `null`.

### POST `/api/nav/tolerance`
`{"xy": 0.05, "yaw_deg": 14}`: Nav2 도착 판정 오차(위치 m, 방향 °)를 설정한다.

- **재시작 없이 2초 안에** 실행 중인 Nav2에 적용된다.
- 저장되므로 Nav2를 다시 시작해도 다시 적용되며, yaml 값보다 우선한다.

| 상태 | 원인 |
|---|---|
| 400 | `xy`/`yaw_deg` 누락, 또는 `limits` 범위 밖 |
| 500 | 저장 실패 |

---

## 위치 다시 잡기

### POST `/api/relocalize`
현재 라이다 스캔을 지도와 맞춰(스캔 매칭) 로봇 위치를 찾고, AMCL에 `/initialpose`로 보낸다(RViz "2D Pose Estimate"와 같음).
로봇은 움직이지 않는다.

| 필드 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `global` | bool | false | `false`: 현재 추정 위치 주변 ±1 m, ±40° 검색 (약 2~3초). `true`: 지도 전체, 모든 방향 검색 (약 10초) |
| `dry_run` | bool | false | `true`면 계산만 하고 AMCL에 보내지 않음 |

응답은 `{"ok": true}`(요청 접수)다. 결과는 `/api/state`의 `reloc`에서 확인한다.
```json
{"state": "done", "message": "위치를 다시 잡았습니다", "dry_run": false, "amcl": true,
 "result": {"x": -3.16, "y": 1.258, "yaw": 0.0335, "match": 0.905, "before": 0.462,
            "ambiguous": false, "global": false, "seconds": 2.1, "shift": 0.228, "turn": 0.0131}}
```
| 필드 | 설명 |
|---|---|
| `state` | `idle` / `running` / `done` / `failed` |
| `result.match` | 찾은 위치의 스캔 일치율 (벽에서 10 cm 안에 든 점의 비율). `reloc_min_match`(0.5) 미만이면 `failed`이고 AMCL에 보내지 않음 |
| `result.before` | 기존 추정 위치의 일치율 |
| `result.shift` / `result.turn` | 보정량 (m, rad) |
| `result.ambiguous` | 다른 곳에도 비슷하게 맞는 후보가 있음. AMCL에 넓은 불확실성으로 보내고 경고 |

| 상태 | 원인 |
|---|---|
| 409 | 이미 실행 중, 또는 Go to 진행 중 |

다음 원인은 비동기로 `reloc.state = failed`가 되며, `message`에 이유가 나온다:
- 지도 없음
- AMCL 없음
- 로봇이 움직이는 중
- 라이다 스캔 없음
- TF 없음
- 스캔 점 부족

---

## 깊이 근접 이동

깊이 카메라는 바닥을 내려다본다(버드뷰). 영상 아래쪽이 로봇에 가깝다.

- **장애물 판정:** 카메라에서 `obstacle_mm`보다 가까운 깊이 픽셀이 장애물이다(바닥은 약 1.3~2 m).
- **근접 영역:** 영상 높이의 `line` 비율 행(근접선)부터, 아래 `ignore_bottom` 비율(로봇 몸체)을 뺀 곳까지다.
- **동작:** 시작하면 `speed`로 전진하고, 근접 영역 안 장애물 픽셀이 `min_px` 이상이 되면 멈춘다.

### GET `/api/approach`
```json
{"line": 0.7, "obstacle_mm": 1100, "ignore_bottom": 0.22, "speed": 0.05, "min_px": 40, "lidar_stop_m": 0.12,
 "state": "idle", "message": "", "moved": 0.0, "zone_px": 387, "zone_invalid": 0.15,
 "max_m": 1.5, "robot_front": 0.16, "lidar_front": 0.356,
 "limits": {"line": [0.05, 0.95], "obstacle_mm": [300, 6000], "ignore_bottom": [0.0, 0.5],
            "speed": [0.02, 0.1], "min_px": [5, 5000], "lidar_stop_m": [0.0, 1.0]}}
```
| 필드 | 설명 |
|---|---|
| `state` | `idle` / `running` / `done`(장애물이 근접선 안으로 들어와 정지) / `stopped`(정지 버튼, 수동 조작 등) / `failed`(안전 정지) |
| `moved` | 이번 실행에서 이동한 거리 (m) |
| `zone_px` | 최신 깊이 프레임에서 근접 영역 안 장애물 픽셀 수. 깊이 영상이 없으면 `null` |
| `zone_invalid` | 근접 영역 중 깊이 측정이 없는 픽셀 비율 |
| `lidar_front` | 차체 앞(x > `robot_front`), 로봇 폭 안에서 가장 가까운 라이다 점 (중심 기준 m) |

### POST `/api/approach/config`
바꿀 항목만 보낸다. 범위는 `limits`를 따른다.

| 필드 | 설명 |
|---|---|
| `line` | 근접선 위치 (영상 위 0 ~ 아래 1) |
| `obstacle_mm` | 장애물 기준 깊이 (mm, 정수) |
| `ignore_bottom` | 판정에서 뺄 아래쪽 비율 (로봇 몸체) |
| `speed` | 전진 속도 (m/s) |
| `min_px` | 정지 기준 장애물 픽셀 수 (정수) |
| `lidar_stop_m` | 라이다 정지 여유: 차체 앞에서 이 거리 안에 물체가 있으면 정지 (중심에서 `robot_front` + 이 값) |

```bash
curl -X POST http://192.168.0.115:8081/api/approach/config -d '{"line":0.65,"obstacle_mm":1000}'
```
- 응답: `GET /api/approach`와 같다.
- 오류: `409`. 범위 밖, 또는 근접선이 하단 제외 영역 안에 있음(`line ≥ 1 - ignore_bottom - 0.02`).

### POST `/api/approach/start`
**로봇이 전진한다.** 응답은 `GET /api/approach`와 같다.

| 상태 | 원인 |
|---|---|
| 409 | 이미 실행 중 / Go to 진행 중 / 드라이버 미연결 / 깊이 카메라 토픽 없음 |

**정지 조건**
| 결과 | 조건 |
|---|---|
| `done` | 근접 영역 안 장애물 ≥ `min_px` (시작할 때 이미 그렇다면 움직이지 않고 바로 `done`) |
| `failed` | 깊이 영상 0.5초 끊김 |
| `failed` | 근접 영역의 50% 넘게 측정 불가 (카메라 최소 거리 약 0.6 m보다 가까움) |
| `failed` | 라이다: 차체 앞 `lidar_stop_m` 안에 물체 |
| `failed` | `approach_max_m`(1.5 m) 이동 |
| `failed` | `approach_timeout_s`(60초) 경과 |
| `stopped` | `/api/approach/stop`, `/api/drive`, `/api/drive/stop`, Go to, 라이다 근접 이동 시작 |

### POST `/api/approach/stop`
정지한다. → `GET /api/approach`와 같은 응답.

---

## 라이다 근접 이동

라이다로 정면의 벽을 찾고, 벽과 수직이 되도록 돈 다음, 최대한 가까이 전진한다.

**벽 인식**
- 정면 ±40°(가까우면 ±65°), 3 m 안의 라이다 점에서 RANSAC으로 직선을 찾는다.
- 벽은 로봇을 마주 보는 직선이어야 한다(벽의 법선이 ±50° 안). 옆 벽은 무시한다.
- 차체 앞(x < `robot_front`)의 점, 즉 로봇 자신의 바퀴·프레임은 제외한다.

**동작 단계**
1. `align`: 벽과 수직이 되도록 제자리 회전. 목표 각도 근처에서는 짧게 돌고 새 스캔으로 다시 확인한다.
2. `approach`: 방향을 보정하며 감속 전진한다. 벽까지 `robot_front + gap`(중심 기준)이 되면 멈춘다.
3. `final`: 마지막으로 각도를 보정한다.

### GET `/api/wall`
```json
{"gap": 0.1, "tol_deg": 1.5, "speed": 0.06,
 "state": "idle", "phase": "align", "message": "", "moved": 0.0,
 "target": 0.26, "max_m": 3.0, "robot_front": 0.16,
 "wall": {"dist": 0.812, "err_deg": -0.3, "inliers": 72, "rms": 0.0044,
          "ends": [[0.81, -0.62], [0.82, 0.55]]},
 "corridor": 0.293,
 "limits": {"gap": [0.02, 0.5], "tol_deg": [0.5, 5.0], "speed": [0.02, 0.1]}}
```
| 필드 | 설명 |
|---|---|
| `state` | `idle` / `running` / `done` / `stopped` / `failed` |
| `phase` | 실행 중 단계: `align` / `approach` / `final` |
| `target` | 정지 거리 = `robot_front` + `gap` (로봇 중심 → 벽, m) |
| `wall` | 현재 인식된 벽. 1초 넘게 못 찾으면 `null` |
| `wall.dist` | 로봇 중심 → 벽 수직 거리 (m) |
| `wall.err_deg` | 벽과 수직이 되려면 돌아야 할 각도 (°, `+` = 왼쪽) |
| `wall.inliers` / `wall.rms` | 벽 직선에 맞은 점 수 / 직선 오차 (m) |
| `wall.ends` | 벽 직선 양 끝 (base_link 좌표 m) |
| `corridor` | 차체 앞, 로봇 폭(±`robot_radius`) 안에서 가장 가까운 라이다 점 (중심 기준 m) |

### POST `/api/wall/config`
| 필드 | 범위 | 설명 |
|---|---|---|
| `gap` | 0.02 ~ 0.5 | 목표 여유: 차체 앞에서 벽까지 남길 거리 (m, 소수 3자리) |
| `tol_deg` | 0.5 ~ 5 | 수직 판정 허용 각도 (°) |
| `speed` | 0.02 ~ 0.10 | 최대 전진 속도 (m/s) |

- 응답: `GET /api/wall`와 같다. 범위 밖이면 `409`.
- 라이다는 0.25 m보다 가까운 점을 버린다. 그래서 `target`이 약 0.27 m 미만이면 정면 점이 보이지 않고 옆쪽 점만으로 벽을 판정한다(정확도 조금 떨어짐).

### POST `/api/wall/start`
**로봇이 회전하고 전진한다.** 응답은 `GET /api/wall`와 같다.

| 상태 | 원인 |
|---|---|
| 409 | 이미 실행 중 / Go to 진행 중 / 드라이버 미연결 / 정면에 벽 없음 / 이미 목표 거리 안 |

**정지 조건**
| 결과 | 조건 |
|---|---|
| `done` | 목표 거리 도달 후 각도 보정 완료. 마지막 각도 보정이 15초 안에 끝나지 않아도 `done`. `message`에 최종 거리·여유·각도 오차·이동 거리가 나옴 |
| `failed` | 라이다 스캔 0.6초 끊김 |
| `failed` | 벽을 1초 넘게 놓침 |
| `failed` | 로봇 폭 안에 물체가 목표 거리보다 1.5 cm 넘게 가까움 (벽 앞의 다른 물체) |
| `failed` | `wall_max_m`(3 m) 이동 |
| `failed` | `wall_timeout_s`(90초) 경과 |
| `stopped` | `/api/wall/stop`, `/api/drive`, `/api/drive/stop`, Go to, 깊이 근접 이동 시작 |

### POST `/api/wall/stop`
정지한다. → `GET /api/wall`와 같은 응답.

---

## 예시: 라벨로 이동한 뒤 벽 앞에 붙기
```bash
B=http://192.168.0.115:8081
ID=$(curl -s $B/api/labels | python3 -c "import json,sys; print([l['id'] for l in json.load(sys.stdin)['labels'] if l['name']=='충전기 앞'][0])")
curl -s -X POST $B/api/nav/goto -d "{\"id\":\"$ID\"}"
# 도착까지 기다리기
until curl -s $B/api/state | python3 -c "import json,sys; n=json.load(sys.stdin)['nav']; sys.exit(0 if n['state'] not in ('sending','active','canceling') else 1)"; do sleep 1; done
curl -s $B/api/state | python3 -c "import json,sys; print(json.load(sys.stdin)['nav']['state'])"   # succeeded?
# 벽과 수직으로 맞추고 차체 앞 10 cm까지
curl -s -X POST $B/api/wall/config -d '{"gap":0.10}' >/dev/null
curl -s -X POST $B/api/wall/start
until curl -s $B/api/wall | python3 -c "import json,sys; sys.exit(0 if json.load(sys.stdin)['state']!='running' else 1)"; do sleep 0.5; done
curl -s $B/api/wall | python3 -c "import json,sys; w=json.load(sys.stdin); print(w['state'], w['message'])"
```
