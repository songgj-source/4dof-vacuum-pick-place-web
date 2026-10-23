# ros2_slam — car2 + Depth 카메라 SLAM / Nav2 (ROS 2 Jazzy)

`car2`(ESP32 + ST3215 서보 4개, 스키드 스티어) 실물 로봇에 Orbbec Astra Pro
depth 카메라를 얹어 ROS 2 Jazzy로 맵을 수집/저장하고, 저장된 맵으로 Nav2
내비게이션을 돌리기 위한 워크스페이스입니다. 시뮬레이션이 아니라 **실제
하드웨어** 대상입니다.

## 구성

```
ros2_slam/
  src/
    car2_driver/     # cmd_vel -> car2 시리얼 프로토콜, 서보 피드백 -> odom/TF
    car2_bringup/     # launch(bringup/mapping/navigation) + slam_toolbox/Nav2 설정
  maps/                # map_saver_cli로 저장한 맵 (yaml + pgm)
  scripts/             # build_ws.sh, run_mapping.sh, save_map.sh, run_navigation.sh
```

### 파이프라인

```
depth 카메라(OpenNI2) --depth image--> depthimage_to_laserscan --/scan-->
  slam_toolbox (매핑) 또는 nav2 AMCL (내비게이션)
car2_driver: /cmd_vel -> 시리얼(D,<left>,<right>) / STAT 피드백 -> /odom, odom->base_link TF
```

- `car2_driver`가 발행하는 `/odom` + `base_to_camera_tf`(정적) + depth 카메라가
  발행하는 `camera_link` 하위 TF들이 합쳐져 `map -> odom -> base_link ->
  camera_link -> ... -> camera_depth_frame`까지 이어지고, 여기서 `/scan`이
  나옵니다. 라이다 없이 depth 카메라만으로 2D SLAM/Nav2를 돌리는 표준적인
  방식입니다.

## 카메라 드라이버: Orbbec Astra Pro는 Orbbec 자체 OpenNI2 런타임이 필요함

Ubuntu apt의 `ros-jazzy-openni2-camera` + `libopenni2-0`(PrimeSense 계열
PS1080 드라이버)만으로는 이 카메라(`lsusb`상 `2bc5:060f` Depth Sensor +
`2bc5:050f` USB 2.0 Camera — RGB가 UVC로 분리 노출되는 Astra Pro 구조)가
**인식되지 않습니다** (`list_devices` → 0 devices). Ubuntu가 배포하는
드라이버가 Orbbec의 OEM 벤더 ID를 지원하지 않기 때문이며
(`hand_eye/test4.py`의 Windows 쪽 메모에 있는 "Orbbec SDK v2는 이 모델을
지원하지 않는다"와 같은 종류의 벤더 호환성 문제), udev 권한 문제가 아닙니다.

**해결: Orbbec이 공식 배포하는 OpenNI2 Linux SDK**
(https://www.orbbec.com/developers/openni-sdk/ → `dl.orbbec3d.com`)에는
Orbbec 벤더 ID가 포함된 자체 `liborbbec.so` 드라이버 + `libOpenNI2.so`
코어가 들어있고, 이 카메라를 정상적으로 찾아 스트리밍합니다. 아래 스크립트가
필요한 부분(코어 라이브러리 + 드라이버 .so, 약 2.5MB)만 내려받아
`third_party/orbbec_openni2/`에 설치하고, 공식 udev 규칙도 등록합니다.

```bash
cd ros2_slam
./scripts/install_orbbec_openni2.sh
```

`bringup.launch.py`는 `openni2_lib_override_dir` 인자(기본값이 바로 이
`third_party/orbbec_openni2/` 경로)로 카메라 프로세스의 `LD_LIBRARY_PATH`
맨 앞에 이 런타임을 끼워 넣어서, `ros-jazzy-openni2-camera` 패키지를
재빌드하지 않고도 Orbbec의 라이브러리로 카메라를 인식하게 합니다. 스크립트를
아직 안 돌렸으면 이 경로가 없으므로 조용히 시스템 기본 라이브러리로
폴백합니다(= 다시 0 devices).

Orbbec의 공식 ROS2 저장소(`orbbec/ros_astra_camera`의 `ros2-development`
브랜치)는 README에 "not stable, please do not use it"이라고 명시되어 있어
사용하지 않았습니다. 대신 기존 `ros-jazzy-openni2-camera` 패키지(ROS
표준 `sensor_msgs/Image` 토픽, `depthimage_to_laserscan`과 바로 호환)에
Orbbec의 런타임만 얹는 방식을 택했습니다.

또한 이 드라이버 버전은 `depth_registration=True`일 때 Astra Pro의 UVC
컬러 스트림을 "640x480@30Hz RGB888"로 열려고 시도하는데, 이 모드를 카메라가
지원하지 않아 컬러 스트림이 실패합니다(로그에 `Unsupported color video
mode` 에러가 찍히지만 무해합니다 — 컬러/뎁스 모드 설정은 각각 독립적으로
처리되어 뎁스에는 영향 없음). 2D 라이다 대체용으로는 컬러 정렬이 필요
없으므로 `bringup.launch.py`는 `depth_registration=False`로 카메라를
직접 구성하고, 미등록 뎁스 스트림 토픽 `depth_raw/image` +
`depth_raw/camera_info`를 구독합니다(등록된 `depth/image`가 아님 — 이
드라이버 버전에는 `depth/image_raw`라는 이름의 토픽 자체가 없습니다).

## 사전 준비

```bash
export ROS_DISTRO=jazzy
sudo apt update
sudo apt install -y ros-jazzy-openni2-camera ros-jazzy-depthimage-to-laserscan \
  ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-teleop-twist-keyboard python3-serial
```

Orbbec 카메라 런타임 + udev 규칙 설치(이미 이 환경엔 적용됨, 새 PC에서
다시 하려면):

```bash
cd ros2_slam
./scripts/install_orbbec_openni2.sh
```

car2 ESP32는 USB(Type-C, CP2102)로 이 PC에 연결합니다. 보통
`/dev/ttyUSB0`로 잡히고, `dialout` 그룹 소속이어야 포트를 열 수 있습니다
(`sudo usermod -aG dialout $USER` 후 재로그인).

빌드:

```bash
cd ros2_slam
./scripts/build_ws.sh
```

## ⚠️ 반드시 실측 후 조정해야 하는 값

이 리포에는 car2의 실제 치수(바퀴 반지름, 좌우 바퀴 간격, 카메라 장착
위치)가 없어서 아래 값들은 **전부 플레이스홀더**입니다. 실측 없이 그대로
쓰면 odometry가 틀리고 Nav2가 엉뚱하게 움직입니다.

| 값 | 위치 | 기본값(placeholder) | 비고 |
|---|---|---|---|
| `wheel_radius_m` | launch 인자 / `car2_driver` 파라미터 | 0.032 | 바퀴 반지름 실측 |
| `wheel_separation_m` | launch 인자 / `car2_driver` 파라미터 | **0.51 (실효 윤거)** | 스키드 스티어 실효 윤거. 실측 윤거는 0.43m지만 회전 시 바퀴 미끄러짐 때문에 0.43이면 yaw가 ~19% 과대 추정됨(RTAB-Map 영상 정합 대비). 축거 0.20m는 운동학에 쓰이지 않음 |
| `base_to_camera_*` (x,y,z,roll,pitch,yaw) | `bringup.launch.py` 인자 | x=0.08, z=0.15, 나머지 0 | base_link(바퀴 중심 지면 투영점) -> 카메라 광학 중심 실측 |
| `robot_radius` | `config/nav2_params.yaml` | 0.26 | 윤거·축거·바퀴 반지름으로 계산한 바퀴 바깥 모서리 반경(약 0.252m). 차체가 바퀴보다 크면 늘릴 것 |
| 속도 제한(vx_max 등) | `config/nav2_params.yaml` | 보수적으로 낮게 설정 | 실측 후 올리기 |

`steps_per_rev`(4096)와 `max_step_speed`(3400)는 `car2/car2/Car.h`의
ST3215 스펙에서 그대로 가져온 값이라 바꿀 필요 없습니다.

`car2_driver`는 `STAT` 명령으로 받은 서보 실측 속도(spd)를 이용해
odometry를 적분합니다(위치값 pos는 회전마다 랩어라운드되는 원시 인코더
값이라 그대로 적분할 수 없어서, 속도 기반 dead-reckoning을 사용).
`car2/car2/Car.cpp`의 `INVERT_DEFAULT`가 바뀌면 `car2_driver/car2_serial_node.py`
상단의 `INVERT` 딕셔너리도 같이 맞춰야 합니다.

## 사용법

### 1) 맵 수집

터미널 A (카메라 + depth->scan + slam_toolbox + RViz):
```bash
cd ros2_slam
./scripts/run_mapping.sh                 # RViz 없이: ./scripts/run_mapping.sh use_rviz:=false
```
RViz(`car2_bringup/rviz/mapping.rviz`)가 같이 뜨며 `/map`, `/scan`, TF, `/odom`을
`map` 프레임 기준으로 보여줍니다. 왼쪽 SlamToolboxPlugin 패널의 **Save Map**으로도
맵을 저장할 수 있습니다. (depth Image 디스플레이는 SlamToolboxPlugin과 함께 쓰면
rviz2가 시작 시 segfault 나서 넣지 않았습니다.)

car2 드라이버는 따로 실행합니다 (로컬 시리얼 또는 로봇 쪽 Raspberry Pi에서):
```bash
./scripts/run_car2_driver.sh /dev/ttyUSB0
```

터미널 B (주행):
```bash
cd ros2_slam
./scripts/run_teleop.sh

```
터미널 C (맵이 충분히 쌓이면 저장):
```bash
cd ros2_slam
./scripts/save_map.sh my_map
```
결과: `maps/my_map.yaml`, `maps/my_map.pgm`

### 2) 저장된 맵으로 내비게이션

```bash
cd ros2_slam
./scripts/run_navigation.sh maps/my_map.yaml /dev/ttyUSB0
```

RViz에서:
1. **2D Pose Estimate**로 로봇의 실제 시작 위치/방향을 지정.
2. **Nav2 Goal**로 목적지를 지정하면 자동 주행.

`myrobot_ws`의 `go_to_pose_service` 패턴처럼 좌표를 코드/서비스로 넘기고
싶다면 `nav2_simple_commander`(이미 설치됨)를 사용하는 작은 스크립트를
추가하면 됩니다 (`myrobot_ws/src/myrobot_nav_service` 참고).

### 3) RTAB-Map 원격 매핑 (로봇: Pi, 매핑: 노트북)

로봇(Raspberry Pi, `user@192.168.0.31`)에는 car2 드라이버와 Astra Pro 카메라만
띄우고, 압축 영상을 WiFi로 받아 노트북에서 RTAB-Map(RGB-D SLAM)을 돌립니다.

```
[Pi] robot_rgbd.launch.py                       [노트북] rtabmap_mapping.launch.py
  car2_serial_node   -> /odom, TF odom->base_link ─────► rtabmap (odom은 /odom 토픽으로 동기화)
  openni2 (depth 320x240@10) -> .../compressedDepth ───► 압축 해제 -> depth_image_proc/register
  usb_cam (RGB 640x480@30, JPEG 60) -> .../compressed ─► 압축 해제 ──┘   (depth -> RGB 프레임)
  static TF base_link->camera_*                            rgbd_sync -> rtabmap -> /map, 3D 맵, map->odom
```

Astra Pro의 RGB는 OpenNI2가 못 여는 별도 UVC 장치라서 `usb_cam`으로 받고,
depth를 RGB 프레임으로 맞추는 정합(register)은 노트북에서 합니다.

**준비 (최초 1회)**
- 노트북: `sudo apt install ros-jazzy-rtabmap-ros ros-jazzy-image-transport-plugins`
- Pi: `sudo apt install ros-jazzy-openni2-camera ros-jazzy-usb-cam ros-jazzy-image-transport-plugins`
- 노트북에서 코드를 Pi로 배포·빌드 (Pi의 `~/ros2_slam_robot`에 복사하므로 Pi의 git
  클론은 건드리지 않음. 첫 배포 때 Orbbec OpenNI2 arm64 런타임도 설치):
  ```bash
  ./scripts/deploy_to_robot.sh            # 기본 user@192.168.0.31
  ```
  코드를 고칠 때마다 다시 실행하면 됩니다.

**실행** (모든 터미널이 같은 `ROS_DOMAIN_ID`여야 함. 스크립트 기본값과 노트북·Pi의
`~/.bashrc` 모두 42로 통일되어 있음)
```bash
# 로봇 쪽 — 노트북에서 원격으로 (Pi의 tmux 'robot_rgbd'에서 실행, 웹 UI 재시작 버튼과 같은 세션)
./scripts/remote_robot_rgbd.sh start      # stop / restart / status / log / attach
#   드라이버만: ./scripts/remote_car2_driver.sh start /dev/ttyS0 -p odom_poll_every_n:=2
#   카메라 웹 UI(영상·각도·RoiRatios·재시작 버튼): ./scripts/remote_camera_web.sh start  → http://192.168.0.31:8080
#   ROS 없이 웹 조종(car2_web.py):                ./scripts/remote_car2_web.sh start    → http://192.168.0.31:8766
#   (시리얼 /dev/ttyS0를 쓰는 드라이버·car2_web은 하나만 — 이미 쓰고 있으면 시작을 거부함)

# 로봇 쪽 — Pi에서 직접 하려면 (ssh user@192.168.0.31)
source /opt/ros/jazzy/setup.bash
cd ~/ros2_slam_robot && ./scripts/run_robot_rgbd.sh
#   RGB 장치가 /dev/video0이 아니면: ./scripts/run_robot_rgbd.sh video_device:=/dev/video1
#   (스크립트가 시작할 때 v4l2-ctl로 찾은 Astra 장치를 보여줌)

# 노트북
./scripts/run_rtabmap_mapping.sh                    # 기존 DB(maps/rtabmap.db)에 이어서
./scripts/run_rtabmap_mapping.sh new_map:=true      # DB 지우고 새로 시작
./scripts/run_rtabmap_mapping.sh use_rtabmap_viz:=true   # RTAB-Map 자체 GUI도 같이
./scripts/run_teleop.sh                             # 주행
./scripts/save_map.sh my_map                        # 2D 격자(/map) 저장 -> Nav2용
```
RViz(`car2_bringup/rviz/rtabmap.rviz`)에는 3D 컬러 맵(MapCloud), 포즈 그래프
(MapGraph, 루프 클로저는 빨간 선), 2D 점유 격자, RGB 영상이 표시됩니다.

**주의**
- **car2 드라이버는 하나만** 띄우세요. `robot_rgbd.launch.py`에 드라이버가 포함돼
  있으므로 `run_car2_driver.sh`를 같이 띄우면 두 프로세스가 `/dev/ttyS0`를 두고
  경쟁해 STAT 응답이 깨지고 `/odom`이 수 초씩 끊깁니다(실측: 60초에 147/600개).
  드라이버를 따로 띄우고 싶으면 `run_robot_rgbd.sh use_car2_driver:=false`.
- RGB 내부 파라미터(`config/astra_pro_rgb.yaml`)는 **임시값**입니다. 정합과 맵
  정확도를 위해 `camera_calibration`으로 보정하고 `rgb_camera_info_url:=file://...`로
  지정하세요 (yaml 상단에 명령 있음).
- 로봇 쪽 odometry는 10Hz(`odom_poll_every_n:=2`)로 올려 두었습니다.
- WiFi 대역폭(실측): depth 30Hz + JPEG 품질 95로는 노트북에 4~6Hz밖에 안 와서,
  depth는 `depth_skip:=2`(10Hz), RGB JPEG 품질은 60(`config/robot_rgb_transport.yaml`)으로
  낮췄습니다. 합계 약 1MB/s.
- RGB는 카메라 기본값인 30fps로 받습니다. 10fps로 요청하면 V4L2 버퍼에 프레임이
  쌓여 약 1.1초 지난 영상이 나와 depth와 짝이 맞지 않습니다(30fps에서도 RGB는
  depth보다 약 0.5초 늦게 도착하므로 노트북 쪽 동기화 큐를 30으로 두었습니다).
- 실행 직후 30초 정도는 `Did not receive data` 경고가 날 수 있습니다. 그 뒤로는
  안정적으로 갱신됩니다.

### 4) RTAB-Map 맵으로 자율 주행 (Nav2)

`run_rtabmap_mapping.sh`로 만든 `maps/rtabmap.db`에서 위치를 추정하고 Nav2로 주행합니다.

```
rtabmap (localization:=true, DB 읽기 전용) -> /map, map->odom      (AMCL/map_server 대신)
depth -> /rtabmap/depth_cloud (rtabmap_util/point_cloud_xyz)        -> 코스트맵·collision_monitor 장애물
Nav2 (navigation_launch.py + config/nav2_rtabmap_params.yaml)      -> /cmd_vel -> 로봇 드라이버
```

```bash
./scripts/remote_robot_rgbd.sh start                    # 로봇 쪽 (드라이버 + 카메라)
./scripts/run_rtabmap_navigation.sh                     # 기본 maps/rtabmap.db
./scripts/run_rtabmap_navigation.sh maps/office.db use_rtabmap_viz:=true
```
RViz(Nav2 화면)에서:
1. **위치 맞추기**: 매핑을 시작/끝낸 자리 근처에 로봇을 두면 rtabmap이 카메라로 아는 장소를 찾아
   스스로 위치를 잡습니다. 안 맞으면 **2D Pose Estimate**로 지정 (`/initialpose` → rtabmap).
2. **Nav2 Goal**로 목적지 지정.

**주의**
- **매핑을 정상 종료(Ctrl-C 후 종료 대기)한 DB**를 쓰세요. rtabmap은 장소 인식용 단어 사전을
  종료할 때 저장하므로, 매핑 중이거나 강제 종료된 DB로는 스스로 위치를 찾지 못합니다.
- 매핑(`run_rtabmap_mapping.sh`)과 동시에 실행하지 마세요 (같은 rtabmap 노드 이름). 두 스크립트 모두
  rtabmap이 이미 떠 있으면 시작을 거부합니다. 같은 DB로 rtabmap 두 개가 뜨면 `database is locked`로
  죽고 DB가 정상 종료되지 않습니다(`rtabmap-info`에서 `0 words`).
- **정상 종료되지 않은 DB 복구**: 사본에 `rtabmap-recovery 사본.db` (원본은 그대로 두세요).
  2026-10-01 맵은 이렇게 복구한 `maps/rtabmap_recovered.db`를 쓰면 됩니다.
- 로봇이 가만히 있으면 카메라가 거의 바닥만 봐서 장소 인식이 잘 안 됩니다. 시작 후 제자리에서
  한 바퀴 돌리면(웹 UI ◀/▶) rtabmap이 위치를 잡습니다.
- 속도 한계: 직진 0.15 m/s, 회전 0.7 rad/s (바퀴 최대 ~0.167 m/s, ~0.77 rad/s). 로봇 반경 0.26 m.
- 장애물은 depth 점 중 바닥 위 0.06~0.7 m, 2.5 m 이내만 씁니다. 화면 아래 22.5%(로봇 몸체)는
  제외 (`body_roi_ratios:="0 0 0 0.225"`, 웹 UI의 RoiRatios와 같은 값으로 맞추세요).
- 카메라 기울기(pitch/roll)가 정확해야 바닥이 장애물로 잡히지 않습니다 — 웹 UI의 "현재 값으로 설정".
- **드라이버 안전 정지**: `car2_serial_node`는 `/cmd_vel`이 1초 끊기면 스스로 멈춥니다
  (`cmd_vel_timeout_s`, 0이면 예전처럼 마지막 명령 유지). Nav2가 죽거나 WiFi가 끊겨도 계속 달리지 않습니다.

### 5) 2D 라이다 (LD 계열) + 4륜 구동

로봇에 추가한 2D 라이다는 LDROBOT LD06/LD19 계열 프로토콜(CP2102 USB 시리얼, 230400 bps,
6 Hz 회전, 약 4000점/초)입니다. apt 드라이버가 없어 `car2_driver/ld_lidar_node.py`로 직접 읽어
`/scan`(LaserScan, `laser_frame`, 0.5° 간격)을 냅니다.

```bash
./scripts/remote_car2_lidar.sh start                          # 드라이버 + 라이다 (Pi tmux 'car2_lidar')
./scripts/remote_car2_lidar.sh start use_car2_driver:=false   # 라이다만 (robot_rgbd와 같이 쓸 때)
./scripts/remote_car2_lidar.sh start --camera                 # 드라이버 + 라이다 + 깊이/RGB 카메라 (robot_rgbd 대신)
./scripts/remote_car2_lidar.sh status | log | attach | restart | stop
# 장착 위치: lidar_x/y/z (m, base_link 기준), lidar_yaw (rad)
./scripts/remote_car2_lidar.sh restart lidar_x:=0.05 lidar_z:=0.25 lidar_yaw:=3.1416
```
- 장착 위치: 바퀴 사이 정중앙 (`lidar_x/y` = 0), 0° 방향이 로봇 정면보다 약 4.5° 왼쪽 (`lidar_yaw` = 0.0785,
  직진 2회 측정). 높이 `lidar_z`(0.2)만 임시값 — 2D에는 영향 없음.
- 라이다 각도는 시계 방향이라 ROS(반시계)로 뒤집어 냅니다 (360° 회전 시험으로 검증).
- 로봇 기둥 4개(라이다에서 약 0.2 m, ±69.5~96.5° 부근)는 `robot_lidar.launch.py`의 `self_mask_deg`로
  제외합니다 — 그 각도에서 0.35 m보다 가까운 점만 버리고, 기둥 사이로 보이는 먼 물체는 남깁니다.
  라이다를 다시 달거나 돌리면 각도를 다시 재야 합니다.

### 6) 라이다 맵핑 · 저장 · 내비게이션 (slam_toolbox + AMCL + Nav2)

로봇 쪽은 `./scripts/remote_car2_lidar.sh start` (드라이버 + 라이다, 카메라도 쓰려면 `--camera`). 나머지는 노트북에서:

```bash
./scripts/run_lidar_mapping.sh                     # 새 맵 (slam_toolbox + RViz)
./scripts/save_lidar_map.sh office                 # 맵핑 중에 저장 -> maps/office.yaml/.pgm + .posegraph/.data
./scripts/run_lidar_mapping.sh office              # 저장한 맵에 이어서 맵핑 (pose graph)
./scripts/run_lidar_navigation.sh maps/office.yaml # AMCL 위치 추정 + Nav2 (RViz에서 Nav2 Goal)
./scripts/run_lidar_navigation.sh maps/office.yaml initial_x:=1.0 initial_y:=0.5 initial_yaw:=1.57
```
- 설정: `config/slam_toolbox_lidar_params.yaml` (0.1~8 m, TF 대기 0.5 s),
  `config/nav2_lidar_params.yaml` (RTAB-Map용과 같은 로봇 한계 + 장애물/AMCL은 `/scan`).
- 내비게이션 시작 4초 뒤 AMCL에 초기 위치를 자동으로 보냅니다(기본: 맵 원점 = 맵핑을 시작한 자리).
  다른 곳에서 시작했으면 `initial_x/y/yaw`를 주거나 RViz **2D Pose Estimate**로 고치세요.
- SLAM/위치추정 노드는 한 번에 하나만 (스크립트가 이미 떠 있으면 거부).
- 라이다: 바퀴 사이 정중앙, 0° 방향은 정면에서 약 4.5° 왼쪽 (`lidar_yaw` = 0.0785, 직진 2회 측정). 각도 방향은 360° 회전으로 검증.
- 바퀴 반지름 `wheel_radius_m` = 0.0329 m (0.87 m 직진을 라이다로 측정: 실제/odom = 1.027).
- 유효 윤거 `wheel_separation_m` = 0.53 m (실제 윤거 0.43 m, 스키드 스티어 미끄러짐 보정 — 360° 회전에서 라이다 기준
  실제 회전이 odom의 0.833배, 반지름 보정에 맞춰 함께 조정).

#### 지도 웹 UI (위치 · 방향 · 주행)

```bash
./scripts/remote_map_web.sh start      # Pi에서 test/map_web.py 실행 → http://192.168.0.31:8081
./scripts/remote_map_web.sh status     # 현재 위치/지도/라이다 요약
```
- 지도(`/map`) 위에 로봇 위치·방향(화살표, 원 = 차체 반경), 라이다 스캔, 주행 궤적, Nav2 경로, AMCL 오차 타원을 표시.
  드래그/휠/핀치로 이동·확대, '따라가기'·'전방 위'(로봇 앞이 화면 위)·'전체' 버튼.
- 위치는 TF `map→base_link`. 노트북에서 SLAM/내비게이션이 안 떠 있으면 `odom→base_link`(바퀴 주행거리) 기준으로 표시.
- 주행 패드(전진·후진·좌/우회전, 속도 슬라이더, 키보드 ↑↓←→/WASD, Space 정지)는 `/cmd_vel`로 보내며
  누르고 있는 동안만 움직입니다(0.5초 내 자동 정지). 주행 코드는 `test/drive_control.py` (camera_test.py와 같은 방식).
- 지도 좌표: 가장자리에 m 눈금, 원점 (0, 0)에 X→/Y↑ 화살표, 커서 위치 좌표, 지도 범위(X/Y)를 표시.
- **위치 라벨**: 이름을 입력하고 '현재 위치 라벨 추가' → 로봇의 현재 map 좌표·방향을 저장
  (Pi의 `~/.ros/car2_map_labels.json`). 지도에 빨간 점 + 이름으로 표시되고, 목록이나 지도 위 점을 눌러 선택합니다.
  map 좌표계가 있을 때만(SLAM/내비게이션 실행 중) 저장되며, 그 지도 기준 좌표이므로 같은 저장 맵으로 내비게이션해야 의미가 있습니다.
- **Go to (Navi)**: 선택한 라벨 위치·방향으로 Nav2 `navigate_to_pose` 목표를 보냅니다(RViz의 Nav2 Goal과 같음).
  노트북에서 `run_lidar_navigation.sh`(또는 `run_rtabmap_navigation.sh`)가 실행 중이어야 하고, 남은 거리·도착/실패를 표시합니다.
  'Navi 취소' 또는 주행 패드를 누르면 목표가 취소됩니다.
- **현재 위치 자동 다시 잡기**: 현재 라이다 스캔을 지도와 맞춰(scan matching) 로봇 위치를 찾고 AMCL에 `/initialpose`로
  보냅니다(RViz 2D Pose Estimate와 같음). 기본은 현재 추정 위치 주변 ±1 m·±40° 검색(Pi에서 약 2~3초),
  위치를 전혀 모를 때는 '전체 지도에서 찾기'(지도 전체·모든 방향, 약 10초). 스캔 일치율이 50% 미만이면 보내지 않고,
  다른 곳에도 비슷하게 맞는 후보가 있으면 경고합니다. 로봇이 멈춘 상태에서만 동작하고, 내비게이션(AMCL)이 실행 중이어야 합니다.
- 라벨이 Nav2 비용지도에서 장애물/로봇 반경(0.26 m) 안이면 목록에 '⚠ 도달 불가'로 표시하고 Go to를 거부합니다.
  실제로는 비어 있는데 그렇다면 지도에 가짜 장애물이 찍힌 것(예: 라이다 range_min 0.25 수정 전 로봇 기둥) — 지도를 새로 만드세요.
- **도착 오차**: 위치(m)·방향(°) 슬라이더/입력 후 '적용' → 실행 중인 Nav2 goal checker(`controller_server`의
  `general_goal_checker.xy/yaw_goal_tolerance`)에 바로 적용. 허용 범위 위치 0.03~0.5 m, 방향 3~90°
  (`-p goal_xy_min/goal_xy_max/goal_yaw_min_deg/goal_yaw_max_deg`로 변경). 값은 Pi의 `~/.ros/car2_nav_tolerance.json`에
  저장되어 내비게이션을 다시 시작해도 자동으로 다시 적용됩니다(yaml 값보다 우선).

## 트러블슈팅

- **`Failed to find a free participant index for domain N` / 노드 생성 실패**:
  `run_mapping.sh`/`run_navigation.sh`/`save_map.sh`는 기본적으로
  `ROS_DOMAIN_ID=42`를 씁니다(기본 도메인 0이나, 이전 세션이 정리 안 되고
  남은 도메인과의 충돌을 피하기 위함). 그래도 이 에러가 나면 뭔가가 42번도
  이미 채워놓은 것이므로, 다른 값으로 바꾸세요: 세 스크립트 모두 실행 전에
  `export ROS_DOMAIN_ID=123` 처럼 원하는 값을 먼저 export하면 그 값을
  우선 사용합니다 (단, 같은 실행 흐름 안의 모든 터미널에 동일하게 적용해야
  서로 통신됩니다 - teleop 터미널 포함). 좀비 프로세스가 쌓인 게
  의심되면 `pkill -9 -f "component_container|ros2-daemon"`로 정리하세요.
- **`ros2 run openni2_camera list_devices`가 0 devices**: `./scripts/install_orbbec_openni2.sh`를
  실행했는지 확인하세요. 실행 후에도 0이면
  `LD_LIBRARY_PATH=ros2_slam/third_party/orbbec_openni2 ros2 run openni2_camera list_devices`로
  override가 실제로 적용됐는지 직접 확인해보세요.
- **`/scan`이 비어 있는데 `component_container`는 떠 있음**: 카메라가 실제로
  depth 이미지를 발행하는지 `ros2 topic hz /camera/depth_raw/image`로 먼저
  확인하세요. USB 허브 체인이나 다른 OpenNI2 프로세스와의 경합으로 이 카메라는
  가끔 첫 시도에 조용히 응답하지 않을 수 있습니다 — 그럴 땐
  `ros2 run openni2_camera usb_reset /dev/bus/usb/<bus>/<dev>`로 리셋
  (`lsusb`에서 Orbbec Depth Sensor의 버스/디바이스 번호 확인) 후 재실행하세요.
- **로봇이 타임아웃으로 멈춤(`TIMEOUT,STOP`)**: `car2_driver`가 살아있는지,
  `/cmd_vel` 구독자가 붙어 있는지(`ros2 topic info /cmd_vel`) 확인하세요.
  car2.ino는 마지막 명령 후 500ms 안에 새 명령이 없으면 자동 정지합니다.
- **포트 권한 오류(`/dev/ttyUSB0` Permission denied)**: `sudo usermod -aG
  dialout $USER` 후 재로그인.
- **Nav2가 목표를 바로 거부**: AMCL 초기 위치가 안 맞아 `map -> base_link`
  TF가 없는 상태입니다. RViz "2D Pose Estimate"로 먼저 위치를 맞추세요.

ros2 run tf2_ros tf2_echo map base_link


라이다 자동차 실행순서

./scripts/remote_car2_lidar.sh start
./scripts/remote_map_web.sh start
./scripts/run_lidar_navigation.sh ./maps/lidarmap.yaml