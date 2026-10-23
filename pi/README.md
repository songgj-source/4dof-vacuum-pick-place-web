# Raspberry Pi(로봇 위)로 옮기기

로봇팔이 이동 플랫폼 위에서 움직이므로, 서보·펌프·카메라를 USB로 잡는 컴퓨터를 로봇 위의 Raspberry Pi로 옮깁니다.
무거운 LLM(Ollama)과 음성 인식(Whisper)은 노트북에 두고 Wi-Fi로 부릅니다.

```
[로봇 위 Raspberry Pi]  arm_api.py (REST·제어·인식), systemd 서비스 arm-api
   ├─ USB  서보 어댑터 → STS3215 ×4
   ├─ USB  ESP32 → L9110S → 진공 펌프
   └─ USB  Astra(탑뷰), 손목 카메라
        ▲ Wi-Fi (REST, 토큰)
[노트북]  브라우저(웹 화면·마이크), Ollama(LLM), stt_server.py(Whisper), 팀 NOMI
```

## 매일 실행하기 (설치가 끝난 뒤)

명령은 노트북의 이 폴더에서 입력합니다. 로봇 이름은 **자비스**입니다.

### 준비
- 노트북 Wi-Fi를 `robot`에 연결합니다(노트북 주소 192.168.0.32, `ip -4 addr show wlp3s0`로 확인).
- Pi를 켭니다. 리튬팩으로 켤 때 충전기를 꽂거나 빼면 Pi가 꺼질 수 있습니다(2026-10-22 실제로 꺼짐).
- 서보 어댑터·ESP32 펌프·손목 카메라는 허브에, **Astra는 Pi에 직접** 꽂습니다(전원 없는 허브에 Astra까지 물리면 허브가 끊김. Pi 포트 1-1.3은 인식 실패가 잦음).
- 서보 전원과 펌프 전원(AA 4개)을 켭니다.

### 한 번에 켜기 (보통은 이것만)
```bash
bash pi/start_all.sh          # 아래 1~3번을 한꺼번에 (이미 켜진 것은 건너뜀)
bash pi/start_all.sh status   # 무엇이 켜져 있는지
bash pi/start_all.sh stop     # 모두 끄기
```
노트북 음성 서버·Ollama 중계를 켜고, 노트북 주소가 바뀌었으면 Pi 설정을 새 주소로 고치고, Pi Wi-Fi 절전을 끄고,
장치를 점검한 뒤 Pi 서버를 켜고 화면을 엽니다(`NO_BROWSER=1`이면 화면은 안 엶). 켜기만 해서는 로봇이 움직이지 않습니다.
아래 1~4번은 같은 일을 손으로 할 때의 명령입니다.

### 1. 노트북: 음성 인식 서버와 Ollama 중계 (노트북을 켤 때마다 한 번)
```bash
# 음성 토큰은 Pi 설정(pi/arm-api.env)의 ARM_STT_TOKEN과 같아야 한다
STT_TOKEN=$(ssh user@192.168.0.31 "grep ^ARM_STT_TOKEN= ~/4dof_ik_pi_test/pi/arm-api.env | cut -d= -f2")
env -u ARM_STT_URL nohup ~/miniconda3/envs/lerobot_312/bin/python scripts/stt_server.py \
  --host 0.0.0.0 --port 8010 --token "$STT_TOKEN" > ~/stt_server.log 2>&1 &

# Pi가 노트북 Ollama(127.0.0.1에서만 받음)를 쓰게 하는 중계. Pi 주소만 허용, sudo 필요 없음
nohup socat TCP4-LISTEN:11434,bind=192.168.0.32,reuseaddr,fork,range=192.168.0.31/32 \
  TCP4:127.0.0.1:11434 > ~/ollama_fwd.log 2>&1 &
```
Pi 설정에는 `ARM_STT_URL=http://192.168.0.32:8010/api/stt`, `OLLAMA_HOST=http://192.168.0.32:11434`가 들어 있습니다.

### 2. Pi: 서버 켜기
```bash
ssh user@192.168.0.31 'sudo iw dev wlan0 set power_save off'   # Pi를 켤 때마다 (절전이면 응답이 수백 ms로 느려짐)
bash pi/remote.sh check     # 장치 점검 (읽기 전용)
bash pi/remote.sh start     # 서버 켜기. 켜기만 해서는 로봇이 움직이지 않는다
```

### 3. 화면 열기
```bash
bash pi/open_jarvis.sh      # 전용 Chrome 창으로 http://192.168.0.31:8000 (터널 없이 마이크 사용 가능). start_all은 주행 화면 :8081도 탭으로 연다
```
처음 한 번 [설정]에 접속 토큰을 넣습니다. 토큰 보기:
`ssh user@192.168.0.31 "grep ^ARM_API_TOKEN= ~/4dof_ik_pi_test/pi/arm-api.env | cut -d= -f2"`

[말하기]를 누르고 말하거나 입력칸에 씁니다. 앞에 "자비스,"를 붙여도 됩니다.

| 말 | 동작 |
|---|---|
| "작업대에 뭐 있어?" | 사진을 찍고 설명 (안 움직임, 약 2.5초) |
| "상태 어때?" | 팔 상태 (안 움직임) |
| "홈으로 돌아가" | 토크를 자동으로 켜고 HOME 자세 (**움직임**) |
| "빨간 큐브 넣어줘", "흰 상자 3번 칸에 넣어" | 한 물체 넣기 (**움직임**) |
| "책상 정리해줘" 또는 [실행] | 책상의 등록 물체를 모두 칸에 넣기 (**움직임**) |
| "다시 해줘" | 마지막 작업 다시 (**움직임**) |
| "멈춰", 정지 버튼 | 즉시 정지 |

- **움직이는 명령은 묻지 않고 바로 실행됩니다.** 팔 옆에서 서보 전원을 바로 끊을 수 있게 해 두세요.
- 팔이 닿는 거리(로봇에서 16~38 cm) 안의 책상에는 정리할 물체만 둡니다. 다른 물건(리튬팩 등)도 물체로 잡혀 집으러 갈 수 있습니다.
- 팔 밑동에 너무 가까운 물체(약 16 cm 안)는 로봇 자신으로 보고 뺍니다.

### 주행 (팀 map_web과 함께)
팀원의 주행 서버(`test/map_web.py`, 같은 Pi의 8081)가 켜져 있으면 자비스가 그 API 두 개만 불러 차를 움직입니다.
- `base_state`(`GET /api/state`, `/api/drive`): 지도 위 위치·방향, 저장된 장소(라벨), 이동 상태 → 자비스 `GET /api/base`, "지금 어디야?"
- `goto_label`(`POST /api/nav/goto`) + 마무리 정렬(`POST /api/drive`): 라벨 자리·방향에 정확히 서기(기본) → "책상으로 이동해줘", 메뉴 8번 "장소로 이동", `POST /robot_arm {"action": "drive", "label": "책상"}`
- `goto_label_wall`(`POST /api/nav/goto_wall`): 장소로 이동한 뒤 앞 벽(작업대)과 수직으로 맞춰 붙음 → `mode: wall`

| 말 | 동작 |
|---|---|
| "책상으로 이동해줘", "휴지통 앞으로 가" | 그 라벨을 만든 **그 자리·방향**으로 주행 (**차가 움직임**) |
| "책상으로 가서 정리해줘", "책상 가서 정리해" | 주행 → 도착하면 사진 → 팔이 닿는 물체를 칸에 정리 → HOME (**차와 팔이 움직임**) |
| "지금 어디야?" | 지도 위치와 갈 수 있는 장소 목록 (안 움직임) |
| "상태 어때?" | 팔 상태 + 차 위치·이동 상태 한 줄 (안 움직임) |
| "멈춰", 정지 버튼 | 주행도 바로 멈춤 (map_web 취소·정지) |

- **라벨 자리에 정확히 서기(기본, `mode: precise`):** Nav2는 장애물 옆 라벨(책상 앞 등)을 비용이 낮은 곳으로 옮겨 가고(최대 1.5 m), 방향도 ±0.25 rad(약 14°)까지 도착으로 칩니다.
  그래서 Nav2로 근처까지 간 뒤(`/api/nav/goto`) 지도 위치를 보며 map_web 주행(`/api/drive`)으로 천천히 마무리합니다: 라벨 쪽으로 돌기 → 전진 또는 후진(덜 도는 쪽, 3~6 cm/s) → 라벨 방향으로 돌기.
  목표는 **위치 ±3 cm, 방향 ±2°**이고, 결과(오차)는 진행 칸과 로그에 남습니다. 라이다로 진행 방향 차체 끝 3 cm 안이나 회전 반경(0.26 m) 안에 물체가 보이면 멈추고,
  Nav2가 라벨에서 60 cm 넘게 떨어져 섰으면 억지로 맞추지 않습니다(위치 추정을 다시 잡으세요). 정확도는 지도 위치 추정(AMCL) 정확도를 넘지 못합니다.
  라벨을 만들 때 책상에 3 cm보다 가까이 붙어 있었다면 안전거리 때문에 그만큼은 못 들어갑니다.
- 예전처럼 앞 벽에 붙고 싶으면 `mode: wall`(`POST /robot_arm {"action": "drive", "label": "책상", "mode": "wall", "gap": 0.05}`).
- **팔이 HOME으로 접혀 있을 때만 주행합니다**(관절마다 HOME에서 120 펄스 안). 아니면 "먼저 홈으로 돌아가"라고 거부합니다.
- 주행 중에는 팔 작업을 받지 않고, 팔 작업 중에는 주행을 받지 않습니다(한 번에 한 작업). 팔 작업은 차가 움직이면 시작하지 않습니다.
- 장소 이름은 map_web에 저장된 라벨 그대로입니다. 지도에 없는 장소를 말하면 있는 장소를 알려 줍니다. 주소는 `ARM_BASE_URL`(기본 `http://127.0.0.1:8081`).
- 진행(이동 중·남은 거리·벽 맞추는 중·도착 후 사진·정리)은 진행 상태 칸에, 실패 이유는 로그창에 나옵니다. 위쪽에 차 위치(가장 가까운 장소·이동 중)가 3초마다 보입니다.
- "가서 정리해"는 도착(벽 근접 성공) 뒤 차가 멈춘 것을 확인하고(최대 4초) 사진을 두 장 찍어 정리합니다. 도착이나 벽 맞추기에 실패하면 정리하지 않습니다.

### 내 주행 스택 (팔과 함께, Pi에서)
`bash pi/start_all.sh`가 팔과 함께 **내 `ros2_slam` 사본으로 주행 스택을 Pi에서 직접** 켭니다(팀 것을 쓰지 않음, `NO_DRIVE=1`이면 팔만).
노트북 없이 로봇 위 Pi 하나로 돕니다. 노트북에는 Nav2가 없고, Pi에는 Nav2·AMCL·지도 서버가 설치돼 있습니다.

| 순서 | tmux 세션 (Pi) | 내용 |
|---|---|---|
| 1 | `arm_drive_robot` | 차 드라이버(`/dev/ttyAMA0`, `ARM_CAR_PORT`) + 라이다(포트 자동 찾기) |
| 2 | `arm_drive_nav` | 지도(`ros2_slam/maps/lidarmap.yaml`, `ARM_DRIVE_MAP`) + AMCL 위치 추정 + Nav2, RViz 없이 |
| 3 | `arm_drive_web` | map_web 8081 (자비스가 이것을 부름) |
| 4 | - | 켜면 '전체 지도에서 위치 찾기'로 처음 위치를 잡음(차는 안 움직임) |

```bash
ssh user@192.168.0.31 'bash ~/4dof_ik_pi_test/pi/drive_ctl.sh status'        # 상태
ssh user@192.168.0.31 'bash ~/4dof_ik_pi_test/pi/drive_ctl.sh relocalize'    # 위치 다시 잡기(차를 손으로 옮긴 뒤)
ssh user@192.168.0.31 'bash ~/4dof_ik_pi_test/pi/drive_ctl.sh log nav'       # 로그 (robot / nav / web)
```
- 팀과 겹치지 않게: ROS 도메인 **43**(팀 42), 세션 이름 `arm_drive_*`, 라벨·도착 오차 파일은 `~/4dof_ik_pi_test/drive_data/`(처음 한 번 팀 라벨 파일을 읽어 복사. 이후 라벨을 새로 만들면 우리 파일에만 저장).
- 팀 주행(드라이버·라이다·map_web·AMCL)이 이미 돌고 있으면 바퀴 드라이버·라이다가 겹치므로 **켜지 않고 알립니다.** 팀 프로그램은 끄지 않습니다.
- 차 시리얼은 이 Pi에서 `/dev/ttyAMA0`입니다(사본 launch 기본값 `/dev/ttyS0`은 입출력 오류로 드라이버가 죽음, 10-23 확인).
- 코드를 바꾸면 `bash pi/deploy_to_robot.sh`가 사본을 Pi로 보내고 빌드합니다. 그 뒤 `bash pi/start_all.sh stop && bash pi/start_all.sh`.
- 지도 위 장소(라벨)는 map_web 화면(`http://192.168.0.31:8081`)에서 만듭니다. 라벨은 책상에서 조금(3 cm 넘게) 떨어진 자리에서 만드세요.

### 팀 주행 코드 사본(`ros2_slam/`)과 같이 쓰기
`ros2_slam/`은 팀 저장소(perpet99/dapier_project)의 같은 폴더를 복사한 **우리 사본**입니다(2026-10-23, 팀 저장소 `f27cbc9`, `ros2_slam` 마지막 변경 `0a7ca73` 10-16).
**고치는 것은 이 사본(우리 환경)에서만 합니다. 팀 저장소와 Pi의 팀 폴더(`~/ros2_slam_robot`)·팀이 띄운 프로그램은 건드리지 않습니다.**
고친 뒤에는 시험(`tests/test_ros2_slam_compat.py`)으로 자비스와 여전히 맞는지 봅니다. Pi에서 지금 도는 주행은 팀 스크립트가 `~/ros2_slam_robot`에서 띄운 것이라,
우리 `deploy_to_robot.sh`는 이 폴더를 보내지 않습니다(우리 사본을 Pi에서 돌리려면 팀 폴더와 다른 곳에 따로 올려야 합니다).
팀의 새 버전을 받아 올 때는 아래처럼 다시 복사합니다. **우리가 고친 내용은 덮어써지니**, 먼저 커밋해 두고 `git diff`로 다시 합칩니다.
```bash
git clone --depth 1 --filter=blob:none --sparse https://github.com/perpet99/dapier_project.git /tmp/dp && git -C /tmp/dp sparse-checkout set ros2_slam
rm -rf ros2_slam && cp -r /tmp/dp/ros2_slam ./ros2_slam && python -m unittest tests.test_ros2_slam_compat
```

| 항목 | 팀 주행 | 자비스(팔) | 같이 쓸 때 |
|---|---|---|---|
| 웹·API 포트 | map_web 8081, 카메라 웹 8080 | 8000 (노트북 음성 8010, Ollama 중계 11434) | 겹치지 않음 |
| 차 드라이버 | `/dev/ttyS0`(GPIO UART) | - | 겹치지 않음 |
| 시리얼 | 라이다 CP2102 `_0001` (기본 by-id) | 펌프 ESP32 CP2102 `_0001` | **이름이 겹침** → 아래 |
| 카메라 | Astra Pro `2bc5:060f` (openni2_camera, 첫 장치) | Astra `2bc5:0401` (USB ID로 골라 엶, `ARM_ASTRA_ID`) | 팀 노드가 먼저 켜지면 우리 Astra를 잡을 수 있음 → 아래 |
| 움직임 | Nav2·수동 주행 | 팔 작업 | 팔 작업은 차가 움직이면 시작하지 않음, 자비스 주행은 팔이 접혀 있을 때만 |

- **우리 사본에서 고친 것:** 라이다 노드에 `port:=auto`를 넣고 `robot_lidar.launch.py` 기본값으로 했습니다. 꽂힌 USB 시리얼을 하나씩 열어 라이다 묶음(47바이트, `0x54 0x2C`)이 나오는 포트를 고르고, 다른 프로그램이 쓰는 포트는 열지 않습니다(배타적 열기). 그래서 우리 사본으로 라이다를 켜면 펌프와 이름이 겹쳐도 펌프를 잡지 않습니다. 팀이 돌리는 `~/ros2_slam_robot`에는 적용되지 않습니다.
- **라이다와 펌프:** 둘 다 CP2102 `_0001`이라 `/dev/serial/by-id` 이름이 하나만 생기고, 어느 쪽을 가리킬지는 꽂은 순서에 따라 바뀝니다(10-22 실제로 라이다 노드가 펌프를 잡음).
  `python scripts/find_pump_port.py --write`로 펌프를 USB 포트 위치 이름으로 정하고, 출력된 `lidar_port:=/dev/serial/by-path/...`로 라이다를 켭니다:
  `./scripts/remote_car2_lidar.sh restart lidar_port:=/dev/serial/by-path/...` (팀 저장소 폴더에서). 포트를 옮겨 꽂으면 다시 정합니다(`start_all.sh`가 펌프는 자동으로 다시 찾음).
- **카메라 2대:** 우리 카메라 코드는 장치 목록에서 `2bc5/0401`을 골라 엽니다(팀 Astra Pro만 꽂혀 있으면 열지 않고 알림). 팀 `openni2_camera`는 `device_id`를 주지 않으면 첫 장치를 열므로, 두 카메라를 함께 쓸 때는 팀 쪽에 Astra Pro를 지정하는 것이 안전합니다. 우리 서버는 팀 카메라 노드가 돌아도 막지 않고 알리기만 합니다.

### 4. 끄기
```bash
bash pi/remote.sh stop      # Pi 서버 (진행 중인 작업은 정리한 뒤 끝남)
pkill -f "scripts/stt_server.py"; pkill -f "socat TCP4-LISTEN:11434"   # 노트북 쪽 (필요할 때만)
```

### 문제가 생기면
- **화면 아래 [로그]창**: 작업의 시작·완료·실패와 실패의 **원문 이유**(화면에는 "알 수 없는 오류"로만 나오는 것)가 쌓입니다. 빨간 줄을 누르면 실행 프로그램의 마지막 출력이 펼쳐지고, [복사]로 통째로 복사할 수 있습니다. 위쪽 "로그" 옆 숫자는 오류 개수입니다(`GET /api/logs`).
- 상태·로그: `bash pi/remote.sh status`, `bash pi/remote.sh log`
- 음성이나 대화가 안 됨: 노트북 주소가 192.168.0.32가 아니면 Pi의 `~/4dof_ik_pi_test/pi/arm-api.env`에서 `ARM_STT_URL`, `OLLAMA_HOST`의 주소를 고치고 `bash pi/remote.sh restart`. socat의 `bind=` 주소도 새 주소로.
- Pi가 안 보임: `ping 192.168.0.31`. 전원이 끊겼는지 확인합니다. 재부팅했으면 2번부터 다시.
- 코드를 고친 뒤 Pi에 반영: `bash pi/deploy_to_robot.sh -- --no-service --no-apt` 후 `bash pi/remote.sh restart`.

## 0. 팀 로봇 Pi(`user@192.168.0.31`)에서 우리 팔만 시험하기

팀원의 주행·카메라·hand_eye가 같이 쓰는 Pi라서, **서비스를 만들지 않고(`--no-service`) 우리 폴더 안에서만** 설치·실행합니다.
팀원의 git 클론, `~/dapier_project/.venv`, ROS 설정은 건드리지 않습니다. Astra 런타임은 Pi에 이미 설치된 Orbbec OpenNI2(`/opt/openni2-orbbec`)를 찾아 씁니다.

```bash
# 노트북(이 폴더)에서
bash pi/deploy_to_robot.sh --dry-run          # 보낼 파일·할 일만 확인
bash pi/deploy_to_robot.sh                    # 코드 복사(~/4dof_ik_pi_test) + Pi에서 install.sh --no-service
bash pi/remote.sh check                       # 장치·드라이버·패키지 점검(읽기 전용)
bash pi/remote.sh start                       # 서버 켜기 → 브라우저 http://192.168.0.31:8000
bash pi/remote.sh stop | status | log
```

- 다른 주소·사용자면 `ROBOT=사용자@주소`를 앞에 붙입니다. 노트북 키를 등록해 두면(`ssh-copy-id user@192.168.0.31`) 비밀번호를 묻지 않습니다.
- apt 패키지가 이미 있으면 `bash pi/deploy_to_robot.sh -- --no-service --no-apt`.
- **Astra는 한 프로그램만 열 수 있습니다.** 팀의 카메라 노드(`remote_robot_rgbd.sh`, `remote_camera_web.sh`)나 `hand_eye`가 돌고 있으면 `start`가 거부합니다. 먼저 팀원과 맞춰 그쪽을 끄세요.
- 팔·펌프 USB를 Pi에 꽂은 뒤 `check`에서 장치 경로가 OK인지 봅니다. 같은 어댑터면 노트북과 by-id 이름이 같아 설정할 것이 없습니다.
- Orbbec 공식 OpenNI2(2.3.0.8x)는 장치 정보 구조체가 `primesense`보다 256바이트 커서, 맞추지 않으면 영상 스트림을 열 때 프로세스가 죽습니다. 카메라 코드가 열 때 맞춥니다(`use_large_device_info`).
- 서버를 끄면 실행 중인 집기 작업에 정지 신호를 보내 게인 복구·팔 고정·펌프 정지가 끝난 뒤 종료합니다.

## 1. 가장 먼저: Astra 드라이버 확인

저장소의 OpenNI2 런타임(`calibration/openni2_redist`)은 **x86-64 전용**이라 Pi에서 쓸 수 없습니다.
Orbbec OpenNI2 SDK의 **ARM64(aarch64)** 빌드를 받아 Pi에 풀고, 그 `Redist` 폴더(`libOpenNI2.so`가 있는 곳)를 설치 때 알려 줍니다.
이 Astra(USB ID `2bc5:0401`)가 Pi에서 깊이·색 영상을 내는지 부품을 더 사기 전에 확인하세요.
안 되면 x86 소형 PC(N100급)나 Jetson으로 갑니다(설치 스크립트는 그대로 쓸 수 있습니다).

## 2. Pi 설치

권장: Raspberry Pi 5 + Ubuntu 24.04(arm64, Python 3.12, 노트북 개발 환경과 같은 버전).

```bash
git clone -b pi https://github.com/songgj-source/4dof-vacuum-pick-place-web.git
cd 4dof-vacuum-pick-place-web
DRY_RUN=1 bash pi/install.sh --openni2 /opt/OpenNI2/Redist     # 할 일만 보기(아무것도 바꾸지 않음)
bash pi/install.sh --openni2 /opt/OpenNI2/Redist                # 설치
```

설치 스크립트가 하는 일:
1. apt 패키지, 프로젝트 `.venv`와 파이썬 패키지(`pi/requirements-pi.txt`: numpy, opencv-headless, pyserial, feetech-servo-sdk, primesense)
2. `dialout`·`video` 그룹(시리얼·카메라 권한), Astra udev 규칙
3. `/etc/arm-api.env` 설정 파일(권한 600, 접속 토큰은 무작위로 만들어 화면에 한 번 보여 줌)
4. systemd 서비스 `arm-api` 등록(부팅 때 자동 시작). `--start`를 주면 바로 켭니다
5. 장치 점검 `scripts/check_devices.py`(읽기 전용)

서비스를 켜는 것만으로 로봇은 움직이지 않습니다. 명령을 받아야 움직입니다.
서비스를 멈추면 집기 프로세스에도 정지 신호(SIGINT)가 가서 게인 복구·팔 고정·펌프 정지를 마친 뒤 끝납니다.

```bash
sudo systemctl start arm-api          # 켜기
systemctl status arm-api              # 상태
journalctl -u arm-api -f              # 로그
sudo nano /etc/arm-api.env && sudo systemctl restart arm-api    # 설정 바꾸기
```

## 3. 노트북 쪽

```bash
# 음성 인식 서버(Whisper만, 로봇 없음). 토큰은 Pi의 ARM_STT_TOKEN과 같게
python scripts/stt_server.py --host 0.0.0.0 --token 음성용비밀값

# Ollama가 다른 컴퓨터(Pi)의 접속을 받게 한다(기본은 이 컴퓨터만)
sudo systemctl edit ollama      # [Service] 아래 Environment="OLLAMA_HOST=0.0.0.0" 추가
sudo systemctl restart ollama
```

Pi의 `/etc/arm-api.env`:

```
OLLAMA_HOST=http://노트북주소:11434
ARM_STT_URL=http://노트북주소:8010/api/stt
ARM_STT_TOKEN=음성용비밀값
```

브라우저는 계속 Pi(`http://Pi주소:8000`) 하나만 엽니다. Pi가 녹음을 노트북 Whisper로 넘기고, 대화는 노트북 Ollama를 씁니다.
`ARM_STT_URL`을 비우면 Pi에서 Whisper를 돌립니다(`install.sh --with-local-stt`, 한 문장 수 초로 느림).

## 4. 설정 값

| 환경 변수 | 뜻 | 기본값 |
|---|---|---|
| `ARM_API_TOKEN` | 접속 토큰(웹 [설정], 팀 NOMI의 `ARM_API_TOKEN`) | 설치 때 무작위 |
| `ARM_API_PORT` | 서버 포트 | 8000 |
| `OLLAMA_HOST` | 대화용 LLM 주소 | 이 컴퓨터 11434 |
| `ARM_STT_URL`, `ARM_STT_TOKEN` | 노트북 음성 인식 서버 | 비우면 이 컴퓨터 Whisper |
| `ARM_SERVO_PORT` | 서보 버스 `/dev/serial/by-id/...` | `config/servo_driver_candidate.json`의 port |
| `ARM_PUMP_PORT` | ESP32 펌프 `/dev/serial/by-id/...` | CP2102 `_0001` |
| `ARM_WRIST_CAMERA` | 손목 카메라 `/dev/v4l/by-id/...-video-index0` | Innomaker U20CAM |
| `ARM_OPENNI2_RUNTIME` | Astra OpenNI2 런타임 폴더 | `calibration/openni2_redist`(x86-64) |

장치 경로는 환경 변수 → `config/devices.json`(있으면, 저장소에 올리지 않음) → 기본값 순으로 정합니다.
같은 USB 장치면 by-id 이름이 어느 리눅스에서나 같아서 보통 비워 둡니다. 직접 지정한 경로는 일련번호 이름 검사를 하지 않지만 `/dev/serial/by-id` 같은 고정 이름인지는 계속 검사합니다.
현재 값 확인: `python scripts/check_devices.py`

## 5. 하드웨어 주의

- **전원**: Pi 5는 5V 5A급. 배터리에서 강압 모듈로 받고, 서보·펌프와 전원 레일을 나누고 GND는 한 점에서 묶습니다. Astra·카메라는 전원 있는 USB 허브를 권합니다.
- **펌프**: 켜질 때 전압이 떨어져 ESP32·Pi가 재부팅될 수 있습니다. 큰 콘덴서와 역기전력 다이오드를 넣습니다.
- **안전**: 서보 전원을 끊는 물리 비상정지 스위치를 둡니다. Wi-Fi가 끊겨도 진행 중인 작업은 Pi 안에서 계속됩니다. 이동 중에는 팔을 HOME으로 접어 둡니다.
- **책상 높이**: 카메라→로봇 보정이 책상 평면 높이를 고정으로 씁니다. 높이가 다른 책상에서는 좌표가 틀어집니다.
- **주소**: Pi에 고정 IP나 호스트명(`robotarm.local`)을 줍니다. 마이크는 브라우저 보안상 https나 localhost에서만 열립니다(Pi 주소로 접속하면 https 설정이 필요).
