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

- 다른 주소·사용자면 `ROBOT=사용자@주소`를 앞에 붙입니다. 비밀번호는 접속할 때 한 번 묻습니다(저장소에 없음, 팀원에게 확인).
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
