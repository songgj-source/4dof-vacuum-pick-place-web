# NOMI 연결: 로봇팔 REST API → NOMI Tool

동료 저장소(https://github.com/1872uu-source/dapier_project11)의 NOMI Agent는 **LLM → ToolRegistry 검증 → Skill → 결과** 순서로 동작합니다. 이 폴더의 파일은 NOMI에 로봇팔 Skill로 그대로 넣을 수 있게 만든 어댑터입니다. 이 Skill은 로봇팔 REST 서버(`scripts/arm_api.py`)를 HTTP로 부릅니다.

```
사용자 말 → Qwen3.5(NOMI) → sort_object(object_name="red_cube")
          → ToolRegistry 검증 → ArmRestSkill → POST /api/menu/2 → /tasks/<id> 완료 대기
          → success + completion_verified=True(로봇 쪽 결과가 성공일 때만)
```

LLM은 관절값을 보내지 않습니다. 물체 이름과 칸 번호만 넘깁니다(NOMI의 ARM_INTEGRATION.md 원칙과 같음).

## 파일

| 파일 | NOMI에서 둘 곳 | 내용 |
|---|---|---|
| `arm_rest_skill.py` | `nomi_agent/skills/arm_rest_skill.py` | Skill 본체. 표준 라이브러리만 사용(ROS·pip 설치 필요 없음) |
| `tool_specs.py` | `nomi_agent/skills/arm_tool_specs.py` | `_build_specs()`에 붙일 ToolSpec 5개 |

## 도구

| 도구 | 하는 일 | REST | timeout |
|---|---|---|---|
| `look_at_table` | 위 카메라로 찍어 작업대 물체·트레이에 이미 있는 물체 반환(움직이지 않음) | `POST /api/image` | 45초 |
| `get_arm_status` | 작업 중인지(지금 단계), HOME인지 | `GET /status` | 12초 |
| `sort_object(object_name, slot?)` | 물체를 집어 칸에 넣고 HOME 복귀 | `/api/menu/5`(칸을 안 말하면 기본 칸), unknown은 `7` | 320초 |
| `sort_all_objects(objects?)` | 책상 정리하기: 책상에 있는 물체(기본 세 개 + 등록 물체, 또는 고른 물체)를 기본 칸에 | `/api/menu/4` | 1220초 |
| `arm_return_home` | HOME 복귀 | `/api/menu/6` | 100초 |

- `object_name`은 `white_box`, `red_cube`, `tape_measure`, `unknown` 또는 사용자가 등록한 물체 이름(예: "파란 컵")입니다. 등록 물체가 늘 수 있어 enum으로 막지 않습니다. Skill이 `GET /api/objects`(10초 캐시)로 이름을 확인하고, 모르는 이름이면 되묻게 돌려줍니다. "흰 상자", "줄자" 같은 한국어 이름도 받습니다.
- 물체 등록은 웹(5번)·터미널 메뉴(5번)·`POST /api/objects`에서 합니다. NOMI 도구로는 두지 않았습니다(물체를 책상에 하나만 두는 준비가 필요해서).
- `look_at_table`은 사진 base64를 빼고 돌려줍니다(9B 모델 문맥 절약). 사진이 필요하면 `data.image_url`을 여세요.
- `look_at_table`은 메뉴 번호(available_menus)를 돌려주지 않습니다. API 번호는 웹·터미널 메뉴 번호(1~4)와 달라 LLM이 헷갈리기 때문입니다.
- 정지: `ArmRestSkill.stop()`은 대기를 끊고 `POST /stop`을 보냅니다. 정지 뒤 3초 안에 들어온 움직이는 명령은 보내지 않습니다(호출 직전에 누른 비상정지가 지워지지 않게).
- 결과 조회가 404(서버 재시작)면 바로, 연결 실패가 약 10초 이어지면 정지를 보낸 뒤 실패로 돌려줍니다.
- 책상 정리하기 결과 메시지에는 물체별 결과(넣음 / 실패 / 건너뜀)가 붙습니다.

## NOMI에 넣는 방법 (tool_registry.py)

```python
# _create_skills()
from nomi_agent.skills.arm_rest_skill import ArmRestSkill
return {..., "arm": ArmRestSkill(node)}

# _build_specs() 끝, return 직전
from nomi_agent.skills.arm_tool_specs import arm_tool_specs
specs += arm_tool_specs(s["arm"], ToolSpec, ArgumentSpec)

# emergency_stop() — 지금은 navigation만 멈춘다. 로봇팔도 멈추게 추가
arm = self.skills.get("arm")
if arm is not None:
    arm.stop()
```

LLM 라우터 프롬프트(`llm_client.py`의 "사용 가능한 TASK" 목록)에 도구 이름과 예시를 추가합니다.

```
look_at_table
get_arm_status
sort_object
sort_all_objects
arm_return_home

예: "빨간 큐브 넣어줘" → sort_object {"object_name": "red_cube"}
    "줄자 1번 칸에 넣어" → sort_object {"object_name": "tape_measure", "slot": 1}
    "다 정리해줘" → sort_all_objects {}
    "흰 상자랑 줄자 치워줘" → sort_all_objects {"objects": "white_box,tape_measure"}
    "책상에 뭐 있어?" → look_at_table {}
```

기존 `pick_object`/`place_object`(따로 집고 따로 놓기)는 이 로봇팔과 맞지 않습니다. 이 팔은 진공으로 집은 뒤 바로 칸에 넣는 동작이 한 번에 묶여 있습니다. 그래서 `sort_object` 하나로 둡니다.

## 환경 변수 (NOMI .env)

```
ARM_API_URL=http://192.168.0.225:8000
ARM_API_TOKEN=<서버 --token 값>
```

로봇팔 PC에서는 서버를 이렇게 켭니다.
`/home/song/miniconda3/envs/lerobot_312/bin/python scripts/arm_api.py --host 0.0.0.0 --token <비밀번호>`

## 시험

`tests/test_nomi_adapter.py`: 가짜 실행기를 단 실제 API 서버에 어댑터를 붙여 확인합니다. 로봇과 카메라는 쓰지 않습니다. 확인 항목은 다음과 같습니다.
- 성공·실패 판정과 completion_verified
- 칸 지정 시 메뉴 5번 호출
- 작업 중 거절
- 정지·timeout 시 로봇 정지 전송
- 토큰 오류와 서버 꺼짐
- 사진 base64 제외
- ToolSpec 형식

NOMI 저장소 안에서 Qwen과 함께 실행한 시험은 하지 않았습니다.
