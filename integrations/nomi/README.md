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
| `look_at_table` | 위 카메라로 찍어 보이는 물체·쓸 수 있는 메뉴 반환(움직이지 않음) | `POST /api/image` | 45초 |
| `get_arm_status` | 작업 중인지, HOME인지 | `GET /status` | 12초 |
| `sort_object(object_name, slot?)` | 물체를 집어 칸에 넣고 HOME 복귀 | `/api/menu/1~3`, 칸 지정 시 `5`, unknown은 `7` | 320초 |
| `sort_all_objects` | 세 물체 모두 기본 칸에 | `/api/menu/4` | 620초 |
| `arm_return_home` | HOME 복귀 | `/api/menu/6` | 100초 |

- `object_name`은 `white_box`, `red_cube`, `tape_measure`, `unknown` 중 하나입니다(enum). Skill 쪽은 "흰 상자", "줄자" 같은 한국어 이름도 받습니다.
- `look_at_table`은 사진 base64를 빼고 돌려줍니다(9B 모델 문맥 절약). 사진이 필요하면 `data.image_url`을 여세요.
- 정지: `ArmRestSkill.stop()`은 대기를 끊고 `POST /stop`을 보냅니다.

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
