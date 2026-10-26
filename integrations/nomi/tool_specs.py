"""NOMI ToolRegistry._build_specs()에 붙일 로봇팔 도구 정의.

NOMI 쪽에서:
    from nomi_agent.skills.arm_rest_skill import ArmRestSkill
    from nomi_agent.skills.arm_tool_specs import arm_tool_specs
    _create_skills():  "arm": ArmRestSkill(node)
    _build_specs():    specs += arm_tool_specs(s["arm"], ToolSpec, ArgumentSpec)

ToolSpec·ArgumentSpec을 인자로 받아 이 파일은 NOMI 패키지를 import하지 않는다(여기서 시험할 수 있게).
timeout은 실제 한 번 넣기(약 1~2분)보다 넉넉히 잡고, Skill 안 대기 시간보다 길게 둬서 Skill이 먼저 정지·실패를 돌려주게 한다.
ToolSpec 필드는 이름으로 넘긴다(NOMI 쪽 필드 순서가 바뀌어도 timeout·category가 엉뚱한 자리에 들어가지 않게).
"""

OBJECT_CHOICES = ('white_box', 'red_cube', 'tape_measure', 'unknown')     # 기본 이름. 사용자가 등록한 물체는 이름(예: '파란 컵') 그대로 받는다
TIDY_TIMEOUT_S = 1220.0     # 기본 3 + 등록 최대 3 = 6개 × Skill 대기 200초 + 여유(Skill이 먼저 끝나게)


def arm_tool_specs(skill, ToolSpec, ArgumentSpec):
    def call(method):
        fn = getattr(skill, method)
        return lambda args: fn(**args)

    return [
        ToolSpec(name="look_at_table",
                 description="로봇팔 위 카메라로 지금 작업대를 찍어 보이는 물체(흰 상자, 빨간 정육면체, 줄자, 등록 안 된 물체)와 "
                             "트레이 칸에 이미 들어 있는 물체를 알려 준다. 로봇은 움직이지 않는다. 사용자가 작업대나 트레이에 무엇이 있는지 "
                             "물으면 되묻지 말고 바로 부른다. 넣거나 정리하라는 명령에는 부르지 말고 바로 sort_object·sort_all_objects를 쓴다.",
                 arguments=(), executor=call("look_at_table"), timeout_seconds=45.0),
        ToolSpec(name="get_arm_status",
                 description="로봇팔이 작업 중인지(작업 중이면 지금 단계), 대기 중이면 HOME 자세인지 읽는다. 움직이지 않는다. "
                             "'지금 뭐 해?', '끝났어?' 같은 물음에 쓴다.",
                 arguments=(), executor=call("get_arm_status"), timeout_seconds=12.0),
        ToolSpec(name="sort_object",
                 description="물체 하나를 찾아 집어 트레이 칸에 넣고 HOME으로 돌아온다(집기와 놓기를 한 번에 한다). "
                             "칸을 말하지 않으면 기본 칸(흰 상자 1, 빨간 정육면체 2, 줄자 3)에 넣는다. unknown(새 물체)은 칸이 꼭 필요하고, "
                             "시작한 뒤에 사람이 물체를 놓으므로 작업대에 아직 없어도 된다(미리 확인하지 않는다). "
                             "사용자가 물체 이름 없이 '이거 넣어줘'라고 하면 부르지 말고 어떤 물체인지 되묻는다. "
                             "물체를 두 개 이상 말하면 이 도구 대신 sort_all_objects에 objects로 넘긴다. "
                             "사용자가 등록한 물체(look_at_table에 이름이 나온다)는 그 이름 그대로 object_name에 넣는다.",
                 arguments=(ArgumentSpec("object_name", "넣을 물체: white_box, red_cube, tape_measure, unknown 또는 사용자가 등록한 물체 이름",
                                         max_length=20),
                            ArgumentSpec("slot", "넣을 트레이 칸 번호(1~3). 사용자가 칸을 말했을 때만 넣고, 기본 칸을 추측해 채우지 않는다",
                                         kind="integer", required=False, minimum=1, maximum=3)),
                 executor=call("sort_object"), timeout_seconds=320.0,
                 category="robot_actuation", requires_completion_evidence=True),
        ToolSpec(name="sort_all_objects",
                 description="책상 정리하기: 작업대에 있는 물체를 흰 상자 → 빨간 정육면체 → 줄자 → 사용자가 등록한 물체 순서로 각자 기본 칸에 넣는다. "
                             "책상에 없는 물체는 알아서 건너뛰고, 이미 트레이에 들어 있는 물체는 건드리지 않으므로 미리 look_at_table로 확인하지 않는다. 등록 안 된 물체는 집지 않고 피해 가며 그대로 둔다. "
                             "한 물체가 실패하면 거기서 멈춘다. '책상 정리해줘', '다 치워줘', '분류해줘'처럼 물체 이름 없이 정리하라는 말과 "
                             "'흰 상자랑 줄자 치워줘'처럼 여러 물체를 말한 경우에 쓴다.",
                 arguments=(ArgumentSpec("objects", "정리할 물체만 고를 때 쉼표로 구분(예: white_box,tape_measure,파란 컵). "
                                         "사용자가 물체를 말하지 않았으면 비운다(책상에 있는 것 전부)",
                                         required=False, max_length=120),),
                 executor=call("sort_all_objects"), timeout_seconds=TIDY_TIMEOUT_S,
                 category="robot_actuation", requires_completion_evidence=True),
        ToolSpec(name="arm_return_home",
                 description="로봇팔을 접힌 HOME 자세로 돌려놓는다. 정지한 뒤나 작업이 실패한 뒤에 팔을 접어 둘 때 쓴다. "
                             "sort_object·sort_all_objects는 끝나면 스스로 HOME으로 돌아오므로 그 뒤에 따로 부르지 않는다.",
                 arguments=(), executor=call("arm_return_home"), timeout_seconds=100.0,
                 category="robot_actuation", requires_completion_evidence=True),
    ]
