"""NOMI ToolRegistry._build_specs()에 붙일 로봇팔 도구 정의.

NOMI 쪽에서:
    from nomi_agent.skills.arm_rest_skill import ArmRestSkill
    from nomi_agent.skills.arm_tool_specs import arm_tool_specs
    _create_skills():  "arm": ArmRestSkill(node)
    _build_specs():    specs += arm_tool_specs(s["arm"], ToolSpec, ArgumentSpec)

ToolSpec·ArgumentSpec을 인자로 받아 이 파일은 NOMI 패키지를 import하지 않는다(여기서 시험할 수 있게).
timeout은 실제 한 번 넣기(약 1~2분)보다 넉넉히 잡고, Skill 안 대기 시간보다 길게 둬서 Skill이 먼저 정지·실패를 돌려주게 한다.
"""

OBJECT_CHOICES = ('white_box', 'red_cube', 'tape_measure', 'unknown')


def arm_tool_specs(skill, ToolSpec, ArgumentSpec):
    def call(method):
        fn = getattr(skill, method)
        return lambda args: fn(**args)

    return [
        ToolSpec("look_at_table",
                 "로봇팔 위 카메라로 지금 작업대를 찍어 보이는 물체(흰 상자, 빨간 정육면체, 줄자, 등록 안 된 물체)와 "
                 "쓸 수 있는 메뉴 번호를 돌려준다. 로봇은 움직이지 않는다. 사용자가 작업대에 무엇이 있는지, 무엇을 넣을 수 있는지 "
                 "물으면 되묻지 말고 바로 부른다. 넣으라는 명령에는 부르지 말고 바로 sort_object를 쓴다.",
                 (), call("look_at_table"), 45.0),
        ToolSpec("get_arm_status", "로봇팔이 작업 중인지, HOME 자세인지, 마지막 작업 단계를 읽는다. 움직이지 않는다.",
                 (), call("get_arm_status"), 12.0),
        ToolSpec("sort_object",
                 "지정한 물체를 찾아 집어 트레이 칸에 넣고 HOME으로 돌아온다(집기와 놓기를 한 번에 한다). "
                 "칸을 말하지 않으면 기본 칸(흰 상자 1, 빨간 정육면체 2, 줄자 3)에 넣는다. unknown(새 물체)은 칸이 꼭 필요하고, "
                 "시작한 뒤에 사람이 물체를 놓으므로 작업대에 아직 없어도 된다(미리 확인하지 않는다).",
                 (ArgumentSpec("object_name", "넣을 물체", choices=OBJECT_CHOICES, max_length=20),
                  ArgumentSpec("slot", "넣을 트레이 칸 번호(1~3). 사용자가 말했을 때만", kind="integer",
                               required=False, minimum=1, maximum=3)),
                 call("sort_object"), 320.0, "robot_actuation", True),
        ToolSpec("sort_all_objects", "흰 상자 → 빨간 정육면체 → 줄자 순서로 세 물체를 모두 기본 칸에 넣는다. 앞 물체가 실패하면 멈춘다. "
                 "'책상 정리해줘', '다 치워줘', '분류해줘'처럼 물체 이름 없이 정리하라는 말도 이 도구다.",
                 (), call("sort_all_objects"), 620.0, "robot_actuation", True),
        ToolSpec("arm_return_home", "로봇팔을 접힌 HOME 자세로 돌려놓는다.",
                 (), call("arm_return_home"), 100.0, "robot_actuation", True),
    ]
