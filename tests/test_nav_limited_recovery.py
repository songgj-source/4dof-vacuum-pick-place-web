"""제자리에서 빙글빙글 도는 Nav2 복구 동작을 줄인다(2026-10-29, 책상 도착 때). ROS 없이 파일과 설정만 확인한다."""
import re
import sys
import types
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
BRINGUP = ROOT / 'ros2_slam/src/car2_bringup'
BT = BRINGUP / 'config/behavior_trees/navigate_to_pose_limited_recovery.xml'


def test_behavior_tree_has_fewer_retries_and_no_spin():
    xml = BT.read_text()
    assert re.search(r'<RecoveryNode number_of_retries="2" name="NavigateRecovery">', xml)
    assert '<Spin' not in xml and '<BackUp' in xml and '<Wait wait_duration="2.0"/>' in xml
    assert xml.count('ClearEntireCostmap') >= 4                          # 지우기 복구는 그대로
    import xml.etree.ElementTree as ET
    assert ET.parse(BT).getroot().tag == 'root'                          # 올바른 XML


def test_planner_tolerance_and_inflation_are_loosened():
    params = yaml.safe_load((BRINGUP / 'config/nav2_lidar_params.yaml').read_text())
    assert params['planner_server']['ros__parameters']['GridBased']['tolerance'] == .3
    for costmap in ('local_costmap', 'global_costmap'):
        inner = params[costmap][costmap]['ros__parameters']
        assert inner['inflation_layer']['inflation_radius'] == .33 and inner['robot_radius'] == .23      # 몸 크기는 그대로


def test_setup_installs_the_behavior_tree():
    assert "'config', 'behavior_trees'" in (BRINGUP / 'setup.py').read_text()


def _launch_helper():
    for name in ('launch', 'launch.actions', 'launch.conditions', 'launch.launch_description_sources', 'launch.substitutions', 'launch_ros',
                 'launch_ros.actions', 'ament_index_python', 'ament_index_python.packages'):
        sys.modules.setdefault(name, types.ModuleType(name))
    stub = sys.modules
    for name, attrs in (('launch', ['LaunchDescription']), ('launch.actions', ['DeclareLaunchArgument', 'ExecuteProcess', 'IncludeLaunchDescription', 'OpaqueFunction', 'TimerAction']),
                        ('launch.conditions', ['IfCondition']), ('launch.launch_description_sources', ['PythonLaunchDescriptionSource']),
                        ('launch.substitutions', ['LaunchConfiguration', 'PythonExpression']), ('launch_ros.actions', ['Node']),
                        ('ament_index_python.packages', ['get_package_share_directory'])):
        for attr in attrs:
            setattr(stub[name], attr, getattr(stub[name], attr, object))
    namespace = {'__name__': 'lidar_navigation_launch'}
    exec(compile((BRINGUP / 'launch/lidar_navigation.launch.py').read_text(), 'lidar_navigation.launch.py', 'exec'), namespace)
    return namespace['with_limited_recovery_bt']


def test_launch_injects_the_installed_tree_path_and_falls_back_safely(tmp_path):
    inject = _launch_helper()
    share = tmp_path / 'share'
    (share / 'config/behavior_trees').mkdir(parents=True)
    (share / 'config/behavior_trees/navigate_to_pose_limited_recovery.xml').write_text('<root/>')
    source = BRINGUP / 'config/nav2_lidar_params.yaml'
    out = inject(str(source), str(share))
    data = yaml.safe_load(Path(out).read_text())
    assert data['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] == str(share / 'config/behavior_trees/navigate_to_pose_limited_recovery.xml')
    assert data['planner_server'] == yaml.safe_load(source.read_text())['planner_server']          # 나머지 설정은 그대로
    assert inject(str(source), str(tmp_path / 'missing')) == str(source)                           # 행동 트리가 없으면 원래 설정
    broken = tmp_path / 'broken.yaml'
    broken.write_text('x: 1\n')
    assert inject(str(broken), str(share)) == str(broken)                                            # bt_navigator 항목이 없어도 죽지 않는다


def test_failed_drive_text_explains_wall_proximity():
    import base_drive
    text = base_drive.nav_failed_text('책상', {'message': ''}, 'aborted')
    assert text.startswith('책상으로 가지 못했습니다: aborted.') and '벽·장애물에 너무 붙어' in text
    assert '벽·장애물' not in base_drive.nav_failed_text('책상', {'message': '취소됨'}, 'canceled')


def test_progress_checker_accepts_the_small_moves_of_the_final_approach():
    # 2026-10-29: 라벨 15cm 앞에서는 0.2m를 못 움직여 'Failed to make progress'로 Goal failed → 벽 근접에 못 넘어갔다
    params = yaml.safe_load((BRINGUP / 'config/nav2_lidar_params.yaml').read_text())
    checker = params['controller_server']['ros__parameters']['progress_checker']
    assert checker['plugin'] == 'nav2_controller::PoseProgressChecker'
    assert checker['required_movement_radius'] <= 0.05 and checker['required_movement_angle'] <= 0.1
    assert 15 < checker['movement_time_allowance'] <= 40                       # 정말 멈췄을 때만 실패로 본다


def test_near_goal_critics_allow_the_turn_needed_to_move_sideways():
    # 2026-10-29: 라벨에서 옆 15cm: GoalAngle·PreferForward·PathAngle(0.5m, 1.0rad)이 돌아서 가는 것을 막아 속도 명령이 안 나왔다
    mppi = yaml.safe_load((BRINGUP / 'config/nav2_lidar_params.yaml').read_text())['controller_server']['ros__parameters']['FollowPath']
    for critic in ('GoalAngleCritic', 'PreferForwardCritic', 'PathAngleCritic'):
        assert mppi[critic]['threshold_to_consider'] <= .15, critic
    assert mppi['PathAngleCritic']['max_angle_to_furthest'] >= 1.57                # 90° 회전까지 허용


def test_robot_radius_is_23cm_everywhere():
    # 2026-10-29 사용자: 장애물 영역(차체) 지름 46cm → 반지름 0.23m. Nav2 지역·전역 비용지도, map_web, 마무리 정렬이 같은 값을 쓴다
    params = yaml.safe_load((BRINGUP / 'config/nav2_lidar_params.yaml').read_text())
    for costmap in ('local_costmap', 'global_costmap'):
        assert params[costmap][costmap]['ros__parameters']['robot_radius'] == .23
    assert "declare_parameter('robot_radius', 0.23)" in (ROOT / 'ros2_slam/test/map_web.py').read_text()
    assert "'radius': 0.23" in (ROOT / 'scripts/base_drive.py').read_text()
