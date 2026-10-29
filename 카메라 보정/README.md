# 카메라 보정/ — 카메라↔로봇 좌표 측정점

팔 FK를 자로 삼아 Astra 카메라 자세를 다시 맞출 때 모은 점입니다. 코드가 이 경로를 직접 읽으므로 **폴더 이름을 바꾸지 마세요**(`scripts/cam_robot_alignment.py`, `tests/test_camera_transforms.py`).

| 파일 | 내용 |
|---|---|
| `재보정_점.jsonl` | 재보정 측정점 |
| `재보정_점_Pi_20261026.jsonl` | Raspberry Pi에서 다시 잰 측정점 (시험에 사용) |
| `aruco_markers_A4.pdf` / `.png` | 인쇄용 ArUco 마커 (A4) |
