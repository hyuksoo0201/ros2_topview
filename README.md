# topview_ws

ROS2 워크스페이스 기반의 탑뷰 로컬라이제이션 프로젝트입니다.  
`topview_localization` 패키지에서 ArUco 마커를 이용해 AMR 포즈를 계산하고 `/amr_pose`로 퍼블리시합니다.

## 1) 프로젝트 개요

- 워크스페이스: `topview_ws`
- 주요 패키지: `src/topview_localization`
- 목적: 탑뷰 카메라 영상 기반 로봇 포즈(`x, y, yaw`) 추정 및 ROS2 토픽 발행

## 2) 패키지/노드 정보

- 패키지명: `topview_localization`
- 노드명: `topview_pose_node`
- 실행 엔트리: `ros2 run topview_localization topview_pose_node`
- 퍼블리시 토픽: `/amr_pose`
- 메시지 타입: `geometry_msgs/PoseStamped`

## 3) 요구 환경

- Ubuntu + ROS2 (Humble 이상 권장)
- Python `3.10` 이상 권장
- 의존성:
  - `rclpy`
  - `geometry_msgs`
  - `opencv-python`
  - `opencv-contrib-python` (`cv2.aruco` 사용)
  - `numpy`
  - `scipy`
  - `colcon`

## 4) 빌드 및 실행

```bash
cd /home/addinedu/topview_ws

# (권장) ROS2 환경 로드
source /opt/ros/humble/setup.bash

# 빌드
colcon build --symlink-install

# 워크스페이스 overlay
source install/setup.bash

# 노드 실행
ros2 run topview_localization topview_pose_node
```

토픽 확인:

```bash
ros2 topic echo /amr_pose
```

## 5) 파라미터/설정 포인트

`topview_pose_node.py` 기준 주요 설정:

- 카메라 인덱스: `CAM_INDEX`
- 캘리브레이션 파일 경로: `CALIB_PATH`
- 디버그 파라미터:
  - `show_debug_view` (기본 `True`)
  - `debug_window_name` (기본 `topview_pose_debug`)

예시:

```bash
ros2 run topview_localization topview_pose_node \
  --ros-args -p show_debug_view:=false
```

## 6) 워크스페이스 구조

```text
topview_ws/
├─ src/
│  └─ topview_localization/
│     ├─ package.xml
│     ├─ setup.py
│     ├─ resource/topview_localization
│     ├─ topview_localization/
│     │  ├─ topview_pose_node.py
│     │  └─ calib_data_topview_v3.npz
│     └─ test/
│        ├─ test_flake8.py
│        ├─ test_pep257.py
│        └─ test_copyright.py
├─ build/
├─ install/
└─ log/
```

## 7) 주의사항

- `build/`, `install/`, `log/` 디렉터리는 버전관리 대상에서 제외합니다.
- 카메라 점유 중복 주의:
  - 다른 프로세스(예: 별도 OpenCV 뷰어)가 카메라를 쓰고 있으면 노드가 카메라를 열지 못할 수 있습니다.
- 하드코딩 경로 주의:
  - `CALIB_PATH`가 절대경로로 설정되어 있으면 환경에 맞게 수정이 필요합니다.

## 8) 테스트

현재 패키지 내 테스트 파일:

- `test/test_flake8.py`
- `test/test_pep257.py`
- `test/test_copyright.py`

실행 예시:

```bash
cd /home/addinedu/topview_ws
source /opt/ros/humble/setup.bash
colcon test
colcon test-result --verbose
```

## 9) 라이선스

`TODO: license to be decided`

