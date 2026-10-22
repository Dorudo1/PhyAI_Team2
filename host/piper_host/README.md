# piper_host — 제어 PC 호스트: 실제 로봇을 README 로봇 인터페이스로

PhyAI_Team2 README 의 "로봇 인터페이스" 표와 같은 이름 · 단위로 **실제** Piper · RealSense 를 내보낸다.
시뮬레이터(MuJoCo)에서 만든 앱이 코드 수정 없이 실제 로봇 데이터를 받는다.
하드웨어(CAN · 드라이버 · 카메라)는 이 PC 의 기존 도구가 켜고, 이 패키지는 이름 · 단위만 바꾼다.

## 대응표

| README | 실제 (이 PC) | 변환 |
|---|---|---|
| `/joint_states` (joint1~8, 위치 · 속도) | `/slave/joint_states_single` | joint7 = 그리퍼 폭/2, joint8 = −폭/2 (piper_ros 공식 규칙과 같음). 200 Hz |
| `/joint_ctrl_single` (joint1~7) | → `/policy/joint_cmd` → **piper-follow** → 슬레이브 | joint7 × 2 = 폭. 빠진 관절은 마지막 명령 유지(시뮬레이터와 같음) |
| `/overhead/color/image_rect_raw` · `depth/image_rect_raw` · `{color,depth}/camera_info` | `/cam2/…` (천장) | frame `overhead_optical`. 깊이 16UC1 mm 그대로 |
| `/wrist/…` | `/cam1/…` (손목) | frame `wrist_optical` |
| `/tf_static` | 캘리브레이션 (`data/calib/calibration.json`) | `world → base_link` (같은 위치), `base_link → overhead_optical`, `link6 → wrist_optical` (end_pose 차이 보정, 아래) |
| `/tf`, `/robot_description` | robot_state_publisher + piper_description URDF | `base_link → link1 … link8` |

이름 대응은 `config/interface.yaml` 한 곳에 있다. README 가 바뀌면 이 파일을 고친다.

## 실행

따로 켤 필요 없음 — `piper-sensors`, `piper-record`, `piper-teleop`, `piper-follow` 가 센서를 켤 때 같이 켜고 끝나면 끈다.
`cam-off` 는 남은 프로세스도 정리한다. 따로 켜기:

```bash
ros2 launch piper_host host.launch.py
```

**ROS 도메인**: 실제 로봇은 `ROS_DOMAIN_ID=10` (제어 PC `~/.bashrc`). 시뮬레이터(42~)와 섞이지 않는다. 팀원 앱도 같은 번호로 실행:
`ROS_DOMAIN_ID=10 bash scripts/run_runtime.sh <앱>`.

**팔을 움직이려면** 다른 터미널에서 `piper-follow` 를 켜고 Enter. 안전 브리지가 범위 제한 · 끊김 정지 · 종료 시 복귀를 맡는다.
piper-follow 가 꺼져 있으면 `/joint_ctrl_single` 명령은 어디에도 가지 않는다 (팔 안 움직임).

## 녹화 · 추출

| 상황 | 명령 | 녹화되는 것 |
|---|---|---|
| 실제 로봇 (텔레오퍼 · 정책) | `piper-record 이름`, `piper-follow --record 이름` | 기존 이름 전부 + `/joint_states` · `/joint_ctrl_single` · `/robot_description` · tf (영상은 중복 저장 안 함) |
| 시뮬레이터 · 이미 켜진 실제 로봇 | `piper-bag [이름]` (장치 안 켬) | README 이름만: `/joint_states`, `/joint_ctrl_single`, `/overhead/…`, `/wrist/…`, tf, `/robot_description`, `/sim/scene` |

`piper-ts export <녹화>` 는 두 형식을 모두 읽는다 (기존 이름이 있으면 그것, 없으면 README 이름).
README 녹화는 마스터 팔이 없으므로 `frames.csv` 에 `slave_*` 와 `cmd_*`(= `/joint_ctrl_single`)가 나오고, 뷰어는 cmd 와 slave 를 비교한다.
시뮬레이터 깊이(32FC1, m)는 실제와 같은 16bit mm PNG 로 저장한다. 시뮬레이터 영상은 `publish_cameras:=true` 일 때만 있다.

## 시뮬레이터와 다른 점

| 항목 | 시뮬레이터 | 실제 |
|---|---|---|
| `world` 원점 | 작업대 중앙 (base_link 는 (+0.03, −0.02)) | **base_link 와 같은 위치** (작업대 원점은 안 쟀음) |
| 손목 카메라 부모 | `link5` (joint6 회전 전) | `link6` (캘리브레이션을 손목 끝 기준으로 함) |
| 영상 | 320×240, 5 Hz, 깊이 32FC1 (m) | 640×480, 30 Hz, 깊이 16UC1 (mm) — discovery 는 둘 다 지원 |
| `/joint_states` | 50 Hz | 200 Hz |
| 팔 응답 | 명령 즉시 | piper-follow 속도(기본 30%)로 따라감 → 같은 경로가 더 오래 걸림 |
| 없는 토픽 | — | `/sim/scene`, `/<cam>/robot_mask`, `/sim/camera_request` |

## 확인 (2026-10-22, 녹화 재생으로 · 팔 안 움직임)

- `/joint_states` 200 Hz, 이름 joint1~8, joint7 = 폭/2 ✓ / 카메라 6개 토픽 약 30 Hz, frame 이름 ✓ / `/robot_description` ✓
- tf: `base_link ← overhead_optical` (502.9, −92.1, 700.3) mm = 캘리브레이션 값 ✓
- `/joint_ctrl_single` joint1~7 → `/policy/joint_cmd` (그리퍼 ×2), joint7 만 온 명령은 나머지 유지 ✓

## 실제 장치 확인 (2026-10-22, 팔 안 움직임)

- `/joint_states` 200 Hz · 카메라 6개 토픽 약 30 Hz · frame 이름 · `/robot_description` ✓ (녹화 재생 결과와 같음)
- **펌웨어 end_pose ≠ URDF link6**: piper_sdk 순기구학은 URDF link6 과 0.05 mm 이내로 같지만, 실제 펌웨어가 보내는 end_pose 는
  link6 기준 (−17.7, 0.05, +15.2) mm (슬레이브), (−16.6, 0.9, +16.9) mm (마스터) 떨어진 점이다 (방향 차 0.02° 이하).
  손목 카메라 캘리브레이션은 end_pose 기준이므로 `end_pose_in_flange` 만큼 옮겨 `link6` 에 붙인다.
- 보정 후 `base_link ← wrist_optical` 이 기존 end_pose 기반 tf 와 0.02 mm · 0.02° 차이 ✓, 천장 카메라 0.00 mm ✓

아직: 다른 자세에서 end_pose 차이가 일정한지(현재 두 팔 · 한 자세씩, 1~2 mm 차이), 팀 example 앱으로 실제 팔 움직이기(로봇 옆 사람 필요).
