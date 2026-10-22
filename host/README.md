# host/ — 제어 PC 전용 (control PC only)

로봇 · 카메라가 연결된 제어 PC(호스트)에서만 빌드 · 실행하는 패키지입니다. 하드웨어를 다루므로 개발 · 런타임 Docker 이미지에는 넣지 않습니다.
`COLCON_IGNORE` 때문에 devcontainer 의 colcon 빌드와 rosdep 설치는 이 폴더를 건너뜁니다.

Packages here run only on the control PC that owns the hardware. They are excluded from the dev and runtime images;
`COLCON_IGNORE` keeps devcontainer colcon builds and rosdep installs from picking them up.

| 패키지 | 내용 |
|---|---|
| [piper_host](piper_host/README.md) | 실제 Piper · RealSense → README 로봇 인터페이스 (`/joint_states`, `/joint_ctrl_single`, `/overhead`, `/wrist`, tf, `/robot_description`) |

호스트에서 빌드: 이 폴더의 패키지를 호스트 colcon 워크스페이스에 링크한 뒤 `colcon build --packages-select piper_host`.
드라이버(piper_ros) · 카메라(realsense-ros) · 안전 브리지(piper-follow)는 호스트에 따로 설치되어 있어야 합니다.
