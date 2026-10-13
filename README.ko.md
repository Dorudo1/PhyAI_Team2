# PhyAI Team2: PIPER 로봇 팔용 ROS 2 앱

[English](README.md) | **한국어**

Docker에서 앱을 개발하고, MuJoCo로 테스트한 뒤 런타임 이미지를 제어 PC에서 실행합니다.

[환경 설정](#1-환경-설정) → [구현](#2-구현) → [테스트](#3-mujoco로-테스트) → [빌드](#4-런타임-이미지-빌드) → [제어 PC에서 실행](#5-제어-pc에서-실행)

## 1. 환경 설정

**목적:** 공용 Ubuntu 22.04 + ROS 2 Humble 개발 환경을 엽니다.

**방법:** VS Code에서 저장소를 열고 **Dev Containers: Reopen in Container**를 실행합니다.

**예상 결과:** ROS 2를 사용할 수 있는 컨테이너 터미널이 열립니다. 저장소 경로는 `~/ws/src/PhyAI_Team2`입니다.

### 사전 요구 사항

| 위치 | 필요한 항목 |
|---|---|
| Docker를 실행하는 개발 PC | Linux (예: Ubuntu 24.04), NVIDIA 드라이버, Docker, NVIDIA Container Toolkit |
| VS Code를 실행하는 PC | Dev Containers 확장; 원격 작업 시 Remote - SSH 확장 |
| 컨테이너 내부 | 앱별 ROS 의존성 (`rosdep`), Python 라이브러리 (`pip`) |

Docker 이미지에 Ubuntu 22.04와 ROS 2 Humble이 포함됩니다. **개발 PC 호스트는 Ubuntu 24.04를 사용해도 됩니다.**
호스트에 Ubuntu 22.04나 ROS 2를 별도로 설치할 필요는 없습니다. 현재 devcontainer는 Linux 호스트 네트워크와
시뮬레이터 화면용 NVIDIA GPU를 사용합니다. SSH 접속용 노트북은 모든 OS를 사용할 수 있습니다.

### 로컬 또는 원격 접속

- **로컬:** 개발 PC에 저장소를 clone하고 VS Code로 연 뒤 컨테이너로 다시 엽니다.
- **원격:** **Remote-SSH: Connect to Host...**로 `<user>@<dev-pc>`에 접속하고,
  해당 PC의 저장소를 연 뒤 컨테이너로 다시 엽니다. Docker는 개발 PC에서 실행됩니다.

처음 컨테이너 빌드에는 몇 분이 걸립니다. `~/ws`가 colcon 워크스페이스입니다.

### 환경 확인

컨테이너에서 실행합니다:

```bash
ros2 doctor --report | head
nvidia-smi
```

ROS 진단 정보와 GPU 정보가 출력되면 됩니다.

## 2. 구현

**목적:** `apps/<name>/`에 앱을 만듭니다.

**방법:** example을 복사하고 ROS 패키지 이름을 변경한 뒤 Python 코드를 수정합니다.

**예상 결과:** `start.sh`로 실행되며 ROS 토픽으로 명령을 보내는 앱이 만들어집니다.

```bash
cd ~/ws/src/PhyAI_Team2
cp -r apps/example apps/<name>
mv apps/<name>/example_app apps/<name>/<your_pkg>
```

### 이름 변경과 폴더 구성

`package.xml`, `setup.py`, `setup.cfg`, `resource/`, Python 패키지 폴더, launch 파일,
`start.sh`의 `example_app`을 `<your_pkg>`로 변경합니다.

```text
apps/<name>/
├── start.sh                 # 앱 실행 명령
├── requirements.txt         # 선택: pip 의존성
└── <your_pkg>/              # 하나 이상의 ROS 패키지
    ├── package.xml
    ├── setup.py / setup.cfg / resource/<your_pkg>
    ├── <your_pkg>/*.py
    └── launch/*.launch.py
```

`<name>`에는 영문 소문자, 숫자, `-`, `_`를 사용합니다. ROS 패키지 이름은 저장소 전체에서 고유해야 합니다.
런타임 빌드는 `apps/<name>/`만 복사하므로 앱 파일은 이 폴더 안에 둡니다.

### 로봇 인터페이스

앱이 목표값을 보내면 시뮬레이터 또는 제어 PC의 브리지가 받아 동작하고 상태를 반환합니다.

| 토픽 | 메시지 | 방향 |
|---|---|---|
| `/joint_ctrl_single` *(임시)* | `sensor_msgs/JointState`: `joint1`–`joint6` 각도 (rad), `joint7` 그리퍼 벌림 폭 (0–0.035 m) | 앱 → 팔 |
| `/joint_states` | `sensor_msgs/JointState`: `joint1`–`joint8` 위치와 속도 | 팔 → 앱 |
| 카메라 토픽 *(미정)* | `sensor_msgs/Image`, `CameraInfo` | 호스트 → 앱 |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | 호스트 → 앱 |
| `/robot_description` | `std_msgs/String` (URDF) | 호스트 → 앱 |
| `/sim/scene` *(시뮬레이터 전용)* | `std_msgs/String`: 블록 위치, 로봇 운동학, 시뮬레이션 시간, 일시 정지 상태와 재시작 ID의 JSON | 시뮬레이터 → example |

임시 토픽과 카메라 이름은 호스트 스택 담당자가 확정합니다. 팔 명령은 현재 `piper_ros` 규칙을 따릅니다.
브리지가 커스텀 메시지를 사용하면 해당 패키지를 base 이미지에 추가해야 합니다.

### 의존성과 설치 파일

| 파일 | 선언할 내용 |
|---|---|
| `package.xml` | 모든 ROS/rosdep 의존성: 실행은 `<exec_depend>`, 빌드는 `<build_depend>`, 둘 다 필요하면 `<depend>` |
| `requirements.txt` | rosdep 키가 없는 비 ROS Python 의존성; 버전 고정 |
| `install_system_deps.sh` | 선택: 시스템 라이브러리 또는 CUDA Toolkit; 이미지 빌드 시 root로 실행 |
| `setup.py` | `console_scripts`에 실행 파일, `data_files`에 launch·설정·URDF·모델 파일 |
| `start.sh` | `exec`를 사용한 실행 명령; LF 줄바꿈 |

실행 파일 등록 예: `'example = example_app.example:main'`.
ROS가 실행 파일을 찾도록 `setup.cfg`와 `resource/<your_pkg>`를 유지합니다. 설치된 파일은
`get_package_share_directory('<your_pkg>')`로 불러옵니다. rosdep 키는 `rosdep resolve <key>`로 확인합니다.

런타임 이미지는 실행 의존성과 설치된 앱 파일만 포함합니다. 선언이 빠지면 배포 후
`ModuleNotFoundError`나 파일 누락이 발생할 수 있습니다. pip 라이브러리는 실제로 import하는 것만 추가하세요.
큰 패키지는 이미지 전송을 느리게 합니다. 하드웨어 SDK와 드라이버는 호스트가 담당합니다.

### 아키텍처와 런타임 제약

```text
제어 PC 호스트: Ubuntu 22.04 + ROS 2 Humble
├── piper_bridge → piper_sdk → CAN → 팔
├── camera driver
└── robot_state_publisher
          ↕ ROS 2 / DDS
런타임 컨테이너 (--network host)
└── 여러분의 앱
```

호스트가 하드웨어를 담당합니다. 앱은 ROS 토픽을 사용하며 디바이스에 직접 접근하지 않습니다.
런타임 앱은 ROS와 앱 환경이 source된 `dev` 사용자로 실행되며 디스플레이가 없습니다.
개발·런타임 이미지는 같은 ROS base를 사용합니다. 배포 전 [런타임 이미지 확인](#4-런타임-이미지-빌드)으로
의존성이 모두 포함되었는지 확인하세요.

## 3. MuJoCo로 테스트

**목적:** 시뮬레이션 팔로 앱을 테스트합니다.

**방법:** devcontainer에서 테스트 스크립트를 실행하고 브라우저 URL을 엽니다.

**예상 결과:** example이 도달 가능한 블록 하나를 집어 들어 올린 뒤 테이블 위에서 놓고
홈 자세로 돌아옵니다. 터미널에 단계와 검증 결과를 출력합니다.

```bash
cd ~/ws/src/PhyAI_Team2
bash scripts/test_sim.sh example
```

**http://localhost:23517**을 엽니다. 종료는 **Ctrl+C**입니다.

### 앱 선택

먼저 앱의 ROS 의존성은 `rosdep`으로, `requirements.txt`의 패키지는 `pip`으로 설치합니다.
그다음 실행합니다:

```bash
bash scripts/test_sim.sh <name>
bash scripts/test_sim.sh          # 메뉴에서 선택
```

스크립트는 선택한 앱과 시뮬레이터를 빌드하고, MuJoCo를 시작하거나 재사용한 뒤
`apps/<name>/start.sh`를 실행합니다. 앱은 [로봇 인터페이스](#로봇-인터페이스)로 동작을 제어합니다.
[example.py](apps/example/example_app/example_app/example.py)는 50 Hz로 부드러운 팔·그리퍼 목표값을
보내 고정 집기·낙하 시나리오를 실행합니다. [pick_drop.py](apps/example/example_app/example_app/pick_drop.py)의
검증된 관절 경유점을 사용하며 런타임 IK 탐색은 하지 않습니다. 알려진 배치의 `block_3` 위치는
(0.40, -0.33) m, 블록 크기는 7.5 × 2.5 × 1.5 cm이며 팔 베이스는 원래 자세를 사용합니다.
`/sim/scene`으로 실제 들어 올림과 착지를 확인합니다. 장면 편집 중 일시 정지는 시나리오도 정지합니다.
두 브라우저 페이지의 **Restart simulation** 버튼은 팔과 블록을 초기화하고 example을 다시 실행합니다.
현재 장면 설정은 유지하며 JSON 파일을 저장하거나 다시 읽지 않습니다. `sim_cmd.sh`는 수동 명령이 필요할 때만 사용합니다.

### 브라우저 접속

| 접속 방식 | URL |
|---|---|
| 개발 PC에서 접속 | `http://localhost:23517` |
| LAN의 다른 PC에서 접속 | `http://<dev-pc-ip>:23517` |
| SSH로 접속 | VS Code **Ports** 패널에서 포워딩한 뒤 `http://localhost:23517` |

다른 포트를 지정했다면 출력된 URL을 사용합니다. GPU/EGL로 화면을 렌더링해 영상을 전송하므로
데스크톱 디스플레이는 필요 없습니다. 시뮬레이터는 개발 전용입니다. 브라우저 카메라 뷰를 제공하지만 ROS 카메라 토픽과 `/tf`는 아직 발행하지 않습니다.
모델은 [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/agilex_piper)를 사용합니다.

### 실험실 장면과 카메라 뷰

기본 장면은 84 × 54 cm 상판 네 개를 2 × 2로 배치한 테이블, 전체 테이블 중앙 근처에
설치한 PIPER, 7.5 × 2.5 × 1.5 cm 블록 여섯 개를 포함합니다.
브라우저의 **Overview**, **Overhead camera**, **Wrist camera** 버튼으로 뷰를 전환합니다.
Overview에는 X/Y/Z 방향 표시가, Overhead camera에는 X/Y 방향 표시가 나타납니다.
화살표는 월드 좌표계의 양의 방향을 가리키며 X는 빨강, Y는 초록, Z는 파랑입니다.
상부 카메라 렌즈는 상판에서 89 cm 위에 있으며 전체 테이블 중앙 기준
X = -3 cm, Y = +2 cm 위치에서 수직으로 내려다봅니다.
손목 카메라는 `joint6` 회전 이전의 팔 케이싱(`link5`)에 고정됩니다. 현재는 이상적인 RGB 영상만 제공하며
장착 프레임은 고정 그리퍼 루트 좌표계(`gripper_root_mount`) 기준으로 배치하며 카메라는
장착 프레임 기준으로 배치합니다. 렌즈는
그리퍼와 같은 전방 방향(그리퍼 루트 +Y, `link6` +Z)을 평행하게 바라봅니다.
참고 사진처럼 은색 카메라 바를 팔에 수직으로 가로 배치하고 얇은 패드로 손목에 직접
부착합니다. 포스트는 없으며 손가락을 향해 기울이지 않습니다. 하우징 치수는 추정값입니다.
`wrist_camera_mount_position`과 `wrist_camera_mount_quaternion`은 패드, 하우징, 렌즈를 함께
이동하거나 회전합니다.
`wrist_camera_roll_degrees`로 하우징과 별도로 영상 방향을 조정합니다.
이 좌표계는 손목 롤이 0일 때의 그리퍼 루트 자세를 사용하며 팔 케이싱에 고정됩니다.
+X는 롤이 0인 `link6`의 +X와 반대입니다. 손목 롤과 손가락 개폐는 카메라를 케이싱에서 움직이지 않습니다.
`wrist_camera_position`과 `wrist_camera_quaternion`은 장착 프레임 기준으로 카메라 렌즈의
위치(m)와 회전(`[w, x, y, z]`)을 설정하며
로봇 손목의 위치나 관절값을 바꾸지 않습니다.
롤 기본값은 0이며 영상 위쪽은 그리퍼 루트 +X, 카메라 바의 가로 방향은 그리퍼 루트 Z입니다.
음수 값은 영상을 반시계 방향으로 회전합니다.
SR305의 노이즈 모델이나 ROS RGB-D 토픽은 포함하지 않습니다.

[lab_scene.json](sim/piper_mujoco_sim/config/lab_scene.json)에서 설정을 수정합니다.
거리 단위는 m이며 원점은 전체 상판 중앙, +Z는 위쪽입니다. +X는 168 cm 방향,
+Y는 108 cm 방향입니다. 로봇 베이스와 포스트 중앙은 원점 기준
X = +3 cm, Y = -2 cm이며 `post_position`으로 설정합니다. 상부 카메라는
같은 좌표계의 `overhead_position`으로 설정합니다. 상판 간격 1 mm, 테이블 높이 75 cm,
포스트 반지름 4 cm, 팔 방향, 블록 질량 20 g 및 마찰, 손목 카메라의 정확한 위치와
방향은 조정 가능한 초기 추정값입니다. 리더 암은 생략했습니다.
기본 브라우저 페이지(`/`)는 읽기 전용 뷰어입니다. `/edit`에서 **Scene editor**를 열거나
`scene_editing:=true`로 실행하면 `/`에도 편집기를 표시합니다:

```bash
ros2 launch piper_mujoco_sim sim.launch.py scene_editing:=true
# 스크립트로 새 시뮬레이터를 시작하는 경우:
SCENE_EDITING=true bash scripts/test_sim.sh
```

스크립트는 두 URL을 출력합니다. 기존 시뮬레이터를 재사용하면 홈페이지 모드는 유지됩니다.
편집 API는 `/edit/scene`에만 있으며 `/` 조회는 다른 뷰어의 편집 모드를 활성화하지 않습니다.
편집기에서 팔 베이스, 카메라 마운트와 렌즈, 상부 카메라,
시야각 및 블록을 편집할 수 있습니다. 위치는 cm, 회전은 도 단위이며 부모 좌표계를 표시합니다.
Preview는 즉시 적용하고 물리와 팔 명령을 일시 정지합니다. **Resume simulation**으로 재개합니다.
**Revert**는 마지막 저장 상태로 복원하고 **Save scene**은 설정 JSON에 저장합니다.
**Reload scene file**은 텍스트 편집기로 변경한 위치와 회전을 불러옵니다. 사용자 지정 MJCF는 뷰만 제공합니다.
테이블 치수 등 구조 설정을 수동 변경한 경우 시뮬레이터를 재시작해야 합니다. `test_sim.sh`가 재사용하는 기존
시뮬레이터에는 변경이 적용되지 않습니다.

앱 없이 장면만 확인하려면 컨테이너에서 실행합니다:

```bash
cd ~/ws
colcon build --symlink-install --packages-select piper_mujoco_sim
source install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch piper_mujoco_sim sim.launch.py
```

### ROS 도메인, 포트, 기존 서버

| 설정 | 기본값 | 용도 |
|---|---|---|
| `ROS_DOMAIN_ID` | `42` | 앱과 시뮬레이터가 공유하는 ROS 도메인 |
| `HTTP_PORT` | `23517` | 고정 브라우저 포트 |

개발자마다 팀원 및 실제 로봇과 다른 도메인을 사용합니다:

```bash
ROS_DOMAIN_ID=43 HTTP_PORT=23518 bash scripts/test_sim.sh <name>
# 또는 이 터미널의 이후 명령에 도메인 설정:
export ROS_DOMAIN_ID=43
```

포트가 사용 중이면 프로세스 이름, PID, 실행 명령을 출력합니다. 같은 ROS 도메인의 MuJoCo 시뮬레이터만
재사용합니다. 도메인이 다르거나 다른 서버이면 안내를 출력하며, 포트를 자동 변경하거나 기존 서버를
종료하지 않습니다. 소유자를 확인할 수 없으면 호스트에서 해당 포트를 확인하세요.

Ctrl+C는 앱과 스크립트가 시작한 시뮬레이터를 종료합니다. 재사용한 시뮬레이터는 계속 실행됩니다.

### 개별 실행과 수동 명령

앱과 시뮬레이터를 별도 터미널에서 실행하려면 두 터미널에 같은 `ROS_DOMAIN_ID`를 설정합니다.
먼저 빌드하고 환경을 불러옵니다:

```bash
cd ~/ws
colcon build --symlink-install --packages-select <your_pkg> piper_mujoco_sim
source install/setup.bash
export ROS_DOMAIN_ID=43
ros2 launch piper_mujoco_sim sim.launch.py
```

다른 터미널에서 `~/ws/install/setup.bash`를 source하고 같은 도메인을 설정한 뒤 앱을 실행합니다:

```bash
ros2 launch <your_pkg> <file>.launch.py
```

수동 동작 확인은 같은 도메인에서 저장소 최상위 폴더에서 실행합니다:

```bash
bash scripts/sim_cmd.sh joint1=0.8 joint7=0.03
```

앱이 목표값을 계속 보내면 수동 명령을 덮어쓰므로 수동 확인 전 제어 앱을 종료합니다.

## 4. 런타임 이미지 빌드

**목적:** 앱 하나를 배포용 이미지로 만듭니다.

**방법:** devcontainer 밖의 **개발 PC 호스트 셸**에서 저장소 최상위 폴더에서 빌드합니다.

**예상 결과:** Docker 이미지 `phyai/app-<name>:latest`가 생성됩니다.

```bash
scripts/build_runtime.sh <name>
```

### CUDA와 PyTorch 포함하기

**목적:** 앱별 GPU 의존성을 선언합니다.

**방법:** `apps/<name>/requirements.txt`와 필요 시 `install_system_deps.sh`를 수정하고 다시 빌드합니다.

**예상 결과:** 선언한 패키지가 빌드·런타임 이미지에 포함되어 실행 시 별도 설치가 필요 없습니다.

예를 들어 CUDA 12.4용 PyTorch wheel은 `requirements.txt`에 선언합니다:

```text
--extra-index-url https://download.pytorch.org/whl/cu124
torch==2.5.1+cu124
```

위 버전 조합은 예시입니다. 앱과 GPU에 맞는 wheel은
[PyTorch 설치 안내](https://pytorch.org/get-started/previous-versions/)에서 선택하세요.
별도 CUDA Toolkit이나 컴파일러가 필요하면
[install_system_deps.sh](apps/example/install_system_deps.sh)의 주석 예시를 수정합니다.
이 스크립트는 pip 설치 전에 의존성 선언 파일만 있는 읽기 전용 폴더에서 실행됩니다.
다운로드는 `/tmp`를 사용하고, 스크립트 하나로 설치할 수 있게 작성하세요.
설치한 패키지는 최종 이미지에 포함되지만 스크립트의 `export`는 이후 단계에 유지되지 않습니다.

개발 컨테이너에서 설치한 패키지는 런타임 이미지에 자동 복사되지 않습니다. 로컬 테스트에는 컨테이너 안에서
같은 선언을 적용하세요 (`sudo bash apps/<name>/install_system_deps.sh`,
`pip3 install -r apps/<name>/requirements.txt`). 테스트 스크립트는 이를 자동 설치하지 않습니다.
빌드 시 다운로드에는 네트워크가 필요하지만 준비된 런타임 이미지는 오프라인으로 실행할 수 있습니다.
제어 PC에는 호환되는 NVIDIA 드라이버와 Container Toolkit이 필요합니다.

### 빌드 전 의존성 확인

devcontainer 안에서 실행합니다:

```bash
rosdep install --from-paths ~/ws/src/PhyAI_Team2/apps/<name> --ignore-src -y --simulate
```

### 빌드된 이미지 확인

개발 PC 호스트 셸에서 실행합니다:

```bash
docker run --rm -it --network host -e ROS_DOMAIN_ID=43 phyai/app-<name>
```

상태 수신과 동작 테스트에는 시뮬레이터와 같은 도메인을 사용합니다. 개발 환경에서는 동작하지만 여기서
실패하면 [의존성과 설치 파일](#의존성과-설치-파일)을 확인하세요. `--symlink-install`은 `data_files` 누락을
숨길 수 있으므로 실제 런타임 이미지로 확인합니다.

## 5. 제어 PC에서 실행

**목적:** 준비된 런타임 이미지를 제어 PC의 호스트 스택과 함께 실행합니다.

**방법:** 제어 PC 셸에서 앱을 시작합니다.

**예상 결과:** 컨테이너가 `start.sh`를 실행하고 ROS 토픽으로 로봇과 통신합니다.

```bash
# 제어 PC 셸:
~/phyai/run_runtime.sh <name>
```

### 제어 PC 요구 사항

- Docker; 앱이 GPU를 사용하면 NVIDIA Container Toolkit.
- 사용자 계정의 `docker` 그룹 권한.
- 제어 PC에 미리 준비된 `phyai/app-<name>:latest` 이미지와 `~/phyai/run_runtime.sh`.
- ROS 2 Humble에서 실행 중인 호스트 스택 (piper_bridge, camera driver, robot_state_publisher).

`run_runtime.sh`는 셸의 `ROS_DOMAIN_ID` (기본값 `0`)를 전달하므로 호스트 스택과 같아야 합니다.
한 번에 하나의 앱만 팔을 제어할 수 있으며,
다른 앱이 실행 중이면 실행 스크립트가 시작을 거부합니다.

### 종료와 디버깅

Ctrl+C를 누르거나 다른 터미널에서 종료합니다. 디버깅하려면 앱 대신 셸을 시작합니다:

```bash
docker stop phyai-<name>
~/phyai/run_runtime.sh <name> bash
```

### DDS 전송

호스트와 컨테이너의 사용자 ID가 다르면 Fast DDS 공유 메모리 전송에서 토픽은 보이지만 데이터가
도착하지 않을 수 있습니다. 런타임 이미지는 이를 피하기 위해 [UDP 전용 프로필](docker/fastdds_udp.xml)을
사용합니다. `FASTRTPS_DEFAULT_PROFILES_FILE`은 이 프로필을 유지하세요.
