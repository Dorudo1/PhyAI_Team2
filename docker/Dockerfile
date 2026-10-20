# Base image for developing ROS 2 apps for the PIPER arm (perception / planning / application).
# Host provides: Linux (e.g. Ubuntu 24.04) + NVIDIA driver. The Humble image includes Ubuntu 22.04.
# App GPU/system dependencies: apps/<name>/install_system_deps.sh and requirements.txt.
# Hardware (piper_sdk, CAN, camera driver) lives on the control PC host, not in these images.
#
# Stages (build context = repo root):
#   base     minimal ROS 2 Humble; the common foundation of dev and runtime images
#   dev      base + dev tools; used by .devcontainer
#   deps     only one app's dependency declarations
#   app-deps base + app system/pip dependencies, shared by build and runtime
#   build    compiles one app (apps/<name>) with colcon
#   runtime  base + the built app only; this is what ships to the control PC
#
#   docker build -f docker/Dockerfile --target runtime --build-arg APP=example -t phyai/app-example .
ARG ROS_DISTRO=humble

# ---------------------------------------------------------------------------
FROM ros:${ROS_DISTRO}-ros-base AS base

ARG ROS_DISTRO
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO}

# No hardware packages here: the control PC host owns piper_sdk, CAN and the camera driver,
# and apps reach them only through ROS 2 topics (see "Robot interface" in README.md).
RUN apt-get update && apt-get install -y --no-install-recommends \
        sudo python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Non-root user whose UID/GID match the host user (avoids root-owned files in the workspace).
# Some base images ship a default 'ubuntu' user at UID 1000; remove it to free the UID.
RUN if id ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# ---------------------------------------------------------------------------
FROM base AS dev

ARG USERNAME=dev

# Dev tools only; the simulator (MuJoCo / Isaac Sim) is the visualization tool, so no RViz/desktop GUI here
RUN apt-get update && apt-get install -y --no-install-recommends \
        git curl wget vim less bash-completion iproute2 net-tools \
        build-essential cmake python3-venv python3-vcstool \
        libegl1 libopengl0 python3-pil \
    && rm -rf /var/lib/apt/lists/*

# MuJoCo quick-trial simulator (stands in for the control PC host stack; see sim/ and README "Simulator").
# Renders offscreen with EGL on the GPU and streams to a browser, so no X11/display is needed.
ARG MUJOCO_VERSION=3.14.0
ARG MENAGERIE_COMMIT=4d038b3feae26ec82b46a4d586379114012a8ac7
RUN pip3 install --no-cache-dir mujoco==${MUJOCO_VERSION} \
    && git clone --filter=blob:none --sparse https://github.com/google-deepmind/mujoco_menagerie /opt/mujoco_menagerie \
    && cd /opt/mujoco_menagerie \
    && git sparse-checkout set agilex_piper \
    && git checkout ${MENAGERIE_COMMIT} \
    && rm -rf .git \
    # The NVIDIA runtime mounts libEGL_nvidia but not its glvnd registration; without it EGL falls back to Mesa
    && mkdir -p /usr/share/glvnd/egl_vendor.d \
    && echo '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}' \
         > /usr/share/glvnd/egl_vendor.d/10_nvidia.json

# rosdep: init is done by the base image; update as the dev user
USER ${USERNAME}
RUN rosdep update --rosdistro ${ROS_DISTRO}

# Source ROS automatically in every shell (and the workspace overlay once built)
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo '[ -f "$HOME/ws/install/setup.bash" ] && source "$HOME/ws/install/setup.bash"' >> ~/.bashrc \
    && echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc

WORKDIR /home/${USERNAME}/ws
CMD ["bash"]

# ---------------------------------------------------------------------------
FROM base AS deps

# Folder name under apps/ to build
ARG APP=example

# Keep only what decides the installed dependencies, in the same layout. The dependency layers below are
# cached on this content, so a code change doesn't reinstall (or re-ship) them; only editing these files does.
COPY apps/${APP} /tmp/app
COPY src /tmp/shared
RUN mkdir -p /deps/app && cd /tmp/app \
    && find . \( -name package.xml -o -name COLCON_IGNORE -o -name AMENT_IGNORE -o -name CATKIN_IGNORE \
                 -o -path ./requirements.txt -o -path ./install_system_deps.sh \) -exec cp --parents {} /deps/app/ \; \
    && mkdir /deps/shared && cd /tmp/shared \
    && find . -name package.xml -exec cp --parents {} /deps/shared/ \;

# ---------------------------------------------------------------------------
FROM base AS app-deps

# Runs as root during image build, before compiling the app. Keep the hook self-contained.
# These installed system/Python packages are inherited by both build and runtime stages.
RUN --mount=type=bind,from=deps,source=/deps,target=/tmp/deps \
    apt-get update \
    && cd /tmp/deps/app \
    && if [ -f install_system_deps.sh ]; then bash -eo pipefail install_system_deps.sh; fi \
    && if [ -f requirements.txt ]; then pip3 install --no-cache-dir -r requirements.txt; fi \
    && rm -rf /var/lib/apt/lists/*

# Shared discovery apps provision SAM2 at image build time, never during arm motion.
ARG DISCOVERY_SAM2=false
COPY src/phyai_discovery/phyai_discovery/setup_sam2.py /opt/phyai-sam2-tools/setup_sam2.py
COPY src/phyai_discovery_ros/config/discovery.json /opt/phyai-sam2-tools/discovery.json
RUN if [ "$DISCOVERY_SAM2" = true ]; then \
      apt-get update && apt-get install -y --no-install-recommends git python3-venv libgomp1 \
      && python3 /opt/phyai-sam2-tools/setup_sam2.py --runtime-dir /opt/phyai-sam2 \
           --config /opt/phyai-sam2-tools/discovery.json --device cuda \
      && rm -rf /var/lib/apt/lists/*; \
    elif [ "$DISCOVERY_SAM2" != false ]; then echo 'DISCOVERY_SAM2 must be true or false' >&2; exit 1; fi

# ---------------------------------------------------------------------------
FROM app-deps AS build

ARG APP=example

WORKDIR /ws
RUN --mount=type=bind,from=deps,source=/deps,target=/tmp/deps \
    apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths /tmp/deps --ignore-src -y --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*

COPY apps/${APP} src/${APP}
COPY src src/shared
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# ---------------------------------------------------------------------------
FROM app-deps AS runtime

ARG APP=example
ARG USERNAME=dev

# Install only the run-time (exec) dependencies of the app's packages.
# System/pip dependencies are already inherited from app-deps.
RUN --mount=type=bind,from=deps,source=/deps,target=/tmp/deps \
    apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths /tmp/deps --ignore-src -y --rosdistro ${ROS_DISTRO} \
         --dependency-types exec \
    && rm -rf /var/lib/apt/lists/* /root/.ros/rosdep

COPY --from=build /ws/install /ws/install
COPY --chmod=755 apps/${APP}/start.sh /ws/start.sh
COPY --chmod=755 docker/runtime_entrypoint.sh /runtime_entrypoint.sh

# The host stack runs as a different UID; shared-memory DDS transport would silently drop data, so use UDP only
COPY docker/fastdds_udp.xml /etc/fastdds_udp.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_udp.xml

LABEL org.phyai.app=${APP}
USER ${USERNAME}
WORKDIR /ws
ENTRYPOINT ["/runtime_entrypoint.sh"]
CMD ["/ws/start.sh"]
