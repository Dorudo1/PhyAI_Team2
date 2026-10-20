#!/bin/bash
# Install the optional GPU segmenter locally without changing system ROS packages.
set -euo pipefail
APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_DIR="$APP_DIR/.models"
python3 -m venv --system-site-packages "$APP_DIR/.venv-sam2"
"$APP_DIR/.venv-sam2/bin/pip" install 'numpy<2' torch==2.5.1 torchvision==0.20.1 --index-url https://download.pytorch.org/whl/cu121
SAM2_BUILD_CUDA=0 "$APP_DIR/.venv-sam2/bin/pip" install --no-build-isolation 'numpy>=1.24,<2' 'hydra-core==1.3.2' 'SAM-2 @ git+https://github.com/facebookresearch/sam2.git@2b90b9f5ceec907a1c18123530e92e794ad901a4'
mkdir -p "$MODEL_DIR"
CHECKPOINT="$MODEL_DIR/sam2.1_hiera_tiny.pt"
if [ ! -s "$CHECKPOINT" ]; then
    curl --fail --location --retry 3 https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_tiny.pt -o "$CHECKPOINT.partial"
    mv "$CHECKPOINT.partial" "$CHECKPOINT"
fi
echo "SAM 2 ready. Run $APP_DIR/run_sim.sh"
