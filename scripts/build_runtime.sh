#!/bin/bash
# Build the runtime image for one app under apps/.
#   scripts/build_runtime.sh <app>        ->  phyai/app-<app>:latest
set -euo pipefail

APP="${1:?usage: $0 <app-name under apps/>}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

[ -d "$ROOT/apps/$APP" ]          || { echo "apps/$APP not found" >&2; exit 1; }
[ -f "$ROOT/apps/$APP/start.sh" ] || { echo "apps/$APP/start.sh missing" >&2; exit 1; }

# Discovery consumers get the advertised SAM2 backend automatically. Query-only
# apps depending on phyai_discovery_interfaces do not need the model.
if [ -z "${DISCOVERY_SAM2:-}" ]; then
    DISCOVERY_SAM2=false
    while IFS= read -r manifest; do
        if grep -Eq '<(exec_depend|depend)>phyai_discovery(_ros)?</' "$manifest"; then
            DISCOVERY_SAM2=true; break
        fi
    done < <(find "$ROOT/apps/$APP" \( -name ".venv*" -o -name .models -o -name build -o -name install \) -prune -o -name package.xml -print)
fi
[[ "$DISCOVERY_SAM2" == true || "$DISCOVERY_SAM2" == false ]] || {
    echo 'DISCOVERY_SAM2 must be true or false' >&2; exit 1;
}

IMAGE="phyai/app-${APP}:latest"
docker build -f "$ROOT/docker/Dockerfile" --target runtime \
    --build-arg APP="$APP" \
    --build-arg DISCOVERY_SAM2="$DISCOVERY_SAM2" \
    -t "$IMAGE" "$ROOT"
echo "Built $IMAGE"
