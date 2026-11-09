#!/usr/bin/env bash
set -euo pipefail
ROOT="$(python3 -c 'import os,sys; print(os.path.dirname(os.path.dirname(os.path.realpath(sys.argv[1]))))' "${BASH_SOURCE[0]}")"
source "$ROOT/scripts/find_shared_mac.sh"
SHARED="$(find_rviz_shared)"
source "$SHARED/scripts/common_mac.sh"
BASE_IMAGE="$IMAGE_TAG"
EGO_IMAGE="${EGO_RVIZ_IMAGE:-ego-swarm-rviz:noetic-arm64-web}"
build_args=(-f "$ROOT/Dockerfile.macos" --platform linux/arm64)
check_docker
check_image
docker build "${build_args[@]}" --build-arg "BASE_IMAGE=$BASE_IMAGE" -t "$EGO_IMAGE" "$ROOT"
