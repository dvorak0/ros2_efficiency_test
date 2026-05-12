#!/usr/bin/env bash
set -euo pipefail

# Always resolve relative to this script's folder
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"
DIR_NAME_RAW="$(basename "${PROJECT_DIR}")"
DIR_NAME="$(echo "${DIR_NAME_RAW}" | tr '[:upper:]' '[:lower:]')"  # for image tags

IMAGE_NAME="${IMAGE_NAME:-${DIR_NAME}-dev}"
DOCKERFILE="${DOCKERFILE:-${SCRIPT_DIR}/Dockerfile}"

echo "Building ${IMAGE_NAME} with:"
echo "  DIR_NAME=${DIR_NAME_RAW}"

if [ -f "${PROJECT_DIR}/.gitmodules" ]; then
  echo "Updating git submodules for Docker build context ..."
  git -C "${PROJECT_DIR}" submodule update --init --recursive
fi

docker build -t "${IMAGE_NAME}" \
  --build-arg DIR_NAME="${DIR_NAME_RAW}" \
  -f "${DOCKERFILE}" "${PROJECT_DIR}"

