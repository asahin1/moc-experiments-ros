#!/usr/bin/env bash
set -euo pipefail

if [ -f ".env" ]; then
  set -a
  . ".env"
  set +a
else
  echo "Error: .env not found" >&2
  exit 1
fi

: "${IMAGE_TAG:?IMAGE_TAG is required in .env}"
: "${WORKSPACE_NAME:?WORKSPACE_NAME is required in .env}"

docker build --build-arg WORKSPACE_NAME=$WORKSPACE_NAME -t "$IMAGE_TAG" .