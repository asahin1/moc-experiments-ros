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
: "${CONTAINER_NAME:?CONTAINER_NAME is required in .env}"
: "${WORKSPACE_NAME:?WORKSPACE_NAME is required in .env}"

if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
  echo "Container with name $CONTAINER_NAME is already running" >&2
  exit 1
fi

docker run -it --rm \
  --name $CONTAINER_NAME \
  --network=host \
  -e DISPLAY=$DISPLAY \
  -v "$PWD/$WORKSPACE_NAME:/home/ubuntu/work/$WORKSPACE_NAME" \
  -v "$PWD/runtime:/home/ubuntu/work/runtime" \
  -v "$PWD/resources:/home/ubuntu/work/resources" \
  $IMAGE_TAG