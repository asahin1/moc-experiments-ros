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

: "${CONTAINER_NAME:?CONTAINER_NAME is required in .env}"

if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
  docker exec -it $CONTAINER_NAME bash
else
  echo "Container with name $CONTAINER_NAME is not running" >&2
  exit 1
fi