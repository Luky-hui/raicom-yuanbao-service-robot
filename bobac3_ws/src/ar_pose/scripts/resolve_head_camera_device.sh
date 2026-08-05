#!/usr/bin/env bash
set -euo pipefail

is_capture_device() {
  local device="$1"
  [ -e "$device" ] || return 1
  if command -v v4l2-ctl >/dev/null 2>&1; then
    v4l2-ctl -d "$device" --all 2>/dev/null | awk '
      /^[[:space:]]*Device Caps/ { in_device_caps = 1; next }
      in_device_caps && /^[[:space:]]*Video Capture/ { found = 1 }
      in_device_caps && /^[^[:space:]]/ { in_device_caps = 0 }
      END { exit found ? 0 : 1 }
    '
    return $?
  fi
  return 0
}

if [ -n "${HEAD_CAMERA_DEVICE:-}" ] && is_capture_device "$HEAD_CAMERA_DEVICE"; then
  printf '%s' "$HEAD_CAMERA_DEVICE"
  exit 0
fi

if is_capture_device /dev/head_camera; then
  printf '%s' /dev/head_camera
  exit 0
fi

for device in /dev/video*; do
  [ -e "$device" ] || continue
  if is_capture_device "$device"; then
    printf '%s' "$device"
    exit 0
  fi
done

printf '%s' "${HEAD_CAMERA_DEVICE:-/dev/video0}"
