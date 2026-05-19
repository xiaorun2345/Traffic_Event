#!/usr/bin/env bash
set -e

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
VERSION="${MEDIAMTX_VERSION:-v1.18.2}"

case "$(uname -m)" in
  aarch64|arm64)
    PLATFORM="linux_arm64"
    ;;
  x86_64|amd64)
    PLATFORM="linux_amd64"
    ;;
  *)
    echo "Unsupported architecture: $(uname -m)" >&2
    exit 1
    ;;
esac

ARCHIVE="mediamtx_${VERSION}_${PLATFORM}.tar.gz"
URL="https://github.com/bluenviron/mediamtx/releases/download/${VERSION}/${ARCHIVE}"
TARGET_DIR="${ROOT}/traffic_platform/mediamtx"
TMP_FILE="/tmp/${ARCHIVE}"

mkdir -p "${TARGET_DIR}"
if command -v wget >/dev/null 2>&1; then
  wget -4 -O "${TMP_FILE}" "${URL}"
else
  curl -4 -L --fail -o "${TMP_FILE}" "${URL}"
fi
tar -xzf "${TMP_FILE}" -C "${TARGET_DIR}" mediamtx
chmod +x "${TARGET_DIR}/mediamtx"
"${TARGET_DIR}/mediamtx" --version
