#!/usr/bin/env bash
# 在 uv 容器裡跑 tools/topview.py(matplotlib + imageio-ffmpeg;套件快取在 out/.uv-cache,主機字型唯讀掛進去給中文用)。
#   tools/topview.sh out/run.csv [topview.py 的參數...]
# run_loop.sh 的 RECORD=1 也是叫這支;既有的 CSV 要補錄影就直接叫。
set -euo pipefail
cd "$(dirname "$0")/.."
PY_IMAGE="${PY_IMAGE:-ghcr.io/astral-sh/uv:python3.12-bookworm-slim}"
mkdir -p out/.uv-cache
FONTS=(); [ -d /usr/share/fonts ] && FONTS=(-v /usr/share/fonts:/usr/share/fonts:ro)
docker run --rm --cpus 2 --memory 3g --pids-limit 256 --log-opt max-size=10m --log-opt max-file=3 \
  --user "$(id -u):$(id -g)" -e HOME=/tmp -e UV_CACHE_DIR=/tmp/uv -e MPLCONFIGDIR=/tmp/mpl \
  -v "$PWD":/w -w /w -v "$PWD/out/.uv-cache":/tmp/uv "${FONTS[@]}" \
  "$PY_IMAGE" uv run --with matplotlib --with numpy --with imageio-ffmpeg python tools/topview.py "$@" 2>&1 \
  | grep -v "UserWarning\|fig.savefig" || true
