#!/usr/bin/env bash
# End-to-end smoke test against a running server (local uvicorn or docker compose).
#   usage: ./scripts/smoke_test.sh [URL]   (default http://localhost:8000)
set -euo pipefail
URL=${1:-http://localhost:8000}

echo "[1/3] waiting for /health ..."
for i in $(seq 1 60); do
  if curl -sf "${URL}/health" >/dev/null; then break; fi
  sleep 5
done
curl -sf "${URL}/health"; echo

echo "[2/3] /info"
curl -sf "${URL}/info"; echo

echo "[3/3] /predict (synthetic frames via client_example.py)"
python "$(dirname "$0")/../examples/client_example.py" --url "${URL}" --steps 2

echo "OK"
