#!/usr/bin/env bash
set -e
VENV="$HOME/swarm_venv"
mkdir -p ~/pip_tmp ~/pip_cache

echo "Installing packages..."
TMPDIR=~/pip_tmp "$VENV/bin/pip" install --cache-dir ~/pip_cache \
    numpy opencv-python-headless pillow ncnn pyserial digi-xbee -q

echo "Verifying imports..."
"$VENV/bin/python3" << 'PYEOF'
results = []
for mod, label in [
    ("cv2",       "opencv"),
    ("ncnn",      "ncnn"),
    ("numpy",     "numpy"),
    ("PIL",       "pillow"),
    ("serial",    "pyserial"),
    ("digi.xbee", "digi-xbee"),
]:
    try:
        m = __import__(mod)
        ver = getattr(m, "__version__", "ok")
        print(f"  [PASS] {label}: {ver}")
    except ImportError as e:
        print(f"  [FAIL] {label}: {e}")
        results.append("FAIL")

if "FAIL" not in results:
    print("\nALL PACKAGES OK - Phase 0 complete!")
else:
    print("\nSome packages missing!")
    raise SystemExit(1)
PYEOF
