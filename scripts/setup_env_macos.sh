#!/bin/sh
# =============================================================================
#  PX4_ulog_plottools - one-time Python environment setup (macOS)
# =============================================================================
#  Creates the project-local environment <project_root>/.venv with the uv
#  binary that ships with the project (tools/uv/macos/).
#
#  Both Apple Silicon and Intel binaries are bundled; the architecture is
#  detected here and the matching one is used.
#
#  Nothing has to be installed beforehand: uv is bundled with the repository,
#  and the Python interpreter it needs is downloaded into the project as well.
#  PATH is never modified and no global package is installed, so deleting the
#  project folder also removes the entire Python environment.
#
#  Usage (first run only, from the project root):
#      ./scripts/setup_env_macos.sh
#      sh scripts/setup_env_macos.sh   # if the executable bit is not set
# =============================================================================

set -eu

# This script lives in <project_root>/scripts/, so the root is one level up.
script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
project_root=$(dirname -- "$script_dir")

if [ "$(uname -s)" != "Darwin" ]; then
    echo "This is not macOS - use ./scripts/setup_env.sh on Linux." >&2
    exit 1
fi

# --- Architecture detection: arm64 (Apple Silicon) vs x86_64 (Intel) --------
case "$(uname -m)" in
    arm64 | aarch64)
        uv_bin="$project_root/tools/uv/macos/uv-aarch64-apple-darwin"
        arch_label="Apple Silicon (arm64)"
        ;;
    x86_64 | amd64)
        uv_bin="$project_root/tools/uv/macos/uv-x86_64-apple-darwin"
        arch_label="Intel (x86_64)"
        ;;
    *)
        echo "Unsupported macOS architecture: $(uname -m)" >&2
        exit 1
        ;;
esac

if [ ! -f "$uv_bin" ]; then
    echo "Bundled uv not found: $uv_bin" >&2
    echo "The repository looks incomplete (tools/uv/macos/ is missing a binary)." >&2
    exit 1
fi
chmod +x "$uv_bin" 2>/dev/null || true

# Gatekeeper quarantines binaries that arrive via a downloaded archive; clear
# the flag for this one file so it can run. This does not change any system
# setting and is a no-op for a git clone.
if xattr -p com.apple.quarantine "$uv_bin" >/dev/null 2>&1; then
    xattr -d com.apple.quarantine "$uv_bin" 2>/dev/null || true
fi

echo "PX4_ulog_plottools - Python environment setup"
echo "Project root: $project_root"
echo "Detected architecture: $arch_label"

# Keep the interpreter inside the project rather than in the user profile, so
# that removing the project removes the whole environment.
UV_PYTHON_INSTALL_DIR="$project_root/.uv-python"
export UV_PYTHON_INSTALL_DIR
# Never reuse a Python already installed on this machine: a managed interpreter
# is downloaded into the project instead, keeping .venv independent of the host.
UV_PYTHON_PREFERENCE=only-managed
export UV_PYTHON_PREFERENCE

cd "$project_root"

echo ""
echo "==> Using the bundled uv"
"$uv_bin" --version

echo ""
echo "==> Creating the project-local environment (uv sync)"
"$uv_bin" sync
echo "    [OK] pyulog installed into ./.venv (no global packages were touched)"

echo ""
echo "==> Verifying ulog2csv"
"$uv_bin" run ulog2csv --help >/dev/null

ulog2csv="$project_root/.venv/bin/ulog2csv"
if [ ! -f "$ulog2csv" ]; then
    echo "ulog2csv was not found at $ulog2csv" >&2
    exit 1
fi
echo "    [OK] ulog2csv is ready: $ulog2csv"

echo ""
echo "Environment setup completed."
echo ""
echo "Next: open MATLAB, run 'load_data_main' (or 'plot_setpoint_response')."
