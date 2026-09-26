#!/bin/sh
# =============================================================================
#  PX4_ulog_plottools - one-time Python environment setup (Linux)
# =============================================================================
#  Creates the project-local environment <project_root>/.venv with the uv
#  binary that ships with the project (tools/uv/linux/uv).
#
#  Nothing has to be installed beforehand: uv is bundled with the repository,
#  and the Python interpreter it needs is downloaded into the project as well.
#  PATH is never modified and no global package is installed, so deleting the
#  project folder also removes the entire Python environment.
#
#  Usage (first run only, from the project root):
#      ./scripts/setup_env.sh
#      sh scripts/setup_env.sh      # if the executable bit is not set
# =============================================================================

set -eu

# This script lives in <project_root>/scripts/, so the root is one level up.
script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
project_root=$(dirname -- "$script_dir")

case "$(uname -s)" in
    Linux) ;;
    Darwin)
        echo "This is macOS - use ./scripts/setup_env_macos.sh instead." >&2
        exit 1
        ;;
    *)
        echo "Unsupported platform: $(uname -s)" >&2
        exit 1
        ;;
esac

# Only the x86_64 build is bundled; report the others clearly instead of
# failing with a confusing "not found".
case "$(uname -m)" in
    x86_64 | amd64) ;;
    *)
        echo "No bundled uv for Linux/$(uname -m) - only x86_64 is provided." >&2
        echo "Install uv from https://docs.astral.sh/uv/ and run 'uv sync' in $project_root." >&2
        exit 1
        ;;
esac

uv_bin="$project_root/tools/uv/linux/uv"

if [ ! -f "$uv_bin" ]; then
    echo "Bundled uv not found: $uv_bin" >&2
    echo "The repository looks incomplete (tools/uv/linux/uv is missing)." >&2
    exit 1
fi
chmod +x "$uv_bin" 2>/dev/null || true

echo "PX4_ulog_plottools - Python environment setup"
echo "Project root: $project_root"

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
