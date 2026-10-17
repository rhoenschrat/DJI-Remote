#!/usr/bin/env bash
# Install the project-local ESP-IDF toolchain into .idf/.
#
# Usage: tools/idf/setup.sh [--force]
#
# Safe to re-run: does nothing if the version from tools/idf/env.sh is already
# installed. --force wipes .idf/ and installs from scratch. A version change in
# env.sh triggers a clean reinstall automatically.
#
# Requirements: macOS or Linux, git, Python >= 3.10 (on macOS:
# "brew install python@3.12").

set -euo pipefail

source "$(dirname "$0")/env.sh"

FORCE=0
[ "${1:-}" = "--force" ] && FORCE=1

if [ "$FORCE" -eq 0 ] && idf_is_installed; then
    echo "ESP-IDF $IDF_VERSION already installed in $IDF_HOME"
    exit 0
fi

# ---------------------------------------------------------------------------
# Prerequisites
# ---------------------------------------------------------------------------
command -v git >/dev/null || { echo "ERROR: git not found." >&2; exit 1; }

find_python() {
    local candidate
    for candidate in python3.13 python3.12 python3.11 python3.10 python3; do
        if command -v "$candidate" >/dev/null &&
           "$candidate" -c 'import sys; sys.exit(sys.version_info < (3, 10))' 2>/dev/null; then
            command -v "$candidate"
            return 0
        fi
    done
    return 1
}

# Skip our own .idf/bin so a stale pin from a previous install is not reused.
PYTHON="$(PATH="${PATH//$IDF_BIN:/}" find_python)" || {
    echo "ERROR: Python >= 3.10 not found." >&2
    echo "       macOS: brew install python@3.12" >&2
    echo "       Linux: install python3.10+ and python3-venv via your package manager" >&2
    exit 1
}
echo "Using $PYTHON ($("$PYTHON" --version))"

# ---------------------------------------------------------------------------
# Fresh install
# ---------------------------------------------------------------------------
rm -rf "$IDF_HOME"
mkdir -p "$IDF_BIN"
ln -s "$PYTHON" "$IDF_BIN/python3"
ln -s "$PYTHON" "$IDF_BIN/python"

echo "Cloning ESP-IDF $IDF_VERSION ..."
git clone --branch "$IDF_VERSION" --depth 1 \
    --recursive --shallow-submodules --jobs 8 \
    https://github.com/espressif/esp-idf.git "$IDF_PATH"

echo "Installing tools for $IDF_TARGETS ..."
"$IDF_PATH/install.sh" "$IDF_TARGETS"
# On macOS/Linux ESP-IDF expects cmake and ninja from the system. Install
# them into .idf/ as well so the project does not depend on Homebrew/apt.
python3 "$IDF_PATH/tools/idf_tools.py" install cmake ninja
# Downloaded archives are only needed during installation (~500 MB).
rm -rf "$IDF_TOOLS_PATH/dist"

echo "$IDF_VERSION" > "$IDF_HOME/.installed-version"

cat <<EOF

ESP-IDF $IDF_VERSION installed in $IDF_HOME

Build with:   ./idf build
With direnv:  direnv allow   (then idf.py is available inside this folder)
EOF
