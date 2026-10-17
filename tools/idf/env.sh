# Shared ESP-IDF configuration for this project. Sourced (not executed) by
# tools/idf/setup.sh, the ./idf wrapper and .envrc.
#
# The toolchain lives entirely inside the project in .idf/ (git-ignored), so
# nothing is installed globally and nothing in the user's shell config changes.
# Bump IDF_VERSION here to move the project to another ESP-IDF release, then
# re-run tools/idf/setup.sh.

IDF_VERSION="v5.5.1"
# Only the chips this project actually targets (M5Stack = esp32,
# Waveshare = esp32s3). Keeps the download small.
IDF_TARGETS="esp32,esp32s3"

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/../.." && pwd)"
IDF_HOME="$PROJECT_ROOT/.idf"

export IDF_PATH="$IDF_HOME/esp-idf"
export IDF_TOOLS_PATH="$IDF_HOME/tools"

# ESP-IDF picks the Python virtualenv by the version of whatever "python3" is
# first in PATH. setup.sh pins the interpreter it installed with as a symlink
# in .idf/bin so activation always finds the matching venv, regardless of
# which Python the system or Homebrew provides later.
IDF_BIN="$IDF_HOME/bin"
case ":$PATH:" in
    *":$IDF_BIN:"*) ;;
    *) export PATH="$IDF_BIN:$PATH" ;;
esac

idf_is_installed() {
    [ -x "$IDF_BIN/python3" ] &&
    [ -f "$IDF_PATH/export.sh" ] &&
    [ "$(cat "$IDF_HOME/.installed-version" 2>/dev/null)" = "$IDF_VERSION" ]
}
