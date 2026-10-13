#!/usr/bin/env bash
# Install bri onto PATH by symlinking it into /usr/local/bin.
# Idempotent: re-running just refreshes the symlink. Called from the dev
# container's postCreateCommand so users don't run it manually.

set -euo pipefail

BRI_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRI="$BRI_DIR/bri"

chmod +x "$BRI" "$BRI_DIR"/commands/*.sh

TARGET="/usr/local/bin/bri"
if [[ -w "$(dirname "$TARGET")" ]]; then
    ln -sf "$BRI" "$TARGET"
else
    sudo ln -sf "$BRI" "$TARGET"
fi

echo "Installed: $TARGET -> $BRI"
