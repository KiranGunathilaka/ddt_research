#!/usr/bin/env bash
# desc: remove build/, install/, log/ from the workspace

set -euo pipefail

cd "$EUFS_MASTER"
rm -rf build install log
echo "Cleaned: build/ install/ log/ under $EUFS_MASTER"
