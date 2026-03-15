#!/bin/bash
# build_mac.sh  —  builds LagerRobot.app and optional .dmg
# Usage: bash build_mac.sh

set -e

echo "==> Creating virtual environment..."
python3 -m venv venv_build
source venv_build/bin/activate

echo "==> Installing dependencies..."
pip install --upgrade pip
pip install pyqt5 pyinstaller

echo "==> Building .app bundle..."
pyinstaller LagerRobot.spec --noconfirm

echo ""
echo "✅  Done! App bundle is at: dist/LagerRobot.app"
echo "    You can move it to /Applications or share it."
echo ""

# Optional: create a .dmg for easy distribution
if command -v create-dmg &> /dev/null; then
    echo "==> Creating .dmg..."
    create-dmg \
        --volname "LagerRobot" \
        --window-size 540 380 \
        --icon-size 128 \
        --icon "LagerRobot.app" 130 180 \
        --app-drop-link 410 180 \
        "dist/LagerRobot.dmg" \
        "dist/LagerRobot.app"
    echo "✅  Disk image: dist/LagerRobot.dmg"
else
    echo "ℹ️   Tip: install create-dmg for a .dmg file:"
    echo "    brew install create-dmg"
fi

deactivate
