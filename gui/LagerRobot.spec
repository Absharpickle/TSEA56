# -*- mode: python ; coding: utf-8 -*-
# LagerRobot.spec
# Run with: pyinstaller LagerRobot.spec

block_cipher = None

a = Analysis(
    ['gui.py'],
    pathex=[],
    binaries=[],
    datas=[],
    hiddenimports=[
        'PyQt5.QtCore',
        'PyQt5.QtGui',
        'PyQt5.QtWidgets',
        'PyQt5.sip',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=['tkinter', 'matplotlib', 'numpy', 'pandas'],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='LagerRobot',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,          # No terminal window
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,       # Auto-detect arm64 / x86_64
    codesign_identity=None,
    entitlements_file=None,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='LagerRobot',
)

app = BUNDLE(
    coll,
    name='LagerRobot.app',
    icon=None,              # Replace with 'icon.icns' if you have one
    bundle_identifier='se.liu.tsea56.lagerrobot',
    version='1.0.0',
    info_plist={
        'NSHighResolutionCapable': True,
        'LSMinimumSystemVersion': '11.0',
        'CFBundleShortVersionString': '1.0.0',
        'NSRequiresAquaSystemAppearance': False,  # Supports dark mode
    },
)
