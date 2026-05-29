@echo off
setlocal enabledelayedexpansion

TITLE RevExBot USD Compiler
echo ========================================================
echo 🏭 REVEXBOT URDF-TO-USD COMPILER (CONDA EDITION)
echo ========================================================

:: 1. Lock the Architecture Paths
set SCRIPT_DIR=%~dp0
set PROJECT_ROOT=%SCRIPT_DIR%..

set URDF_FILE=%PROJECT_ROOT%\assets\urdf\revexbot.urdf
set USD_FILE=%PROJECT_ROOT%\assets\usd\revexbot.usd
set CONVERTER_SCRIPT=%PROJECT_ROOT%\IsaacLab\_isaac_sim\scripts\tools\convert_urdf.py

echo [INFO] Target URDF: %URDF_FILE%
echo [INFO] Output USD: %USD_FILE%

:: 2. Execute via Active Conda Environment
echo ⚡ Igniting Isaac Lab USD Converter...
python "%CONVERTER_SCRIPT%" ^
    "%URDF_FILE%" ^
    "%USD_FILE%" ^
    --merge-joints ^
    --make-instanceable

if %errorlevel% equ 0 (
    echo.
    echo ✅ [SUCCESS] USD Master File Forged: %USD_FILE%
) else (
    echo.
    echo ❌ [FATAL] Conversion failed. Is your 'env_isaaclab' Conda environment activated?
)

pause