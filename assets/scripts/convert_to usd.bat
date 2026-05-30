@echo off
setlocal enabledelayedexpansion

TITLE RevExBot USD Compiler
echo ========================================================
echo 🏭 REVEXBOT URDF-TO-USD COMPILER (CONDA EDITION)
echo ========================================================

:: 1. Absolute Path Resolution
FOR %%A IN ("%~dp0..\..") DO SET "PROJECT_ROOT=%%~fA"

:: 🚨 Assumes your assets are stored here based on your previous config.
:: Adjust these paths if they are nested inside revex_ext.
SET "URDF_FILE=%PROJECT_ROOT%\assets\urdf\revexbot.urdf"
SET "USD_FILE=%PROJECT_ROOT%\assets\usd\revexbot.usd"

:: Isaac Lab internal symlink path
SET "CONVERTER_SCRIPT=%PROJECT_ROOT%\IsaacLab\_isaac_sim\scripts\tools\convert_urdf.py"

echo 📍 Project Root: %PROJECT_ROOT%
echo 📥 Target URDF: %URDF_FILE%
echo 📤 Output USD: %USD_FILE%

:: Fallback Check: Isaac Sim 4.0+ occasionally moves this to 'utils/import_robot.py'
IF NOT EXIST "%CONVERTER_SCRIPT%" (
    echo ⚠️ [WARNING] convert_urdf.py not found at primary path. Attempting fallback...
    SET "CONVERTER_SCRIPT=%PROJECT_ROOT%\IsaacLab\_isaac_sim\scripts\utils\import_robot.py"
)

:: 2. Execute via Active Conda Environment
echo.
echo ⚡ Igniting Isaac Lab USD Converter...

:: Anchor context to the root so Isaac Lab pathing rules apply
pushd "%PROJECT_ROOT%"

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
    echo ❌ [FATAL] Conversion failed. 
    echo 🔧 Ensure your Conda environment (e.g., 'isaaclab') is activated.
)

popd
pause