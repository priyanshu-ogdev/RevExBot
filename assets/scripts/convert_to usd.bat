@echo off
setlocal enabledelayedexpansion

TITLE RevExBot USD Compiler
echo ========================================================
echo 🏭 REVEXBOT URDF-TO-USD COMPILER (CONDA EDITION)
echo ========================================================

:: 1. Absolute Path Resolution
:: %~dp0 gets the directory of the script (assumes it's in scripts/)
FOR %%A IN ("%~dp0..\..") DO SET "PROJECT_ROOT=%%~fA"

:: 🚨 Set the specific folder containing your URDF
SET "URDF_DIR=%PROJECT_ROOT%\assets\urdf"
SET "URDF_FILE=%URDF_DIR%\revexbot1.urdf"

:: Set the output folder
SET "USD_DIR=%PROJECT_ROOT%\assets\usd"
SET "USD_FILE=%USD_DIR%\revexbot1.usd"

:: Ensure output directory exists
if not exist "%USD_DIR%" mkdir "%USD_DIR%"

:: Isaac Lab internal symlink path
SET "CONVERTER_SCRIPT=%PROJECT_ROOT%\IsaacLab\_isaac_sim\scripts\tools\convert_urdf.py"

echo 📍 Project Root: %PROJECT_ROOT%
echo 📁 URDF Directory: %URDF_DIR%
echo 📥 Target URDF: %URDF_FILE%
echo 📤 Output USD: %USD_FILE%

:: Fallback Check
IF NOT EXIST "%CONVERTER_SCRIPT%" (
    echo ⚠️ [WARNING] convert_urdf.py not found at primary path. Attempting fallback...
    SET "CONVERTER_SCRIPT=%PROJECT_ROOT%\IsaacLab\_isaac_sim\scripts\utils\import_robot.py"
)

:: 2. Execute via Active Conda Environment
echo.
echo ⚡ Igniting Isaac Lab USD Converter...

:: 🚨 FIX: Change directory to the URDF folder so relative mesh paths (../meshes/) resolve!
pushd "%URDF_DIR%"

python "%CONVERTER_SCRIPT%" ^
    "%URDF_FILE%" ^
    "%USD_FILE%" ^
    --make-instanceable

:: 🚨 Removed --merge-joints to protect your fixed end-effector links

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