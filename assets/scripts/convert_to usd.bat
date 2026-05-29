@echo off
setlocal enabledelayedexpansion

TITLE RevExBot USD Compiler
echo ========================================================
echo 🏭 REVEXBOT URDF-TO-USD COMPILER (ISAAC LAB)
echo ========================================================

:: 1. Lock the Architecture Paths
set SCRIPT_DIR=%~dp0
set PROJECT_ROOT=%SCRIPT_DIR%..

set URDF_FILE=%PROJECT_ROOT%\assets\urdf\revexbot.urdf
set USD_FILE=%PROJECT_ROOT%\assets\usd\revexbot.usd
set ISAACLAB_DIR=%PROJECT_ROOT%\IsaacLab

echo [INFO] Project Root: %PROJECT_ROOT%
echo [INFO] Target URDF: %URDF_FILE%
echo [INFO] Output USD: %USD_FILE%

:: 2. Verify Pre-requisites
if not exist "%URDF_FILE%" (
    echo.
    echo [ERROR] URDF not found at %URDF_FILE%
    echo ❌ Compile your master_assembly.xacro into a URDF first!
    pause
    exit /b 1
)

if not exist "%ISAACLAB_DIR%\isaaclab.bat" (
    echo.
    echo [ERROR] Isaac Lab not found at %ISAACLAB_DIR%
    echo ❌ Execute the clone and symlink protocol before compiling.
    pause
    exit /b 1
)

:: 3. Locate the Internal Isaac Sim Converter
set CONVERTER_SCRIPT=
set POSSIBLE_PATHS=
set "POSSIBLE_PATHS=%POSSIBLE_PATHS% _isaac_sim\exts\omni.isaac.urdf\scripts\convert_urdf.py"
set "POSSIBLE_PATHS=%POSSIBLE_PATHS% _isaac_sim\scripts\tools\convert_urdf.py"
set "POSSIBLE_PATHS=%POSSIBLE_PATHS% source\extensions\omni.importer.urdf\omni\importer\urdf\scripts\convert_urdf.py"

for %%P in (%POSSIBLE_PATHS%) do (
    if exist "%ISAACLAB_DIR%\%%P" (
        set CONVERTER_SCRIPT=%ISAACLAB_DIR%\%%P
        goto :found_converter
    )
)

echo [ERROR] Could not find the Isaac Sim convert_urdf.py script.
echo ❌ Ensure the _isaac_sim symlink was created correctly.
pause
exit /b 1

:found_converter
echo [INFO] Found Engine Converter: %CONVERTER_SCRIPT%

:: 4. Execute via Isaac Lab Python Wrapper
echo.
echo ⚡ Igniting Isaac Lab USD Converter...
call "%ISAACLAB_DIR%\isaaclab.bat" -p "%CONVERTER_SCRIPT%" ^
    "%URDF_FILE%" ^
    "%USD_FILE%" ^
    --merge-joints ^
    --make-instanceable

if %errorlevel% equ 0 (
    echo.
    echo ✅ [SUCCESS] USD Master File Forged: %USD_FILE%
) else (
    echo.
    echo ❌ [FATAL] Conversion failed. Check the traceback above.
)

pause