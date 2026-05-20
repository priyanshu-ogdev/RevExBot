@echo off
setlocal enabledelayedexpansion

:: ============================================================
:: 1. AUTO-DETECT ISAAC LAB INSTALLATION
:: ============================================================
set ISAACLAB_DIR=
set FOUND_CONVERTER=

:: Check environment variable
if defined ISAACLAB_PATH (
    set ISAACLAB_DIR=%ISAACLAB_PATH%
    goto :check_converter
)

:: Check common installation directories
set CANDIDATES=
set "CANDIDATES=%CANDIDATES% C:\Users\%USERNAME%\IsaacLab"
set "CANDIDATES=%CANDIDATES% C:\Program Files\NVIDIA Corporation\IsaacLab"
set "CANDIDATES=%CANDIDATES% C:\IsaacLab"
set "CANDIDATES=%CANDIDATES% D:\IsaacLab"

for %%D in (%CANDIDATES%) do (
    if exist "%%D\_isaac_sim\python.bat" (
        set ISAACLAB_DIR=%%D
        goto :check_converter
    )
    if exist "%%D\python.bat" (
        set ISAACLAB_DIR=%%D
        goto :check_converter
    )
)

:: Ask user if not found
echo [ERROR] Could not locate Isaac Lab automatically.
echo Please enter the full path to your Isaac Lab installation (e.g., C:\IsaacLab):
set /p ISAACLAB_DIR=Path: 
if not exist "!ISAACLAB_DIR!" (
    echo [ERROR] Path does not exist.
    pause
    exit /b 1
)

:check_converter
echo [INFO] Isaac Lab found at: %ISAACLAB_DIR%

:: ============================================================
:: 2. LOCATE CONVERTER SCRIPT (multiple possible paths)
:: ============================================================
set CONVERTER_SCRIPT=

:: List of possible relative paths to convert_urdf.py (newest to oldest)
set POSSIBLE_PATHS=
set "POSSIBLE_PATHS=%POSSIBLE_PATHS% scripts\tools\convert_urdf.py"
set "POSSIBLE_PATHS=%POSSIBLE_PATHS% source\extensions\omni.importer.urdf\omni\importer\urdf\scripts\convert_urdf.py"
set "POSSIBLE_PATHS=%POSSIBLE_PATHS% tools\convert_urdf.py"
set "POSSIBLE_PATHS=%POSSIBLE_PATHS% _isaac_sim\scripts\tools\convert_urdf.py"

for %%P in (%POSSIBLE_PATHS%) do (
    if exist "%ISAACLAB_DIR%\%%P" (
        set CONVERTER_SCRIPT=%ISAACLAB_DIR%\%%P
        goto :found_converter
    )
)

:: If not found, ask user
echo [ERROR] Could not find convert_urdf.py in standard Isaac Lab locations.
echo Please enter the full path to convert_urdf.py (e.g., C:\IsaacLab\scripts\tools\convert_urdf.py):
set /p CONVERTER_SCRIPT=Path: 
if not exist "!CONVERTER_SCRIPT!" (
    echo [ERROR] File not found.
    pause
    exit /b 1
)

:found_converter
echo [INFO] Converter script: %CONVERTER_SCRIPT%

:: ============================================================
:: 3. DETERMINE PATHS RELATIVE TO THIS SCRIPT
:: ============================================================
set SCRIPT_DIR=%~dp0
set PROJECT_ROOT=%SCRIPT_DIR%..
set URDF_FILE=%PROJECT_ROOT%\urdf\revexbot.urdf
set USD_FILE=%PROJECT_ROOT%\usd\revexbot.usd

echo [INFO] URDF: %URDF_FILE%
echo [INFO] USD output: %USD_FILE%

if not exist "%URDF_FILE%" (
    echo [ERROR] URDF not found at %URDF_FILE%
    echo Please generate the URDF first (run compile_robot.py or xacro).
    pause
    exit /b 1
)

:: ============================================================
:: 4. LOCATE PYTHON EXECUTABLE INSIDE ISAAC LAB
:: ============================================================
set PYTHON_EXE=
if exist "%ISAACLAB_DIR%\_isaac_sim\python.bat" (
    set PYTHON_EXE=%ISAACLAB_DIR%\_isaac_sim\python.bat
) else if exist "%ISAACLAB_DIR%\python.bat" (
    set PYTHON_EXE=%ISAACLAB_DIR%\python.bat
) else if exist "%ISAACLAB_DIR%\python\python.exe" (
    set PYTHON_EXE=%ISAACLAB_DIR%\python\python.exe
) else (
    set PYTHON_EXE=python
    echo [WARNING] Using system Python; ensure Isaac Lab modules are in PYTHONPATH.
)

echo [INFO] Using Python: %PYTHON_EXE%

:: ============================================================
:: 5. RUN CONVERTER
:: ============================================================
%PYTHON_EXE% "%CONVERTER_SCRIPT%" ^
    "%URDF_FILE%" ^
    "%USD_FILE%" ^
    --merge-joints ^
    --make-instanceable

if %errorlevel% equ 0 (
    echo [SUCCESS] USD file created: %USD_FILE%
) else (
    echo [ERROR] Conversion failed. Check the output above.
)

pause