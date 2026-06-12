@echo off
setlocal EnableDelayedExpansion
TITLE RevEx Data Forge Launcher
color 0A

:: 1. Resolve project root (we are in pipeline/; go up one level)
FOR %%A IN ("%~dp0..") DO SET "PROJECT_ROOT=%%~fA"

echo ==================================================
echo 🤖 REVEX DATA FORGE LAUNCHER
echo ==================================================
echo 📍 Project root: %PROJECT_ROOT%
echo.

:: 2. (Optional) Activate conda environment
:: CALL conda activate isaaclab

:: 3. Verify Python
python --version >nul 2>&1
IF %ERRORLEVEL% NEQ 0 (
    echo ❌ Python not found. Activate your environment first.
    pause
    exit /b 1
)

:: 4. Step 1 – YouTube Harvester
echo [1/5] Searching & downloading videos...
python "%PROJECT_ROOT%\pipeline\scrape_youtube.py" --download
IF %ERRORLEVEL% NEQ 0 GOTO ERROR

:: 5. Step 2 – Scene Splitter
echo.
echo [2/5] Splitting videos into atomic clips...
python "%PROJECT_ROOT%\pipeline\ingest_media.py"
IF %ERRORLEVEL% NEQ 0 GOTO ERROR

:: 6. Step 3 – Kinematic Extractor
echo.
echo [3/5] Extracting 3D skeletons (YOLO + MediaPipe)...
start /B /WAIT /HIGH python "%PROJECT_ROOT%\pipeline\extract_kinematics.py"
IF %ERRORLEVEL% NEQ 0 GOTO ERROR

:: 7. Step 4 – Retargeter
echo.
echo [4/5] Retargeting to 39-DOF RevExBot...
start /B /WAIT /HIGH python "%PROJECT_ROOT%\pipeline\retarget_urdf.py"
IF %ERRORLEVEL% NEQ 0 GOTO ERROR

:: 8. Step 5 – Compiler
echo.
echo [5/5] Compiling unified motion library...
python "%PROJECT_ROOT%\pipeline\build_library.py"
IF %ERRORLEVEL% NEQ 0 GOTO ERROR

echo ==================================================
echo 🎉 Data pipeline complete!
echo 📚 Library saved to data/unified_motion_library.json
echo ==================================================
pause
exit /b 0

:ERROR
echo.
color 0C
echo ❌ Pipeline step failed with error code %ERRORLEVEL%.
echo ❌ Check the console output above for details.
pause
exit /b %ERRORLEVEL%