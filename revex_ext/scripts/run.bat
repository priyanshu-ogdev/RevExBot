@echo off
TITLE RevExBot ASE Training Pipeline
SETLOCAL EnableDelayedExpansion

:: 1. Resolve paths
FOR %%A IN ("%~dp0..") DO SET "PROJECT_ROOT=%%~fA"
SET "LOG_DIR=%PROJECT_ROOT%\logs"
IF NOT EXIST "%LOG_DIR%" mkdir "%LOG_DIR%"

:: 2. (Optional) Activate your conda environment
:: CALL conda activate isaaclab

:: 3. Verify Python
python --version >nul 2>&1
IF %ERRORLEVEL% NEQ 0 (
    echo ❌ Python not found. Activate your environment first.
    pause
    exit /b 1
)

:: 4. Parse arguments
IF "%1"=="" GOTO USAGE
IF /I "%1"=="1" GOTO PHASE1
IF /I "%1"=="2" GOTO PHASE2
GOTO USAGE

:PHASE1
echo ========================================================
echo 🔥 PHASE 1: Base Locomotion Training (8192 envs)
echo ========================================================
echo 📂 Console output also written to: %LOG_DIR%\phase1_loco.log
echo.

:: 🚨 HIGH PRIORITY – prevents background processes from starving the physics engine
start /B /WAIT /HIGH python "%PROJECT_ROOT%\scripts\train.py" --phase 1 --headless > "%LOG_DIR%\phase1_loco.log" 2>&1

IF %ERRORLEVEL% NEQ 0 GOTO ERROR_HANDLER
echo ✅ Phase 1 completed. Checkpoint saved.
GOTO END

:PHASE2
echo ========================================================
echo 🔥 PHASE 2: ASE Style Training (4096 envs)
echo ========================================================
IF "%2"=="" (
    echo ⚠️ No checkpoint provided. Starting Phase 2 from scratch.
    SET "CKPT_ARG="
) ELSE (
    echo 📦 Loading Phase 1 checkpoint: %2
    SET "CKPT_ARG=--checkpoint %2"
)
echo 📂 Console output also written to: %LOG_DIR%\phase2_ase.log
echo.

start /B /WAIT /HIGH python "%PROJECT_ROOT%\scripts\train.py" --phase 2 --headless %CKPT_ARG% > "%LOG_DIR%\phase2_ase.log" 2>&1

IF %ERRORLEVEL% NEQ 0 GOTO ERROR_HANDLER
echo ✅ Phase 2 completed. Checkpoint saved.
GOTO END

:USAGE
echo ========================================================
echo RevExBot ASE Training Pipeline
echo ========================================================
echo Usage: run.bat [phase] [checkpoint_path]
echo.
echo   Phase 1:  run.bat 1
echo   Phase 2:  run.bat 2 "checkpoint_phase1_iter15000.pt"
echo ========================================================
pause
exit /b 1

:ERROR_HANDLER
echo.
echo ❌ FATAL: Training crashed with error code %ERRORLEVEL%.
echo ❌ Check the log file in %LOG_DIR% for the full stack trace.
pause
exit /b %ERRORLEVEL%

:END
ENDLOCAL