@echo off
TITLE RevExBot 3-Expert Forge (Loco, Agile, Precision)
SETLOCAL EnableDelayedExpansion

:: 1. Absolute Path Resolution
FOR %%A IN ("%~dp0..\..") DO SET "PROJECT_ROOT=%%~fA"
SET "LOG_DIR=%PROJECT_ROOT%\logs"
IF NOT EXIST "%LOG_DIR%" mkdir "%LOG_DIR%"

echo ========================================================
echo 🔥 IGNITING REVEXBOT 3-EXPERT FORGE
echo ========================================================
echo 📍 Project Root: %PROJECT_ROOT%
echo 📂 Logging to: %LOG_DIR%

:: Anchor the execution context
pushd "%PROJECT_ROOT%"

:: Phase 1: Locomotion (Foundation)
echo.
echo [1/4] Forging Loco Foundation (Target: 1500 Epochs)...
python revex_ext\scripts\train.py --task RevEx-Loco-v0 --headless > "%LOG_DIR%\phase1_loco.log" 2>&1
IF %ERRORLEVEL% NEQ 0 GOTO ERROR_HANDLER

:: Phase 2: Agile (Terrain Recovery)
echo [2/4] Forging Agile Recovery (Target: 2500 Epochs)...
python revex_ext\scripts\train.py --task RevEx-Agile-v0 --headless > "%LOG_DIR%\phase2_agile.log" 2>&1
IF %ERRORLEVEL% NEQ 0 GOTO ERROR_HANDLER

:: Phase 3: Precision (Force Impedance & Soft Grasp)
echo [3/4] Forging Precision Expert (Target: 2500 Epochs)...
python revex_ext\scripts\train.py --task RevEx-Precision-v0 --headless > "%LOG_DIR%\skill_precision.log" 2>&1
IF %ERRORLEVEL% NEQ 0 GOTO ERROR_HANDLER

:: Phase 4: Automatic Registration
echo.
echo ========================================================
echo 📦 [4/4] SEALING EXPERT REGISTRY
echo ========================================================
python revex_ext\scripts\register_experts.py
IF %ERRORLEVEL% NEQ 0 GOTO ERROR_HANDLER

echo ========================================================
echo 🎉 3-EXPERT FORGE COMPLETE. 
echo 🚀 Ready for Router Distillation: python revex_ext\scripts\train_moe_skrl.py
echo ========================================================
popd
pause
exit /b 0

:ERROR_HANDLER
echo.
echo ❌ FATAL: PIPELINE CRASHED AT PHASE %ERRORLEVEL%.
echo ❌ Check the trace files in %LOG_DIR% for the Python stack trace.
popd
pause
exit /b %ERRORLEVEL%