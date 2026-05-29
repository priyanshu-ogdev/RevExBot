@echo off
TITLE RevExBot Production Forge - Full Pipeline
SETLOCAL EnableDelayedExpansion

:: 1. Define Paths
SET "ROOT_DIR=%~dp0"
SET "LOG_DIR=%ROOT_DIR%logs"
IF NOT EXIST "%LOG_DIR%" mkdir "%LOG_DIR%"

echo ========================================================
echo 🔥 IGNITING REVEXBOT FULL TRAINING PIPELINE
echo ========================================================

:: Phase 1: Locomotion (Foundation)
echo [1/5] Training Loco Foundation...
python revex_ext\scripts\train.py --task RevEx-Loco-v0 --headless > "%LOG_DIR%\phase1_loco.log" 2>&1
IF %ERRORLEVEL% NEQ 0 GOTO ERROR_HANDLER

:: Phase 2: Agile (Terrain Recovery)
echo [2/5] Training Agile Recovery...
python revex_ext\scripts\train.py --task RevEx-Agile-v0 --headless > "%LOG_DIR%\phase2_agile.log" 2>&1
IF %ERRORLEVEL% NEQ 0 GOTO ERROR_HANDLER

:: Phase 3: Combat (Tactical AMP)
echo [3/5] Training Combat Expert...
python revex_ext\scripts\train.py --task RevEx-Combat-v0 --headless > "%LOG_DIR%\skill_combat.log" 2>&1
IF %ERRORLEVEL% NEQ 0 GOTO ERROR_HANDLER

:: Phase 4: Dance (Rhythmic AMP)
echo [4/5] Training Dance Expert...
python revex_ext\scripts\train.py --task RevEx-Dance-v0 --headless > "%LOG_DIR%\skill_dance.log" 2>&1
IF %ERRORLEVEL% NEQ 0 GOTO ERROR_HANDLER

:: Phase 5: Precision (Force Impedance)
echo [5/5] Training Precision Expert...
python revex_ext\scripts\train.py --task RevEx-Precision-v0 --headless > "%LOG_DIR%\skill_precision.log" 2>&1
IF %ERRORLEVEL% NEQ 0 GOTO ERROR_HANDLER

echo ========================================================
echo 🎉 ALL EXPERTS FORGED SUCCESSFULLY.
echo 📂 Logs saved to: %LOG_DIR%
echo 🚀 Run 'python scripts/register_experts.py' to seal the registry.
echo ========================================================
pause
GOTO END

:ERROR_HANDLER
echo ❌ PIPELINE CRASHED AT PHASE %ERRORLEVEL%.
echo ❌ Check logs in %LOG_DIR% for stack trace.
pause
exit /b %ERRORLEVEL%

:END
ENDLOCAL