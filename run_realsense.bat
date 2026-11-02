@echo off
rem ============================================================
rem  SCARA - start with the drives and the RealSense camera.
rem  Same as run.bat --realsense --confirm.
rem  Plain ASCII on purpose: cmd.exe mangles non-ASCII in .bat files.
rem ============================================================
chcp 65001 >nul
set PYTHONUTF8=1
cd /d "%~dp0"

if exist ".venv\Scripts\python.exe" goto start
echo.
echo   Not installed yet. Run install.bat first.
echo.
pause
exit /b 1

:start
.venv\Scripts\python.exe tools\arm_control.py --realsense --confirm %*
if errorlevel 1 pause
