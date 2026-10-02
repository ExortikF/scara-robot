@echo off
rem ============================================================
rem  SCARA - start the control window.
rem
rem    run.bat              with drives and camera
rem    run.bat --simulate   no hardware, just look at it
rem
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
.venv\Scripts\python.exe tools\arm_control.py %*
if errorlevel 1 pause
