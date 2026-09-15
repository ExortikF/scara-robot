@echo off
rem Запуск управления рукой.
rem
rem   запустить.bat              с приводами и камерой
rem   запустить.bat --simulate   без железа, посмотреть
chcp 65001 >nul
set PYTHONUTF8=1
cd /d "%~dp0"

if exist ".venv\Scripts\python.exe" goto :запуск

echo.
echo   Окружение не создано. Сначала запустите установить.bat
echo.
pause
exit /b 1

:запуск
.venv\Scripts\python.exe tools\arm_control.py %*
if errorlevel 1 pause
