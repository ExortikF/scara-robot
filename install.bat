@echo off
rem ============================================================
rem  SCARA - installer. Double-click this file.
rem ============================================================
rem
rem  EVERYTHING HERE IS PLAIN ASCII ON PURPOSE.
rem
rem  cmd.exe reads a .bat file byte by byte while running it, and it
rem  does NOT reliably handle non-ASCII text - not even after
rem  "chcp 65001". A label named with Cyrillic letters got mangled
rem  into a command: cmd printed "'_py' is not recognized" and the
rem  installer died before it started.
rem
rem  So this file stays ASCII and does the bare minimum: find python,
rem  hand over to tools\install.py. All Russian text is printed by
rem  python, which handles UTF-8 properly.
rem ============================================================
chcp 65001 >nul
set PYTHONUTF8=1
cd /d "%~dp0"

where py >nul 2>nul
if %errorlevel%==0 goto run_py

where python >nul 2>nul
if %errorlevel%==0 goto run_python

where python3 >nul 2>nul
if %errorlevel%==0 goto run_python3

echo.
echo   Python not found.
echo.
echo   1. If you JUST installed Python - close this window and open
echo      it again. Windows only picks up the new PATH in new windows.
echo.
echo   2. If Python is not installed - get it here:
echo        https://www.python.org/downloads/
echo      Take the 64-bit installer. Do NOT take an "alpha" or "beta".
echo      Tick "Add Python to PATH" and "tcl/tk and IDLE".
echo.
pause
exit /b 1

:run_py
py -3 tools\install.py %*
goto done

:run_python
python tools\install.py %*
goto done

:run_python3
python3 tools\install.py %*
goto done

:done
echo.
pause
