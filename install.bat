@echo off
rem ============================================================
rem  SCARA - full installer for Windows. Double-click this file.
rem
rem  Does everything, from Python to a working program:
rem    1. finds Python 3.12, or downloads and installs it itself
rem       (for this user only - no administrator rights needed);
rem    2. hands over to tools\install.py: libraries, camera,
rem       recognition models, self-test, desktop shortcuts.
rem
rem  Options (pass them after the name in a console window):
rem    install.bat --voice       also voice commands (+90 MB)
rem    install.bat --big-voice   voice with the big model (+1.9 GB)
rem
rem  EVERYTHING HERE IS PLAIN ASCII ON PURPOSE.
rem  cmd.exe reads a .bat file byte by byte while running it and does
rem  NOT reliably handle non-ASCII text, even after "chcp 65001": a
rem  Cyrillic label became "'_py' is not recognized" and the installer
rem  died. All Russian text is printed by python instead.
rem ============================================================
chcp 65001 >nul
set PYTHONUTF8=1
cd /d "%~dp0"

set "PYVER=3.12.10"
set "PYURL=https://www.python.org/ftp/python/%PYVER%/python-%PYVER%-amd64.exe"
set "PYDIR=%LOCALAPPDATA%\Programs\Python\Python312"
set "PYEXE=%PYDIR%\python.exe"
set "PYSETUP=%TEMP%\python-%PYVER%-amd64.exe"
set "PY="

echo ==============================================================
echo   SCARA installer
echo ==============================================================
echo.
echo [0] Looking for Python 3.12 ...

rem --- 1. Python 3.12 that this installer put here earlier ----------
if not exist "%PYEXE%" goto try_launcher
"%PYEXE%" tools\python_ok.py
if errorlevel 1 goto try_launcher
set "PY=%PYEXE%"
goto have_python

rem --- 2. Python 3.12 the user installed themselves -----------------
:try_launcher
where py >nul 2>nul
if errorlevel 1 goto install_python
py -3.12 tools\python_ok.py >nul 2>nul
if errorlevel 1 goto install_python
for /f "delims=" %%i in ('py -3.12 tools\python_ok.py --where') do set "PY=%%i"
if not defined PY goto install_python
goto have_python

rem --- 3. None suitable: download and install it --------------------
:install_python
if /i "%PROCESSOR_ARCHITECTURE%"=="x86" if not defined PROCESSOR_ARCHITEW6432 goto win32

echo     not found - downloading Python %PYVER% (about 25 MB) ...
curl.exe -L --fail --silent --show-error -o "%PYSETUP%" "%PYURL%"
if errorlevel 1 goto download_ps
goto run_setup

:download_ps
echo     curl did not work, trying PowerShell ...
powershell -NoProfile -ExecutionPolicy Bypass -Command "[Net.ServicePointManager]::SecurityProtocol='Tls12'; Invoke-WebRequest -UseBasicParsing -Uri '%PYURL%' -OutFile '%PYSETUP%'"
if errorlevel 1 goto no_download

:run_setup
echo     installing Python %PYVER% for this user, 1-2 minutes ...
echo     (no window will appear - this is normal)
start "" /wait "%PYSETUP%" /quiet InstallAllUsers=0 TargetDir="%PYDIR%" PrependPath=1 Include_tcltk=1 Include_pip=1 Include_launcher=1 InstallLauncherAllUsers=0 Include_test=0 Include_doc=0 Shortcuts=0
del "%PYSETUP%" >nul 2>nul
if not exist "%PYEXE%" goto setup_failed
"%PYEXE%" tools\python_ok.py
if errorlevel 1 goto setup_failed
set "PY=%PYEXE%"

:have_python
echo     using: %PY%
echo.
"%PY%" tools\install.py %*
goto done

rem --- errors --------------------------------------------------------
:win32
echo.
echo   This is 32-bit Windows. The libraries this program needs
echo   (numpy, mediapipe, opencv) do not exist for 32-bit Windows.
echo   A 64-bit Windows 10 or 11 is required.
goto done

:no_download
echo.
echo   Could not download Python. Check the internet connection
echo   and run install.bat again.
echo   Or install it by hand:  %PYURL%
echo   (tick "Add Python to PATH"), then run install.bat again.
goto done

:setup_failed
echo.
echo   Python installer finished, but Python 3.12 did not appear in
echo     %PYDIR%
echo   Most often an older broken Python 3.12 is in the way:
echo   Settings - Apps - remove every "Python 3.12", then run
echo   install.bat again.
goto done

:done
echo.
pause
