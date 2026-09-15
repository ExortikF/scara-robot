@echo off
rem Установка одной командой. Двойной щелчок по этому файлу.
rem
rem Здесь намеренно почти ничего нет: вся работа в tools\install.py.
rem Командный файл Windows плохо дружит с кириллицей и с проверкой
rem ошибок, поэтому его дело - найти питон и передать управление.
rem
rem Переходы метками, а не вложенные if: внутри блока в скобках
rem %errorlevel% подставляется ДО выполнения блока, и проверка внутри
rem него всегда видит старое значение. Классическая ловушка bat.
chcp 65001 >nul
set PYTHONUTF8=1
cd /d "%~dp0"

where py >nul 2>nul
if %errorlevel%==0 goto :через_py

where python >nul 2>nul
if %errorlevel%==0 goto :через_python

where python3 >nul 2>nul
if %errorlevel%==0 goto :через_python3

echo.
echo   Питон не найден.
echo.
echo   Если вы только что его поставили - ЗАКРОЙТЕ это окно и откройте
echo   заново. Windows подхватывает новый PATH только в новых окнах,
echo   поэтому "py" и "python" в старом окне так и останутся
echo   нераспознанными командами.
echo.
echo   Если питона нет: https://www.python.org/downloads/
echo   Нужен 64-битный, файл с пометкой "Windows installer (64-bit)".
echo   При установке отметьте "Add Python to PATH" и "tcl/tk and IDLE".
echo.
pause
exit /b 1

:через_py
py -3 tools\install.py %*
goto :конец

:через_python
python tools\install.py %*
goto :конец

:через_python3
python3 tools\install.py %*
goto :конец

:конец
echo.
pause
