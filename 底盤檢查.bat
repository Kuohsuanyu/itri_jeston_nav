@echo off
REM ASCII only on purpose.
REM CMD parses each line before "chcp 65001" takes effect, so UTF-8 Chinese
REM inside a .bat gets split into garbage bytes and the tail of the line ends
REM up being executed as a command. All Chinese text lives in the .py instead.
chcp 65001 >nul
title Chassis Check - chassis_driver
cd /d "%~dp0"

set PY=C:\Python313\python.exe
if not exist "%PY%" set PY=python

"%PY%" chassis_check.py

echo.
pause
