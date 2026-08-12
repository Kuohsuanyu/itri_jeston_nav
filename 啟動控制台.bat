@echo off
chcp 65001 >nul
title Jetson LiDAR Demo Console
cd /d "%~dp0"

set PY=C:\Python313\python.exe
if not exist "%PY%" set PY=python

"%PY%" console.py

echo.
echo Console stopped.
pause
