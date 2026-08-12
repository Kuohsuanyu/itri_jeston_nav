@echo off
REM ASCII only on purpose - see the note in the header of chassis_check.py.
chcp 65001 >nul
title Chassis Test UI
cd /d "%~dp0"

set PLINK="C:\Program Files\PuTTY\plink.exe"
set HK=SHA256:ph8AvnetrS39dH8fnsuW8FPp9tsIg3wGecSXe/egfI4
set TGT=andykuo@192.168.40.98

echo Starting chassis_console on the Jetson (idempotent)...
%PLINK% -ssh -batch -hostkey %HK% -pw 2919 %TGT% "~/chassis_test/start.sh"

echo.
echo Opening http://192.168.40.98:8091/
start "" "http://192.168.40.98:8091/"

echo.
echo The UI keeps running on the Jetson after this window closes.
echo To stop it:  plink ... "pkill -f chassis_console.py"
echo.
pause
