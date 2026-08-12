@echo off
REM URDF Sensor Studio - 在任意 URDF 上擺放感測器並匯出
REM 雙擊即可,或在 cmd 裡執行:  run.cmd  [urdf路徑]
cd /d "%~dp0"
python studio.py %*
if errorlevel 1 pause
