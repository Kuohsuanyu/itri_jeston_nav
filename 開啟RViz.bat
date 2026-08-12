@echo off
chcp 65001 >nul
title RViz2 - FAST-LIO (WSL2)

REM 直接開 RViz,不經過控制台。
REM 腳本會先 ping Jetson、等 topic 出現,再啟動 RViz2。

REM 帶參數可指定自己的設定檔:  開啟RViz.bat my.rviz
if "%~1"=="" (
  wsl -d Ubuntu-22.04 -- bash -lc "~/lidar_view/start_rviz.sh"
) else (
  wsl -d Ubuntu-22.04 -- bash -lc "~/lidar_view/start_rviz.sh ~/lidar_view/%~1"
)

echo.
echo RViz closed.
pause
