@echo off
chcp 65001 >nul
title QBOT 模型檢視 - RViz2 (WSL2,不連 Jetson)

REM 純本機看 URDF 模型和 TF 樹,車子不用開機。
REM 第一次跑、或改過 robot_tf.sh 之後,先執行「更新模型.bat」。

wsl -d Ubuntu-22.04 -- bash -lc "~/lidar_view/view_model.sh"

echo.
pause
