@echo off
chcp 65001 >nul
title 更新 QBOT 模型 (robot_tf.sh -> URDF)

REM 改過 jetson_deploy\scripts\robot_tf.sh 的座標之後跑這支。
REM   1. gen_view_urdf.py  讀 robot_tf.sh -> 產生 wsl\qbot_view.xacro
REM   2. 把 wsl\ 底下的腳本同步進 WSL 的 ~/lidar_view
REM   3. setup_model.sh    xacro 展開成 ~/lidar_view/qbot_view.urdf

cd /d "%~dp0"

echo === 1/3 從 robot_tf.sh 產生 xacro ===
python wsl\gen_view_urdf.py || goto :err

echo.
echo === 2/3 同步腳本到 WSL ===
wsl -d Ubuntu-22.04 -- bash -c "cd '/mnt/c/Users/ag133/Desktop/工作資料/程式/wheeled-robot-lidar-nav/wsl' && mkdir -p ~/lidar_view && cp *.xml *.rviz *.sh ~/lidar_view/ && sed -i 's/\r$//' ~/lidar_view/*.sh && chmod +x ~/lidar_view/*.sh && echo OK" || goto :err

echo.
echo === 3/3 展開 URDF ===
wsl -d Ubuntu-22.04 -- bash -lc "~/lidar_view/setup_model.sh" || goto :err

echo.
echo 完成。可以執行「檢視模型.bat」
pause
exit /b 0

:err
echo.
echo *** 失敗 ***
pause
exit /b 1
