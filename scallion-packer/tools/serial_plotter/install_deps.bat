@echo off
chcp 65001 >nul
cd /d "%~dp0"
set MIRROR=https://mirrors.aliyun.com/pypi/simple/
set HOST=mirrors.aliyun.com
set PIPOPTS=-r requirements.txt -i %MIRROR% --trusted-host %HOST% --default-timeout=600

echo 使用阿里云镜像安装（超时 600 秒，仅 Essentials 无 Addons 大包）...
python -m pip install %PIPOPTS%
if errorlevel 1 (
  echo.
  echo 阿里云失败时可换清华源重试：
  echo py -3 -m pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple --default-timeout=600
  pause
  exit /b 1
)
echo 完成。运行: python main.py
pause
