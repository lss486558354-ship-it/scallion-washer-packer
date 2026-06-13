@echo off
chcp 65001 >nul 2>&1
echo Rename step 1: inner folder...
cd /d "C:\Users\yolov8\Desktop\新建文件夹 (2)\大葱切割-清洗-打包一体机_freeRTOS\大葱切割-清洗-打包一体机"
ren "大葱切割-清洗-打包一体机" "1"
echo Result1: %errorlevel%
echo.
echo Rename step 2: middle folder...
cd /d "C:\Users\yolov8\Desktop\新建文件夹 (2)"
ren "大葱切割-清洗-打包一体机_freeRTOS" "1"
echo Result2: %errorlevel%
echo.
echo Rename step 3: outer folder...
cd /d "C:\Users\yolov8\Desktop"
ren "新建文件夹 (2)" "freertos_machine"
echo Result3: %errorlevel%
echo DONE - please restart Cursor IDE
pause