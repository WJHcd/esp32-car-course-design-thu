@echo off
rem ============================================================
rem  voice-control-task flash: flash built firmware to ESP32-S3
rem  usage: flash.bat [COM port]   (default COM8, CP210x USB-UART)
rem  example: flash.bat COM5
rem  NOTE: 烧录含两部分：应用固件 voice-control-task.bin
rem        + 语音模型 srmodels.bin（自动写入 model 分区）
rem ============================================================

set PORT=%~1
if "%PORT%"=="" set PORT=COM8

set IDF_PYTHON=C:\Espressif\tools\python\v5.4.4\venv\Scripts\python.exe

if not exist "%~dp0build\flash_args" goto :nobuild
if not exist "%IDF_PYTHON%" goto :nopython

echo ====== flashing to %PORT% ======
pushd "%~dp0build"
"%IDF_PYTHON%" -m esptool --chip esp32s3 -p %PORT% -b 460800 --before default_reset --after hard_reset write_flash @flash_args
set FLASH_RC=%ERRORLEVEL%
popd
if %FLASH_RC%==0 goto :ok
goto :err

:ok
echo.
echo ************ Flash OK ************
goto :eof

:nobuild
echo ERROR: build\flash_args not found. Run build.bat first.
exit /b 1

:nopython
echo ERROR: python not found at %IDF_PYTHON%
exit /b 1

:err
echo.
echo ************ Flash FAILED ************
exit /b 1
