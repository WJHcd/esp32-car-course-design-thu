@echo off
rem ============================================================
rem  task2 flash: flash built firmware to ESP32-S3
rem  usage: flash.bat [COM port]   (default COM8, CP210x USB-UART)
rem  example: flash.bat COM5
rem ============================================================

set PORT=%~1
if "%PORT%"=="" set PORT=COM8

set IDF_PYTHON=C:\Espressif\tools\python\v5.4.4\venv\Scripts\python.exe

if not exist "%~dp0build\flash_args" goto :nobuild
if not exist "%IDF_PYTHON%" goto :nopython

echo ====== flashing to %PORT% ======
pushd "%~dp0build"
"%IDF_PYTHON%" -m esptool --chip esp32s3 -p %PORT% -b 460800 --before no_reset --after hard_reset write_flash @flash_args
set FLASH_RC=%ERRORLEVEL%
popd
if not "%FLASH_RC%"=="0" goto :err

echo.
echo ************ Flash OK ************
goto :eof

:nobuild
echo.
echo ************ Flash FAILED: build\flash_args not found; run build-fast.bat first ************
exit /b 1

:nopython
echo.
echo ************ Flash FAILED: ESP-IDF Python not found at %IDF_PYTHON% ************
exit /b 1

:err
echo.
echo ************ Flash FAILED: hold BOOT, press RESET once, then retry ************
exit /b 1
