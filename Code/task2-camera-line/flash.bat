@echo off
rem ============================================================
rem  task2 flash: flash built firmware to ESP32-S3
rem  usage: flash.bat [COM port]   (default COM8, CP210x USB-UART)
rem  example: flash.bat COM5
rem ============================================================

set PORT=%~1
if "%PORT%"=="" set PORT=COM8

set IDF_PATH=C:\esp\v5.4.4\esp-idf
set IDF_TOOLS_PATH=C:\Espressif\tools
set IDF_PYTHON_ENV_PATH=C:\Espressif\tools\python\v5.4.4\venv
set IDF_PYTHON_CHECK_CONSTRAINTS=0
set MSYSTEM=

set PATH=C:\Espressif\tools\cmake\3.30.2\bin;^
C:\Espressif\tools\ninja\1.12.1;^
C:\Espressif\tools\xtensa-esp-elf\esp-14.2.0_20260121\xtensa-esp-elf\bin;^
C:\Espressif\tools\riscv32-esp-elf\esp-14.2.0_20260121\riscv32-esp-elf\bin;^
C:\Espressif\tools\python\v5.4.4\venv\Scripts;%PATH%

cd /d %~dp0

echo ====== flashing to %PORT% ======
python "%IDF_PATH%\tools\idf.py" -p %PORT% flash
if errorlevel 1 goto :err

echo.
echo ************ Flash OK ************
goto :eof

:err
echo.
echo ************ Flash FAILED: hold BOOT, press RESET once, then retry ************
exit /b 1
