@echo off
rem ============================================================
rem  task2 incremental build (skips set-target so that the
rem  sdkconfig generated from sdkconfig.defaults is preserved).
rem  Only use build.bat (with set-target) when changing target
rem  or when sdkconfig must be regenerated from defaults.
rem ============================================================

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

echo ====== building (incremental) ======
python "%IDF_PATH%\tools\idf.py" build
if errorlevel 1 goto :err

echo.
echo ************ Build OK ************
goto :eof

:err
echo.
echo ************ Build FAILED ************
exit /b 1
