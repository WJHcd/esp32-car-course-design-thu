@echo off
rem ============================================================
rem  task1 one-click build script (Windows CMD)
rem  Usage: run this .bat, it will set-target esp32s3 then build.
rem  Edit the IDF paths below if your install differs.
rem
rem  NOTE: 本环境为 EIM (VS Code ESP-IDF 扩展管理器) 安装，
rem        工具链位于 C:\Espressif\tools\<tool>\<version>，
rem        且没有 tools.json，故标准 export.bat 会失败。
rem        这里改为手动把工具链目录加入 PATH，不依赖 export。
rem ============================================================

set IDF_PATH=C:\esp\v5.4.4\esp-idf
set IDF_TOOLS_PATH=C:\Espressif\tools
set IDF_PYTHON_ENV_PATH=C:\Espressif\tools\python\v5.4.4\venv
set IDF_PYTHON_CHECK_CONSTRAINTS=0
rem clear MSYSTEM in case of MSYS environment pollution
set MSYSTEM=

rem ---- 手动加入工具链到 PATH（按需增删） ----
set PATH=C:\Espressif\tools\cmake\3.30.2\bin;^
C:\Espressif\tools\ninja\1.12.1;^
C:\Espressif\tools\xtensa-esp-elf\esp-14.2.0_20260121\xtensa-esp-elf\bin;^
C:\Espressif\tools\riscv32-esp-elf\esp-14.2.0_20260121\riscv32-esp-elf\bin;^
C:\Espressif\tools\python\v5.4.4\venv\Scripts;%PATH%

cd /d %~dp0

echo.
echo ====== set target esp32s3 ======
python "%IDF_PATH%\tools\idf.py" set-target esp32s3
if errorlevel 1 goto :err

echo.
echo ====== building ======
python "%IDF_PATH%\tools\idf.py" build
if errorlevel 1 goto :err

echo.
echo ************ Build OK ************
goto :eof

:err
echo.
echo ************ Build FAILED ************
exit /b 1
