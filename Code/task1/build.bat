@echo off
rem ============================================================
rem  task1 one-click build script (Windows CMD)
rem  Usage: run this .bat, it will set-target esp32s3 then build.
rem  Edit the IDF paths below if your install differs.
rem
rem  NOTE: IDF_TOOLS_PATH must point to the PARENT of the "tools"
rem        directory (C:\Espressif), i.e. where <tool>\<version>
rem        folders live, otherwise idf_tools.py cannot find them.
rem ============================================================

set IDF_PATH=C:\esp\v5.4.4\esp-idf
set IDF_TOOLS_PATH=C:\Espressif
set IDF_PYTHON_ENV_PATH=C:\Espressif\tools\python\v5.4.4\venv
set IDF_PYTHON_CHECK_CONSTRAINTS=0
rem clear MSYSTEM in case of MSYS environment pollution
set MSYSTEM=

call %IDF_PATH%\export.bat
if errorlevel 1 goto :err

cd /d %~dp0

echo.
echo ====== set target esp32s3 ======
idf.py set-target esp32s3
if errorlevel 1 goto :err

echo.
echo ====== building ======
idf.py build
if errorlevel 1 goto :err

echo.
echo ************ Build OK ************
goto :eof

:err
echo.
echo ************ Build FAILED ************
exit /b 1
