@echo off
call "D:\VisualStudio\Packages\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 -winsdk=10.0.26100.0 >nul
if errorlevel 1 exit /b %errorlevel%
if not exist "%~dp0build" mkdir "%~dp0build"
pushd "%~dp0build"
cl /nologo /std:c++20 /EHsc /utf-8 /W3 /MD /O2 /I"D:\sunmmer_camp\computerGraphics\OceanRenderEngine\src" "%~dp0InspectLibrary.cpp" /Fe:InspectLibrary.exe /link /INCREMENTAL:NO
if errorlevel 1 exit /b %errorlevel%
if "%~1"=="build-only" exit /b 0
InspectLibrary.exe "%~dp0raw\ecostress_raw" "%~dp0reader-audit.csv"
exit /b %errorlevel%
