@echo off
setlocal

cd /d "%~dp0"

rem Override any of these by setting the variable before running the script.
if not defined QT_DIR set "QT_DIR=C:\Qt\6.11.0\mingw_64"
if not defined CMAKE_EXE set "CMAKE_EXE=C:\Qt\Tools\CMake_64\bin\cmake.exe"
if not defined NINJA_EXE set "NINJA_EXE=C:\Qt\Tools\Ninja\ninja.exe"
if not defined MINGW_DIR set "MINGW_DIR=C:\Qt\Tools\mingw1310_64"

set "QTFRAMEWORK_BYPASS_LICENSE_CHECK=1"
set "PATH=%MINGW_DIR%\bin;%QT_DIR%\bin;C:\Qt\Tools\Ninja;%PATH%"

if not exist "%CMAKE_EXE%" (
    echo Khong tim thay CMake: %CMAKE_EXE%
    pause
    exit /b 1
)

if not exist "%NINJA_EXE%" (
    echo Khong tim thay Ninja: %NINJA_EXE%
    pause
    exit /b 1
)

if not exist "%QT_DIR%\bin\Qt6Core.dll" (
    echo Khong tim thay Qt tai: %QT_DIR%
    pause
    exit /b 1
)

echo Dang configure C++ project...
"%CMAKE_EXE%" -S . -B build -G Ninja ^
    -DCMAKE_PREFIX_PATH="%QT_DIR%" ^
    -DCMAKE_MAKE_PROGRAM="%NINJA_EXE%" ^
    -DCMAKE_C_COMPILER="%MINGW_DIR%\bin\gcc.exe" ^
    -DCMAKE_CXX_COMPILER="%MINGW_DIR%\bin\g++.exe"

if errorlevel 1 (
    echo Configure that bai.
    pause
    exit /b 1
)

echo Dang build main.cpp...
"%CMAKE_EXE%" --build build

if errorlevel 1 (
    echo Build that bai.
    pause
    exit /b 1
)

echo Dang chay test parser MAVLink/MSP...
build\test_protocol.exe
if errorlevel 1 (
    echo Test that bai.
    pause
    exit /b 1
)

echo Dang chay DroneAutomationCpp...
build\DroneAutomationCpp.exe

endlocal
