@echo off
REM Build script for Neural Network Generator - Windows

setlocal enabledelayexpand

echo Building Neural Network Generator...

REM Get script directory
set SCRIPT_DIR=%~dp0
set SRC_DIR=%SCRIPT_DIR%src
set BACKEND_GENERATOR_DIR=%SCRIPT_DIR%backend\generator

if not exist "%SRC_DIR%" (
    echo Error: src directory not found at %SRC_DIR%
    exit /b 1
)

echo Compiling C++ generator from %SRC_DIR%...
cd /d "%SRC_DIR%"

REM Run make to build the generator
REM Note: Requires MinGW or similar with g++ and make installed
make clean >nul 2>&1
make generator

REM Check if compilation was successful
if not exist "gen.exe" (
    if not exist "gen" (
        echo Error: Compilation failed. 'gen.exe' or 'gen' executable not found.
        exit /b 1
    )
)

echo. Compilation successful

REM Copy to backend/generator as 'main.exe'
echo Copying executable to %BACKEND_GENERATOR_DIR%...
if exist "gen.exe" (
    copy /Y gen.exe "%BACKEND_GENERATOR_DIR%\main.exe"
) else (
    copy /Y gen "%BACKEND_GENERATOR_DIR%\main.exe"
)

echo. Executable ready at %BACKEND_GENERATOR_DIR%\main.exe

echo.
echo Build complete! Ready for development.
endlocal
