@echo off
setlocal
cd /d "%~dp0"
title XPBD Baker - Build

if not exist "mvnw.cmd" (
    echo ERROR: mvnw.cmd was not found.
    pause
    exit /b 1
)

set "DISCOVERED_JAVA_HOME="
for /f "usebackq delims=" %%J in (`powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0distribution\find-java.ps1" -Purpose Build`) do if not defined DISCOVERED_JAVA_HOME set "DISCOVERED_JAVA_HOME=%%J"
if not defined DISCOVERED_JAVA_HOME (
    echo ERROR: No compatible 64-bit JDK 17+ installation was found.
    echo A build requires both bin\java.exe and bin\javac.exe.
    if not defined XPBD_NO_PAUSE pause
    exit /b 1
)
set "JAVA_HOME=%DISCOVERED_JAVA_HOME%"
echo Using JDK: %JAVA_HOME%

echo Building and assembling the JavaFX release folder...
call mvnw.cmd clean package
if errorlevel 1 (
    echo.
    echo BUILD FAILED
    if not defined XPBD_NO_PAUSE pause
    exit /b 1
)

echo.
echo BUILD SUCCESS
echo Release folder: target\release
echo Double-click target\release\run.bat to start.
if not defined XPBD_NO_PAUSE pause
endlocal
