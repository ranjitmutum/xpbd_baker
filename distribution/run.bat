@echo off
setlocal
cd /d "%~dp0"
title XPBD Bone Baker

set "DISCOVERED_JAVA_HOME="
for /f "usebackq delims=" %%J in (`powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0find-java.ps1" -Purpose Runtime`) do if not defined DISCOVERED_JAVA_HOME set "DISCOVERED_JAVA_HOME=%%J"

if not defined DISCOVERED_JAVA_HOME (
    echo ERROR: No compatible Java installation was found.
    echo Install a 64-bit Java 17 or newer runtime, then try again.
    if not defined XPBD_NO_PAUSE pause
    exit /b 1
)

set "JAVA_HOME=%DISCOVERED_JAVA_HOME%"
set "JAVA_CMD=%JAVA_HOME%\bin\java.exe"
echo Using Java home: %JAVA_HOME%

set "MAIN_JAR="
set "MULTIPLE_MAIN_JARS="
for /f "delims=" %%J in ('dir /b /a-d "xpbd-baker-*.jar" 2^>nul') do call :selectMainJar "%%J"
if not defined MAIN_JAR (
    echo ERROR: The xpbd-baker main JAR is missing.
    if not defined XPBD_NO_PAUSE pause
    exit /b 1
)
if defined MULTIPLE_MAIN_JARS (
    echo ERROR: More than one xpbd-baker main JAR was found.
    echo Keep exactly one versioned main JAR in this folder.
    if not defined XPBD_NO_PAUSE pause
    exit /b 1
)
set "MISSING_LIBRARY="
for %%L in (
    "gson-2.10.1.jar"
    "gdx-1.14.1.jar"
    "gdx-jnigen-loader-2.5.2.jar"
    "gdx-bullet-1.14.1.jar"
    "gdx-bullet-platform-1.14.1-natives-desktop.jar"
    "javafx-base-21.0.2.jar"
    "javafx-base-21.0.2-win.jar"
    "javafx-controls-21.0.2.jar"
    "javafx-controls-21.0.2-win.jar"
    "javafx-graphics-21.0.2.jar"
    "javafx-graphics-21.0.2-win.jar"
) do if not exist "lib\%%~L" if not defined MISSING_LIBRARY set "MISSING_LIBRARY=%%~L"
if defined MISSING_LIBRARY (
    echo ERROR: The lib folder is incomplete; missing %MISSING_LIBRARY%.
    if not defined XPBD_NO_PAUSE pause
    exit /b 1
)

"%JAVA_CMD%" --module-path "lib" --add-modules javafx.controls,javafx.graphics -cp "%MAIN_JAR%;lib\*" xpbd.MainApp
set "APP_EXIT=%ERRORLEVEL%"
if not "%APP_EXIT%"=="0" (
    echo.
    echo The application exited with error code %APP_EXIT%.
    if not defined XPBD_NO_PAUSE pause
)
endlocal & exit /b %APP_EXIT%

:selectMainJar
if defined MAIN_JAR (
    set "MULTIPLE_MAIN_JARS=1"
) else (
    set "MAIN_JAR=%~1"
)
exit /b 0
