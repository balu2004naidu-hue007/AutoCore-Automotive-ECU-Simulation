@echo off
setlocal
cd /d "%~dp0"
echo ==========================================================
echo AutoCore v1.6.1 - Automatic Setup, Build, Test and Dashboard
echo ==========================================================
echo Missing CMake, Visual C++ Build Tools and Python will be installed automatically with winget.
echo An internet connection is required for first-time setup. Windows may display an installer/UAC prompt.
echo.
powershell -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\build_and_test.ps1"
set "EXITCODE=%ERRORLEVEL%"
if not "%EXITCODE%"=="0" (
  echo.
  echo AutoCore setup or launch failed. Read the error messages above.
  pause
)
endlocal & exit /b %EXITCODE%
