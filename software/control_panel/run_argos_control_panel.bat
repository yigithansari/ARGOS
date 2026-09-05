@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "SPYDER_PY=C:\Users\yigit\AppData\Local\spyder-6\envs\spyder-runtime\python.exe"

if exist "%SPYDER_PY%" (
  "%SPYDER_PY%" "%SCRIPT_DIR%argos_control_panel.py"
  goto :end
)

where python >nul 2>nul
if %ERRORLEVEL% EQU 0 (
  python "%SCRIPT_DIR%argos_control_panel.py"
  goto :end
)

echo Python bulunamadi. Spyder runtime veya normal Python kurulu olmali.
pause

:end
endlocal
