@echo off
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
  echo First run: follow README.md to create .venv and install requirements.
  pause
  exit /b 1
)
".venv\Scripts\python.exe" -m robot_lab
if errorlevel 1 pause
