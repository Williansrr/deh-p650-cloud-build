@echo off
setlocal
cd /d "%~dp0"
echo ================================================
echo  DEH-P650 GITHUB CLOUD BUILD - CHECAGEM
echo ================================================
if not exist ".github\workflows\build.yml" (
  echo [ERRO] .github\workflows\build.yml nao encontrado.
  echo Extraia o ZIP inteiro antes de enviar ao GitHub.
  pause
  exit /b 1
)
if not exist "overlay\main\main.cpp" (
  echo [ERRO] overlay incompleto.
  pause
  exit /b 1
)
python tools\validate_cloud_package.py
if errorlevel 1 (
  echo [ERRO] Validacao falhou.
  pause
  exit /b 1
)
echo.
echo [OK] Pasta pronta para enviar ao GitHub.
pause
