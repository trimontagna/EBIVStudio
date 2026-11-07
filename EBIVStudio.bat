@echo off
rem Starts EBIV Studio with the OpenEB 5.2 runtime environment (DLLs, HAL plugins).
rem Optional arguments: a recording (.raw, .hdf5, .dat, .evt) and / or an EBIV parameter file (.cfg).
setlocal
call "%~dp0..\openeb_env.bat" || exit /b 1
cd /d "%~dp0"
start "" "%~dp0build\Release\EBIVStudio.exe" %*
