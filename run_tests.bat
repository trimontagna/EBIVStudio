@echo off
rem Runs the EBIV unit tests in the OpenEB environment. Extra arguments go to the test binary
rem (e.g. run_tests.bat --gtest_filter=MotionComp.*).
setlocal
call "%~dp0..\openeb_env.bat" || exit /b 1
"%~dp0build\Release\ebiv_tests.exe" %*
