@echo off
"%~dp0\bin\premake5.exe" --file="%~dp0\premake5.lua" vs2019 %*
pause
