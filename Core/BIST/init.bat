@ECHO OFF

IF NOT EXIST build (
  mkdir build
)
cd build

"C:\Program Files\CMake\bin\cmake.exe" -G"Visual Studio 16 2019" ..

IF %ERRORLEVEL% EQU 0 (
  start "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\IDE\devenv.exe" BIST.sln
) ELSE (
  pause
)
