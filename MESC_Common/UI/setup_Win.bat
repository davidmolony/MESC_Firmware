@ECHO OFF

@REM Copyright 2023 cod3b453
@REM
@REM Redistribution and use in source and binary forms, with or without
@REM modification, are permitted provided that the following conditions are met:
@REM
@REM 1. Redistributions of source code must retain the above copyright notice, this
@REM    list of conditions and the following disclaimer.
@REM
@REM 2. Redistributions in binary form must reproduce the above copyright notice,
@REM    this list of conditions and the following disclaimer in the documentation
@REM    and/or other materials provided with the distribution.
@REM
@REM 3. Neither the name of the copyright holder nor the names of its contributors
@REM    may be used to endorse or promote products derived from this software
@REM    without specific prior written permission.
@REM
@REM THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
@REM AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
@REM IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
@REM DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
@REM FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
@REM DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
@REM SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
@REM CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
@REM OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
@REM OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

SET "SELFDIR=%~dp0"

SET "LNKNAME=www"
SET "LNKDIR=Arduino\WebServer"
SET "LNKPATH=%SELFDIR%\%LNKDIR%\%LNKNAME%"

@REM mkdir -p "${LNKPATH}"
IF NOT EXIST "%LNKPATH%" (
    mkdir "%LNKPATH%"
)

cp "%SELFDIR%\%LNKNAME%\*.*" "%LNKPATH%"
