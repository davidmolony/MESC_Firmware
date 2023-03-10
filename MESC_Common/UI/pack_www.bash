#!/bin/bash

# Copyright 2023 cod3b453
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

WWWDIR="$(builtin cd "$(dirname "${BASH_SOURCE[@]}")" && pwd -P)/www"

echo "INFO: WWWDIR is '${WWWDIR}'"

URLOUT="${WWWDIR}/url.cpp"
WWWOUT="${WWWDIR}/www.cpp"

> "${URLOUT}"
> "${WWWOUT}"

pushd "${WWWDIR}" || return 1

while read -r wwwfil
do
    srcfil="${wwwfil:2}"
    echo "INFO: Processing '${srcfil}'"
    cppfil="${srcfil}.cpp"
    xxd -include "${srcfil}" | sed 's:^unsigned char :static char const :g; s:^unsigned int .*$::g; s:^};$:, 0x00\n};:' > "${cppfil}"
    printf '#include "%s"\n' "${cppfil}" >> "${WWWOUT}"
    srcmime='text/plain'
    if [[ "${srcfil}" =~ css$ ]]
    then
        srcmime='text/css'
    elif [[ "${srcfil}" =~ html$ ]]
    then
        srcmime='text/html'
    elif [[ "${srcfil}" =~ js$ ]]
    then
        srcmime='text/javascript'
    fi
    lblnam=$(echo "${srcfil}" | tr '.' '_')
    printf '    MESC_WWW_ENTRY( "/%s", "%s", %s ),\n' "${srcfil}" "${srcmime}" "${lblnam}" >> "${URLOUT}"
done < <(find . -type f -name '*.css' -o -name '*.html' -o -name '*.js')

popd

