/*
* Copyright 2021 cod3b453
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "util_ntc.h"

static NTCNode const ntc_T_R_table[] =
{
    { - 23.0f, 91222.0f },
    { - 20.0f, 77523.0f },
    { - 15.0f, 59606.0f },
    { - 10.0f, 46290.0f },
    { -  5.0f, 36290.0f },
    {    0.0f, 28704.0f },
    {    5.0f, 22897.0f },
    {   10.0f, 18410.0f },
    {   15.0f, 14916.0f },
    {   20.0f, 12171.0f },
    {   25.0f, 10000.0f },
    {   30.0f,  8269.0f },
    {   35.0f,  6881.0f },
    {   40.0f,  5759.0f },
    {   45.0f,  4847.0f },
    {   50.0f,  4101.0f },
    {   55.0f,  3488.0f },
    {   60.0f,  2981.0f },
    {   65.0f,  2559.0f },
    {   70.0f,  2207.0f },
    {   75.0f,  1912.0f },
    {   80.0f,  1662.0f },
    {   85.0f,  1451.0f },
    {   90.0f,  1272.0f },
    {   95.0f,  1118.0f },
    {  100.0f,   987.0f },
    {  105.0f,   874.0f },
    {  110.0f,   776.0f },
    {  115.0f,   692.0f },
    {  120.0f,   618.0f },
};

NTCNode const * const ntc_T_R       = ntc_T_R_table;
size_t          const ntc_T_R_count = (sizeof(ntc_T_R_table) / sizeof(*ntc_T_R_table));
