/*
* Copyright 2023 cod3b453
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

#ifndef MESC_UI_WIFI_HTTPPROTOCOL_HPP
#define MESC_UI_WIFI_HTTPPROTOCOL_HPP

#include <deque>
#include <string>

namespace HTTP
{
    enum class Status
    {
        OK         = 200,
        NO_CONTENT = 204,

        INTERNAL_SERVER_ERROR = 500,
    };

    enum class ContentType
    {
        APPLICATION_JSON, // application/json

        APPLICATION_OCTETSTREAM, // application/octet-stream

        IMAGE_PNG,
        IMAGE_SVGXML, // image/svg+xml

        TEXT_CSS,
        TEXT_CSV,
        TEXT_HTML,
        TEXT_JAVASCRIPT,
        
        TEXT_PLAIN,
        TEXT_XML,
    };

    class ResponseHeader
    {
    private:
        Status      m_status;
        ContentType m_content_type;
    public:
        ResponseHeader();
        void setStatus( Status const s );
        void setContentType( ContentType const ct );
        void generate_response( std::deque< std::string > & buf );
    };
}

std::string to_string( HTTP::Status const s );
std::string to_string( HTTP::ContentType const ct );

#endif
