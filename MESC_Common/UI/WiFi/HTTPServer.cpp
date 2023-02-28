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

#include "HTTPServer.hpp"

#include "HTTPProtocol.hpp"

namespace MESC
{
    namespace UI
    {
        namespace WiFi
        {
            uint16_t  HTTPServer::getPort() const
            {
                return UINT16_C(8080);
            }

            void HTTPServer::process_request( char const c )
            {
                if (c == '\r')
                {
                    return;
                }

                if (c == '\n')
                {
                    Serial.print( "HTTP REQ>" );
                    Serial.println( reqbuf.c_str() );

                    if (reqbuf.size() == 0)
                    {
                        Serial.println( "INFO: Processing HTTP request" );
                        // Peek reqbuf for 'GET ...'
                        // Peek request path '... /file.name ...'
                        // path_entry = lookup( req_path )
                        // if path_entry not found, process dynamic request e.g. update?var=val&...
                        HTTP::ResponseHeader header;

                    // TODO populate resp_pay with path or abort
                    //  resp_pay.push_back( path_entry );

                        header.generate_response( resp_hdr );
                    }
                    else
                    {
                        req.push_back( reqbuf );
                        reqbuf.clear();
                    }
                }
                else
                {
                    reqbuf.append( 1, c );
                }
            }

            bool HTTPServer::response_available() const
            {
                return !resp_hdr.empty() || !resp_pay.empty();
            }

            std::string HTTPServer::get_response_line()
            {
                std::string ret;
                std::deque< std::string > * p = NULL;

                if (!resp_hdr.empty())
                {
                    p = &resp_hdr;
                }
                else if (!resp_pay.empty())
                {
                    p = &resp_pay;
                }

                if (p)
                {
                    ret = p->front();
                    ret.append( "\r\n" );
                    p->pop_front();
                }

                return ret;
            }
        }
    }
}
