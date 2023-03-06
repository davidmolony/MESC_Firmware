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
#ifdef VIRT
#include "../virt/serial.hpp"
#endif
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

            std::pair< bool, HTTPServer::URLEntry > HTTPServer::lookup( std::string const url ) const
            {
                URLEntry e;
                return std::make_pair<>( false, e );
            (void)url;
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
                        HTTP::ResponseHeader header;

                        Serial.println( "INFO: Processing HTTP request" );
                        
                        std::string s = req.front();
                        req.pop_front();

                        if (s.compare(0,5,"GET /") == 0)
                        {
                            size_t const end = s.find( ' ', 4 );
                            std::string const url = s.substr(4,end-4);
                            Serial.println( ("INFO: URL '" + url + "'").c_str() );
                            std::pair< bool, URLEntry > const pay = lookup( url );
                            if (pay.first == false)
                            {
                                header.setStatus( HTTP::Status::NOT_FOUND );
                            }
                            else
                            {
                                size_t const len = pay.second.size;
                                if (len == 0)
                                {
                                    header.setStatus( HTTP::Status::NO_CONTENT );
                                }
                                else
                                {
                                    header.setContentLength( len );
                                    resp_pay.push_back( pay.second.data );
                                }
                            }
                        }/*
                        else
                        {
                            // error
                        }*/

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
