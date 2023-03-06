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

#include <iostream>
#include <string>

#include <cstdlib>
#include <cstring>

#include "serial.hpp"

#include "../Provider/ESP32.hpp"

class Virt : public MESC::UI::WiFi::HTTPServer
{
public:
    virtual bool     startHTTP() override
    {
        return true;
    };

    virtual void     runHTTP() override
    {
        static char const req[]
        {
            "GET / HTTP/1.1\r\n"
            "Host: 192.168.4.1\r\n"
            "Accept: test/html\r\n"
            "\r\n"
        };

        for ( size_t n { strlen(req) }, i {}; i < n; ++i )
        {
            char const c { req[i] };
            std::cout << c;
            process_request( c );

            while (response_available())
            {
                std::string s = "HTTP RESP>";
                s.append( get_response_line() );
                std::cout << s;
            }
        }
    }

    virtual std::pair< bool, URLEntry > lookup( std::string const url ) const
    {
        URLEntry e;

        if (url == "/")
        {
            static char const data[] = {'H','e','l','l','o',' ','w','o','r','l','d'};
            e.path = "index.html";
            e.data = data;
            e.size = sizeof(data);
            return std::make_pair<>( true, e );
        }
        
        return std::make_pair<>( false, e );
    }
};

int main( int argc, char ** argv )
{
    Virt http_server {};

    http_server.runHTTP();

    return EXIT_SUCCESS;
(void)argc;
(void)argv;
}
