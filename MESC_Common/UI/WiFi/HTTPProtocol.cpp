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

#include "HTTPProtocol.hpp"

#include <map>

namespace HTTP
{
    ResponseHeader::ResponseHeader()
    :   m_status        ( Status::OK             )
    ,   m_content_type  ( ContentType::TEXT_HTML )
    ,   m_content_length( 0                      )
    {

    }

    void ResponseHeader::setStatus( Status const s )
    {
        m_status = s;
    }

    void ResponseHeader::setContentType( ContentType const ct )
    {
        m_content_type = ct;
    }

    void ResponseHeader::setContentLength( size_t const l )
    {
        m_content_length = l;
    }

    void ResponseHeader::generate_response( std::deque< std::string > & buf )
    {
        std::string line;

        line.assign( "HTTP/1.1 ");
        line.append( to_string( m_status ) );
        buf.push_back( line );

        line.assign( "Server: MESCUI ");
        buf.push_back( line );

        line.assign( "Content-Length: ");
        line.append( std::to_string( m_content_length ) );
        buf.push_back( line );

        line.assign( "Content-Type: ");
        line.append( to_string( m_content_type ) );
        buf.push_back( line );

        line.assign( "Connection: close");
        buf.push_back( line );

        // End of Header
        line.clear();
        buf.push_back( line );     
    }

}

template < class T > using underlying_type_t = typename std::underlying_type< T >::type;
template < class T > auto UNBOX( T const t ) -> underlying_type_t< T > { return static_cast< underlying_type_t< T > >( t ); }

std::string to_string( HTTP::Status const s )
{
    static std::map< HTTP::Status, std::string > const lut =
    {
        { HTTP::Status::OK                   , "OK"                    },
        { HTTP::Status::NO_CONTENT           , "No Content"            },
        { HTTP::Status::NOT_FOUND            , "Not Found"             },
        { HTTP::Status::INTERNAL_SERVER_ERROR, "Internal Server Error" },
    };
    
    auto it = lut.find( s );

    if (it == lut.cend())
    {
        it = lut.find( HTTP::Status::INTERNAL_SERVER_ERROR );
    }

    return std::to_string( UNBOX( it->first ) ) + " " + it->second;
}

std::string to_string( HTTP::ContentType const ct )// TODO charset
{
    static std::map< HTTP::ContentType, std::string > const lut =
    {
        { HTTP::ContentType::TEXT_HTML, "text/html" },
    };
    
    auto it = lut.find( ct );

    if (it == lut.cend())
    {
        return std::string( "500 Internal Server Error" );
    }

    return it->second; // TODO charset
}
