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

#include "ESP32.hpp"

#include <WiFi.h>

#include <map>
// STARTING_AUTO_GENERATED_WWW_DATA
#include "../www/www.cpp"
// FINISHED_AUTO_GENERATED_WWW_DATA
static WiFiServer * g_esp32_wifi_server = NULL;
static IPAddress    g_esp32_ip;

static std::map< std::string, MESC::UI::WiFi::HTTPServer::URLEntry > const url_content
{
    { "/", MESC::UI::WiFi::HTTPServer::URLEntry{ {"index.html"}, index_html, sizeof(index_html) } },
// STARTING_AUTO_GENERATED_URL_ENTRY
#include "../www/url.cpp"
// FINISHED_AUTO_GENERATED_URL_ENTRY
};

// MESC::UI::WiFi::AccessPoint
bool MESC_ESP32::startAP()
{
    std::string const & ssid   = getSSID();
    std::string const  pw      = "password";
    int         const  channel = getChannel();

    bool const ret = WiFi.softAP( ssid.c_str(), pw.c_str(), channel );

    if (ret)
    {
        g_esp32_ip = WiFi.softAPIP();
    }

    return ret;
}

std::string MESC_ESP32::getIP() const
{
    return  std::to_string(g_esp32_ip[0])
    + "." + std::to_string(g_esp32_ip[1])
    + "." + std::to_string(g_esp32_ip[2])
    + "." + std::to_string(g_esp32_ip[3]);
}
// MESC::UI::WiFi::HTTPServer
bool MESC_ESP32::startHTTP()
{
    if (g_esp32_wifi_server)
    {
        return false;
    }

    uint16_t const port = getPort();
    g_esp32_wifi_server = new WiFiServer( port );

    if (g_esp32_wifi_server)
    {
        g_esp32_wifi_server->begin();
        return true;
    }

    return false;
}

void MESC_ESP32::runHTTP()
{
    WiFiClient client = g_esp32_wifi_server->available();

    if (!client)
    {
        return;
    }

    Serial.println( "INFO: Starting HTTP client session" );

    while (client.connected())
    {
        if (client.available())
        {
            char const c = client.read();
            process_request( c );

            while (response_available())
            {
                std::string s = "HTTP RESP>";
                s.append( get_response_line() );
                Serial.print( s.c_str());
                client.print( s.c_str() );
            }
        }
    }

    Serial.println( "INFO: Finished HTTP client session" );
    
    client.stop();
}

std::pair< bool, MESC::UI::WiFi::HTTPServer::URLEntry > MESC_ESP32::lookup( std::string const url ) const
{
    std::map< std::string, MESC::UI::WiFi::HTTPServer::URLEntry >::const_iterator it = url_content.find( url );

    if (it == url_content.cend())
    {
        MESC::UI::WiFi::HTTPServer::URLEntry e;
        return std::make_pair<>( false, e );
    }
    else
    {
        return std::make_pair<>( true, it->second );
    }
}
