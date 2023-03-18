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

#include <WiFi.h>
#include <WebServer.h>

#include <string>
#include <tuple>
#include <vector>

#define USE_UART

#ifdef USE_UART
#define UART Serial2
#endif

static char     const * g_esp32_webserver_ssid    = "MESC_UI_AP_1234";
static char     const * g_esp32_webserver_pswd    = "password";
static int      const   g_esp32_webserver_channel = 11;
static uint16_t const   g_esp32_webserver_port    = 80;

static WebServer  * g_esp32_webserver = NULL;
static IPAddress    g_esp32_ip;
// STARTING_AUTO_GENERATED_WWW_DATA
#include "www/www.cpp"
// FINISHED_AUTO_GENERATED_WWW_DATA
#define MESC_WWW_ENTRY(path,mime,data) std::make_tuple<>( path, mime, data )
static std::vector< std::tuple< char const *, char const *, char const * > > const g_root =
{
    MESC_WWW_ENTRY( "/", "text/html", index_html ),
// STARTING_AUTO_GENERATED_URL_ENTRY
#include "www/url.cpp"
// FINSHED_AUTO_GENERATED_URL_ENTRY
};

static std::string getAddress()
{
    return  std::to_string(g_esp32_ip[0])
    + "." + std::to_string(g_esp32_ip[1])
    + "." + std::to_string(g_esp32_ip[2])
    + "." + std::to_string(g_esp32_ip[3])
    + ":" + std::to_string(g_esp32_webserver_port);
}

void setup()
{
    Serial.begin( 9600 );
    Serial.println( "INFO: Starting MESC WebServer Initialisation" );
#ifdef USE_UART
    Serial.println("INFO: Starting UART");

    Serial2.begin( 9600 );
#endif
    Serial.println("INFO: Starting Access Point");

    WiFi.softAP( g_esp32_webserver_ssid, g_esp32_webserver_pswd, 11 );

    Serial.println("INFO: Started Access Point");

    g_esp32_ip = WiFi.softAPIP();
    
    Serial.print("INFO: Address is ");
    std::string const addr = getAddress();
    Serial.println( addr.c_str() );

    g_esp32_webserver = new WebServer( g_esp32_webserver_port );

    Serial.println("INFO: Adding fall-back handler");

    g_esp32_webserver->onNotFound(
        []()
        {
            g_esp32_webserver->send( 404, "text/plain", "nope" );
        }
    );

    Serial.println("INFO: Adding UART handler");

    g_esp32_webserver->on( "/UART", // [GET] UART?command=<COMMAND> _OR_ [POST] UART command=<COMMAND
        []()
        {
            if (!g_esp32_webserver->hasArg( "command" ))
            {
                g_esp32_webserver->send( 400 );
                return;
            }

            String const command = g_esp32_webserver->arg( "command" );

            Serial.print("INFO: UART [");
            Serial.print( command );
            Serial.println("]");

            HTTPMethod const method = g_esp32_webserver->method();

            switch (method)
            {
                case HTTP_GET:
                {    
#ifdef USE_UART
                    UART.println( command );
#endif
                    std::string resp;
#ifdef USE_UART
                    while (UART.available())
                    {
                        char const c = UART.read();

                        resp.append( 1, c );
                    }
#endif
                    g_esp32_webserver->send( 200, "text/plain", resp.c_str() );
                    return;
                }
                case HTTP_POST:
#ifdef USE_UART
                    UART.println( command );
#endif
                    g_esp32_webserver->send( 202 );
                    return;
                default:
                  break;
            }

            g_esp32_webserver->send( 400 );
        }
    );

    Serial.println("INFO: Adding MESC_WWW_ENTRY entries");

    for ( auto const & e : g_root )
    {
        Serial.print("    ");
        char const * path = std::get<0>(e);
        Serial.println( path );

        g_esp32_webserver->on( path,
            [e]()
            {
                g_esp32_webserver->send( 200, std::get<1>(e), std::get<2>(e) );
            }
        );
    }

    Serial.println("INFO: Starting MESC WebServer");

    g_esp32_webserver->begin();

    Serial.println("INFO: Finished MESC WebServer Initialisation");
}

void loop()
{
    g_esp32_webserver->handleClient();
    delay(10);
}
