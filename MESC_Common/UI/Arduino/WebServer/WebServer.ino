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

static WebServer * g_esp32_webserver = NULL;
static IPAddress   g_esp32_ip;
// STARTING_AUTO_GENERATED_WWW_DATA
#include "www/www.cpp"
// FINISHED_AUTO_GENERATED_WWW_DATA
// STARTING_AUTO_GENERATED_URL_ENTRY
static std::vector< std::pair< char const *, char const * > > g_root =
{
    std::make_pair<>( "/index.html" , index_html ),
    std::make_pair<>( "/MESCUI.html", MESCUI_html),
    std::make_pair<>( "/MESCUI.css" , MESCUI_css ),
    std::make_pair<>( "/MESCUI.js"  , MESCUI_js  ),
};
// FINSHED_AUTO_GENERATED_URL_ENTRY
void setup()
{
    Serial.begin(9600);
    Serial.println("INFO: Starting MESC WebServer Initialisation");

    WiFi.softAP( "MESC_UI_AP_1234", "password", 11 );
    g_esp32_ip = WiFi.softAPIP();
    
    g_esp32_webserver = new WebServer( 8080 );

    for ( auto e : g_root )
    {
        g_esp32_webserver->on( e.first, [e]() { g_esp32_webserver->send( 200, "text/html", e.second ); } );
    }

    g_esp32_webserver->onNotFound( []() { g_esp32_webserver->send( 404, "text/plain", "nope" ); } );

    g_esp32_webserver->begin();

    Serial.println("INFO: Finished MESC WebServer Initialisation");
}

void loop()
{
    g_esp32_webserver->handleClient();
    delay(10);
}
