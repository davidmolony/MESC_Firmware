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

#include "Provider/ESP32.hpp"

#define MESC_UI_WIFI_PROVIDER MESC_ESP32

MESC_UI_WIFI_PROVIDER * wp;

void setup()
{
    Serial.begin(9600);
    Serial.println("INFO: Starting MESC WiFi Initialisation");

    wp = new MESC_UI_WIFI_PROVIDER();
    
    if (!wp)
    {
        Serial.println("ERROR: Failed to create WiFi Provider");
        return;
    }

    Serial.println("INFO: Starting Access Point");

    if (!wp->startAP())
    {
        Serial.println("ERROR: Failed to create WiFi Access Point");
        return;
    }

    Serial.println("INFO: Started Access Point");
    Serial.println(wp->getIP().c_str());

    Serial.println("INFO: Starting HTTP Server");

    if (!wp->startHTTP())
    {
        Serial.println("ERROR: Failed to create WiFi HTTP Server");
        return;
    }

    Serial.println("INFO: Started HTTP Server");
    Serial.println(wp->getPort());

    Serial.println("INFO: Finished MESC WiFi Initialisation");
}

void loop()
{
    if (wp)
    {
        wp->runHTTP();
    }
}

#include "Provider/ESP32.cpp"

#include "WiFi/AccessPoint.cpp"
#include "WiFi/HTTPProtocol.cpp"
#include "WiFi/HTTPServer.cpp"
