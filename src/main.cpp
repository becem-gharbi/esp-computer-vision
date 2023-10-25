#include <Arduino.h>
#include <ei_cam.h>
#include <WiFi.h>
#include <credentials.h>

EICam eiCam = EICam();
void connectWiFi();

void setup()
{
    Serial.begin(115200);

    eiCam.begin(true);

    connectWiFi();

    eiCam.startStream();

    Serial.print("Camera Stream Ready! Go to: http://");
    Serial.print(WiFi.localIP());
    Serial.printf("\n");
}

void loop()
{
    eiCam.loop();
}

void connectWiFi()
{
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
}