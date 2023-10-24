#include <Arduino.h>
#include <ei_cam.h>

EICam eiCam = EICam();

void setup()
{
    Serial.begin(115200);

    eiCam.begin();
}

void loop()
{
    eiCam.loop();
}