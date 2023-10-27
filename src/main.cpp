#include <Arduino.h>
#include <ei_cam.h>
#include <WiFi.h>
#include <config.h>

EICam eiCam = EICam();
void connectWiFi();
void handlePredictions(ei_impulse_result_t *predictions);

void setup()
{
    Serial.begin(115200);

    eiCam.begin(true);

    connectWiFi();

    eiCam.startStream();

    eiCam.controlLED(0);
}

void loop()
{
    ei_impulse_result_t predictions = eiCam.predict();

    handlePredictions(&predictions);
}

void handlePredictions(ei_impulse_result_t *predictions)
{
    Serial.printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                  predictions->timing.dsp, predictions->timing.classification, predictions->timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    bool bb_found = predictions->bounding_boxes[0].value > 0;
    for (size_t ix = 0; ix < predictions->bounding_boxes_count; ix++)
    {
        auto bb = predictions->bounding_boxes[ix];
        if (bb.value == 0)
        {
            continue;
        }
        Serial.printf("%s (%f) [ x: %u, y: %u, width: %u, height: %u ]\n", bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
    }
    if (!bb_found)
    {
        Serial.printf("No objects found\n");
    }
#else
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
    {
        Serial.printf("%s: %.5f\n", predictions->classification[ix].label,
                      predictions->classification[ix].value);
    }
#endif

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    _log("    anomaly score: %.3f\n", predictions->anomaly);
#endif

    Serial.printf("\n");
}

void connectWiFi()
{
    if (!WiFi.config(staticIP, gateway, subnet))
    {
        Serial.printf("Failed to set static IP\n");
    }

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.printf(".");
    }

    const char *ip = staticIP.toString().c_str();

    Serial.printf("\nWiFi connected on %s\n", ip);
}