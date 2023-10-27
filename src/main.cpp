#include <Arduino.h>
#include <ei_cam.h>
#include <WiFi.h>
#include <credentials.h>

EICam eiCam = EICam();
void connectWiFi();
void handlePredictions(ei_impulse_result_t *predictions);

void setup()
{
    Serial.begin(115200);

    eiCam.begin(true);

    connectWiFi();

    eiCam.startStream();

    Serial.print("Camera Stream Ready! Go to: http://");
    Serial.print(WiFi.localIP());
    Serial.printf("\n");

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
    Serial.printf("\n");
#else
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
    {
        Serial.printf("    %s: %.5f\n", predictions->classification[ix].label,
                      predictions->classification[ix].value);
    }
#endif

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    _log("    anomaly score: %.3f\n", predictions->anomaly);
#endif
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