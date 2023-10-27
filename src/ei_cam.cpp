#include "Inference.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include <ei_cam.h>

bool EICam::_logEnabled = false;
bool EICam::_camInitialized = false;
bool EICam::_isCapturingForInference = false;
uint8_t *EICam::_snapshotBufferForInference = nullptr;
httpd_handle_t EICam::_streamHttpd = NULL;

void EICam::_log(const char *format, ...)
{
    if (_logEnabled)
    {
        va_list args;
        va_start(args, format);
        Serial.printf(format, args);
        va_end(args);
    }
}

void EICam::begin(bool logEnabled)
{
    _logEnabled = logEnabled;

    if (_initCam() == false)
    {
        _log("Failed to initialize Camera!\r\n");
    }
    else
    {
        _log("Camera initialized\r\n");
    }

    _initLED();

    _log("\nStarting continuous Inference in 2 seconds...\n");
    ei_sleep(2000);
}

void EICam::end()
{
    stopStream();
    _deinitCam();
}

ei_impulse_result_t EICam::predict()
{
    ei_impulse_result_t result = {};

    // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
    if (ei_sleep(5) != EI_IMPULSE_OK)
    {
        return result;
    }

    _snapshotBufferForInference = (uint8_t *)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

    // check if allocation was successful
    if (_snapshotBufferForInference == nullptr)
    {
        _log("ERR: Failed to allocate snapshot buffer!\n");
        return result;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &_getDataCamForInference;

    if (_captureCamForInference((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, _snapshotBufferForInference) == false)
    {
        free(_snapshotBufferForInference);
        return result;
    }

    // Run the classifier

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);

    free(_snapshotBufferForInference);

    if (err != EI_IMPULSE_OK)
    {
        _log("Failed to run classifier (%d)\n", err);
        return result;
    }

    return result;
}

int EICam::_getDataCamForInference(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0)
    {
        out_ptr[out_ptr_ix] = (_snapshotBufferForInference[pixel_ix] << 16) + (_snapshotBufferForInference[pixel_ix + 1] << 8) + _snapshotBufferForInference[pixel_ix + 2];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix += 3;
        pixels_left--;
    }
    // and done!
    return 0;
}

bool EICam::_captureCamForInference(uint32_t img_width, uint32_t img_height, uint8_t *out_buf)
{
    bool do_resize = false;

    if (_camInitialized == false)
    {
        _log("Camera is not initialized\r\n");
        return false;
    }

    _isCapturingForInference = true;

    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb)
    {
        _log("[capture] Camera capture failed\n");
        _isCapturingForInference = false;
        return false;
    }

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, _snapshotBufferForInference);

    esp_camera_fb_return(fb);

    _isCapturingForInference = false;

    if (!converted)
    {
        _log("[capture] Conversion failed\n");
        return false;
    }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS) || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS))
    {
        do_resize = true;
    }

    if (do_resize)
    {
        ei::image::processing::crop_and_interpolate_rgb888(
            out_buf,
            EI_CAMERA_RAW_FRAME_BUFFER_COLS,
            EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            out_buf,
            img_width,
            img_height);
    }

    return true;
}

void EICam::_deinitCam()
{
    // deinitialize the camera
    esp_err_t err = esp_camera_deinit();

    if (err != ESP_OK)
    {
        _log("Camera deinit failed\n");
        return;
    }

    _camInitialized = false;
    return;
}

bool EICam::_initCam()
{
    if (_camInitialized == true)
        return true;

    const camera_config_t _camConfig = {
        .pin_pwdn = PWDN_GPIO_NUM,
        .pin_reset = RESET_GPIO_NUM,
        .pin_xclk = XCLK_GPIO_NUM,
        .pin_sscb_sda = SIOD_GPIO_NUM,
        .pin_sscb_scl = SIOC_GPIO_NUM,

        .pin_d7 = Y9_GPIO_NUM,
        .pin_d6 = Y8_GPIO_NUM,
        .pin_d5 = Y7_GPIO_NUM,
        .pin_d4 = Y6_GPIO_NUM,
        .pin_d3 = Y5_GPIO_NUM,
        .pin_d2 = Y4_GPIO_NUM,
        .pin_d1 = Y3_GPIO_NUM,
        .pin_d0 = Y2_GPIO_NUM,
        .pin_vsync = VSYNC_GPIO_NUM,
        .pin_href = HREF_GPIO_NUM,
        .pin_pclk = PCLK_GPIO_NUM,

        // XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
        .xclk_freq_hz = 20000000,

        .pixel_format = PIXFORMAT_JPEG, // YUV422,GRAYSCALE,RGB565,JPEG
        .frame_size = FRAMESIZE_QVGA,   // QQVGA-UXGA Do not use sizes above QVGA when not JPEG

        .jpeg_quality = 10, // 0-63 lower number means higher quality
        .fb_count = 1,      // if more than one, i2s runs in continuous mode. Use only with JPEG
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    };

    // initialize the camera
    esp_err_t err = esp_camera_init(&_camConfig);
    if (err != ESP_OK)
    {
        _log("Camera init failed with error 0x%x\n", err);
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID)
    {
        s->set_vflip(s, 1);      // flip it back
        s->set_brightness(s, 1); // up the brightness just a bit
        s->set_saturation(s, 0); // lower the saturation
    }

    _camInitialized = true;

    _log("Camera initialized");

    return true;
}

void EICam::startStream()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = _streamHandler,
        .user_ctx = NULL};

    if (httpd_start(&_streamHttpd, &config) == ESP_OK)
    {
        httpd_register_uri_handler(_streamHttpd, &index_uri);
    }
}

void EICam::stopStream()
{
    if (_streamHttpd)
    {
        httpd_stop(_streamHttpd);
    }
}

esp_err_t EICam::_streamHandler(httpd_req_t *req)
{
    const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
    const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
    const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    char *part_buf[64];

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK)
    {
        return res;
    }

    while (true)
    {
        if (_isCapturingForInference)
        {
            delay(1);
            continue;
        }

        fb = esp_camera_fb_get();
        if (!fb)
        {
            _log("[stream] Camera capture failed");
            res = ESP_FAIL;
        }
        else
        {
            if (fb->width > 0)
            {

                if (fb->format != PIXFORMAT_JPEG)
                {
                    bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                    esp_camera_fb_return(fb);
                    fb = NULL;
                    if (!jpeg_converted)
                    {
                        _log("[stream] JPEG compression failed");
                        res = ESP_FAIL;
                    }
                }
                else
                {
                    _jpg_buf_len = fb->len;
                    _jpg_buf = fb->buf;
                }
            }
        }
        if (res == ESP_OK)
        {
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if (fb)
        {
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        }
        else if (_jpg_buf)
        {
            free(_jpg_buf);
            _jpg_buf = NULL;
        }
        if (res != ESP_OK)
        {
            break;
        }
    }

    return res;
}

void EICam::controlLED(uint8_t intensity)
{
    ledcWrite(LEDC_CHANNEL, intensity); 
}

void EICam::_initLED()
{
    ledcSetup(LEDC_CHANNEL, 4000, 8);
    ledcAttachPin(4, LEDC_CHANNEL);
    controlLED(0);
}