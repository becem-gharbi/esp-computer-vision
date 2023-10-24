#ifndef H_EI_CAM
#define H_EI_CAM

#include "edge-impulse-sdk/classifier/ei_classifier_types.h"
#include "esp_camera.h"

#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240
#define EI_CAMERA_FRAME_BYTE_SIZE 3

class EICam
{
public:
    void begin();
    void end();
    void loop();

private:
    static camera_config_t _camConfig;
    static uint8_t *_snapshotBuffer;
    bool _camInitialized;
    bool _initCam(void);
    void _deinitCam(void);
    bool _captureCam(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
    static int _getDataCam(size_t offset, size_t length, float *out_ptr);
    void _handlePredictions(ei_impulse_result_t *predictions);
};

#endif