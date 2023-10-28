# ESP32 computer vision

This project is a starter to run computer vision applications (image classification, object detection..) directly on ESP32 cam. No extra coding or deep ML knowledge is required as the process is done with [Edge Impulse](https://edgeimpulse.com). It's a platform that takes input data (image, sound..) and extracts features, trains the model and exports the code as library for multiple platforms including embedded devices (Arduino, Espressif..).

## Getting started

This project is built with `PlatformIO` and supports only `ESP32 CAM` boads. By default the model used is `Car Detection` provided by this public [project](https://studio.edgeimpulse.com/public/28056/latest). It uses Transfer learning method for image classification with labels `cars` and `unknown`.

First make sure to set WiFi configuration on `config.h` header file. Then build and upload the binary. Finally run the Serial monitor and the streaming app located under `app` folder.

## Usage
To create and deploy your own model, start by collecting camera snapshots via the streaming app and use the [Uploader](https://docs.edgeimpulse.com/docs/edge-impulse-studio/data-acquisition/uploader) tool to upload the files and label them. 

Then choose the processing block [Image](https://docs.edgeimpulse.com/docs/edge-impulse-studio/processing-blocks/image) for feature extraction.

Then choose the learning block [Transfer learning](https://docs.edgeimpulse.com/docs/edge-impulse-studio/learning-blocks/transfer-learning-images) for image classification or [FOMO](https://docs.edgeimpulse.com/docs/edge-impulse-studio/learning-blocks/object-detection/fomo-object-detection-for-constrained-devices) for object detection.

Then start the training and verify the model performance.

Finally for deployment choose Arduino library, build and download it. The extracted source files should be placed under the `src` folder. The header file should be renamed to `inferencing.h`.


## Credits

- https://docs.edgeimpulse.com/docs
- https://github.com/espressif/esp32-camera
- https://github.com/alanesq/esp32cam-demo
