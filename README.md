# Reflective_AimBot
This program is used to detect the retro-reflective targets using frame differencing under flashing IR LED light.

## Contents
- [Image differencing - solution 1](https://github.com/Jarvis-X/Reflective_AimBot/blob/main/Goal_detection_frame_differencing_camera_test.py)
  - An issue explored and partially fixed: the electronic rolling shutter of the image sensor leads
    to asynchronous LED switch and image capture, which may cause a frame partially lit up by the IR
    LED, making the frame difference obstructed. It turns out waiting for a substantially long time
    (> 2*shutter/exposure time) after switching the LED can solve the rolling shutter problem.
    Cross-checked with Edward.
  - DONE: Robust blob detection for goals of different colors
  - TODO: blob filtering and tracking 
