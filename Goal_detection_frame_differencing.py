'''
Author       : Karen Li, Jiawei Xu
Date         : 2023-09-28 15:45:04
LastEditors  : Jiawei Xu
LastEditTime : 2023-10-06 19:16:00
Description  : This program is used to detect the retro-reflective goal
               using frame differencing under the LED light.
Editor Notes :
    1. The function `reflective_highlighter` takes two consectutive
       images, with the infrared LED on and off, respectively, then
       compute the difference between the two images to get the
       highlighted area, presumably the retro-reflective targets
    2. Since the image sensor takes images from its onboard buffer
       which has a maximum fresh rate, there exists a maximum
       feasible refresh rate for the sensor to properly capture
       the LED on and off. `WAIT_TIME_US` is the user-defined time
       for such a maximum refresh rate.
    3. Since we are taking two pictures at each iteration/detection,
       we get two blocks of time that we can use for some computation
       instead of plainly wait. That is what "time blocks" are for
       (search for `time block 1` and `time block 2`)
'''

import math
import sensor, image, time, pyb, omv, os
import image, network, rpc, struct

from pyb import UART
from machine import Pin

INFRARED_LED_PIN = "PC4"
GOAL_COLOR_THRESHOLD = (52, 65, 55, 75, 30, 50)
WAIT_TIME_US = 40000

def init_sensor_goal(pixformat=sensor.RGB565, framesize=sensor.HQVGA, windowsize=None,
                     brightness=3, autoexposure=False, exposure=5000, autowhitebal=False,
                     white_balance=(0, 0, 0), contrast=0, saturation=0) -> None:
    """
    description: Initializes the sensor for goal detection
    return      {*}: None
    """
    sensor.reset()                        # Initialize the camera sensor.
    sensor.set_pixformat(pixformat)       # Set pixel format to RGB565 (or GRAYSCALE)
    sensor.set_framesize(framesize)       # 160x120 resolution
    if windowsize is not None:            # Set windowing to reduce the resolution of the image
        sensor.set_windowing(windowsize)
    sensor.skip_frames(time=1000)         # Let new settings take affect.
    sensor.set_auto_exposure(False)
    sensor.set_auto_whitebal(False)
    sensor.__write_reg(0xfe, 0b00000000) # change to registers at page 0
    sensor.__write_reg(0x03, 0b00000000) # high bits of exposure control
    sensor.__write_reg(0x04, 0b10000000) # low bits of exposure control
    sensor.__write_reg(0xb0, 0b10000000) # global gain
    sensor.__write_reg(0xad, 0b01100000) # R ratio
    sensor.__write_reg(0xae, 0b01100000) # G ratio
    sensor.__write_reg(0xaf, 0b01100000) # B ratio
    # RGB gains
    sensor.__write_reg(0xa3, 0b10100000) # G gain odd
    sensor.__write_reg(0xa4, 0b10100000) # G gain even
    sensor.__write_reg(0xa5, 0b10100000) # R gain odd
    sensor.__write_reg(0xa6, 0b10100000) # R gain even
    sensor.__write_reg(0xa7, 0b10100000) # B gain odd
    sensor.__write_reg(0xa8, 0b10100000) # B gain even
    sensor.__write_reg(0xa9, 0b10100000) # G gain odd 2
    sensor.__write_reg(0xaa, 0b10100000) # G gain even 2

    sensor.__write_reg(0xfe, 0b00000010) # change to registers at page 2
    #sensor.__write_reg(0xd0, 0b00000000) # change global saturation,
                                          # strangely constrained by auto saturation
    sensor.__write_reg(0xd1, 0b01110000) # change Cb saturation
    sensor.__write_reg(0xd2, 0b01100000) # change Cr saturation
    sensor.__write_reg(0xd3, 0b01000000) # luma contrast
    sensor.__write_reg(0xd5, 0b00000000) # luma offset
    sensor.skip_frames(time=2000)          # Let the camera adjust.


def draw_rectangle(img, blob):
    img.draw_rectangle(blob.rect(), color=(0, 255, 0))
    # img.flush()


def reflective_highlighter():
    global time_last_snapshot # we need to wait for the sensor for some time from
                              # the last snapshot to avoid a partial new image
    clock.tick()
    omv.disable_fb(True)
    elapsed = WAIT_TIME_US - (int((time.time_ns() - time_last_snapshot)/1000))
    if elapsed > 0:
        time.sleep_us(elapsed)
    extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.RGB565)
    extra_fb.replace(sensor.snapshot())
    time_last_snapshot = time.time_ns()
    led_pin.value(0)

    # time block 2:
    # blob filtering

    elapsed = WAIT_TIME_US - (int((time.time_ns()-time_last_snapshot)/1000))
    if elapsed > 0:
        time.sleep_us(elapsed)
    img = sensor.snapshot()
    img.sub(extra_fb, reverse = True)
    sensor.dealloc_extra_fb()
    time_last_snapshot = time.time_ns()
    led_pin.value(1)
    img.lens_corr(1.65).mean(1)
    omv.disable_fb(False)
    img.negate()
    img.flush()

    # time block 1:
    # blob detection


if __name__ == "__main__":
    # Initialize the camera sensor for goal detection
    init_sensor_goal(pixformat=sensor.RGB565, windowsize=None, autoexposure=False,
                     autowhitebal=False, white_balance=(-6.02073, -5.49487, -1.804),
                     exposure=10000, contrast=-3, saturation=0, brightness=0)
    clock = time.clock()                        # Tracks FPS.
    led_pin = Pin(INFRARED_LED_PIN, Pin.OUT)    # Initialize the LED pin
    led_state = 0                               # Initialize the LED as off
    led_pin.value(led_state)
    img = sensor.snapshot()                     # Take an initial snapshot w/ led off
    time_last_snapshot = time.time_ns()         # Initialize the time record the time
    while True:
        clock.tick()
        reflective_highlighter()
        print(clock.fps())
