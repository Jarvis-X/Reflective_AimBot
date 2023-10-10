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


""" The first three functions are for image sensor tests
"""
def camera_filter_highlighter_mask_test():
    """
    description: A test function that visualizes the mask for
                 image difference
    return  {*}: None
    """
    omv.disable_fb(True) # disable display on the IDE
    led_pin.value(0) # turn off the led
    time_last_snapshot = time.time_ns() # time the LED change
    # allocate extra buffer spaces to hold the
    extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(),
                                     sensor.GRAYSCALE) # grayscale image w/ LED on
    hold_up_for_sensor_refresh(time_last_snapshot)
    extra_fb.replace(sensor.snapshot().to_grayscale())
    """ We have ONE image now:
        one on the framebuffer: extra_fb2 (grayscale)
    """

    time_last_snapshot = time.time_ns() # time the LED change
    hold_up_for_sensor_refresh(time_last_snapshot)
    img = sensor.snapshot().to_grayscale(copy=False) # second image in grayscale
    """ We have TWO images now:
        one on the framebuffer: extra_fb (grayscale)
        one on the framebuffer: img (grayscale)
    """

    img.difference(extra_fb) # motion map (single channel)!!!
    img.binary([(18, 255)]).dilate(1).erode(1) # motion map (binary)
    sensor.dealloc_extra_fb()
    omv.disable_fb(False)
    img.negate()
    img.flush()


def reflective_highlighter_1step_test():
    """
    description: A test function that visualizes the frame
                 w/ LED on and off to see the rolling shutter
    return  {*}: None
    """
    omv.disable_fb(True)
    led_pin.value(0)
    img = sensor.snapshot()
#    time_last_snapshot = time.time_ns() # time the LED change
#    hold_up_for_sensor_refresh(time_last_snapshot, wait_time=5000)
    img = sensor.snapshot()
    led_pin.value(1)
    time_last_snapshot = time.time_ns() # time the LED change
    hold_up_for_sensor_refresh(time_last_snapshot, wait_time=1000)
    omv.disable_fb(False)
    img.flush()
    return None


def reflective_highlighter_2steps_test():
    """
    description: A test function that visualizes the frame difference w/ LED on and off
    return  {*}: None
    """
    omv.disable_fb(True)
    led_pin.value(0)
    time_last_snapshot = time.time_ns() # time the LED change
    extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.RGB565)
    hold_up_for_sensor_refresh(time_last_snapshot, wait_time=8425)

    extra_fb.replace(sensor.snapshot())
    time_last_snapshot = time.time_ns() # time the LED change
    hold_up_for_sensor_refresh(time_last_snapshot, wait_time=10015)

    led_pin.value(1)
    time_last_snapshot = time.time_ns() # time the LED change
    hold_up_for_sensor_refresh(time_last_snapshot, wait_time=8425)
    img = sensor.snapshot()
    time_last_snapshot = time.time_ns() # time the LED change
    hold_up_for_sensor_refresh(time_last_snapshot, wait_time=10015)
    img.sub(extra_fb, reverse = True)
    omv.disable_fb(False)
    img.negate()
    img.flush()
    sensor.dealloc_extra_fb()
    return None


INFRARED_LED_PIN = "PC4"
goal_thresholds = [(35, 55, -10, 30, 20, 50), (50, 75, -50, -20, 40, 70)]
WAIT_TIME_US = 1000

def init_sensor_goal(pixformat=sensor.RGB565, framesize=sensor.HQVGA, windowsize=None) -> None:
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
    sensor.__write_reg(0x82, 0b00000100) # enable anti blur, disable AWB
    sensor.__write_reg(0x03, 0b00000001) # high bits of exposure control
    sensor.__write_reg(0x04, 0b00000000) # low bits of exposure control
    sensor.__write_reg(0xb0, 0b11100000) # global gain
#    sensor.__write_reg(0xad, 0b10100000) # R ratio
#    sensor.__write_reg(0xae, 0b10100000) # G ratio
#    sensor.__write_reg(0xaf, 0b10100000) # B ratio
#    # RGB gains
#    sensor.__write_reg(0xa3, 0b10100000) # G gain odd
#    sensor.__write_reg(0xa4, 0b10100000) # G gain even
#    sensor.__write_reg(0xa5, 0b10100000) # R gain odd
#    sensor.__write_reg(0xa6, 0b10100000) # R gain even
#    sensor.__write_reg(0xa7, 0b10100000) # B gain odd
#    sensor.__write_reg(0xa8, 0b10100000) # B gain even
#    sensor.__write_reg(0xa9, 0b10100000) # G gain odd 2
#    sensor.__write_reg(0xaa, 0b10100000) # G gain even 2

    sensor.__write_reg(0xfe, 0b00000010) # change to registers at page 2
#    sensor.__write_reg(0xd0, 0b00000000) # change global saturation,
#                                         # strangely constrained by auto saturation
    sensor.__write_reg(0xd1, 0b01100000) # change Cb saturation
    sensor.__write_reg(0xd2, 0b01100000) # change Cr saturation
    sensor.__write_reg(0xd3, 0b01000000) # luma contrast
#    sensor.__write_reg(0xd5, 0b00000000) # luma offset
    sensor.skip_frames(time=2000)         # Let the camera adjust.


def draw_rectangle(img, blob):
    img.draw_rectangle(blob.rect(), color=(0, 255, 0))
    # img.flush()



def hold_up_for_sensor_refresh(last_time_stamp, wait_time=WAIT_TIME_US) -> None:
    """
    description: wait for the sensor for some time from the
                 last snapshot to avoid a partial new image
    return  {*}: None
    """
    elapsed = wait_time - (int((time.time_ns() - last_time_stamp)/1000))
    if elapsed > 0:
        time.sleep_us(elapsed)

    return None



""" The actual funtion we use for goal detection and tracking

"""
def camera_filter_highlighter():
    omv.disable_fb(True) # disable display
    led_pin.value(0)
    time_last_snapshot = time.time_ns() # time the LED change
    hold_up_for_sensor_refresh(time_last_snapshot, wait_time=10000)

    # allocate extra buffer spaces for:
    #    1. motion filter
    #    2. image difference
    extra_fb1 = sensor.alloc_extra_fb(sensor.width(), sensor.height(),
                                      sensor.RGB565) # first one for full image w/ LED off
    extra_fb2 = sensor.alloc_extra_fb(sensor.width(), sensor.height(),
                                      sensor.GRAYSCALE) # second one for grayscale image w/ LED off

    time_last_snapshot = time.time_ns() # time the LED change
    extra_fb2.replace(sensor.snapshot().to_grayscale()) # first image (LED off)
    hold_up_for_sensor_refresh(time_last_snapshot, wait_time=10000)
    """ We have ONE image now:
        one on the framebuffer: extra_fb2 (grayscale)
    """
    time_last_snapshot = time.time_ns() # time the LED change
    extra_fb1.replace(sensor.snapshot()) # second image (LED off)
    """ We have TWO images now:
        one on the framebuffer: extra_fb2 (grayscale)
        one on the framebuffer: extra_fb1 (RGB565)
    """
    img_gray = extra_fb1.to_grayscale(copy=True) # second image in grayscale
    """ We have THREE images now:
        one on the framebuffer: extra_fb2 (grayscale)
        one on the framebuffer: extra_fb1 (RGB565)
        one on the heap: img_gray(grayscale)
    """
    img_gray.difference(extra_fb2) # motion map (single channel, scalar)!!!
    sensor.dealloc_extra_fb()
    """ We have TWO images now:
        one on the framebuffer: extra_fb1 (RGB565)
        one on the heap (motion map): img_gray(grayscale)
    """
    img_gray.binary([(16, 255)], invert=False).erode(1).dilate(1).erode(1) # motion map (binary)
    hold_up_for_sensor_refresh(time_last_snapshot, wait_time=10000)

    led_pin.value(1) # turn off the led
    time_last_snapshot = time.time_ns()
    hold_up_for_sensor_refresh(time_last_snapshot, wait_time=10000)
    img = sensor.snapshot() # third image (LED on)
    """ We have THREE images now:
        one on the framebuffer: extra_fb1 (RGB565)
        one on the framebuffer: img (RGB565)
        one on the heap (motion map): img_gray(grayscale)
    """
    img.sub(extra_fb1, reverse = True, mask=img_gray)
    img.negate()
    blobs = goal_blob_detection(img, goal_thresholds)

    sensor.dealloc_extra_fb()
    omv.disable_fb(False)
    img.flush()


def camera_filter_highlighter_superslow():
    omv.disable_fb(True) # disable display
    led_pin.value(0)
    time_last_snapshot = time.time_ns() # time the LED change
    hold_up_for_sensor_refresh(time_last_snapshot, wait_time=100000)

    # allocate extra buffer spaces for:
    #    1. motion filter
    #    2. image difference
    extra_fb1 = sensor.alloc_extra_fb(sensor.width(), sensor.height(),
                                      sensor.RGB565) # first one for full image w/ LED off
    extra_fb2 = sensor.alloc_extra_fb(sensor.width(), sensor.height(),
                                      sensor.GRAYSCALE) # second one for grayscale image w/ LED off

    time_last_snapshot = time.time_ns() # time the LED change
    extra_fb2.replace(sensor.snapshot().to_grayscale()) # first image (LED off)
    """ We have ONE image now:
        one on the framebuffer: extra_fb2 (grayscale)
    """
    time_last_snapshot = time.time_ns() # time the LED change
    hold_up_for_sensor_refresh(time_last_snapshot, wait_time=20000)
    extra_fb1.replace(sensor.snapshot()) # second image (LED off)
    """ We have TWO images now:
        one on the framebuffer: extra_fb2 (grayscale)
        one on the framebuffer: extra_fb1 (RGB565)
    """
    img_gray = extra_fb1.to_grayscale(copy=True) # second image in grayscale
    """ We have THREE images now:
        one on the framebuffer: extra_fb2 (grayscale)
        one on the framebuffer: extra_fb1 (RGB565)
        one on the heap: img_gray(grayscale)
    """
    img_gray.difference(extra_fb2) # motion map (single channel, scalar)!!!
    sensor.dealloc_extra_fb()
    """ We have TWO images now:
        one on the framebuffer: extra_fb1 (RGB565)
        one on the heap (motion map): img_gray(grayscale)
    """
    img_gray.binary([(16, 255)], invert=False).erode(1).dilate(1).erode(1) # motion map (binary)
    led_pin.value(1) # turn off the led
    hold_up_for_sensor_refresh(time_last_snapshot, wait_time=100000)
    img = sensor.snapshot() # third image (LED on)
    """ We have THREE images now:
        one on the framebuffer: extra_fb1 (RGB565)
        one on the framebuffer: img (RGB565)
        one on the heap (motion map): img_gray(grayscale)
    """
    img.sub(extra_fb1, reverse = True, mask=img_gray)
    img.negate()
    blobs = goal_blob_detection(img, goal_thresholds)

    sensor.dealloc_extra_fb()
    omv.disable_fb(False)
    img.flush()



def reflective_highlighter_2steps():
    """
    description: A test function that visualizes the frame difference w/ LED on and off
    return  {*}: None
    """
    omv.disable_fb(True)
    led_pin.value(0)
    time_last_snapshot = time.time_ns() # time the LED change
    extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.RGB565)
    hold_up_for_sensor_refresh(time_last_snapshot, wait_time=8425)

    extra_fb.replace(sensor.snapshot())
    time_last_snapshot = time.time_ns() # time the LED change
    hold_up_for_sensor_refresh(time_last_snapshot, wait_time=15000)

    led_pin.value(1)
    time_last_snapshot = time.time_ns() # time the LED change
    hold_up_for_sensor_refresh(time_last_snapshot, wait_time=8425)
    img = sensor.snapshot()
    time_last_snapshot = time.time_ns() # time the LED change
    img.sub(extra_fb, reverse = True)
    omv.disable_fb(False)
    img.negate()
    blobs = goal_blob_detection(img, goal_thresholds)
    hold_up_for_sensor_refresh(time_last_snapshot, wait_time=15000)
    img.flush()
    sensor.dealloc_extra_fb()
    return None


def goal_blob_detection(img, goal_thresholds, verbose=True):
    green_blobs = img.find_blobs(goal_thresholds,
                                 area_threshold=20,
                                 pixels_threshold=20,
                                 margin=int(sensor.height()/40),
                                 merge=True)
    if verbose:
        for green_blob in green_blobs:
            img.draw_rectangle(green_blob.rect(), color = (178,15,184))

    return green_blobs





if __name__ == "__main__":
    # Initialize the camera sensor for goal detection
    init_sensor_goal()
    clock = time.clock()                        # Tracks FPS.
    led_pin = Pin(INFRARED_LED_PIN, Pin.OUT)    # Initialize the LED pin
    while True:
        clock.tick()
        camera_filter_highlighter_superslow()
        print(clock.fps())
