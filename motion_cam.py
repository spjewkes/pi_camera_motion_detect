#!/usr/bin/env python
import sys,os
import io
import numpy as np

try:
    from picamera import PiCamera, PiCameraCircularIO
except ImportError:
    exit('This script requires the picamera module\nInstall with: sudo pip install picamera')

try:
    from PIL import Image, ImageFilter
except ImportError:
    exit('This script requires the pillow module\nInstall with: sudo pip install pillow')

prior_image = None

def compare(img1, img2):
    im = [None, None] # to hold two arrays
    for i, f in enumerate([img1, img2]):
        im[i] = (np.array(f.convert('L')            # convert to grayscale using PIL
                          .resize((32,32), resample=Image.BICUBIC) # reduce size and smooth a bit using PIL
                          .filter(ImageFilter.GaussianBlur(radius=2))) # blur using PIL
        ).astype(np.int)   # convert from unsigned bytes to signed int using numpy
    return np.abs(im[0] - im[1]).sum()

def detect_motion(camera):
    global prior_image
    stream = io.BytesIO()
    camera.capture(stream, format='jpeg', use_video_port=True)
    stream.seek(0)
    if prior_image is None:
        prior_image = Image.open(stream)
        return False
    else:
        current_image = Image.open(stream)
        # Compare current_image to prior_image to detect motion. This is
        # left as an exercise for the reader!
        result = compare(current_image, prior_image)
        print(result)
        # Once motion detection is done, make the prior image the current
        prior_image = current_image
        if result > 2000:
            return True
        return False

def main():
    with PiCamera() as camera:
        count = 0
        camera.resolution = (1280, 720)
        stream = PiCameraCircularIO(camera, seconds=10)
        camera.start_recording(stream, format='h264')
        try:
            while True:
                camera.wait_recording(1)
                if detect_motion(camera):
                    print('Motion detected!')
                    # As soon as we detect motion, split the recording to
                    # record the frames "after" motion
                    camera.split_recording('after_{:04d}.h264'.format(count))
                    # Write the 10 seconds "before" motion to disk as well
                    stream.copy_to('before_{:04d}.h264'.format(count), seconds=10)
                    stream.clear()
                    # Wait until motion is no longer detected, then split
                    # recording back to the in-memory circular buffer
                    while detect_motion(camera):
                        camera.wait_recording(1)
                    print('Motion stopped!')
                    camera.split_recording(stream)
                    count += 1
        finally:
            camera.stop_recording()

if __name__ == "__main__":
    main()

