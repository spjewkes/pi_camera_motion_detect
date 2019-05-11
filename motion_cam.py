#!/usr/bin/env python3

from datetime import datetime, timedelta
import io
import numpy as np
import os

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
    global motion_enabled
    global motion_time

    stream = io.BytesIO()
    camera.capture(stream, format='jpeg', use_video_port=True)
    stream.seek(0)

    if prior_image is None:
        prior_image = Image.open(stream)
        motion_enabled = False
    else:
        current_image = Image.open(stream)
        # Compare current_image to prior_image to detect motion. This is
        # left as an exercise for the reader!
        result = compare(current_image, prior_image)
        print(result)
        # Once motion detection is done, make the prior image the current
        prior_image = current_image
        if result > 1800:
            # This resets the timer even if motion detect is already enabled. Giving a set
            # time with no motion before stopping the camera
            motion_time = datetime.now() + timedelta(seconds=5)
            motion_enabled = True
        elif motion_enabled and result < 1000 and datetime.now() > motion_time:
            motion_enabled = False

    return motion_enabled

def user_freespace_gb():
    """
    Returns the amount of free space user space in GB
    """
    fd = os.open('.', os.O_RDONLY | os.O_DIRECTORY)
    info = os.fstatvfs(fd)
    os.close(fd)
    return (info.f_bsize * info.f_bavail) / (1024 * 1024 * 1024)

def main():
    with PiCamera() as camera:
        count = 0
        camera.resolution = (1280, 720)
        stream = PiCameraCircularIO(camera, seconds=10)
        camera.start_recording(stream, format='h264')
        try:
            camera.wait_recording(3)
            while True:
                camera.wait_recording(0)
                if detect_motion(camera):
                    print('Motion detected!')
                    # As soon as we detect motion, split the recording to
                    # record the frames "after" motion
                    camera.split_recording('{}_after_{:04d}.h264'.format(datetime.now().isoformat('_'), count))
                    # Write the 10 seconds "before" motion to disk as well
                    # stream.copy_to('{}_before_{:04d}.h264'.format(datetime.now().isoformat('_'), count), seconds=5)
                    stream.clear()
                    # Wait until motion is no longer detected, then split
                    # recording back to the in-memory circular buffer
                    while detect_motion(camera):
                        camera.wait_recording(0)
                    print('Motion stopped!')
                    camera.split_recording(stream)
                    count += 1

                if user_freespace_gb() < 0.5:
                    print("CLosing program to avoid using all free space.")
                    print("There is {} GB remaining.".format(user_freespace_gb()))
                    break
                        
        finally:
            camera.stop_recording()

if __name__ == "__main__":
    main()

