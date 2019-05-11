#!/usr/bin/env python3

from datetime import datetime, timedelta
import io
import numpy as np
import os
import argparse

try:
    from picamera import PiCamera, PiCameraCircularIO
except ImportError:
    exit('This script requires the picamera module\nInstall with: sudo pip install picamera')

try:
    from PIL import Image, ImageFilter
except ImportError:
    exit('This script requires the pillow module\nInstall with: sudo pip install pillow')

def compare(img1, img2):
    im = [None, None] # to hold two arrays
    for i, f in enumerate([img1, img2]):
        # im[i] = (np.array(f.convert('L')            # convert to grayscale using PIL
        #                   .resize((32,32), resample=Image.BICUBIC) # reduce size and smooth a bit using PIL
        #                   .filter(ImageFilter.GaussianBlur(radius=2))) # blur using PIL
        # ).astype(np.int)   # convert from unsigned bytes to signed int using numpy
        im[i] = (np.array(f.convert('L')            # convert to grayscale using PIL
                          .resize((256,256), resample=Image.BICUBIC) # reduce size and smooth a bit using PIL
                          .filter(ImageFilter.GaussianBlur(radius=2))) # blur using PIL
        ).astype(np.int)   # convert from unsigned bytes to signed int using numpy
    return np.abs(im[0] - im[1]).sum()

def user_freespace_gb():
    """
    Returns the amount of free space user space in GB
    """
    fd = os.open('.', os.O_RDONLY | os.O_DIRECTORY)
    info = os.fstatvfs(fd)
    os.close(fd)
    return (info.f_bsize * info.f_bavail) / (1024 * 1024 * 1024)

class MotionDetect(object):
    def __init__(self, args):
        self.dryrun = args.dryrun

        # Motion detection values that control starting and stoping recording
        self.start_threshold = args.start
        self.stop_threshold = args.stop
        self.prior_image = None
        
        # Stats for motion detection value
        self.result_max = 0
        self.result_min = float('inf')
        self.result_avg = None

        # Used to track when motion detection is on
        self.motion_time = datetime.now()
        self.motion_enabled = False
        
    def run(self):
        with PiCamera() as camera:
            count = 0
            camera.resolution = (1280, 720)
            stream = PiCameraCircularIO(camera, seconds=10)
            camera.start_recording(stream, format='h264')
            try:
                camera.wait_recording(3)
                while True:
                    camera.wait_recording(0)
                    if self.detect_motion(camera):
                        print('Motion detected!')

                        if not self.dryrun:
                            # As soon as we detect motion, split the recording to
                            # record the frames "after" motion
                            camera.split_recording('{}_capture_{:04d}.h264'.format(datetime.now().isoformat('_'), count))
                            # Write the 10 seconds "before" motion to disk as well
                            stream.copy_to('{}_before_{:04d}.h264'.format(datetime.now().isoformat('_'), count), seconds=10)

                        stream.clear()

                        # Wait until motion is no longer detected, then split
                        # recording back to the in-memory circular buffer
                        while self.detect_motion(camera):
                            camera.wait_recording(0)

                        print('Motion stopped!')
                        camera.split_recording(stream)

                        count += 1

                    if user_freespace_gb() < 0.5:
                        print("CLosing program to avoid using all free space.")
                        print("There is {} GB remaining.".format(user_freespace_gb()))
                        break

            except KeyboardInterrupt:
                print("Stopped")
                        
            finally:
                camera.stop_recording()

                print("Motion detection values:")
                print("\tMin: {}".format(self.result_min))
                print("\tMax: {}".format(self.result_max))
                print("\tAvg: {}".format(self.result_avg))

    def detect_motion(self, camera):
        # motion_start_threshold = 1800
        # motion_stop_threshold = 1000
        # motion_start_threshold = 50000
        # motion_stop_threshold = 28000

        stream = io.BytesIO()
        camera.capture(stream, format='jpeg', use_video_port=True)
        stream.seek(0)

        if self.prior_image is None:
            self.prior_image = Image.open(stream)
            self.motion_enabled = False
        else:
            current_image = Image.open(stream)
            # Compare current_image to prior_image to detect motion. This is
            # left as an exercise for the reader!
            result = compare(current_image, self.prior_image)
            self.store_result(result)
            # Once motion detection is done, make the prior image the current
            self.prior_image = current_image
            if result > self.start_threshold:
                # This resets the timer even if motion detect is already enabled. Giving a set
                # time with no motion before stopping the camera
                self.motion_time = datetime.now() + timedelta(seconds=5)
                self.motion_enabled = True
            elif self.motion_enabled and result < self.stop_threshold and datetime.now() > self.motion_time:
                self.motion_enabled = False

        return self.motion_enabled

    def store_result(self, result):
        print(result)
        self.result_max = max(self.result_max, result)
        self.result_min = min(self.result_min, result)
        if self.result_avg:
            self.result_avg = (self.result_avg + result) / 2.0
        else:
            self.result_avg = result

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Capture motion detection as video')
    parser.add_argument('--dryrun', action='store_true', help="Allows observing motion detection values without recording")
    parser.add_argument('--start', type=int, default=50000, help="The start motion detection value (when not recording)")
    parser.add_argument('--stop', type=int, default=28000, help="The end motion detection value (when recording)")

    args = parser.parse_args()

    MotionDetect(args).run()

