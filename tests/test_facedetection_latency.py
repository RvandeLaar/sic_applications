import queue
import time

import cv2
from sic_framework.core import utils_cv2
from sic_framework.core.message_python2 import (
    BoundingBoxesMessage,
    CompressedImageMessage,
)
from sic_framework.devices.common_desktop.desktop_camera import DesktopCameraConf
from sic_framework.devices.desktop import Desktop
from sic_framework.services.face_detection.face_detection import FaceDetection

from utils.redis_latency_test import RedisLatencyTester

# CUSTOM FACE DETECTION EXAMPLE
# from custom_components.custom_face_detection import CustomFaceDetection

"""
This demo recognizes faces from your webcam and displays the result on your laptop.

IMPORTANT
face-detection service needs to be running:
1. run-face-detection
"""

imgs_buffer = queue.Queue(maxsize=1)
faces_buffer = queue.Queue(maxsize=1)


def on_image(image_message: CompressedImageMessage):
    # will print out average end-to-end latency after 100 messages
    latency_tester.end_to_end_latency_test(image_message._timestamp, average_count=100)
    imgs_buffer.put(image_message.image)


def on_faces(message: BoundingBoxesMessage):
    faces_buffer.put(message.bboxes)


# Get the internet speed (if you want to test it)
latency_tester = RedisLatencyTester()
download_speed, upload_speed = latency_tester.perform_speed_test()
print("Download Speed: {:.2f} Mbps".format(download_speed))
print("Upload Speed: {:.2f} Mbps".format(upload_speed))

# Create camera configuration using fx and fy to resize the image along x- and y-axis, and possibly flip image
conf = DesktopCameraConf(fx=1.0, fy=1.0, flip=1)

# Connect to the services
desktop = Desktop(camera_conf=conf)

face_rec = FaceDetection()
# CUSTOM FACE DETECTION EXAMPLE
# face_rec = CustomFaceDetection()

# Feed the camera images into the face recognition component
face_rec.connect(desktop.camera)

# Send back the outputs to this program
desktop.camera.register_callback(on_image)
face_rec.register_callback(on_faces)

# Initialize variables for calculating FPS
frame_count = 0
start_time = time.time()

while True:
    img = imgs_buffer.get()
    faces = faces_buffer.get()

    for face in faces:
        utils_cv2.draw_bbox_on_image(face, img)

    # Calculate FPS
    frame_count += 1
    elapsed_time = time.time() - start_time
    if elapsed_time >= 1.0:  # update FPS every second
        # it depends on the camera you are using, the camera (Integrated_Webcam_HD) I am using is capped at 30fps
        # you can use cmd "v4l2-ctl -d /dev/video0 --list-formats-ext"
        # to check the supported formats, resolutions, and frame rates for your camera
        fps = frame_count / elapsed_time
        print(f"FPS: {fps:.2f}")
        # reset the variables
        frame_count = 0
        start_time = time.time()

    cv2.imshow("", img)
    cv2.waitKey(1)
