#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from rospy.topics import Publisher
from sensor_msgs.msg import Image

from gopro_cam.gopro import GoPro

if __name__ == '__main__':
    rospy.init_node('gopro_stream_node')

    if(not rospy.has_param('~ip_address')):
        rospy.logfatal('Need to provide ip address of the camera')
        exit(1)

    if(not rospy.has_param('~camera_name')):
        rospy.logfatal('Need to provide camera name')
        exit(1)

    ip_address = rospy.get_param('~ip_address')
    camera_name = rospy.get_param('~camera_name')
    resolution = rospy.get_param('~resolution', '720')
    device = rospy.get_param('~device', '/dev/video3')

    fov = rospy.get_param('~fov', 'wide')
    fov_command = '0'

    if(fov == 'medium'):
        fov_command = '4'
    elif(fov == 'narrow'):
        fov_command = '6'

    cv_backend = cv2.CAP_V4L2
    if(not '/dev/video' in device):
        device = device + '?overrun_nonfatal=1&fifo_size=50000000&buffer_size=425984'
        cv_backend = cv2.CAP_FFMPEG

    total_tries = 5
    gopro_intialized = False

    cvbridge = CvBridge()

    for i in range(5):
        gopro_camera = GoPro(ip_address)
        gopro_camera.webcamFOV(fov_command)
        gopro_camera.startWebcam(resolution)
        cap = cv2.VideoCapture(device, cv_backend)

        if(not cap.isOpened()):
            rospy.logfatal('Could not open video device')
        else:
            gopro_intialized = True
            break

        rospy.sleep(5)

    if(not gopro_intialized):
        rospy.logfatal('Could not initialize gopro camera')
        exit(1)

    image_topic = '/{}/image_raw'.format(camera_name)
    pub = rospy.Publisher(image_topic, Image, queue_size=10)

    while(not rospy.is_shutdown()):

        ret, frame = cap.read()

        if not ret:
            print('frame empty')
            continue

        try:
            img_msg = cvbridge.cv2_to_imgmsg(frame, encoding='rgb8')
        except CvBridgeError as e:
            print(e)
            continue
        img_msg.header.stamp = rospy.Time.now()
        img_msg.header.frame_id = camera_name

        pub.publish(img_msg)

    gopro_camera.stopWebcam()
