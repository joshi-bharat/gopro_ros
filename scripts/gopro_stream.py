#!/usr/bin/env python

import rospy
import cv2

from gopro_cam.gopro import GoPro


if __name__ == '__main__':
    rospy.init_node('gopro_stream_node')

    if(not rospy.has_param('~ip_address')):
        rospy.logfatal('Need to provide ip address of the camera')

    ip_address = rospy.get_param('~ip_address')
    gopro_camera = GoPro(ip_address)
    gopro_camera.startWebcam('480')

    cap = cv2.VideoCapture('/dev/video2', cv2.CAP_V4L2)

    while(not rospy.is_shutdown()):
        rospy.spin()

    # gopro_camera.stopWebcam()
