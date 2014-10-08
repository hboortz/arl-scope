#!/usr/bin/env python
import sys
import rospy
import rospkg
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os.path
import numpy as np


class FiducialFinder(object):
    def __init__(self, model, kp, des):
        self.image_sub = rospy.Subscriber("video_topic",
                                          Image,
                                          self.img_received)
        self.image_pub = rospy.Publisher("video_annotated",
                                         Image,
                                         queue_size=10)

        self.model = model
        self.model_keypoints = kp
        self.model_descriptors = des

        self.bridge = CvBridge()
        self.sift = cv2.SIFT()
        self.flann = cv2.FlannBasedMatcher({'algorithm': 0, 'trees': 5},
                                           {'checks': 50})

    def img_received(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imshow('img', img)
        mask = None
        data_keypoints, data_descriptors = self.sift.detectAndCompute(img, mask)

        matches = self.flann.knnMatch(self.model_descriptors,
                                      data_descriptors, k=2)

        good = [m for (m, n) in matches if self.lowes_ratio_test(m, n)]

        if len(good) > 10:  # 10 is arbitrary
            H, mask = self.find_homography(data_keypoints, good)
            self.draw_outline(img, H)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

    def lowes_ratio_test(self, m, n):
        return m.distance < 0.7 * n.distance

    def find_homography(self, data_keypoints, matches):
        src_pts = np.float32([self.model_keypoints[m.queryIdx].pt
                              for m in matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([data_keypoints[m.trainIdx].pt
                              for m in matches]).reshape(-1, 1, 2)
        return cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

    def draw_outline(self, img, M):
        h, w, _ = self.model.shape  # discard color band with _
        corners = np.float32([[0, 0], [0, h-1], [w-1, h-1], [w-1, 0]])
        corners = corners.reshape(-1, 1, 2)
        dest = cv2.perspectiveTransform(corners, M)

        cv2.polylines(img, [np.int32(dest)], True, 255, 3)


def get_default_fiducial():
    pack = rospkg.RosPack()
    package_path = pack.get_path("fiducial_finder")
    return os.path.join(package_path, "src", "fiducial.jpg")


def main(args):
    fiducial_img = get_default_fiducial()
    if len(args) >= 1:
        fiducial_img = args[0]

    model = cv2.imread(fiducial_img)
    sift = cv2.SIFT()
    kp, des = sift.detectAndCompute(model, None)

    ff = FiducialFinder(model, kp, des)
    rospy.init_node('video_subscriber', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__ == '__main__':
    main(sys.argv[1:])
