#!/usr/bin/env python
# 4/1/2015
# Charles O. Goddard

import tf
import cffi
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import apriltag


class TagFinder(object):
	def __init__(self):
		self.broadcaster = tf.TransformBroadcaster()
		self.bridge = CvBridge()
		self.detector = apriltag.Detector(apriltag.tag36h11)
		rospy.Subscriber("/camera/image_rect", Image, self.process_image)
		print 123

	def process_image(self, img):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(img, "mono8")
		except CvBridgeError, e:
			print e
			return

		res = self.detector.detect(cv_image.tostring())
		for detection in res:
			print detection.id

if __name__ == '__main__':
	rospy.init_node('TagFinder')
	t = TagFinder()
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		print 'So die I.'
