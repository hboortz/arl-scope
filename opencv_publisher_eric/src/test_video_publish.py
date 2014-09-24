#!/usr/bin/env python
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("video_topic", Image, queue_size=10)

    # cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    # self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)

    self.cap = cv2.VideoCapture(1)
    # self.cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 400);
    # self.cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 400);
    # self.cap.set(cv2.cv.CV_CAP_PROP_FPS, 60);

  # def callback(self,data):
  #   try:
  #     cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
  #   except CvBridgeError, e:
  #     print e

  #   (rows,cols,channels) = cv_image.shape
  #   if cols > 60 and rows > 60 :
  #     cv2.circle(cv_image, (50,50), 10, 255)

  #   cv2.imshow("Image window", cv_image)
  #   cv2.waitKey(3)

  #   try:
  #     self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
  #   except CvBridgeError, e:
  #     print e

def main(args):
  ic = image_converter()
  rospy.init_node('video_publisher', anonymous=True)

  rospy.loginfo("Press 'q' to quit WHEN FOCUSED ON CV WINDOW")

  try:
    while(True):
      # Capture frame-by-frame
      ret, frame = ic.cap.read()

      # Our operations on the frame come here
      gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

      # Display the resulting frame
      cv2.imshow('frame',frame)
      cv2.imshow('gray',gray)
      if cv2.waitKey(1) & 0xFF == ord('q'):
        break

      try:
        ic.image_pub.publish(ic.bridge.cv2_to_imgmsg(frame, "bgr8"))
      except CvBridgeError, e:
        print e

  except KeyboardInterrupt:
    print "Shutting down"

  ic.cap.release()
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)