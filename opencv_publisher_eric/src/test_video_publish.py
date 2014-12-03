#!/usr/bin/env python
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageConverter(object):
  def __init__(self):
    self.image_pub = rospy.Publisher("video_topic", Image, queue_size=10)
    self.bridge = CvBridge()
    self.cap = cv2.VideoCapture(0)

  def read_frame(self):
    return_val, frame = self.cap.read()
    height=frame.shape[0]
    width =frame.shape[1]
    cv2.circle(frame,(width/2,height/2),4,(0,0,255),1)
    cv2.imshow('frame', frame)
    return frame
    
  def publish_ros_img(self, frame):
    try:
      rosimg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
      self.image_pub.publish(rosimg)
    except CvBridgeError, e:
      print e

def main(args):
  ic = ImageConverter()
  rospy.init_node('video_publisher', anonymous=True)
  rospy.loginfo("Press 'q' to quit WHEN FOCUSED ON CV WINDOW")
  try:
    while(True):
      frame = ic.read_frame()
      if not frame == None:
        ic.publish_ros_img(frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
          break
      else:
        rospy.loginfo('You read an empty image - check your video device')

  except KeyboardInterrupt:
    print "Shutting down"
  ic.cap.release()
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
