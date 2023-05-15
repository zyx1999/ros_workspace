#!/usr/bin/env python
# simple script to publish a image from a file.
import rospy
import rospkg
import cv2
from grid_map_demos.srv import *

def handle_request(req):
    res = img2PointCloudResponse()
    global imagePath

    img = cv2.imread(imagePath, cv2.IMREAD_UNCHANGED)
    img = cv2.resize(img, (120,120))
#    print img.shape
#    print img.size
#    print img.dtype.itemsize

    rosimage = sensor_msgs.msg.Image()
    if img.dtype.itemsize == 2:
       if len(img.shape) == 3:
           if img.shape[2] == 3:
               rosimage.encoding = 'bgr16'
           if img.shape[2] == 4:
           	   rosimage.encoding = 'bgra16'
       else:
           rosimage.encoding = 'mono16'
    if img.dtype.itemsize == 1:
       if len(img.shape) == 3:
           if img.shape[2] == 3:
               rosimage.encoding = 'bgr8'
           if img.shape[2] == 4:
           	   rosimage.encoding = 'bgra8'
       else:
           rosimage.encoding = 'mono8'
#    print "Encoding: ", rosimage.encoding

    rosimage.width = img.shape[1]
    rosimage.height = img.shape[0]
    rosimage.step = img.strides[0]
    rosimage.data = img.tostring()
    rosimage.header.stamp = rospy.Time.now()
    rosimage.header.frame_id = 'map'

    res.img = rosimage
    return res

#Main function initializes node and subscribers and starts the ROS loop
def main_program():
    global imagePath
    rospack = rospkg.RosPack()
    rospy.init_node('img2PointCloudServer')
    imagePath = rospy.get_param('~image_path')
    s = rospy.Service('img2PC', img2PointCloud, handle_request)
    print("Img2PointCloud Server Ready...")
    rospy.spin()

if __name__ == '__main__':
    try:
        main_program()
    except rospy.ROSInterruptException: pass
