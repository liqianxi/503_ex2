#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, CameraInfo, Image



"""
Reference:

I looked at my teammate Jihoon Og's code.

https://github.com/jihoonog/CMPUT-503-Exercise-2/blob/v2/packages/exercise-2/src/my_camera_subscriber_node.py



"""

class MyCameraSubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyCameraSubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        
        # construct publisher
        self.sub = rospy.Subscriber(f'/{os.environ["VEHICLE_NAME"]}/camera_node/camera_info', CameraInfo, self.callback)
        
        # Subscriber to get the compressed image from the camera
        self.sub_2 = rospy.Subscriber(f'/{os.environ["VEHICLE_NAME"]}/camera_node/image/compressed', CompressedImage, self.camera_image_callback)
        
        # Publish the compressed image as a custom topic
        self.pub = rospy.Publisher(f'my_camera_image/compressed', CompressedImage, queue_size=10)
        self.image_data = None


    def callback(self, data):
        rospy.loginfo("Width %d, height %d", data.width, data.height)
    
    def camera_image_callback(self, data):
        # Must do a deep copy and not a copy by reference other wise it won't work
        self.image_data = CompressedImage()
        self.image_data.format = "jpeg"
        self.image_data.data = data.data
        self.image_data.header.stamp = rospy.Time.now()

    def camera_image_pub(self):
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            if self.image_data is not None:
                self.pub.publish(self.image_data)
            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = MyCameraSubscriberNode(node_name='my_cam_subscriber_node')
    node.camera_image_pub()
    # keep spinning
    rospy.spin()