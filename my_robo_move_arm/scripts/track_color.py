#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Twist, Vector3, Pose, Point, Quaternion
from visualization_msgs.msg import Marker
import cv2
from cv_bridge import CvBridge
import numpy as np
import subprocess
import tf
import math

import moveit_commander



from convert_depth_to_phys_coord import convert_depth_to_phys_coord_using_realsense
from find_specific_color import find_specific_color



def createMarker():
    marker_data = Marker()
    marker_data.header.frame_id = "base_link"
    marker_data.header.stamp = rospy.Time.now()

    marker_data.ns = "basic_shapes"
    marker_data.id = 0

    marker_data.action = Marker.ADD
    marker_data.pose.orientation.x=0.0
    marker_data.pose.orientation.y=0.0
    marker_data.pose.orientation.z=1.0
    marker_data.pose.orientation.w=0.0
    marker_data.color.r = 1.0
    marker_data.color.g = 0.0
    marker_data.color.b = 0.0
    marker_data.color.a = 1.0

    marker_data.scale.x = 0.05
    marker_data.scale.y = 0.05
    marker_data.scale.z = 0.05
    marker_data.lifetime = rospy.Duration(1)
    marker_data.type = 1
    return marker_data



class TrackColor:
  def __init__(self):
    self.bridge = CvBridge()
    self.color_image = None
    self.depth_array = None
    self.cameraInfo = None

    rospy.init_node('calc_coord_node')

    rospy.Subscriber("/d435/image_raw", Image, self.image_listener)
    rospy.Subscriber("/d435/depth/image_raw", Image, self.depth_image_listener)
    rospy.Subscriber("/d435/camera_info", CameraInfo, self.cameraInfo_listener)

    self.tf_listener = tf.TransformListener()


    self.pub_position_marker = rospy.Publisher("position_marker", Marker, queue_size=10)
    self.pub_detected_position_image = rospy.Publisher("/detected_position_image", Image, queue_size=1)


    r = rospy.Rate(1)
    while not rospy.is_shutdown():

      self.do_something()

      r.sleep()



  def do_something(self):
    if self.cameraInfo is None:
      return
    if self.depth_array is None:
      return
    if self.color_image is None:
      return
      

    position = find_specific_color(self.color_image)

    if position is not None:
      copy_im = self.color_image.copy()
      cv2.circle(copy_im, position, 10, (0, 0, 255), -1)
      msg = self.bridge.cv2_to_imgmsg(copy_im, encoding="bgr8")
      self.pub_detected_position_image.publish(msg)
      #print(position)
      #print(self.depth_array[position[1]][position[0]])
    else:
      return

    marker_data = createMarker()
    marker_data.id = 1
    X, Y, Z = convert_depth_to_phys_coord_using_realsense(position[0],position[1], self.depth_array[position[1]][position[0]], self.cameraInfo)
    p = PointStamped()
    p.header.frame_id = "d435_link"
    p.header.stamp =rospy.Time(0)
    p.point.x= X
    p.point.y = Y
    p.point.z = Z
    p = self.tf_listener.transformPoint("base_link",p)
    marker_data.pose.position = p.point
    self.pub_position_marker.publish(marker_data)


    #
    #
    #move arm
    #moveit_commander.roscpp_initialize()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_pose_reference_frame("base_link")
    orientation = Quaternion(0,0.707106,0,0.707106)
    pose = Pose(p.point, orientation)
    rospy.loginfo('[move_to_pose] Moving arm to ' + str(pose))
    arm.set_pose_target(pose)
    arm.go()



  def cameraInfo_listener(self, data):
    self.cameraInfo = data

  def image_listener(self, data):
    try:
      image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.color_image = image
      
    except Exception as err:
          print(err)

  def depth_image_listener(self,data):
      try:
          
          depth_image = self.bridge.imgmsg_to_cv2(data, 'passthrough')
          depth_array = np.array(depth_image, dtype=np.float32)
          self.depth_array = depth_array
          #self.depth_array = depth_array / 1000.0

      except Exception as err:
          print(err)



if __name__ == '__main__':
    try:
        TrackColor()
    except rospy.ROSInterruptException:
        pass
