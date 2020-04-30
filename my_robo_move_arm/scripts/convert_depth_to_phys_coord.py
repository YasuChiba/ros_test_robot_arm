#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import pyrealsense2

#x→右方向  y→下方向　のピクセル int
#depth (meter)   float
#cameraInfo: sensor_msgs/CameraInfo
def convert_depth_to_phys_coord_using_realsense(x, y, depth, cameraInfo):

  _intrinsics = pyrealsense2.intrinsics()
  _intrinsics.width = cameraInfo.width
  _intrinsics.height = cameraInfo.height
  _intrinsics.ppx = cameraInfo.K[2]
  _intrinsics.ppy = cameraInfo.K[5]
  _intrinsics.fx = cameraInfo.K[0]
  _intrinsics.fy = cameraInfo.K[4]
  #_intrinsics.model = cameraInfo.distortion_model
  _intrinsics.model  = pyrealsense2.distortion.none 	

  _intrinsics.coeffs = [i for i in cameraInfo.D]

  result = pyrealsense2.rs2_deproject_pixel_to_point(_intrinsics, [x, y], depth)
  #print(result)

  #result[0]: right
  #result[1]: down
  #result[2]: forward
  return result[2], -result[0], -result[1]

