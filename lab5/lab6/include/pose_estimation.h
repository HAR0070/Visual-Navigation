/*
 * @file pose_estimation.h
 * @brief Include file for pose_estimation.cpp
 * Estimates the pose from frame to frame.
 * (TODO) Feel free to use a header file to organize your solution.
 */

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <memory>

#include <Eigen/Eigen>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <stdio.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Eigen>
#include <fstream>
#include <functional>
#include <geometry_msgs/msg/detail/transform__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <iostream>
#include <memory>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/detail/camera_info__struct.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "lab_5/akaze_feature_tracker.h"
#include "lab_5/brisk_feature_tracker.h"
#include "lab_5/feature_tracker.h"
#include "lab_5/orb_feature_tracker.h"
#include "lab_5/sift_feature_tracker.h"