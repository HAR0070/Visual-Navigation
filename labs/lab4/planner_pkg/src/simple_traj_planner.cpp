#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>

class SimpleTrajPlanner : public rclcpp::Node {
public:
  SimpleTrajPlanner() : Node("simple_traj_planner") {
    // Subscriber for desired trajectory vertices
    desire_traj_vertices_sub_ =
        this->create_subscription<geometry_msgs::msg::PoseArray>(
            "desired_traj_vertices", 10,
            std::bind(&SimpleTrajPlanner::trajCB, this, std::placeholders::_1));

    // Publisher for desired states for controller
    desired_state_pub_ = this->create_publisher<
        trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>("desired_state", 1);

    // Transform broadcaster
    br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr
      desire_traj_vertices_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::
      SharedPtr desired_state_pub_;
  
  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;

  void trajCB(const geometry_msgs::msg::PoseArray::SharedPtr traj_msg) {
    // Sanity check for traj_msg size
    if (traj_msg->poses.empty()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Empty trajectory vertices msg.");
      return;
    }


    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 0 |  16.485 - Fall 2020  - Lab 4 coding assignment  (10 pts)
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //
    //  As a simple warm up exercise before we get to the actual 'real deal',
    //  let's just make our quadcopter fly to the first gate in the course.
    //  In this section:
    //   1. Extract the first vertex of the trajectory
    //   2. Set the acceleration and velocities to zero
    //   3. Publish the desired MultiDOFJointTrajectoryPoint
    //   4. Create and publish TF transform of the desired pose
    // ~~~~ begin solution

    geometry_msgs::msg::Transform transform;
    geometry_msgs::msg::Twist zero_twist;
    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint msg; 

    transform.translation.x = traj_msg->poses[0].position.x ;
    transform.translation.y = traj_msg->poses[0].position.y ;
    transform.translation.z = traj_msg->poses[0].position.z ;
    transform.rotation = traj_msg->poses[0].orientation ;   // both are quaternion

    msg.transforms.push_back(transform); 
    msg.velocities.push_back(zero_twist); 
    msg.accelerations.push_back(zero_twist);

    desired_state_pub_->publish(msg);
    
    geometry_msgs::msg::TransformStamped origin;
    geometry_msgs::msg::Transform tf_trans;
    tf_trans.rotation.w = 1;  // identity quaternion 

    origin.header.frame_id = "map" ;
    origin.child_frame_id = "drone" ;
    origin.transform = tf_trans; 
    br_->sendTransform(origin);

    // ~~~~ end solution
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //                                 end part 0
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Logging to terminal
    std::stringstream ss;
    ss << "Trajectory Position"
       << " x:" << transform.translation.x << " y:" << transform.translation.y
       << " z:" << transform.translation.z;
    RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleTrajPlanner>());
  rclcpp::shutdown();
  return 0;
}
