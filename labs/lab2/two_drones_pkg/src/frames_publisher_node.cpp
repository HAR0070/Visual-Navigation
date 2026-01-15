#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>

class FramesPublisherNode : public rclcpp::Node {
  rclcpp::Time startup_time;

  rclcpp::TimerBase::SharedPtr heartbeat;

  // TODO: Declare a
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  // These 3 are member functions - defining some functions that is going to be detailed later

 public:
  FramesPublisherNode() : Node("frames_publisher_node") {
    // NOTE: This method is run once, when the node is launched.

    // TODO: Instantiate the Transform Broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // ....

    startup_time = now();

    heartbeat = this->create_timer(
                             std::chrono::duration<double>(0.02),
                             std::bind(&FramesPublisherNode::onPublish, this));
    heartbeat->reset();
  }

  void onPublish() {
    // NOTE: This method is called at 50Hz, due to the timer created on line 25.

    //
    // 1. TODO: Compute time elapsed in seconds since the node has been started
    //    i.e. the time elapsed since startup_time (defined on line 13)
    //   HINTS:
    //   - Get the current time with this->now()
    //   - use the - operator between the current time and startup_time
    //   - convert the resulting Duration to seconds, store result into a double
    double time_t = (this->now() - startup_time).seconds() ;

    // Here we declare two geometry_msgs::msg::TransformStamped objects, which
    // need to be populated
    geometry_msgs::msg::TransformStamped world_T_av1;
    geometry_msgs::msg::TransformStamped world_T_av2;

    // 2. TODO: Populate the two transforms for the AVs, using the variable "time"
    //    computed above. Specifically:
    //     - world_T_av1 should have origin in [cos(time), sin(time), 0.0] and
    //       rotation such that:
    //        i) its y axis stays tangent to the trajectory and
    //       ii) the z vector stays parallel to that of the world frame
    // -----------------
    //
    // Setting yaw = t - - will make the object tangential to the trajectory - ie drone has to rotate to be tangential to circle
    //
    //--------------
    world_T_av1.header.stamp = this->now();
    world_T_av1.header.frame_id = "world";   // parent frame id
    world_T_av1.child_frame_id = "av1";       // id of object i am defining the tf for  - how is av1 wrt world - is what is defined here
    std::vector<float> av1_translation(3);
    tf2::Quaternion q;
    double yaw = time_t;
    q.setRPY(0,0,yaw);
    av1_translation = {std::cos(time_t),std::sin(time_t),0} ;

    world_T_av1.transform.translation.x = av1_translation[0];
    world_T_av1.transform.translation.y = av1_translation[1];
    world_T_av1.transform.translation.z = av1_translation[2];
    world_T_av1.transform.rotation.x = q.x();
    world_T_av1.transform.rotation.y = q.y();
    world_T_av1.transform.rotation.z = q.z();
    world_T_av1.transform.rotation.w = q.w();
    //
    //     - world_T_av2 shoud have origin in [sin(time), 0.0, cos(2*time)], the
    //       rotation is irrelevant to our purpose.
    //    NOTE: world_T_av1's orientation is crucial for the rest fo the
    //    assignment,
    //          make sure you get it right
    //
    world_T_av2.header.stamp = this->now();
    world_T_av2.header.frame_id = "world";
    world_T_av2.child_frame_id = "av2";

    std::vector<float> av2_translation(3);
    tf2::Quaternion q2;

    av2_translation = {std::sin(time_t),0,std::cos(2*time_t)};
    q2.setRPY(0,0,0);

    world_T_av2.transform.translation.x = av2_translation[0];
    world_T_av2.transform.translation.y = av2_translation[1];
    world_T_av2.transform.translation.z = av2_translation[2];
    world_T_av2.transform.rotation.x = q2.x();
    world_T_av2.transform.rotation.y = q2.y();
    world_T_av2.transform.rotation.z = q2.z();
    world_T_av2.transform.rotation.w = q2.w();
    //    HINTS:
    //    - check out the ROS tf2 Tutorials:
    //    https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html
    //    - consider the setRPY method on a tf2::Quaternion for world_T_av1

    // ...

    // 3. TODO: Publish the transforms, namely:
    //     - world_T_av1 with frame_id "world", child_frame_id "av1"
    //     - world_T_av2 with frame_id "world", child_frame_id "av2"
    //    HINTS:
    //         1. you need to define a
    //         std::unique_ptr<tf2_ros::TransformBroadcaster> as a member of the
    //            node class (line 18) and use its sendTransform method below
    tf_broadcaster->sendTransform(world_T_av1);
    tf_broadcaster->sendTransform(world_T_av2);
    //         2. the frame names are crucial for the rest of the assignment,
    //            make sure they are as specified, "av1", "av2" and "world"

    // ...
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramesPublisherNode>());
  return 0;
}
