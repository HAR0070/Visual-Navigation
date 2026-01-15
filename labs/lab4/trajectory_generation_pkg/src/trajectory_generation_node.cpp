#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>  // for nonlinear optimization
#include <mav_trajectory_generation/trajectory.h>

#include <eigen3/Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>   // this has tf2::getyaw

class WaypointFollower : public rclcpp::Node {

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr currentStateSub;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr poseArraySub;

  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::
      SharedPtr desiredStatePub;

  // Current state
  Eigen::Vector3d x; // current position of the UAV's c.o.m. in the world frame
  bool have_state = false;  // to check if we have received curr position yet
  double yawd = 0.0;

  rclcpp::TimerBase::SharedPtr desiredStateTimer;

  rclcpp::Time trajectoryStartTime;
  mav_trajectory_generation::Trajectory trajectory;
  mav_trajectory_generation::Trajectory yaw_trajectory;

  void onCurrentState(nav_msgs::msg::Odometry const &cur_state) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 1.1 |  16.485 - Fall 2024  - Lab 4 coding assignment (5 pts)
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //
    //  Populate the variable x, which encodes the current world position of the
    //  UAV
    // ~~~~ begin solution
    x << cur_state.pose.pose.position.x,
        cur_state.pose.pose.position.y,
        cur_state.pose.pose.position.z;

    tf2::Quaternion q;
    tf2::fromMsg(cur_state.pose.pose.orientation , q);
    yawd = tf2::getYaw(q);
    have_state = true;

    // ~~~~ end solution
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //                                 end part 1.1
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }

  void generateOptimizedTrajectory(geometry_msgs::msg::PoseArray const &poseArray) {
    if (poseArray.poses.size() < 1) {
      RCLCPP_ERROR(get_logger(),
                  "Must have at least one pose to generate trajectory!");
      trajectory.clear();
      yaw_trajectory.clear();
      return;
    }

    if (!trajectory.empty())
      return;

    if (!have_state) {
      RCLCPP_WARN(get_logger(), "No current state yet, skipping trajectory generation");
      return;
    }
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 1.2 |  16.485 - Fall 2024  - Lab 4 coding assignment (35 pts)
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //
    //  We are using the mav_trajectory_generation library
    //  (https://github.com/ethz-asl/mav_trajectory_generation) to perform
    //  trajectory optimization given the waypoints (based on the position and
    //  orientation of the gates on the race course).
    //  We will be finding the trajectory for the position and the trajectory
    //  for the yaw in a decoupled manner.
    //  In this section:
    //  1. Fill in the correct number for D, the dimension we should apply to
    //  the solver to find the positional trajectory
    //  2. Correctly populate the Vertex::Vector structure below (vertices,
    //  yaw_vertices) using the position of the waypoints and the yaw of the
    //  waypoints respectively
    //
    //  Hints:
    //  1. Use vertex.addConstraint(POSITION, position) where position is of
    //  type Eigen::Vector3d to enforce a waypoint position.
    //  2. Use vertex.addConstraint(ORIENTATION, yaw) where yaw is a double
    //  to enforce a waypoint yaw.
    //  3. Remember angle wraps around 2 pi. Be careful!
    //  4. For the ending waypoint for position use .makeStartOrEnd as seen with
    //  the starting waypoint instead of .addConstraint as you would do for the
    //  other waypoints.
    //
    // ~~~~ begin solution

    // for access to SNAP
    // namespace is inside the function -- hence wont be seen outside -- so its fine
    using namespace mav_trajectory_generation::derivative_order;

    const int D = 3; // dimension of each position vertex in the trajectory
    const int yd = 1; // dimension of each yaw Vertex in trajectory
    mav_trajectory_generation::Vertex::Vector vertices;
    mav_trajectory_generation::Vertex::Vector yaw_vertices;
    // reserve the size of the array
    vertices.reserve(poseArray.poses.size()+1);
    yaw_vertices.reserve(poseArray.poses.size()+1);

    mav_trajectory_generation::Vertex start(D), end(D);
    mav_trajectory_generation::Vertex y_start(1), y_end(1);

    // begining
    // is current position
    start.makeStartOrEnd(x , SNAP);
    vertices.push_back(start);

    y_start.makeStartOrEnd(yawd , ANGULAR_VELOCITY);
    yaw_vertices.push_back(y_start);

    // Middle points
    for (int i = 0; i<poseArray.poses.size()-1; ++i){
      mav_trajectory_generation::Vertex middle(D);
      Eigen::Vector3d middle_v;
      middle_v << poseArray.poses[i].position.x,
                  poseArray.poses[i].position.y,
                  poseArray.poses[i].position.z;
      middle.addConstraint(POSITION ,middle_v);
      vertices.push_back(middle);

      mav_trajectory_generation::Vertex y_mid(1);
      tf2::Quaternion q_mid;
      tf2::fromMsg(poseArray.poses[i].orientation , q_mid);
      double yaw_mid = tf2::getYaw(q_mid);
      y_mid.addConstraint(ORIENTATION , yaw_mid);
      yaw_vertices.push_back(y_mid);
    }

    // End points
    Eigen::Vector3d end_v;
    end_v << poseArray.poses.back().position.x,
            poseArray.poses.back().position.y,
            poseArray.poses.back().position.z;
    end.makeStartOrEnd(end_v, SNAP);
    vertices.push_back(end);

    tf2::Quaternion q;
    tf2::fromMsg(poseArray.poses.back().orientation , q);
    double yaw_end = tf2::getYaw(q);
    y_end.makeStartOrEnd(yaw_end , ANGULAR_VELOCITY);
    yaw_vertices.push_back(y_end);

    // ~~~~ end solution
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //                                 end part 1.2
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // ============================================================
    // Estimate the time to complete each segment of the trajectory
    // ============================================================

    std::vector<double> segment_time;
    const double v_max = 8.0;
    const double a_max = 2;
    segment_time = estimateSegmentTimes(vertices, v_max, a_max);

    const double ang_v_max = 1.50;
    const double ang_a_max = 2.0;

    // Shouldn't do this -- if there are 2 different segment times - then yaw and pos - will mismatch
    // std::vector<double> segment_time_yaw;
    // segment_time_yaw = estimateSegmentTimes(yaw_vertices, ang_v_max, ang_a_max);

    // =====================================================
    // Solve for the optimized trajectory (linear optimizer)
    // =====================================================
    // // Position
    // const int N = 10;
    // mav_trajectory_generation::PolynomialOptimization<N> opt(D);
    // opt.setupFromVertices(vertices, segment_times, SNAP);
    // opt.solveLinear();

    // // Yaw
    // mav_trajectory_generation::PolynomialOptimization<N> yaw_opt(1);
    // yaw_opt.setupFromVertices(yaw_vertices, segment_times, SNAP);
    // yaw_opt.solveLinear();

    // =====================================================
    // Solve for the optimized trajectory (Non-linear optimizer)
    // =====================================================
    // Using default parameters 
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    parameters.max_iterations = 1000;
    parameters.f_rel = 0.05;
    parameters.x_rel = 2;
    parameters.time_penalty = 5000.0;
    parameters.initial_stepsize_rel = 0.1;
    parameters.inequality_constraint_tolerance = 1;

    // Position
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(D, parameters);
    opt.setupFromVertices(vertices, segment_time, SNAP);
    opt.addMaximumMagnitudeConstraint(VELOCITY, v_max); 
    opt.addMaximumMagnitudeConstraint(ACCELERATION, a_max);
    opt.optimize();

    // Yaw - either you should stick to both segment times being same 
    // in that case  last param - stick to time should be True in both case 
    // or get first segment times - then do yaw
    const int Ny = 6;
    mav_trajectory_generation::NonlinearOptimizationParameters params;
    params.max_iterations = 1000;
    params.f_rel = 0.05;
    params.x_rel = 10;
    params.time_penalty = 500.0;
    params.initial_stepsize_rel = 0.1;
    params.inequality_constraint_tolerance = 2;

    mav_trajectory_generation::PolynomialOptimizationNonLinear<Ny> yaw_opt(yd, params);
    yaw_opt.setupFromVertices(yaw_vertices, segment_time, ANGULAR_VELOCITY);
    yaw_opt.addMaximumMagnitudeConstraint(ANGULAR_VELOCITY, ang_v_max); 
    yaw_opt.addMaximumMagnitudeConstraint(ANGULAR_ACCELERATION, ang_a_max);
    yaw_opt.optimize();

    // ============================
    // Get the optimized trajectory
    // ============================
    mav_trajectory_generation::Segment::Vector segments;
    // opt.getSegments(&segments); // Unnecessary?  -- can we get segment time through this? 

    opt.getTrajectory(&trajectory);
    yaw_opt.getTrajectory(&yaw_trajectory);
    trajectoryStartTime = now();

    RCLCPP_INFO(get_logger(),
                "Generated optimizes trajectory from %zu waypoints",
                vertices.size());
  }

  void publishDesiredState() {
    if (trajectory.empty())
      return;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 1.3 |  16.485 - Fall 2024  - Lab 4 coding assignment (15 pts)
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //
    //  Finally we get to send commands to our controller! First fill in
    //  properly the value for 'nex_point.time_from_start' and 'sampling_time'
    //  (hint: not 0) and after extracting the state information from our
    //  optimized trajectory, finish populating next_point.
    //
    // ~~~~ begin solution

    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint nex_point;

    double elapsed_time = (now() - trajectoryStartTime).seconds();

    // Use elapsed time + 0.02 to sample the trajectory
    // since this sample is for nxt point
    double sampling_time = elapsed_time + 0.02;

    // Set the time_from_start correctly
    builtin_interfaces::msg::Duration time_from_start;
    time_from_start.sec = static_cast<int32_t>(elapsed_time);
    time_from_start.nanosec = static_cast<uint32_t>((elapsed_time - time_from_start.sec) * 1e9);
    nex_point.time_from_start = time_from_start;

    int derivative_order_p = mav_trajectory_generation::derivative_order::POSITION;
    Eigen::Vector3d sample_p = trajectory.evaluate(sampling_time, derivative_order_p);

    int derivative_order_v = mav_trajectory_generation::derivative_order::VELOCITY;
    Eigen::Vector3d sample_v = trajectory.evaluate(sampling_time, derivative_order_v);

    int derivative_order_a = mav_trajectory_generation::derivative_order::ACCELERATION;
    Eigen::Vector3d sample_a = trajectory.evaluate(sampling_time, derivative_order_a);

    int derivative_order_y = mav_trajectory_generation::derivative_order::ORIENTATION;
    Eigen::VectorXd sample_yaw = yaw_trajectory.evaluate(sampling_time, derivative_order_y);

    tf2::Quaternion q;
    q.setRPY(0, 0, sample_yaw[0]);
    // Using only the first pose
    geometry_msgs::msg::Transform transform;
    transform.translation.x = sample_p[0];
    transform.translation.y = sample_p[1];
    transform.translation.z = sample_p[2];
    transform.rotation = tf2::toMsg(q);

    // Create zero velocity and acceleration
    geometry_msgs::msg::Twist velocity_twist;
    velocity_twist.linear.x = sample_v[0];
    velocity_twist.linear.y = sample_v[1];
    velocity_twist.linear.z = sample_v[2];
    velocity_twist.angular.x = 0;
    velocity_twist.angular.y = 0;
    velocity_twist.angular.z = 0;

    // Create zero velocity and acceleration
    geometry_msgs::msg::Twist accel_twist;
    accel_twist.linear.x = sample_a[0];
    accel_twist.linear.y = sample_a[1];
    accel_twist.linear.z = sample_a[2];
    accel_twist.angular.x = 0;
    accel_twist.angular.y = 0;
    accel_twist.angular.z = 0;

    // Add single transform and zero twists
    nex_point.transforms.push_back(transform);
    nex_point.velocities.push_back(velocity_twist);
    nex_point.accelerations.push_back(accel_twist);

    ///////---------------------////////////////
    RCLCPP_INFO(get_logger(), "Publishing desired state");
    desiredStatePub->publish(nex_point);

    // ~~~~ end solution
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // ~
    //                                 end part 1.3
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }

public:
  explicit WaypointFollower() : Node("waypoint_follower_node") {
    currentStateSub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/current_state", 1,
        std::bind(&WaypointFollower::onCurrentState, this,
                  std::placeholders::_1));

    poseArraySub = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/desired_traj_vertices", 1,
        std::bind(&WaypointFollower::generateOptimizedTrajectory, this,
                  std::placeholders::_1));

    desiredStatePub = this->create_publisher<
        trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>("/desired_state",
                                                            1);

    desiredStateTimer = this->create_timer(
                      std::chrono::duration<double>(0.02),
                      std::bind(&WaypointFollower::publishDesiredState, this));
    desiredStateTimer->reset();
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointFollower>());
  return 0;
}
