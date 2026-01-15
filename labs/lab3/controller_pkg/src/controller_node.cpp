#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <mav_msgs/msg/actuators.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>

#define PI M_PI

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  PART 0 |  16.485 - Fall 2024  - Lab 3 coding assignment
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//  In this code, we ask you to implement a geometric controller for a
//  simulated UAV, following the publication:
//
//  [1] Lee, Taeyoung, Melvin Leoky, N. Harris McClamroch. "Geometric tracking
//      control of a quadrotor UAV on SE (3)." Decision and Control (CDC),
//      49th IEEE Conference on. IEEE, 2010
//
//  We use variable names as close as possible to the conventions found in the
//  paper, however, we have slightly different conventions for the aerodynamic
//  coefficients of the propellers (refer to the lecture notes for these).
//  Additionally, watch out for the different conventions on reference frames
//  (see Lab 3 Handout for more details).
//
//  The include below is strongly suggested [but not mandatory if you have
//  better alternatives in mind :)]. Eigen is a C++ library for linear algebra
//  that will help you significantly with the implementation. Check the
//  quick reference page to learn the basics:
//
//  https://eigen.tuxfamily.org/dox/group__QuickRefPage.html

#include <eigen3/Eigen/Dense>
typedef Eigen::Matrix<double, 4, 4> Matrix4d;

// If you choose to use Eigen, tf2 provides useful functions to convert tf2
// messages to eigen types and vice versa.
#include <tf2_eigen/tf2_eigen.hpp>

// FOR exit(1) FOR DEBUGGING
#include <cstdlib>

class ControllerNode : public rclcpp::Node {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  PART 1 |  Declare ROS callback handlers
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  // In this section, you need to declare:
  //   1. two subscribers (for the desired and current UAVStates)
  //   2. one publisher (for the propeller speeds)
  //   3. a timer for your main control loop
  //
  // ~~~~ begin solution

  rclcpp::Publisher<mav_msgs::msg::Actuators>::SharedPtr publisher_ ;
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr sub_desired_state ;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_current_state ;
  rclcpp::TimerBase::SharedPtr heartbeat ;

  // define unique_ptr based on the data type of the messages
  // desired_state = std::unique_ptr<tf2_geometry_msgs::msg::multi_dof_joint_trajectory_point>();
  // current_state = std::unique_ptr<nav_msgs::msg::odometry>();
  // propeller_speed = std::unique_ptr<mav_msgs::msg::actuators>();

  // desiredState_sub() : Node("desiredState_sub") , count_(0){
  //   subscription_ = this->create_subscription<tf2_geometry_msgs::msg::multi_dof_joint_trajectory_point>("desired_state", 50); // 50 is que_size
  //   timer_ = this->create_wall_timer(
  //     // creates a smart pointer timer - which calls timer_callback function
  //     100ms , std::bind(&desiredState_sub::timer_callback, this)) ;
  //     // now the timer_callback should be specific to one callable object
  //     // and here it timer_callback function of this->desiredState_sub object
  // }
  //
  // currentState_sub() : Node("currentState_sub") , count_(0){
  //   subscription_ = this->create_subscription<nav_msgs::msg::odometry>("current_state", 50 );
  //   timer_ = this->create_wall_timer(
  //     100ms , std::bind(&currentState_sub::timer_callback, this));
  // }
  //
  // propSpeed_pub(): Node("propSpeed_pub") , count_(0){
  //   publisher_ = this->create_publisher<mav_msgs::msg::actuators>("rotor_speed_cmds" , 50);
  //   timer_ this->create_wall_timer(
  //     100ms ,  std::bind(&propSpeed_pub::timer_callback, this)) ;
  // }


  // ~~~~ end solution
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                                 end part 1
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // Controller parameters
  double kx, kv, kr, komega; // controller gains - [1] eq (15), (16)

  // Physical constants (we will set them below)
  double m;            // mass of the UAV
  double g;            // gravity acceleration
  double d;            // distance from the center of propellers to the c.o.m.
  double cf,           // Propeller lift coefficient
      cd;              // Propeller drag coefficient
  Eigen::Matrix3d J;   // Inertia Matrix
  Eigen::Vector3d e3;  // [0,0,1]
  Eigen::Matrix4d F2W; // Wrench-rotor speeds map

  // Controller internals (you will have to set them below)
  // Current state
  Eigen::Vector3d x; // current position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d v; // current velocity of the UAV's c.o.m. in the world frame
  Eigen::Matrix3d R; // current orientation of the UAV
                    // This is rotation matrix from world to body
  Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame

  // Desired state
  Eigen::Vector3d xd; // desired position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d vd; // desired velocity of the UAV's c.o.m. in the world frame
  Eigen::Vector3d ad;      // desired acceleration of the UAV's c.o.m. in the world frame
  double yawd; // desired yaw angle


  int64_t hz; // frequency of the main control loop

  static Eigen::Vector3d Vee(const Eigen::Matrix3d &in) {
    Eigen::Vector3d out;
    out << in(2, 1), in(0, 2), in(1, 0);
    return out;
  }

  static double signed_sqrt(double val) {
    return val > 0 ? sqrt(val) : -sqrt(-val);
  }

public:
  ControllerNode() : Node("controller_node"), e3(0, 0, 1), hz(100) {
    // declare ROS parameters
    declare_parameter<double>("kx");
    declare_parameter<double>("kv");
    declare_parameter<double>("kr");
    declare_parameter<double>("komega");

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 2 |  Initialize ROS callback handlers
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //
    // In this section, you need to initialize your handlers from part 1.
    // Specifically:
    //  - bind controllerNode::onDesiredState() to the topic "desired_state"
    //  - bind controllerNode::onCurrentState() to the topic "current_state"
    //  - bind controllerNode::controlLoop() to the created timer, at frequency
    //    given by the "hz" variable
    //
    // Hints:
    //  - make sure you start your timer with reset()
    //
    // ~~~~ begin solution

    sub_desired_state = this->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>(
      "desired_state", 10 , std::bind(&ControllerNode::onDesiredState , this , std::placeholders::_1));

    sub_current_state = this->create_subscription<nav_msgs::msg::Odometry>(
      "current_state" , 10 , std::bind(&ControllerNode::onCurrentState , this , std::placeholders::_1));

    publisher_ = this->create_publisher<mav_msgs::msg::Actuators>("rotor_speed_cmds", 50);

    heartbeat = this->create_timer(
                            std::chrono::duration<double>(1/hz),
                            std::bind(&ControllerNode::controlLoop, this));

    heartbeat->reset();
    // ~~~~ end solution
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //                                 end part 2
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    if (!(get_parameter("kx", kx) && get_parameter("kv", kv) &&
          get_parameter("kr", kr) && get_parameter("komega", komega))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to get controller gains from parameter server");
      exit(1);
    }

    // Initialize constants
    m = 1.0;
    cd = 1e-5;
    cf = 1e-3;
    g = 9.81;
    d = 0.3;
    J = Eigen::Matrix3d::Identity();
    double a = cf * d / sqrt(2);
    F2W << cf, cf, cf, cf, a, a, -a, -a, -a, a, a, -a, cd, -cd, cd, -cd;
  }

  void onDesiredState(
      const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint &des_state) {

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 3 | Objective: fill in xd, vd, ad, yawd
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //
    // 3.1 Get the desired position, velocity and acceleration from the in-
    //     coming ROS message and fill in the class member variables xd, vd
    //     and ad accordingly. You can ignore the angular acceleration.
    //
    // Hint: use "v << vx, vy, vz;" to fill in a vector with Eigen.
    //

    xd << des_state.transforms[0].translation.x ,
         des_state.transforms[0].translation.y ,
         des_state.transforms[0].translation.z;

    vd << des_state.velocities[0].linear.x ,
        des_state.velocities[0].linear.y ,
        des_state.velocities[0].linear.z ;

    ad << des_state.accelerations[0].linear.x ,
         des_state.accelerations[0].linear.y ,
         des_state.accelerations[0].linear.z ;

    //
    // 3.2 Extract the yaw component from the quaternion in the incoming ROS
    //     message and store in the yawd class member variable
    //
    //  Hints:
    //    - look into the functions tf2::getYaw(...) and tf2::fromMsg
    //
    tf2::Quaternion desired_yaw ;
    tf2::fromMsg(des_state.transforms[0].rotation, desired_yaw );
    yawd = tf2::getYaw(desired_yaw);
    //
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //                                 end part 3
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }

  void onCurrentState(const nav_msgs::msg::Odometry &cur_state) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 4 | Objective: fill in x, v, R and omega
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //
    // Get the current position and velocity from the incoming ROS message and
    // fill in the class member variables x, v, R and omega accordingly.
    x << cur_state.pose.pose.position.x ,
       cur_state.pose.pose.position.y,
       cur_state.pose.pose.position.z ;

    v << cur_state.twist.twist.linear.x ,
        cur_state.twist.twist.linear.y ,
        cur_state.twist.twist.linear.z;
    // R is the current orientation of UAV
    tf2::Quaternion state_r ;
    tf2::fromMsg(cur_state.pose.pose.orientation , state_r);

    Eigen::Quaternion state_q(state_r.w() , state_r.x() , state_r.y(), state_r.z());
    // rotation matrix
    R = state_q.toRotationMatrix();  // current orientation of UAV as rotation matrix

    //  CAVEAT: cur_state.twist.twist.angular is in the world frame, while omega
    //          needs to be in the body frame!

    // world to body
    Eigen::Matrix3d R_bw = R.transpose();
    Eigen::Vector3d omega_world;
    omega_world << cur_state.twist.twist.angular.x,
                              cur_state.twist.twist.angular.y,
                              cur_state.twist.twist.angular.z;
    omega = R_bw * omega_world ;  // order matters - dimenstions 3x3 and 3x1
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //                                 end part 4
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }

  void controlLoop() {
    Eigen::Vector3d ex, ev, er, eomega;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 5 | Objective: Implement the controller!
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //
    // 5.1 Compute position and velocity errors. Objective: fill in ex, ev.
    //  Hint: [1], eq. (6), (7)
    //
    ex = x - xd;
    ev = v - vd;
    // roll and pitch depends on trajectory


    // 5.2 Compute the Rd matrix.
    //
    //  Hint: break it down in 3 parts:
    //    - b3d vector = z-body axis of the quadrotor, [1] eq. (12)
    //    - check out [1] fig. (3) for the remaining axes [use cross product]
    //    - assemble the Rd matrix, eigen offers: "MATRIX << col1, col2, col3"
    //
    //  CAVEATS:
    //    - Compare the reference frames in the Lab 3 handout with Fig. 1 in the
    //      paper. The z-axes are flipped, which affects the signs of:
    //         i) the gravity term and
    //        ii) the overall sign (in front of the fraction) in equation (12)
    //            of the paper
    //    - remember to normalize your axes!
    Eigen::Vector3d e3(0,0,1);
    Eigen::Vector3d U = -kx*ex - kv*ev + m*g*e3 + m*ad ;
    double U_norm = U.norm();
    if (U_norm < 1e-9) {
      RCLCPP_WARN(this->get_logger(), "Degenerate desired thrust vector");
      return;
    }
    Eigen::Vector3d b3d = U / U_norm;

    // Build b3d vector
    // Rotation of first body fix axis - desired yaw
    tf2::Quaternion q;
    q.setRPY(0,0,yawd);
    Eigen::Quaternion q_eig(q.w(), q.x(), q.y() , q.z());
    Eigen::Matrix3d q_rot = q_eig.toRotationMatrix();
    Eigen::Vector3d b1d = q_rot*Eigen::Vector3d::UnitX();
    b1d.normalize();

    Eigen::Vector3d b2d = b3d.cross(b1d);
    double denom = b2d.norm();
    if (denom < 1e-9) {
      // b1d is (nearly) parallel to b3d — choose an alternative b1d (e.g., [0,1,0])
      Eigen::Vector3d alt(0.0, 1.0, 0.0);
      b2d = b3d.cross(alt).normalized();
    } else {
      b2d.normalize();
    }

    Eigen::Matrix3d R_d;
    Eigen::Vector3d b23d = b2d.cross(b3d).normalized();
    R_d.col(0) = b23d;
    R_d.col(1) = b2d ;
    R_d.col(2) = b3d ;

    // 5.3 Compute the orientation error (er) and the rotation-rate error
    // (eomega)
    //  Hints:
    //     - [1] eq. (10) and (11)
    //     - you can use the Vee() static method implemented above
    //
    //  CAVEAT: feel free to ignore the second addend in eq (11), since it
    //          requires numerical differentiation of Rd and it has negligible
    //          effects on the closed-loop dynamics.
    //

    Eigen::Matrix3d R_rel = R_d.transpose() * R;
    double cos_theta = (R_rel.trace() - 1.0) / 2.0;
    cos_theta = std::clamp(cos_theta, -1.0, 1.0);
    double theta = std::acos(cos_theta);

    Eigen::Vector3d phi;
    if (theta < 1e-8) {
      // small-angle approx: log(R) ≈ 0.5*(R - R^T)
      Eigen::Matrix3d S = 0.5 * (R_rel - R_rel.transpose());
      er = Vee(S) ;
    } else {
      double k = theta / (2.0 * std::sin(theta));
      Eigen::Matrix3d S = k * (R_rel - R_rel.transpose());  // skew matrix = log(R)
      er = Vee(S) ;                      // vee(S)
    }

    // angular distance = theta error
    //
    eomega = omega ;//- R.transpose()*R_d*Omega_d;

    // 5.4 Compute the desired wrench (force + torques) to control the UAV.
    //  Hints:
    //     - [1] eq. (15), (16)

    // CAVEATS:
    //    - Compare the reference frames in the Lab 3 handout with Fig. 1 in the
    //      paper. The z-axes are flipped, which affects the signs of:
    //         i) the gravity term
    //        ii) the overall sign (in front of the bracket) in equation (15)
    //            of the paper
    //
    //    - feel free to ignore all the terms involving \Omega_d and its time
    //      derivative as they are of the second order and have negligible
    //      effects on the closed-loop dynamics.
    Eigen::Vector3d f = -kx*ex -kv*ev + m*g*e3 + m*ad ;
    double F = f.dot(R.col(2));
    Eigen::Vector3d M = -kr*er - komega*eomega + omega.cross(J*omega);
    // higher order omega is neglected - doesnt make much difference for controller


    // 5.5 Recover the rotor speeds from the wrench computed above
    //
    //  Hints:
    //     - [1] eq. (1)

    // CAVEATs:
    //     - we have different conventions for the arodynamic coefficients,
    //       Namely: C_{\tau f} = c_d / c_f
    //               (LHS paper [1], RHS our conventions [lecture notes])
    //
    //     - Compare the reference frames in the Lab 3 handout with Fig. 1 in
    //     the
    //       paper. In the paper [1], the x-body axis [b1] is aligned with a
    //       quadrotor arm, whereas for us, it is 45° from it (i.e., "halfway"
    //       between b1 and b2). To resolve this, check out equation 6.8 in the
    //       lecture notes!
    Eigen::Vector4d wrench;
    wrench << F, M(0), M(1), M(2);
    Eigen::Vector4d W = F2W.inverse()* wrench;

    //     - The thrust forces are **in absolute value** proportional to the
    //       square of the propeller speeds. Negative propeller speeds -
    //       although uncommon - should be a possible outcome of the controller
    //       when appropriate. Note that this is the case in unity but not in
    //       real life, where propellers are aerodynamically optimized to spin
    //       in one direction!
    //

    //
    // 5.6 Populate and publish the control message
    //
    // Hint: do not forget that the propeller speeds are signed (maybe you want
    // to use signed_sqrt function).
    //
    mav_msgs::msg::Actuators msg ;
    msg.angular_velocities.resize(4);

    for (int i = 0; i < 4; ++i) {
      msg.angular_velocities[i] = signed_sqrt(W(i)); // signed sqrt -> rotor speed (rad/s)
    }

    msg.header.stamp = this->now();
    publisher_->publish(msg);
    //
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //           end part 5, congrats! Start tuning your gains (part 6)
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv); // Initialize the ROS 2 system
  rclcpp::spin(std::make_shared<ControllerNode>()); // Spin the node so it
                                                    // processes callbacks
  rclcpp::shutdown(); // Shutdown the ROS 2 system when done
  return 0;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  PART 6 [NOTE: save this for last] |  Tune your gains!
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
// Live the life of a control engineer! Tune these parameters for a fast
// and accurate controller.
//
// Modify the gains kx, kv, kr, komega in controller_pkg/config/params.yaml
// and re-run the controller.
//
// Can you get the drone to do stable flight?
//
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//  You made it! Congratulations! You are now a control engineer!
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
