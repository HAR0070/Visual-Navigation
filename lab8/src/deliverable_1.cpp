/**
 * @file Locating tedy with yolo
 * @brief GTSAM bundle adjustment
 */

#include "rclcpp/rclcpp.hpp"
#include <ultralytics_ros/msg/yolo_result.hpp>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"

#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

#include <sensor_msgs/msg/camera_info.hpp>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

#include "deliverable_1.hpp"
#include <gtsam/nonlinear/ISAM2.h>

using namespace std;
using namespace gtsam;

class MyNode : public rclcpp::Node
{
public:
    MyNode(): Node("my_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        subscription_ = this->create_subscription<ultralytics_ros::msg::YoloResult>(
            "/yolo_result", 1, std::bind(&MyNode::topic_callback, this, std::placeholders::_1));
        
        // calibration_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        //     "/camera/rgb/camera_info", 1, std::bind(&MyNode::camera_calibration, this, std::placeholders::_1));
        
        tedy_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/tedy_marker", 1);
        // camera_trj_pub = this->create_publisher<visualization_msgs::msg::Marker>("/camera_trj", 1);
        camera_pos_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/camera_pos",1);
        
        // camera calibration 
        {
        double fx = 537.960321985614; 
        double fy = 539.597659163239; 
        double cx = 319.183641033155; 
        double cy = 247.053820358135; 
        double k1 = 0.0263704224826013; 
        double k2 = -0.10008645619921; 
        double p1 = 0.00313758409632316; 
        double p2 = 0.00242072236844001;
        K = boost::make_shared<Cal3_S2>(fx, fy , 0.0, cx, cy); // , k1, k2, p1 , p2);
        }
        // noise models 
        measurementNoise = noiseModel::Isotropic::Sigma(2, 1);  // one pixel in u and v
        prior_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.2)).finished()); // modify

        gtsam::SmartProjectionParams smart_params;
        // Lower the threshold required to attempt triangulation
        smart_params.setRankTolerance(1e-9); 
        smart_params.setEnableEPI(true); // Epipolar constraint filtering

        smart_params.setDegeneracyMode(gtsam::DegeneracyMode::ZERO_ON_DEGENERACY);
        smart_factor = boost::make_shared<gtsam::SmartProjectionPoseFactor<Cal3_S2>>(
            measurementNoise, K, smart_params);

        // params.setVerbosity("ERROR");
        // params.setAbsoluteErrorTol(1e-08);

        gtsam::ISAM2Params isam_params;
        isam_params.relinearizeThreshold = 0.1; // Re-evaluate old math if error gets too high
        isam_params.relinearizeSkip = 1;        // Check every step
        isam = gtsam::ISAM2(isam_params);
    }

private:
    // ros2 interface show ultralytics_ros/msg/YoloResult
    void topic_callback(const ultralytics_ros::msg::YoloResult::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "====================");
        RCLCPP_INFO(this->get_logger(), "I received a message");
        RCLCPP_INFO(this->get_logger(), "====================");

        int location_id = 0;        // To check if teddy bear is detected 

        for (auto det :  msg->detections.detections)
        {
            for (auto res :  det.results)
            {
                auto class_id = res.hypothesis.class_id;
                auto score = res.hypothesis.score;
                RCLCPP_INFO(this->get_logger(), "class_id: '%s'", class_id.c_str());
                RCLCPP_INFO(this->get_logger(), "score: %.02f%%", score*100);

                if (class_id == "teddy bear") {
                    location_id = 1; 
                    break;
                }
                
            }
            auto x = det.bbox.center.position.x;
            auto y = det.bbox.center.position.y;
            
            RCLCPP_INFO(this->get_logger(), "center: (%.02f, %.02f)", x, y);
            RCLCPP_INFO(this->get_logger(), "--------------------");

            std::string source_frame = "world";
            std::string target_frame = "openni_rgb_frame";
            geometry_msgs::msg::TransformStamped transformStamped; 
            try {
                transformStamped = tf_buffer_.lookupTransform(
                    source_frame, target_frame, tf2::TimePointZero
                );
                RCLCPP_INFO(this->get_logger(), "Camera Position    ::  x: %.02f,  y: %.02f,  z: %.02f",
                    transformStamped.transform.translation.x,
                    transformStamped.transform.translation.y,
                    transformStamped.transform.translation.z
                );
                RCLCPP_INFO(this->get_logger(), "Camera Orientation :: qx: %.02f, qy: %.02f, qz: %.02f, qw: %.02f",
                    transformStamped.transform.rotation.x,
                    transformStamped.transform.rotation.y,
                    transformStamped.transform.rotation.z,
                    transformStamped.transform.rotation.w
                );

                if (location_id){
                        
                    Pose3 camera_pose = transformToPose3(transformStamped);  // come back
                    // camera_poses.push_back(camera_pose);               // ground truth camera pose

                    // just for this frame
                    gtsam::NonlinearFactorGraph graph;
                    gtsam::Values initialEstimate;

                    // Pose3 prior_pose = camera_pose;
                    graph.add(PriorFactor<Pose3>(Symbol('x', current_index), camera_pose, prior_noise));

                    if (current_index == 0) {
                        // graph.add(PriorFactor<Pose3>(Symbol('x', 0), camera_pose, prior_noise));
                        auto landmark_prior_noise = noiseModel::Isotropic::Sigma(3, 2); 
                        Point3 initial_landmark_guess = camera_pose.transformFrom(Point3(0.0, 0.0, 2.0));
                        graph.add(PriorFactor<Point3>(landmarkKey, initial_landmark_guess, landmark_prior_noise));
                        // Optional: initial landmark guess (e.g., at (0,0,0) or triangulated from first two views)
                        initialEstimate.insert(landmarkKey, camera_pose.transformFrom(Point3(0.0, 0.0, 2.0))); // rough guess
                    }

                    // Add projection factor for this observation
                    graph.add(GenericProjectionFactor<Pose3, Point3, Cal3_S2>(
                        Point2(x, y), measurementNoise, Symbol('x', current_index), landmarkKey, K));

                    initialEstimate.insert(Symbol('x', current_index), camera_pose);

                    isam.update(graph, initialEstimate);
                    result = isam.calculateEstimate();

                    // Publish marker using result.at<Point3>(landmarkKey)
                    std::vector<Point3> tedy_landmarks = { result.at<Point3>(landmarkKey) };
                    DrawPoint3(tedy_marker_pub, now(), tedy_landmarks, marker_color::color_init);

                    std::vector<Pose3> cam_poses;
                    for (int i = current_index; i > (current_index > 5 ? current_index - 5 : 0); --i) {
                        cam_poses.push_back(result.at<Pose3>(Symbol('x', i)));
                    }
                    DrawPoses3(camera_pos_pub, now(), cam_poses);

                    // gtsam::Key pose_key = gtsam::Symbol('x', current_index);
                    // gtsam::Key point_key = gtsam::Symbol('l', location_id); // landmark is teddy bear, which is the first landmark in our problem
                    // GenericProjectionFactor<Pose3, Point3, Cal3_S2> factor(Point2(x,y), measurementNoise , pose_key, point_key, K );
                    // graph.add(factor);

                    // // Add the 2D measurement to the smart factor, linking it to the current camera pose
                    // smart_factor->add(Point2(x, y), Symbol('x', current_index));

                    // // if (current_index == 0) {
                    // //     graph.add(smart_factor);
                        
                    // // }
                    // auto smart_factor_snapshot = boost::make_shared<gtsam::SmartProjectionPoseFactor<Cal3_S2>>(*smart_factor);
                    // graph.add(smart_factor_snapshot);

                    // // if (!result.empty()){
                    // //     initialEstimate = result; // use the previous optimization result as the initial estimate for the next optimization
                    // // } 
                    // initialEstimate.insert( Symbol('x', current_index ), camera_pose); //camera pose as the initial estimate 

                    // // Change to isam 
                    // // isam.update(graph, initialEstimate);
                    // // isam.update();
                    // gtsam::ISAM2Result isam_res = isam.update(graph, initialEstimate, smart_factor_indices);
                    // isam.update(); // Second update to relinearize

                    // smart_factor_indices.clear();
                    // smart_factor_indices.push_back(isam_res.newFactorsIndices.back());

                    // result = isam.calculateEstimate();
                    // // cout << "ISAM2 Error: " << isam.error() << endl;
                    // cout << "ISAM2 Error: " << isam.getFactorsUnsafe().error(result) << endl;

                    // plotInRviz();

                        // Optimize the graph.
                    // if (current_index > 0){
                    //     result = LevenbergMarquardtOptimizer(graph, initialEstimate, params).optimize();

                    //     // Marginals marginals(graph, result);
                    //     // measurementNoise = marginals.marginalCovariance(Symbol('x', current_index)); // its adding info twice 

                    //     result.print("Final results:\n");
                    //     cout << "initial error = " << graph.error(initialEstimate) << endl;
                    //     cout << "final error = " << graph.error(result) << endl;
                        
                    //     plotInRviz();
                    // }
                    current_index++;
                    break;
                }   
                

            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Transform not available: %s", ex.what());
            }
            RCLCPP_INFO(this->get_logger(), "--------------------");

        }
        
    }

    void camera_calibration(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        double fx = msg->k[0];
        double fy = msg->k[4];
        double cx = msg->k[2];
        double cy = msg->k[5];
        double k1 = msg->d[0]; 
        double k2 = msg->d[1]; 
        double p1 = msg->d[2]; 
        double p2 = msg->d[3];
        K = boost::make_shared<Cal3_S2>(fx, fy , 0.0, cx, cy) ; //, k1, k2, p1 , p2);
    }

    gtsam::Pose3 transformToPose3( const geometry_msgs::msg::TransformStamped& tf) {
            const auto& t = tf.transform.translation;
            const auto& q = tf.transform.rotation;

            gtsam::Rot3 R = gtsam::Rot3::Quaternion(
                q.w, q.x, q.y, q.z);   // NOTE order: w,x,y,z

            gtsam::Point3 T(t.x, t.y, t.z);

            return gtsam::Pose3(R, T);
        }

    void plotInRviz(){

    std::vector<Point3> tedy_landmarks;
    // tedy_landmarks.push_back(result.at<Point3>(Symbol('l', 1)));  // 1 is location_id for teddy bear
    // DrawPoint3(tedy_marker_pub, now(), tedy_landmarks, marker_color::color_init);

    if (smart_factor->point()) { 
            tedy_landmarks.push_back(*(smart_factor->point()));
        }
    DrawPoint3(tedy_marker_pub, now(), tedy_landmarks, marker_color::color_init);

    std::vector<Pose3> cam_poses;
    int j = 0;
    for (int i = camera_poses.size() - 1; i >= 0; --i) {
        cam_poses.push_back(result.at<Pose3>(Symbol('x', i)));
        j++;
        if (j >= 5) break; 
    }
    DrawPoses3(camera_pos_pub, now(), cam_poses);

    }
    
    // Camera calibration - distortion included - parameters are from the camera_info topic -
    // used as fallback if the camera_info topic is not available
    gtsam::Cal3_S2::shared_ptr K; 
    // This replaces GenericProjectionFactor
    gtsam::SmartProjectionPoseFactor<Cal3_S2>::shared_ptr smart_factor;

    // Define the camera observation noise model
    noiseModel::Isotropic::shared_ptr measurementNoise;
    // Prior noise for the first pose
    noiseModel::Diagonal::shared_ptr prior_noise;

    // Create a factor graph
    // NonlinearFactorGraph graph;

    gtsam::ISAM2 isam;
    gtsam::FactorIndices smart_factor_indices;
    
    size_t current_index = 0;
    std::vector<gtsam::Pose3> camera_poses;
    gtsam::Values result;
    gtsam::Key landmarkKey = gtsam::Symbol('l', 1);
    // gtsam::Values initialEstimate;
    // gtsam::LevenbergMarquardtParams params;

    // subscribe to yolo results
    rclcpp::Subscription<ultralytics_ros::msg::YoloResult>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr calibration_sub;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Publishers for visualization
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tedy_marker_pub;
    // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr camera_trj_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr camera_pos_pub;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}
