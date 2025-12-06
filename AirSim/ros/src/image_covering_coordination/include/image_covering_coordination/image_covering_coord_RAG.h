/*
NOTE: This header contains the algorithms RBG and BSG only; for RAG-H view multi_target_tracking_coord_RAG.cpp
 */
#ifndef IMAGE_COVERING_COORD_RAG_H
#define IMAGE_COVERING_COORD_RAG_H

// C++ headers
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

// Custom message headers
#include <airsim_ros_pkgs/Neighbors.h>
#include <airsim_ros_pkgs/NeighborsArray.h>
#include <image_covering_coordination/ImageCovering.h>

// Airsim library
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

// #include "airsim_settings_parser.h"
#include "common/AirSimSettings.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "sensors/lidar/LidarSimpleParams.hpp"
#include "ros/ros.h"
#include "sensors/imu/ImuBase.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "yaml-cpp/yaml.h"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <math.h>
#include <math_common.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <rosgraph_msgs/Clock.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <unordered_map>
#include<map>
#include <memory>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <rosgraph_msgs/Clock.h>
#include <random>
#include <chrono>
#include <thread>
#include <algorithm>
#include <limits>

#include <ros/spinner.h>
#include <opencv2/imgproc.hpp>

// Airsim specific namespaces and typedefs
using namespace msr::airlib;
typedef ImageCaptureBase::ImageRequest ImageRequest;
typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageType ImageType;
typedef common_utils::FileSystem FileSystem;

class ImageCoveringCoordRAG
{

public:
    ImageCoveringCoordRAG(ros::NodeHandle& nh);
    ~ImageCoveringCoordRAG(); // destructor

private:
    struct ImageInfoStruct {
        geometry_msgs::Pose pose;
        int camera_orientation;
        sensor_msgs::Image image;
        double best_marginal_gain_val;
    };

    // ROS node handle
    ros::NodeHandle node_handle_;
    int curr_time_step_samp_;
    int num_robots_;
    int num_evaders_;
    int robot_idx_self_;
    double total_data_msg_size_; // in KB
    double communication_data_rate_; // in KB/s
    bool takeoff_wait_flag_;
    double max_sensing_range_;  // max sensing range among all robots
    double altitude_flight_level_;
    double flight_level_;
    double drone_lin_vel_factor_;
    double yaw_rate_;
    double yaw_duration_;
    int communication_round_;

    std::chrono::time_point<std::chrono::_V2::system_clock> start_;

    // ROS subscribers
    ros::Subscriber clock_sub_;
    ros::Subscriber marginal_gain_pub_clock_sub_;
    ros::Subscriber check_drones_selection_sub_;

    // ROS publishers
    ros::Publisher mg_rag_msg_pub_;

    // TF listener
    tf::TransformListener* tf_listener_;

    // Class variables
    std::string drone_name_self_;
    std::string camera_name_;
    double communication_range_;
    double objective_function_value_;
    // double previous_robots_obj_func_val_; // normalized reward not used in RAG-H
    Eigen::MatrixXd reward_;
    bool PEdata_new_msg_received_flag_;
    double fov_x_;
    double fov_y_;
    double best_marginal_gain_self_;
    std::map<int, bool> all_robots_selected_map_;
    bool terminate_decision_making_;

    // Callback functions
    void clockCallback(const rosgraph_msgs::Clock& msg);
    void marginalGainPublisherclockCallback(const rosgraph_msgs::Clock& msg);
    void updateAllRobotsSelectionStatusCallback(const rosgraph_msgs::Clock& msg);

    void objectiveFunctionRAG(double& obj_func_val, std::vector<int>& in_neigh_ids_select_actions, bool& include_self_robot, int& self_robot_action_id);
    void marginalGainFunctionRAG(double& best_marginal_gain, int& optimal_action_id, geometry_msgs::Pose latest_pose, std::map<int, 
                                        ImageInfoStruct>& prev_selected_action_inneigh_ids_img);
    void bestInitActionObjFunctionRAG(double& best_marginal_gain, int& optimal_action_id, std::vector<int>& inneighs_selected_action);

    void collectRotatedCameraImages();

    void simulatedCommTimeDelayRAG();  // for RAG-H
    void simulatedCommTimeDelayRAG(double& processed_time);

    std::pair<double, double> computeRangeBearing(const std::vector<double>& pose1, const std::vector<double>& pose2);

    void getSelectedActionsInNeighborIDs(std::map<int, double>& selected_action_inneigh_ids_mg);
    void getSelectedActionsInNeighborIDImages(std::map<int, ImageInfoStruct>& selected_action_inneigh_ids_mg);


    void getNonSelectedActionsInNeighborIDs(std::map<int, double>& non_selected_action_inneigh_ids_mg);
    void getSelectedActionsInNeighborIDImagesLinePath(std::map<int, ImageInfoStruct>& selected_action_inneigh_ids_mg);
    void getSelectedActionsInNeighborIDImagesNearestNeighbors(std::map<int, ImageInfoStruct>& selected_action_inneigh_ids_mg, int num_in_neighbors);

    void getNearestNeighborRobots(std::vector<int>& nn_ids, int num_in_neighbors);

    void isMyMarginalGainBest(bool& is_my_mg_best, std::map<int, double>& non_selected_action_inneigh_ids_mg);
    void takeUnionOfMapsDataStruct(std::map<int, double>& selected_action_inneigh_ids_mg, std::map<int, double>& new_selected_action_inneigh_ids_mg, std::map<int, double>& combined_map);
    void takeUnionOfMapsDataStruct(std::map<int, double>& selected_action_inneigh_ids_mg, std::map<int, double>& new_selected_action_inneigh_ids_mg);
    void takeUnionOfMapsDataStruct(std::map<int, ImageInfoStruct>& selected_action_inneigh_ids_mg, std::map<int, ImageInfoStruct>& new_selected_action_inneigh_ids_mg);

    sensor_msgs::ImagePtr get_img_msg_from_response(const ImageResponse& img_response);
    ros::Time airsim_timestamp_to_ros(const msr::airlib::TTimePoint& stamp) const;
    ros::Time chrono_timestamp_to_ros(const std::chrono::system_clock::time_point& stamp) const;

    bool areAllValuesTrue();
    bool areAllValuesTrue3Robots(); // FOR DEBUGGING
    bool areAllValuesTrue(std::map<int, bool> &all_robots_selected_map);


    msr::airlib::Pose giveGoalPoseFormat(msr::airlib::MultirotorState& curr_pose, float disp_x, float disp_y, float disp_z, float new_yaw);
    Eigen::Quaternionf quaternionFromEuler(float roll, float pitch, float yaw);
    Eigen::Vector3f eulerFromQuaternion(const Eigen::Quaternionf& q);
    msr::airlib::Pose giveCameraGoalPoseFormat(msr::airlib::MultirotorState& curr_pose, float disp_x, float disp_y, float disp_z, float new_pitch,
                                                float curr_roll, float curr_yaw);

    std::vector<double> computeWeightedScores(const std::vector<sensor_msgs::Image>& images);
    double computeWeightedScoreSingleImage(const sensor_msgs::Image& image,  bool convertBGR2RGB);
    double computeWeightedScoreSingleImage(const cv::Mat& img);

    std::vector<cv::Point> toCvPointVector(const std::vector<cv::Point2f>& points);
    std::vector<cv::Point2f> findIntersection(const std::vector<cv::Point2f>& poly1, const std::vector<cv::Point2f>& poly2);
    std::vector<cv::Point2f> getRectangleFromPose(const geometry_msgs::Pose& pose, int orientation);
    sensor_msgs::Image findNonOverlappingRegions(const sensor_msgs::Image& curr_image, const geometry_msgs::Pose& curr_pose,
                                            int curr_orientation, std::map<int, std::pair<geometry_msgs::Pose, 
                                            int>>& in_neighbor_poses_and_orientations, bool convertBGR2RGB);
    std::vector<cv::Point2f> getRectangleFromPoseInNeighbor(const geometry_msgs::Pose& pose_self_robot, const geometry_msgs::Pose& pose_inn_robot, 
                                                                        int self_rob_img_orient, int inn_img_orient);

    geometry_msgs::Pose convertToGeometryPose(const msr::airlib::MultirotorState& pose1);
    double PoseEuclideanDistance(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2, int cam_orient_1, int cam_orient_2);
    void saveImage(std::vector<ImageResponse>& response);
    void viewImage(std::vector<ImageResponse>& response);
    void viewImageCvMat(cv::Mat& image);
    void viewImageCvMat(cv::Mat& image, bool switchBGR_RGB_flag);
    void viewImageCvMat(cv::Mat& image, bool switchBGR_RGB_flag, const std::string filename);
    void saveImageCvMat(cv::Mat& image, bool switchBGR_RGB_flag, const std::string filename);

    void rotateImagesInVector();
    double calculateAngleFromOrientation(int cam_orient);
    void printVector(const std::vector<int>& vec);
    void printMap(const std::map<int, double>& map);

    double horizon_;
    double n_time_steps_;

    // actions set
    int selected_action_ind_; // a_i,t_RAG-H
    bool finished_action_selection_;
    bool waiting_for_robots_to_finish_selection_;
    std::vector<int> in_neigh_ids_select_actions_; // I_i,t ; agents in N^-_i,t (in-neighbors) that have already selected an action to execute
    Eigen::MatrixXd actions_;
    Eigen::MatrixXd actions_unit_vec_;
    Eigen::Matrix<double, 3, 1> last_selected_displacement_;

    // Airsim client and constants
    float client_timeout_sec_;
    const msr::airlib::YawMode client_default_yaw_mode_;
    float client_lookahead_;
    float client_adaptive_lookahead_ ;
    float client_margin_;
    double yaw_rate_timestep_;
    double drone_linear_velocity_;
    msr::airlib::DrivetrainType client_drivetrain_;
    msr::airlib::MultirotorRpcLibClient client_;

    // to store drone evader data in a buffer
    airsim_ros_pkgs::NeighborsArray neighbors_data_;

    std::unique_ptr<ros::AsyncSpinner> async_spinner_;

    // objects related to image processing for objective function and marginal gain calculations
    std::vector<sensor_msgs::Image> camera_rotated_imgs_vec_;
    double camera_pitch_theta_;
    double camera_downward_pitch_angle_;
    // current_image_request_ = {ImageRequest(curr_camera_name, msr::airlib::ImageCaptureBase::ImageType::Segmentation, false, false)} ;
    std::vector<ImageRequest> current_image_requests_vec_;
    msr::airlib::ImageCaptureBase::ImageType camera_image_type_;
    sensor_msgs::Image best_self_image_;
    int best_image_cam_orientation_;
    std::string img_save_path_;

    // CAMERA PARAMETERS******
    double y_half_side_len_m_;
    double x_half_side_len_m_;
    double y_dir_px_m_scale_;
    double x_dir_px_m_scale_;
    double delta_C_; // displacement in center due to viewing angle
    double diagonal_frame_dist_;
    double diagonal_px_scale_;
    int pasted_rect_height_pix_;
    int pasted_rect_width_pix_;
    bool take_new_pics_;
    double road_pixel_weight_;
    // CAMERA PARAMETERS******end

    std::string graph_topology_RAG_;
    std::string algorithm_to_run_;
    double control_action_disp_scale_;
    int default_num_nearest_neighbors_;
    int assump_counter_;
    int random_seed_;
    std::map<int, bool> check_drones_msg_dict_; // to confirm there were no dropped messages from drones which are still making decisions
    int num_nearest_neighbors_;
    int is_server_experiment_;

};

#endif // IMAGE_COVERING_COORD_RAG_H
