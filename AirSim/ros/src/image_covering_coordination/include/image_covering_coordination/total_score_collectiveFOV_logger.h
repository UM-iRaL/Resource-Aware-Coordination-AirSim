/*
NOTE: This header contains the algorithms RBG and BSG only; for RAG-H view multi_target_tracking_coord_RAG.cpp
 */
#ifndef TOTAL_SCORE_COLLECTIVE_FOV_H
#define TOTAL_SCORE_COLLECTIVE_FOV_H

// C++ headers
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

// Custom message headers
#include <airsim_ros_pkgs/Neighbors.h>
#include <airsim_ros_pkgs/NeighborsArray.h>
// #include <multi_target_tracking/PursuerEvaderData.h>
// #include <multi_target_tracking/PursuerEvaderDataArray.h>
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
#include <utility>
#include <fstream>

#include <ros/spinner.h>
#include <opencv2/imgproc.hpp>

// Airsim specific namespaces and typedefs
using namespace msr::airlib;
typedef ImageCaptureBase::ImageRequest ImageRequest;
typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageType ImageType;
typedef common_utils::FileSystem FileSystem;

class TotalScoreCollectiveFovLogger
{

public:
    TotalScoreCollectiveFovLogger(ros::NodeHandle& nh);
    ~TotalScoreCollectiveFovLogger(); // destructor
    void writeDataToCSV();

private:
    struct ImageInfoStruct {
        geometry_msgs::Pose pose;
        int camera_orientation;
        sensor_msgs::Image image;
        double best_marginal_gain_val;
    };

    // ROS node handle
    ros::NodeHandle node_handle_;
    int num_robots_;

    std::chrono::time_point<std::chrono::_V2::system_clock> start_;

    // ROS subscribers
    ros::Subscriber get_drones_images_sub_;
    ros::Subscriber check_drones_selection_sub_;

    // Class variables
    double fov_x_;
    double fov_y_;
    double best_marginal_gain_self_;
    std::map<int, bool> all_robots_selected_map_;
    bool terminate_decision_making_;

    // Callback functions
    void getRobotsBestImages(const rosgraph_msgs::Clock& msg);
    double computeWeightedScoreSingleImage(const cv::Mat& img);
    double computeWeightedScoreSingleImageNORGBconv(const cv::Mat& img);

    sensor_msgs::Image findNonOverlappingRegions(const sensor_msgs::Image& curr_image, const geometry_msgs::Pose& curr_pose,
                                            int curr_orientation, std::map<int, std::pair<geometry_msgs::Pose, 
                                            int>>& in_neighbor_poses_and_orientations);
    void allRobotsImagesPasted(cv::Mat& mask_returned, bool& gotallimages);
    void updateAllRobotsSelectionStatusCallback(const rosgraph_msgs::Clock& msg);

    double PoseEuclideanDistance(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2, int cam_orient_1, int cam_orient_2);
    void saveImage(std::vector<ImageResponse>& response);
    void viewImage(std::vector<ImageResponse>& response);
    void viewImageCvMat(cv::Mat& image);
    void viewImageCvMat(cv::Mat& image, bool switchBGR_RGB_flag);
    void viewImageCvMat(cv::Mat& image, bool switchBGR_RGB_flag, const std::string filename);
    void saveImageCvMat(cv::Mat& image, const std::string filename);

    double calculateAngleFromOrientation(int cam_orient);
    void logTotalScoreCollectiveFOV();
    void logTotalScoreCollectiveFOV(double duration_last);

    bool areAllValuesTrue();

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

    std::unique_ptr<ros::AsyncSpinner> async_spinner_;

    // objects related to image processing for objective function and marginal gain calculations
    double camera_pitch_theta_;
    double camera_downward_pitch_angle_;
    int best_image_cam_orientation_;
    std::string img_save_path_;
    std::map<int, ImageInfoStruct> robots_ids_images_map_;

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

    geometry_msgs::Pose frame_center_pose_;
    // std::vector<std::pair<std::chrono::steady_clock::time_point, double>> score_data_;
    std::vector<std::pair<double, double>> score_data_;
    int pic_name_counter_;
    std::string algorithm_to_run_;
    int experiment_number_;
    double time_elapsed_sum_;

    double t_minus_1_stamp_;
    bool firstlogflag_;
    int num_nearest_neighbors_ = 0;
    int is_server_experiment_;
};

#endif // TOTAL_SCORE_COLLECTIVE_FOV_H
