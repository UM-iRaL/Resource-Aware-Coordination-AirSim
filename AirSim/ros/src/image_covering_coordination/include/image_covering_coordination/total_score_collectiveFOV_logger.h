#ifndef TOTAL_SCORE_COLLECTIVE_FOV_H
#define TOTAL_SCORE_COLLECTIVE_FOV_H

// C++ headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>

// Custom message headers (修改为 ROS2 版本)
#include <airsim_ros_pkgs/msg/neighbors.hpp>
#include <airsim_ros_pkgs/msg/neighbors_array.hpp>
#include <multi_target_tracking/msg/marginal_gain_rag.hpp>
#include <image_covering_coordination/msg/image_covering.hpp>

// Airsim library
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif 
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "common/AirSimSettings.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "sensors/lidar/LidarSimpleParams.hpp"
#include "sensors/imu/ImuBase.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "yaml-cpp/yaml.h"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <image_transport/image_transport.hpp>
#include <iostream>
#include <math.h>
#include <math_common.h>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/msg/image_encodings.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <unordered_map>
#include <map>
#include <memory>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <random>
#include <chrono>
#include <thread>
#include <algorithm>
#include <utility>
#include <fstream>
#include <opencv2/imgproc.hpp>

// AirSim specific namespaces and typedefs
using namespace msr::airlib;
typedef ImageCaptureBase::ImageRequest ImageRequest;
typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageType ImageType;
typedef common_utils::FileSystem FileSystem;

class TotalScoreCollectiveFovLogger : public rclcpp::Node,
                                      public std::enable_shared_from_this<TotalScoreCollectiveFovLogger> 
{
public:
    TotalScoreCollectiveFovLogger();
    ~TotalScoreCollectiveFovLogger();
    void writeDataToCSV();

private:
    struct ImageInfoStruct {
        geometry_msgs::msg::Pose pose;
        int camera_orientation;
        sensor_msgs::msg::Image image;
        double best_marginal_gain_val;
    };

    int num_robots_;
    std::chrono::time_point<std::chrono::_V2::system_clock> start_;

    // ROS subscribers
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr get_drones_images_sub_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr check_drones_selection_sub_;

    // Class variables
    double fov_x_;
    double fov_y_;
    double best_marginal_gain_self_;
    std::map<int, bool> all_robots_selected_map_;
    bool terminate_decision_making_;

    // Callback functions
    void getRobotsBestImages(const rosgraph_msgs::msg::Clock::SharedPtr msg);
    double computeWeightedScoreSingleImage(const cv::Mat& img);
    double computeWeightedScoreSingleImageNORGBconv(const cv::Mat& img);

    sensor_msgs::msg::Image findNonOverlappingRegions(
        const sensor_msgs::msg::Image& curr_image, const geometry_msgs::msg::Pose& curr_pose,
        int curr_orientation, std::map<int, std::pair<geometry_msgs::msg::Pose, int>>& in_neighbor_poses_and_orientations);

    void allRobotsImagesPasted(cv::Mat& mask_returned, bool& gotallimages);
    void updateAllRobotsSelectionStatusCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg);

    double PoseEuclideanDistance(const geometry_msgs::msg::Pose& pose1, 
                                 const geometry_msgs::msg::Pose& pose2, 
                                 int cam_orient_1, int cam_orient_2);

    void saveImage(std::vector<ImageResponse>& response);
    void viewImage(std::vector<ImageResponse>& response);
    void viewImageCvMat(cv::Mat& image);
    void viewImageCvMat(cv::Mat& image, bool switchBGR_RGB_flag);
    void viewImageCvMat(cv::Mat& image, bool switchBGR_RGB_flag, const std::string& filename);
    void saveImageCvMat(cv::Mat& image, const std::string& filename);

    double calculateAngleFromOrientation(int cam_orient);
    void logTotalScoreCollectiveFOV();
    void logTotalScoreCollectiveFOV(double duration_last);
    bool areAllValuesTrue();


    template<typename MessageT>
    typename MessageT::SharedPtr waitForMessage(
        const std::string & topic,
        const std::chrono::duration<double> timeout);

    double horizon_;
    double n_time_steps_;

    int selected_action_ind_;
    bool finished_action_selection_;
    bool waiting_for_robots_to_finish_selection_;
    std::vector<int> in_neigh_ids_select_actions_;
    Eigen::MatrixXd actions_;
    Eigen::MatrixXd actions_unit_vec_;
    Eigen::Matrix<double, 3, 1> last_selected_displacement_;


    double camera_pitch_theta_;
    double camera_downward_pitch_angle_;
    int best_image_cam_orientation_;
    std::string img_save_path_;
    std::map<int, ImageInfoStruct> robots_ids_images_map_;


    double y_half_side_len_m_;
    double x_half_side_len_m_;
    double y_dir_px_m_scale_;
    double x_dir_px_m_scale_;
    double delta_C_;
    double diagonal_frame_dist_;
    double diagonal_px_scale_;
    int pasted_rect_height_pix_;
    int pasted_rect_width_pix_;
    bool take_new_pics_;

    geometry_msgs::msg::Pose frame_center_pose_;
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
