/*
NOTE: This script contains the algorithm for RAG
 */
#include "image_covering_coordination/total_score_collectiveFOV_logger.h"

TotalScoreCollectiveFovLogger::TotalScoreCollectiveFovLogger()
    : rclcpp::Node("total_score_collective_fov_logger")
{
    // Get ROS params
    this->declare_parameter<int>("number_of_drones", 1);
    this->get_parameter("number_of_drones", num_robots_);

    this->declare_parameter<std::string>("algorithm_to_run", "default");
    this->get_parameter("algorithm_to_run", algorithm_to_run_);

    this->declare_parameter<int>("experiment_number", 0);
    this->get_parameter("experiment_number", experiment_number_);

    this->declare_parameter<int>("num_nearest_neighbors", 3);
    this->get_parameter("num_nearest_neighbors", num_nearest_neighbors_);

    this->declare_parameter<int>("is_server_experiment", 0);
    this->get_parameter("is_server_experiment", is_server_experiment_);

    std::cout << "ALGORITHM: " << algorithm_to_run_ << std::endl;

    // async_spinner_.reset(new ros::AsyncSpinner(2));
    // async_spinner_->start();

    // ROS subscribers
    // NOTE!!! Subscribe to /clock topic for putting a callback which checks whether n_time_step_ amount of time has passed
    get_drones_images_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", 
        1,
        std::bind(&TotalScoreCollectiveFovLogger::getRobotsBestImages, this, std::placeholders::_1)
    );
    // check_drones_selection_sub_ = node_handle_.subscribe("/clock", 1, &TotalScoreCollectiveFovLogger::updateAllRobotsSelectionStatusCallback, this);

    fov_y_ = M_PI / 2.; // in rads; 90 deg horizontal fov
    fov_x_ = 1.093; // in rads; 62.6 deg vertical fov

    actions_unit_vec_.resize(3, 8);
    actions_unit_vec_ << 1., 0.707, 0., -0.707, -1., -0.707, 0., 0.707,
        0., 0.707, 1., 0.707, 0., -0.707, -1., -0.707,
        0., 0., 0., 0., 0., 0., 0., 0.;

    // CAMERA SETTINGS*******
    camera_pitch_theta_ = 0.2182; // 12.5 deg
    camera_downward_pitch_angle_ = -1.571; // DO NOT CHANGE! -90 deg
    y_half_side_len_m_ = 30.; //m
    x_half_side_len_m_ = 18.25; //m
    y_dir_px_m_scale_ = 10.67; // pixels per m in the y dir
    x_dir_px_m_scale_ = 13.15; // pixels per m in the x dir
    delta_C_ = 10.44; // m, displacement in center due to viewing angle when delta = 0.2182 rads
    diagonal_frame_dist_ = sqrt(pow((2. * y_half_side_len_m_), 2) + pow((2. * x_half_side_len_m_), 2)); // + 2.*delta_C_;  // shortest distance needed to consider overlap from in-neighbors
    diagonal_px_scale_ = sqrt(pow(y_dir_px_m_scale_, 2) + pow(x_dir_px_m_scale_, 2));
    pasted_rect_height_pix_ = 480;
    pasted_rect_width_pix_ = 640;
    take_new_pics_ = true;
    // CAMERA SETTINGS*******end

    start_ = std::chrono::high_resolution_clock::now(); // absolute time, wall clock
    // img_save_path_ = "/home/sgari/sandilya_ws/unity_airsim_workspace/AirSim/ros/src/image_covering_coordination/data/";
    // img_save_path_ = "/home/airsim_user/Airsim/ros/src/image_covering_coordination/data/";
    img_save_path_ = "/mnt/ros/logs/data/";

    // setting frame center pose
    geometry_msgs::Point point_cf;
    point_cf.x = -13.5;
    point_cf.y = 11.3;
    frame_center_pose_.position = point_cf;
    pic_name_counter_ = 0;

    for (int i = 0; i < num_robots_; i++) {
        all_robots_selected_map_[i + 1] = false; // initialized all to be false, no drone has initially selected an action
    }

    // Initialize logical timestamp
    unsigned long long logical_time = 0;
    time_elapsed_sum_ = 0.;
    t_minus_1_stamp_ = 0.;

    firstlogflag_ = true;
}

// destructor
TotalScoreCollectiveFovLogger::~TotalScoreCollectiveFovLogger()
{
    // writeDataToCSV();
    // async_spinner_->stop();
}


/////////////////////--------------------------------------------------------------------////////////////////////////

void TotalScoreCollectiveFovLogger::updateAllRobotsSelectionStatusCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
    /* SUBSCRIBES to marginal gain message which contains action selection flag */

    // auto timeout = rclcpp::Duration::from_seconds(2.5);
    for (int i = 0; i < num_robots_; i++) {
        int other_robot_id = i + 1;
        std::string mg_msg_topic = "Drone" + std::to_string(other_robot_id) + "/mg_rag_data";
        // std::cout << "MG MSG TOPIC: " << mg_msg_topic << std::endl;

        using ImageCoveringMsg = image_covering_coordination::msg::ImageCovering;
        auto mg_data_msg = waitForMessage<image_covering_coordination::msg::ImageCovering>(
            this->shared_from_this(),
            mg_msg_topic,
            std::chrono::seconds(2)
        );
        
        if (mg_data_msg) {
            image_covering_coordination::msg::ImageCovering mg_msg_topic_data = *mg_data_msg;
            all_robots_selected_map_[i + 1] = mg_msg_topic_data.flag_completed_action_selection;
            // std::cout <<  "got mg msg from robot " + std::to_string(other_robot_id) << std::endl;
        }
        else {
            // Handle the case where no message was received within the timeout
            all_robots_selected_map_[i + 1] = false;
            //    std::cout << "could not get mg msg from robot " + std::to_string(other_robot_id) << std::endl;
        }
    }
}

void TotalScoreCollectiveFovLogger::getRobotsBestImages(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
    /* SUBSCRIBES to marginal gain message which contains action selection flag */

    // auto timeout = rclcpp::Duration::from_seconds(2.5); // Timeout seconds
    for (int i = 0; i < num_robots_; i++) {
        int other_robot_id = i + 1;
        std::string mg_msg_topic = "Drone" + std::to_string(other_robot_id) + "/mg_rag_data";
        // std::cout << "MG MSG TOPIC: " << mg_msg_topic << std::endl;
        using ImageCoveringMsg = image_covering_coordination::msg::ImageCovering;
        auto mg_data_msg = waitForMessage<image_covering_coordination::msg::ImageCovering>(
            this->shared_from_this(),
            mg_msg_topic,
            std::chrono::seconds(2)
        );        

        if (mg_data_msg) {
            image_covering_coordination::msg::ImageCovering mg_msg_topic_data = *mg_data_msg;
            ImageInfoStruct image_info;
            image_info.pose = mg_msg_topic_data.pose;
            image_info.image = mg_msg_topic_data.image;
            image_info.camera_orientation = mg_msg_topic_data.camera_orientation_setting;
            // image_info.best_marginal_gain_val = mg_msg_topic_data.best_marginal_gain;
            robots_ids_images_map_[other_robot_id] = image_info;
            all_robots_selected_map_[i + 1] = mg_msg_topic_data.flag_completed_action_selection;
            if (all_robots_selected_map_[i + 1] == false) {
                // start_ = std::chrono::high_resolution_clock::now();
                return; //break out of function here itself
            }
        }
        else {
            all_robots_selected_map_[i + 1] = false;
            RCLCPP_WARN(this->get_logger(), "could nOT get mg msg from robot %d", other_robot_id);
        }
    }

    // check if all robots finished selecting actions and are publishing their latest best image
    bool robots_finished_selection = areAllValuesTrue();

    auto t1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = t1 - start_;
    double logging_interval = duration.count();

    if (robots_finished_selection) {
        // time_elapsed_sum_ += logging_interval;

        RCLCPP_INFO(this->get_logger(), "Logging interval: %f", logging_interval);
        // std::cout << "time elapsed: " << time_elapsed_sum_ << std::endl;

        double time_wo_imcol;
        if (firstlogflag_) {
            time_wo_imcol = logging_interval - 20.; // this just shifted to the left by 20 sec, not actually subtracting the 20sec imcol delay
            firstlogflag_ = false;
        }
        else {
            if (logging_interval > 20) {
                time_wo_imcol = t_minus_1_stamp_ + (logging_interval - 20.);
            }
            else {
                time_wo_imcol = t_minus_1_stamp_ + logging_interval;
            }
        }

        RCLCPP_INFO(this->get_logger(), "time_wo_imcol: %f", time_wo_imcol);

        logTotalScoreCollectiveFOV(time_wo_imcol); // function to calculate and log total score

        t_minus_1_stamp_ = time_wo_imcol; // recording this after logging; this is the last timestamp
        RCLCPP_INFO(this->get_logger(), "t-1 timestamp: %f", t_minus_1_stamp_);

        start_ = std::chrono::high_resolution_clock::now();
    }
    else {
        // start_ = std::chrono::high_resolution_clock::now();
        return;
    }
}

void TotalScoreCollectiveFovLogger::logTotalScoreCollectiveFOV(double duration_last)
{

    cv::Mat robots_all_pasted_img;
    bool got_all_images = true;
    allRobotsImagesPasted(robots_all_pasted_img, got_all_images);

    if (got_all_images == false) {
        return;
    }

    // std::string picname = "fullimage" + std::to_string(pic_name_counter_) + ".png";
    // saveImageCvMat(robots_all_pasted_img, picname);
    double total_score_collective_fov = computeWeightedScoreSingleImageNORGBconv(robots_all_pasted_img);
    // save to csv file: 0th column is timestamp, 1st column is the total_score_collective_fov
    score_data_.emplace_back(duration_last, total_score_collective_fov);
    pic_name_counter_++;

    RCLCPP_INFO(this->get_logger(), "total score: %f", total_score_collective_fov);
}

void TotalScoreCollectiveFovLogger::logTotalScoreCollectiveFOV()
{
    cv::Mat robots_all_pasted_img;
    bool got_all_images = true;
    allRobotsImagesPasted(robots_all_pasted_img, got_all_images);

    if (got_all_images == false) {
        return;
    }

    // std::string picname = "fullimage" + std::to_string(pic_name_counter_) + ".png";
    // saveImageCvMat(robots_all_pasted_img, picname);
    double total_score_collective_fov = computeWeightedScoreSingleImageNORGBconv(robots_all_pasted_img);

    auto t1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = t1 - start_;
    double time_elapsed = duration.count();
    // save to csv file: 0th column is timestamp, 1st column is the total_score_collective_fov
    score_data_.emplace_back(time_elapsed, total_score_collective_fov);
    pic_name_counter_++;

    RCLCPP_INFO(this->get_logger(), "total score: %f", total_score_collective_fov);
}

void TotalScoreCollectiveFovLogger::writeDataToCSV()
{
    // std::string filepath = img_save_path_ + "totalscore_"+ std::to_string(experiment_number_) + "_" + algorithm_to_run_ +  ".csv"; // w/o num nearest neighbors
    std::string filepath = img_save_path_ + "totalscore_" + std::to_string(experiment_number_) + "_" + algorithm_to_run_ + "_" + std::to_string(num_nearest_neighbors_) +"_"+ std::to_string(num_robots_)+ ".csv";
    std::ofstream file(filepath);

    if (file.is_open()) {
        // file << "Timestamp,TotalScore\n";  // puts the first row with string
        for (const auto& data : score_data_) {
            auto timestamp = data.first;
            file << timestamp << "," << std::fixed << std::setprecision(2) << data.second << "\n";
        }
        file.close();
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file '%s' for writing.", filepath.c_str());
    }
}

double TotalScoreCollectiveFovLogger::computeWeightedScoreSingleImage(const cv::Mat& img)
{
    // cv::cvtColor(img, img, cv::COLOR_BGR2RGB); // NOTE double check if you need to do this to input
    cv::Mat rgb_img;
    cv::cvtColor(img, rgb_img, cv::COLOR_BGR2RGB);  // 使用临时变量保存

    double score = 0.0;

    if (is_server_experiment_ == 1) {
        for (int r = 0; r < img.rows; ++r) {
            for (int c = 0; c < img.cols; ++c) {
                cv::Vec3b pixel = img.at<cv::Vec3b>(r, c);

                // Check if the pixel is not black
                if (pixel != cv::Vec3b(0, 0, 0)) {
                    score += 1.0; // Increment the score for each non-black pixel
                }
            }
        }
    }

    else {
        // Iterate through pixels
        for (int r = 0; r < img.rows; ++r) {
            for (int c = 0; c < img.cols; ++c) {

                cv::Vec3b pixel = img.at<cv::Vec3b>(r, c);

                // Check pixel color and add weighted score
                // if (pixel == cv::Vec3b(144, 9, 201)) { // road_1
                //     score += 1.;
                // }
                // else if (pixel == cv::Vec3b(130, 9, 200)) { // road_2
                //     score += 1.;
                // }
                // else if (pixel == cv::Vec3b(53, 73, 65)) { // pavement_1
                //     score += 0.05;
                // }
                // else if (pixel == cv::Vec3b(0, 72, 128)) { // pavement_2
                //     score += 0.05;
                // }

                // // for use with unity editor 46DEG X 60 DEG FOV!!!
                // if ((pixel == cv::Vec3b(32, 97, 80))) { // road_1
                //     score += 1.;
                // }
                // else if ((pixel == cv::Vec3b(176, 97, 24))) { // road_2
                //     score += 1.;
                // }
                // else if ((pixel == cv::Vec3b(0, 73, 194))) { // pavement_1
                //     score += 0.05;
                // }
                // else if ((pixel == cv::Vec3b(34, 97, 16))) { // pavement_2
                //     score += 0.05;
                // }

                // NOTE!! for use with unity EXECUTABLE 46DEG X 60 DEG FOV, 15 drones!!!
                if ((pixel == cv::Vec3b(50, 0, 128))) { // road_1
                    score += 1.;
                }
                else if ((pixel == cv::Vec3b(18, 73, 200))) { // road_2
                    score += 1.;
                }
                else if ((pixel == cv::Vec3b(18, 9, 128))) { // pavement_1
                    score += 0.05;
                }
                else if ((pixel == cv::Vec3b(128, 72, 128))) { // pavement_2
                    score += 0.05;
                }
            }
        }
    }

    return score;
}

double TotalScoreCollectiveFovLogger::computeWeightedScoreSingleImageNORGBconv(const cv::Mat& img)
{
    // cv::cvtColor(img, img, cv::COLOR_BGR2RGB); // NOTE double check if you need to do this to input
    double score = 0.0;

    if (is_server_experiment_ == 1) {
        for (int r = 0; r < img.rows; ++r) {
            for (int c = 0; c < img.cols; ++c) {
                cv::Vec3b pixel = img.at<cv::Vec3b>(r, c);

                // Check if the pixel is not black
                if (pixel != cv::Vec3b(0, 0, 0)) {
                    score += 1.0; // Increment the score for each non-black pixel
                }
            }
        }
    }

    else {
        // Iterate through pixels
        for (int r = 0; r < img.rows; ++r) {
            for (int c = 0; c < img.cols; ++c) {

                cv::Vec3b pixel = img.at<cv::Vec3b>(r, c);

                // for use with unity executable, 10 robots
                // if (pixel == cv::Vec3b(201, 9, 144)) { // road_1
                //     score += 1.;
                // }
                // else if (pixel == cv::Vec3b(200, 9, 130)) { // road_2
                //     score += 1.;
                // }
                // else if (pixel == cv::Vec3b(65, 73, 53)) { // pavement_1
                //     score += 0.05;
                // }
                // else if (pixel == cv::Vec3b(128, 72, 0)) { // pavement_2
                //     score += 0.05;
                // }

                // for use with unity editor 46DEG X 60 DEG FOV, 10 robots!!!
                // if ((pixel == cv::Vec3b(80, 97, 32))) { // road_1
                //     score += 1.;
                // }
                // else if ((pixel == cv::Vec3b(24, 97, 176))) { // road_2
                //     score += 1.;
                // }
                // else if ((pixel == cv::Vec3b(194, 73, 0))) { // pavement_1
                //     score += 0.05;
                // }
                // else if ((pixel == cv::Vec3b(16, 97, 34))) { // pavement_2
                //     score += 0.05;
                // }

                // NOTE!! for use with unity editor 46DEG X 60 DEG FOV, 15 robots!!! BGR
                // if ((pixel == cv::Vec3b(25, 97, 134))) { // road_1
                //     score += 1.;
                // }
                // else if ((pixel == cv::Vec3b(16, 41, 4))) { // road_2
                //     score += 1.;
                // }
                // else if ((pixel == cv::Vec3b(194, 73, 0))) { // pavement_1
                //     score += 0.05;
                // }
                // else if ((pixel == cv::Vec3b(25, 97, 132))) { // pavement_2
                //     score += 0.05;
                // }

                // NOTE!! for use with unity EXECUTABLE 46DEG X 60 DEG FOV, 15 drones!!!
                if ((pixel == cv::Vec3b(50, 0, 128))) { // road_1
                    score += 1.;
                }
                else if ((pixel == cv::Vec3b(18, 73, 200))) { // road_2
                    score += 1.;
                }
                else if ((pixel == cv::Vec3b(18, 9, 128))) { // pavement_1
                    score += 0.05;
                }
                else if ((pixel == cv::Vec3b(128, 72, 128))) { // pavement_2
                    score += 0.05;
                }
            }
        }
    }

    return score;
}

void TotalScoreCollectiveFovLogger::allRobotsImagesPasted(cv::Mat& mask_returned, bool& gotallimages)
{
    // PSEUDOCODE get each image, cam orient, and pose from robots_ids_images_map_ and paste them onto a mask at their correct poses and angles

    // FIND RECTANGLE CORNERS*****
    std::map<int, std::pair<geometry_msgs::msg::Pose, int>> robot_poses_and_orientations;
    for (const auto& robot : robots_ids_images_map_) {
        std::pair<geometry_msgs::msg::Pose, int> pose_cam_orient;
        pose_cam_orient.first = robot.second.pose;
        pose_cam_orient.second = robot.second.camera_orientation;
        robot_poses_and_orientations[robot.first] = pose_cam_orient;
    }
    std::vector<std::vector<cv::Point>> box_points_vec;
    for (const auto& robot : robot_poses_and_orientations) {

        if (robot.second.second == -1) {
            continue;
        }

        // get pixel coordinates of the inn rectangle relative to center_mask as center
        double dx_m = (robot.second.first.position.x + (delta_C_ * actions_unit_vec_(0, robot.second.second))) - (frame_center_pose_.position.x); // in m
        double dy_m = (robot.second.first.position.y + (delta_C_ * actions_unit_vec_(1, robot.second.second))) - (frame_center_pose_.position.y); // in m

        double dx_pix = dx_m * x_dir_px_m_scale_;
        double dy_pix = dy_m * y_dir_px_m_scale_;

        double rect_center_x_pix = dx_pix;
        double rect_center_y_pix = -dy_pix;
        cv::Point2f center_rect(rect_center_x_pix, rect_center_y_pix);
        double angle_rect = calculateAngleFromOrientation(robot.second.second); // remember to negate; this is in deg

        // get a BoxPoints or RotatedRect object based on the robot's pose and camera orientation in pixel coords
        // Create the rotated rectangle
        cv::RotatedRect rect(center_rect, cv::Size2f(pasted_rect_width_pix_, pasted_rect_height_pix_), angle_rect);
        std::vector<cv::Point2f> box_points(4);
        rect.points(box_points.data());

        // convert Point2f to Point
        std::vector<cv::Point> polygon_points(box_points.begin(), box_points.end());
        box_points_vec.push_back(polygon_points);
    }

    // Initialize min and max bounds with extreme values
    double min_x = std::numeric_limits<double>::max(), max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max(), max_y = std::numeric_limits<double>::lowest();

    // Iterate through all box points to find min and max bounds
    for (const auto& points : box_points_vec) {
        for (const auto& point : points) {
            min_x = std::min(min_x, static_cast<double>(point.x));
            max_x = std::max(max_x, static_cast<double>(point.x));
            min_y = std::min(min_y, static_cast<double>(point.y));
            max_y = std::max(max_y, static_cast<double>(point.y));
        }
    }

    // if ((min_x < 0) || (max_x < 0) || (min_y < 0) || (max_y < 0))
    // {
    //     gotallimages = false;
    //     return; // exit function
    // }
    // FIND RECTANGLE CORNERS*****end

    // Calculate the width and height required for the mask
    int mask_width = static_cast<int>(max_x - min_x);
    int mask_height = static_cast<int>(max_y - min_y);

    if ((mask_width < 0) || (mask_height < 0)) {
        gotallimages = false;
        return; // exit function
    }

    // Optionally add padding to the mask
    int padding = static_cast<int>(diagonal_px_scale_ * diagonal_frame_dist_);
    cv::Mat mask(mask_height + 2 * padding, mask_width + 2 * padding, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Point2f center_mask((mask.cols - 1) / 2.0, (mask.rows - 1) / 2.0);

    for (const auto& robot : robots_ids_images_map_) {
        const auto& robot_img_info = robot.second; // Assuming this has an image, pose, and orientation

        if (robot_img_info.camera_orientation == -1) {
            continue;
        }

        cv_bridge::CvImagePtr cv_ptr;
        cv::Mat img; // Your code to convert robot_img_info.image to cv::Mat
        cv_ptr = cv_bridge::toCvCopy(robot.second.image, robot.second.image.encoding);
        img = cv_ptr->image;
        cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

        // Calculate the robot's image center in the mask coordinates
        auto pose_cam_orient = robot_poses_and_orientations[robot.first];
        double dx_m = (pose_cam_orient.first.position.x + (delta_C_ * actions_unit_vec_(0, pose_cam_orient.second))) - (frame_center_pose_.position.x); // in m
        double dy_m = (pose_cam_orient.first.position.y + (delta_C_ * actions_unit_vec_(1, pose_cam_orient.second))) - (frame_center_pose_.position.y); // in m

        double dx_pix = dx_m * x_dir_px_m_scale_;
        double dy_pix = dy_m * y_dir_px_m_scale_;

        cv::Point2f center_rect(center_mask.x + dx_pix, center_mask.y - dy_pix); // Adjusted for mask center

        double angle = calculateAngleFromOrientation(pose_cam_orient.second); // Function to get angle from orientation
        cv::Mat rot = cv::getRotationMatrix2D(cv::Point2f(img.cols / 2.0, img.rows / 2.0), -angle, 1.0); // Rotate around image center
        // cv::Mat rot = cv::getRotationMatrix2D(center_rect, -angle, 1.0);   // rotate around transformed center
        cv::Rect2f bbox = cv::RotatedRect(center_rect, img.size(), -angle).boundingRect2f();

        // adjust transformation matrix
        rot.at<double>(0, 2) += bbox.width / 2.0 - img.cols / 2.0;
        rot.at<double>(1, 2) += bbox.height / 2.0 - img.rows / 2.0;

        cv::Mat rotated;
        cv::warpAffine(img, rotated, rot, bbox.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);

        rotated.copyTo(mask(cv::Rect(center_rect.x - rotated.cols / 2,
                                     center_rect.y - rotated.rows / 2,
                                     rotated.cols,
                                     rotated.rows)));

        // viewImageCvMat(mask, false);
        // saveImageCvMat(mask, "debug1.png");
    }

    // sensor_msgs::Image robots_all_pasted_img;
    // cv_bridge::CvImage cv_image{ cv_ptr->header, cv_ptr->encoding, mask};
    // cv_image.toImageMsg(robots_all_pasted_img);

    // return mask;
    mask_returned = mask;
}

void TotalScoreCollectiveFovLogger::saveImage(std::vector<ImageResponse>& response)
{
    for (const ImageResponse& image_info : response) {
        RCLCPP_INFO(this->get_logger(), "Image uint8 size: %zu", image_info.image_data_uint8.size());
        RCLCPP_INFO(this->get_logger(), "Image float size: %zu", image_info.image_data_float.size());

        std::string file_path = FileSystem::combine(img_save_path_, std::to_string(image_info.time_stamp));
        if (image_info.pixels_as_float) {
            Utils::writePFMfile(image_info.image_data_float.data(), image_info.width, image_info.height, file_path + ".pfm");
        }
        else {
            std::ofstream file(file_path + ".png", std::ios::binary);
            file.write(reinterpret_cast<const char*>(image_info.image_data_uint8.data()), image_info.image_data_uint8.size());
            file.close();
        }
    }
}

void TotalScoreCollectiveFovLogger::saveImageCvMat(cv::Mat& image, const std::string filename)
{
    // Save the image as a PNG file
    std::string path_filename = img_save_path_ + filename;
    cv::imwrite(path_filename, image);
}

void TotalScoreCollectiveFovLogger::viewImage(std::vector<ImageResponse>& response)
{
    for (const ImageResponse& image_info : response) {
        RCLCPP_INFO(this->get_logger(), "Image uint8 size: %zu", image_info.image_data_uint8.size());
        RCLCPP_INFO(this->get_logger(), "Image float size: %zu", image_info.image_data_float.size());


        cv::Mat image;
        if (image_info.pixels_as_float) {
            // Allocate memory for uint8 image
            cv::Mat float_image(image_info.height, image_info.width, CV_32FC3, const_cast<float*>(image_info.image_data_float.data()));

            // Convert to uint8 range [0, 255] and CV_8UC3 format
            cv::convertScaleAbs(float_image, image, 255.0);
            image.convertTo(image, CV_8UC3);
        }
        else {
            // Assuming image_data_uint8 stores uint8 data in BGR format
            image = cv::Mat(image_info.height, image_info.width, CV_8UC3, const_cast<uchar*>(image_info.image_data_uint8.data())).clone();
        }

        // Flip the image vertically
        cv::flip(image, image, 0);

        // If image is in BGR format, swap R and B channels
        if (image.channels() == 3 && image_info.pixels_as_float == false) {
            cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
        }

        cv::imshow("Image", image);
        cv::waitKey(0); // Wait for a key press
    }
}

void TotalScoreCollectiveFovLogger::viewImageCvMat(cv::Mat& image)
{
    // Flip the image vertically
    // cv::flip(image, image, 0);

    // If image is in BGR format, swap R and B channels
    if (image.channels() == 3) {
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    }

    cv::imshow("Image", image);
    cv::waitKey(0); // Wait for a key press
}

// override
void TotalScoreCollectiveFovLogger::viewImageCvMat(cv::Mat& image, bool switchBGR_RGB_flag)
{
    // Flip the image vertically
    // cv::flip(image, image, 0);

    // If image is in BGR format, swap R and B channels
    if ((image.channels() == 3) && (switchBGR_RGB_flag)) {
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    }

    cv::imshow("Image", image);
    cv::waitKey(0); // Wait for a key press
}

void TotalScoreCollectiveFovLogger::viewImageCvMat(cv::Mat& image, bool switchBGR_RGB_flag, const std::string filename)
{
    // If image is in BGR format, swap R and B channels
    if ((image.channels() == 3) && (switchBGR_RGB_flag)) {
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    }

    // Show the image
    cv::imshow("Image", image);
    cv::waitKey(0); // Wait for a key press

    // Save the image as a PNG file
    std::string path_filename = img_save_path_ + filename;
    cv::imwrite(path_filename, image);
}

double TotalScoreCollectiveFovLogger::PoseEuclideanDistance(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2, int cam_orient_1, int cam_orient_2)
{
    /* 
    cam_orient_1: camera orientation of pose1
    cam_orient_2: camera orientation of pose2
     */
    // has to account for delta C with image orient

    double cx_p1 = delta_C_ * actions_unit_vec_(0, cam_orient_1);
    double cy_p1 = delta_C_ * actions_unit_vec_(1, cam_orient_1);

    double cx_p2 = delta_C_ * actions_unit_vec_(0, cam_orient_2);
    double cy_p2 = delta_C_ * actions_unit_vec_(1, cam_orient_2);

    double dx = (pose1.position.x + cx_p1) - (pose2.position.x + cx_p2);
    double dy = (pose1.position.y + cy_p1) - (pose2.position.y + cy_p2);
    double dz = pose1.position.z - pose2.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double TotalScoreCollectiveFovLogger::calculateAngleFromOrientation(int cam_orient)
{

    double angle_deg;
    if ((cam_orient == 0) || (cam_orient == 4)) {
        angle_deg = 0.;
    }
    else if ((cam_orient == 1) || (cam_orient == 5)) {
        angle_deg = 45.;
    }
    else if ((cam_orient == 2) || (cam_orient == 6)) {
        angle_deg = 90.;
    }
    else if ((cam_orient == 3) || (cam_orient == 7)) {
        angle_deg = 135.;
    }

    return angle_deg;
}

bool TotalScoreCollectiveFovLogger::areAllValuesTrue()
{
    for (const auto& pair : all_robots_selected_map_) {
        if (!pair.second) {
            return false; // Found a false value, return false
        }
    }
    return true; // All values are true
}

template<typename MessageT>
typename MessageT::SharedPtr waitForMessage(
        const std::string & topic,
        const std::chrono::duration<double> timeout)
{
    // Create a promise and future to asynchronously wait for the incoming message
    auto promise = std::make_shared<std::promise<typename MessageT::SharedPtr>>();
    auto future = promise->get_future();
    // Create a subscriber that sets the received message into the promise
    auto subscription = this->create_subscription<MessageT>(
        topic,
        10,  // Queue size (adjustable as needed)
        [promise](typename MessageT::SharedPtr msg) {
            // Check if promise hasn't been set yet, then set received message
            if (promise->get_future().valid()) {
                promise->set_value(msg);
            }
        }
    );
    // Record start time for timeout tracking
    auto start_time = std::chrono::steady_clock::now();
    // Define loop rate (frequency) for spinning
    rclcpp::WallRate loop_rate(50);  // Spin at 50 Hz, adjust as desired
    typename MessageT::SharedPtr result_msg = nullptr;
    // Loop continuously until the message is received or timeout occurs
    while (rclcpp::ok()) {
        // Execute pending callbacks
        rclcpp::spin_some(this->get_node_base_interface());
        // Check if future is ready (message received)
        if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            result_msg = future.get();  // Retrieve the received message
            break;
        }
        // Check if timeout has been reached
        if (std::chrono::steady_clock::now() - start_time > timeout) {
            // Timeout reached without receiving a message
            break;
        }
        // Sleep to maintain loop frequency
        loop_rate.sleep();
    }
    // Return the received message or nullptr if timeout occurred
    return result_msg;
}




int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TotalScoreCollectiveFovLogger>(); 
    rclcpp::spin(node);
    node->writeDataToCSV();
    rclcpp::shutdown();
    return 0;
}

