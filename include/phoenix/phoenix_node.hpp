/// @file phoenix_node.hpp
///
/// @author Ben Potter
/// @date 21 Aug 2024

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include "phoenix/phoenix.hpp"

#pragma once

namespace phx {

/// 
/// @brief A ROS camera publisher interface.
///
class PhoenixNode {

public:
    PhoenixNode();
    virtual ~PhoenixNode();
    
    ///
    /// @brief Begin publishing images.
    ///
    void spin();

private:

    ///
    /// @brief Propagate camera parameters to camera interface, returning false
    /// on failure.
    ///
    bool setup_camera();

    ///
    /// @brief Capture an image and publish it.
    ///
    void take_and_publish();

    ///
    /// @brief Spin up a test image without polling the camera, and publish it.
    ///
    void load_test_pattern_and_publish();

    ///
    /// @brief Shutdown the node, cleaning up any dangling references.
    ///
    void shutdown();

private:
    
    ros::NodeHandle node;

    image_transport::CameraPublisher image_pub;

    // std::string camera_name;
    // std::string pixel_format_str;

    // We do not really _choose_ image size. These are intrinsic to the image
    // sensor size in the camera.
    
    // int image_width;
    // int image_height;

    int frame_rate;

    // int exposure;
    // int brightness;
    // int contrast;
    // int saturation;
    // int sharpness;
    // int focus;
    // int white_balance;
    // int gain;

    // bool auto_focus;
    // bool auto_exposure;
    // bool auto_white_balance;
    
    boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager;

    Phoenix camera;

};

}

