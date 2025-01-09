/// @file phoenix_node.cpp
///
/// @author Ben Potter
/// @date 21 Aug 2024

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <sensor_msgs/image_encodings.h>
#include "phoenix/phoenix_node.hpp"

namespace phx {

PhoenixNode::PhoenixNode():
    node()
{

    image_transport::ImageTransport image_transport(node);
    image_pub = image_transport.advertiseCamera("phoenix/image_raw", 1);

    frame_rate = 5;

}

PhoenixNode::~PhoenixNode() {

    camera.shutdown();
}

void PhoenixNode::spin() {
    ROS_INFO("start phoenix node");

    // Attempt to setup the camera.
    bool camera_did_init = setup_camera();

    ros::Rate loop_rate(frame_rate);
    while(camera_did_init && node.ok()) {

        take_and_publish();  
        // load_test_pattern_and_publish();

        loop_rate.sleep();
    }

    ROS_INFO("stop phoenix node");
    shutdown();
}

bool PhoenixNode::setup_camera() {
    ROS_INFO("init camera");

    config conf;

    std::string camera_name = "PHX050S-P";

    // Filepath to a camera info YAML.
    // Enables camera calibration pipeline.
    std::string camera_info_url = "";

    camera_info_manager.reset(
        new camera_info_manager::CameraInfoManager(node, camera_name, camera_info_url));

    // Checks if camera info has been loaded. If not, an attempt is made.
    // If it fails, this method returns false and the camera info is empty.
    // camera_info_manager->isCalibrated();
    
    // Camera info is published with each image when CameraPublisher is used.
    // This message contains information about the camera calibration.
    // Typically, this info is parsed from a YAML configuration file. Instead,
    // we choose to ignore this at present by setting defaults. This prevents
    // the node from warning the user that there is a configuration file 
    // could not be found.

    sensor_msgs::CameraInfo camera_info;
    camera_info.header.frame_id = "phoenix";
    camera_info.width = 2448;
    camera_info.height = 2048;
    
    camera_info_manager->setCameraInfo(camera_info);

    try {

        // The camera driver will throw an exception if the setup procedure
        // fails. We catch the problem here, and report it by returning T/F
        // based on whether the setup was successful.
        camera.setup(conf);

    } catch(std::runtime_error &e) {
        ROS_FATAL("camera failed to start: %s", e.what());
        return false;
    } catch(...) {
        ROS_FATAL("camera failed to start: unknown error");
        return false;
    }

    return true;
}

void PhoenixNode::take_and_publish() {
    ROS_DEBUG("take image");

    sensor_msgs::Image image_msg; 
    image_msg.header.frame_id = "phoenix";
    image_msg.header.stamp.sec = 0;
    image_msg.header.stamp.nsec = 0;
    image_msg.width = 2448;
    image_msg.height = 2048;
    image_msg.encoding = sensor_msgs::image_encodings::MONO8;
    image_msg.is_bigendian = false; // TODO: I am not sure what the byte ordering actually is.
    image_msg.step = 2448; // Image data is tightly packed.

    boost::shared_ptr<frame> f;
    f = camera.take();

    // TODO: This copies the frame vector into the image_msg vector. We want to
    // not do this because we already copied the data out of the image sent by 
    // the device.
    image_msg.data = f->data;
    
    sensor_msgs::CameraInfo camera_info;
    camera_info = camera_info_manager->getCameraInfo();
    camera_info.header.frame_id = image_msg.header.frame_id;
    camera_info.header.stamp = image_msg.header.stamp;

    image_pub.publish(image_msg, camera_info);
}

void PhoenixNode::load_test_pattern_and_publish() {
    ROS_INFO("test pattern image");

    // https://docs.ros.org/en/ros2_packages/rolling/api/sensor_msgs/interfaces/msg/Image.html
    sensor_msgs::Image image_msg; 
    image_msg.header.frame_id = "phoenix";
    image_msg.header.stamp.sec = 0;
    image_msg.header.stamp.nsec = 0;
    image_msg.width = 2448;
    image_msg.height = 2048;
    image_msg.encoding = sensor_msgs::image_encodings::MONO8;
    image_msg.is_bigendian = false; // TODO: I am not sure what the byte ordering actually is.
    image_msg.step = 2448; // Image data is tightly packed.

    // Generate test pattern: all pixels are grey.
    image_msg.data.resize(image_msg.step * image_msg.height);
    for(auto &pixel : image_msg.data)
        pixel = 0x80;
    
    sensor_msgs::CameraInfo camera_info;
    camera_info = camera_info_manager->getCameraInfo();
    camera_info.header.frame_id = image_msg.header.frame_id;
    camera_info.header.stamp = image_msg.header.stamp;

    image_pub.publish(image_msg, camera_info);
}

void PhoenixNode::shutdown() {
    
    camera.shutdown();
}

}


int main(int argc, char *argv[]) {

    ros::init(argc, argv, "phoenix");
    phx::PhoenixNode phx_node;

    boost::thread th(boost::bind(&ros::spin));

    phx_node.spin();

    ROS_INFO("node completed");

    return 0;
}
