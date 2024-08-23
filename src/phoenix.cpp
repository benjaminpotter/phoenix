/// @file phoenix.cpp
///
/// @author Ben Potter
/// @date 21 Aug 2024

#include <ros/ros.h> // Depends on ROS for logging only.
#include "phoenix/phoenix.hpp"

namespace phx {

Phoenix::Phoenix() {

    // There is no point in handling a OpenSystem failure since it implies we
    // will never be able to connect with a camera.
    system = Arena::OpenSystem();
    device = nullptr;
}

Phoenix::~Phoenix() {
    shutdown();

    if(system != nullptr)
        Arena::CloseSystem(system);
}

void Phoenix::setup(config conf) {

    // Notes
    // - Take setup params via conf.
    // - Throw an exception if setup params are invalid.
    //   - Catch exception at the node level and report to user.
    // - Set the device state.
    // - Set camera params.
    // - Open camera stream.

    // FIXME: While updating devices, SIGINT does not happen?
    int timeout_ms = 1000;
    system->UpdateDevices(timeout_ms);

    std::vector<Arena::DeviceInfo> devices;
    devices = system->GetDevices();

    if(devices.size() == 0) {
        throw std::runtime_error("no devices found");
    }

    // Establish connection with device.
    // TODO: Catch GenICam exception that is thrown when the device IP is not
    // configured correctly. In this case, the device is found, but a call to
    // CreateDevice fails.
    Arena::DeviceInfo selected_device = devices[0];
    device = system->CreateDevice(selected_device);

    std::string device_model_name = (std::string) selected_device.ModelName();
    ROS_INFO("found device: %s", device_model_name.c_str());

    // Configure the camera stream.
    ROS_INFO("configuring device");

    // Set the acquisition mode.
    // TODO: What is acquisition mode?
    Arena::SetNodeValue<GenICam::gcstring>(
        device->GetNodeMap(),
        "AcquisitionMode",
        "Continuous");

    // Set the buffer handling mode.
    // When set to 'NewestOnly' the buffer will always stream the newest image
    // even if it requires discarding frames.
    Arena::SetNodeValue<GenICam::gcstring>(
        device->GetTLStreamNodeMap(),
        "StreamBufferHandlingMode",
        "NewestOnly");

    // Set the packet size method.
    // When set to 'StreamAutoNegotiatePacketSize' the device will request the 
    // largest packet size that the system will allow.
    Arena::SetNodeValue<bool>(
        device->GetTLStreamNodeMap(),
        "StreamAutoNegotiatePacketSize",
        true);

    // Set packet resend flag.
    // When set to 'true' the device will ensure that image UDP packets are 
    // retrieved and redelivered in the correct order.
    Arena::SetNodeValue<bool>(
        device->GetTLStreamNodeMap(),
        "StreamAutoNegotiatePacketSize",
        true);

    // Start the device streaming.
    device->StartStream();

}

boost::shared_ptr<frame> Phoenix::take() {

    // Notes
    // - Grab most recent image from buffer.
    // - Wrap the image data in a frame struct.
    // - Return a shared_ptr to the newly allocated struct.
    // - Throw an exception if the camera stream was interrupted.
    //   - Catch exception at the node level and report to user.

    // Time to wait for the device to send an image.
    int timeout_ms = 2000;

    // If an image is not received before timeout_ms, an exception is thrown.
    Arena::IImage *image = device->GetImage(timeout_ms);

    size_t width = image->GetWidth();
    size_t height = image->GetHeight();

    int32_t endianness = image->GetPixelEndianness();

    // This value is encoded as a 64b integer that represents a pixel format.
    uint64_t pixel_format = image->GetPixelFormat();

    size_t bits_per_pixel = image->GetBitsPerPixel();
    size_t bytes_per_pixel = bits_per_pixel / 8;

    uint64_t timestamp_ns = image->GetTimestampNs();

    // Get the pixel data from the image in bytes. The interpretation of the 
    // bytes is determined by the pixel format.
    const uint8_t *data = image->GetData();
    
    boost::shared_ptr<frame> f = boost::make_shared<frame>();
    f->data.insert(f->data.end(), &data[0], &data[width * height * bytes_per_pixel]);

    device->RequeueBuffer(image);

    return f; 
}

void Phoenix::shutdown() {

    if(device != nullptr) {
        device->StopStream();
        system->DestroyDevice(device);
        device = nullptr;
    }

}

}

