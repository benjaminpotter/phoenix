/// @file phoenix.hpp
///
/// @author Ben Potter
/// @date 21 Aug 2024

#include <boost/shared_ptr.hpp>
#include <ArenaApi.h>

#pragma once

namespace phx {

typedef struct frame {

    std::vector<uint8_t> data;

} frame;

typedef struct config {

    // Only contains values which are set by the user.
    

} config;

class Phoenix {

public:
    Phoenix();
    virtual ~Phoenix();

    ///
    /// @brief Apply user configuration and open camera stream.
    ///
    void setup(config conf);

    ///
    /// @brief Return a shared pointer to the latest camera frame.
    ///
    boost::shared_ptr<frame> take();

    ///
    /// @brief Close camera stream and perform clean up.
    ///
    void shutdown();


private:

    // DeviceState state;

    // ArenaSDK uses raw pointers for these objects.
    Arena::ISystem *system;
    Arena::IDevice *device;

};

}

