## Gazebo plugin programming

### [Control plugin](http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5)

A plugin is a C++ library that is loaded by Gazebo at runtime. A plugin has access to Gazebo's API, which allows a plugin to perform a wide variety of tasks including moving objects, adding/removing objects, and accessing sensor data.

#### Install gazebo headers
On some operating systems the development package must be installed prior to building a plugin.

```bash
sudo apt install libgazebo8-dev
```

#### Write a plugin
* Step 1: Create a workspace

```bash
mkdir ~/velodyne_plugin
cd ~/velodyne_plugin
```

* Step 2: Create the plugin source file

**velodyne_plugin.cc**

```C++
#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class VelodynePlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: VelodynePlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Just output a message for now
      std::cerr << "\nThe velodyne plugin is attach to model[" <<
        _model->GetName() << "]\n";
    }
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
#endif
```

* Step 3: Create the CMake build script

**CMakeLists.txt**

```bash
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(velodyne_plugin SHARED velodyne_plugin.cc)
target_link_libraries(velodyne_plugin ${GAZEBO_libraries})
```

* Step 4: Attach the plugin to the Velodyne sensor

We will utilize SDF's <include> capability to test out our plugin without touching the main Velodyne SDF file.

**velodyne.world**

```xml
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>
    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- A testing model that includes the Velodyne sensor model -->
    <model name="my_velodyne">
      <include>
        <uri>model://velodyne_hdl32</uri>
      </include>

      <!-- Attach the plugin to this model -->
      <plugin name="velodyne_control" filename="libvelodyne_plugin.so"/>
    </model>

  </world>
</sdf>
```

* Step 5: Build and test

```bash
mkdir build
cd build
cmake ..
make
cd ~/velodyne_plugin/build
gazebo --verbose ../velodyne.world
```

#### Plugin modification
* PID controller

**velodyne_plugin.cc**

```C++
public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Safety check
  if (_model->GetJointCount() == 0)
  {
    std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
    return;
  }

  // Store the model pointer for convenience.
  this->model = _model;

  // Get the first joint. We are making an assumption about the model
  // having one joint that is the rotational joint.
  this->joint = _model->GetJoints()[0];

  // Setup a P-controller, with a gain of 0.1.
  this->pid = common::PID(0.1, 0, 0);

  // Apply the P-controller to the joint.
  this->model->GetJointController()->SetVelocityPID(
      this->joint->GetScopedName(), this->pid);

  // Set the joint's target velocity. This target velocity is just
  // for demonstration purposes.
  this->model->GetJointController()->SetVelocityTarget(
      this->joint->GetScopedName(), 10.0);
}
```

* Plugin Configuration

We have hardcoded the rotational speed, however a configurable speed that doesn't require re-compilation would be better. In this section we'll modify the plugin to read a custom SDF parameter that is the target velocity of the Velodyne.

Start by adding a new element as a child of the <plugin>. The new element can be anything, as long as it is valid XML. Our plugin will have access to this value in the Load function.

```xml
<plugin name="velodyne_control" filename="libvelodyne_plugin.so">
  <velocity>25</velocity>
</plugin>
```

```C++
// Default to zero velocity
double velocity = 0;

// Check that the velocity element exists, then read the value
if (_sdf->HasElement("velocity"))
  velocity = _sdf->Get<double>("velocity");

// Set the joint's target velocity. This target velocity is just
// for demonstration purposes.
this->model->GetJointController()->SetVelocityTarget(
    this->joint->GetScopedName(), velocity);
```

* Plugin API

Adjusting the target velocity via SDF is very convenient, but it would be even better to support dynamic adjustments. 

There are two API types that we can use: message passing, and functions. Message passing relies on Gazebo's transport mechanism, and would involve creating a named topic on which a publisher can send double values. The plugin would receive these messages, containing a double, and set the velocity appropriately. Message passing is convenient for inter-process communication.

The function approach would create a new public function that adjusts the velocity. For this to work, a new plugin would inherit from our current plugin. The child plugin would be instantiated by Gazebo instead of our current plugin, and would control the velocity by calling our function. This type of approach is most often used when interfacing Gazebo to ROS.

**velodyne_plugin.cc**

```C++
#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class VelodynePlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: VelodynePlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Safety check
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
        return;
      }

      // Store the model pointer for convenience.
      this->model = _model;

      // Get the first joint. We are making an assumption about the model
      // having one joint that is the rotational joint.
      this->joint = _model->GetJoints()[0];

      // Setup a P-controller, with a gain of 0.1.
      this->pid = common::PID(0.1, 0, 0);

      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetVelocityPID(
          this->joint->GetScopedName(), this->pid);

      // Default to zero velocity
      double velocity = 0;

      // Check that the velocity element exists, then read the value
      if (_sdf->HasElement("velocity"))
        velocity = _sdf->Get<double>("velocity");

      this->SetVelocity(velocity);

      // Create the node
      this->node = transport::NodePtr(new transport::Node());
      #if GAZEBO_MAJOR_VERSION < 8
      this->node->Init(this->model->GetWorld()->GetName());
      #else
      this->node->Init(this->model->GetWorld()->Name());
      #endif

      // Create a topic name
      std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";

      // Subscribe to the topic, and register a callback
      this->sub = this->node->Subscribe(topicName,
         &VelodynePlugin::OnMsg, this);
    }

    /// \brief Set the velocity of the Velodyne
    /// \param[in] _vel New target velocity
    public: void SetVelocity(const double &_vel)
    {
      // Set the joint's target velocity.
      this->model->GetJointController()->SetVelocityTarget(
          this->joint->GetScopedName(), _vel);
    }

    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    private: void OnMsg(ConstVector3dPtr &_msg)
    {
      this->SetVelocity(_msg->x());
    }

    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;

    /// \brief A PID controller for the joint.
    private: common::PID pid;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
#endif
```

Testing code: **vel.cc**

```C++
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// Gazebo's API has changed between major releases. These changes are
// accounted for with #if..#endif blocks in this file.
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo as a client
#if GAZEBO_MAJOR_VERSION < 6
  gazebo::setupClient(_argc, _argv);
#else
  gazebo::client::setup(_argc, _argv);
#endif

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Publish to the  velodyne topic
  gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::Vector3d>("~/my_velodyne/vel_cmd");

  // Wait for a subscriber to connect to this publisher
  pub->WaitForConnection();

  // Create a a vector3 message
  gazebo::msgs::Vector3d msg;

  // Set the velocity in the x-component
#if GAZEBO_MAJOR_VERSION < 6
  gazebo::msgs::Set(&msg, gazebo::math::Vector3(std::atof(_argv[1]), 0, 0));
#else
  gazebo::msgs::Set(&msg, ignition::math::Vector3d(std::atof(_argv[1]), 0, 0));
#endif

  // Send the message
  pub->Publish(msg);

  // Make sure to shut everything down.
#if GAZEBO_MAJOR_VERSION < 6
  gazebo::shutdown();
#else
  gazebo::client::shutdown();
#endif
}
```

Add a few lines to the **CMakeLists.txt**

```bash
# Build the stand-alone test program
add_executable(vel vel.cc)
target_link_libraries(vel ${GAZEBO_LIBRARIES})
```

### [Plugin for ROS](http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i6)

* 1.Add a header files to the **velodyne_plugin.cc** file.

```C++
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
```

* 2.Add a few member variables to the plugin.

```C++
/// \brief A node use for ROS transport
private: std::unique_ptr<ros::NodeHandle> rosNode;

/// \brief A ROS subscriber
private: ros::Subscriber rosSub;

/// \brief A ROS callbackqueue that helps process messages
private: ros::CallbackQueue rosQueue;

/// \brief A thread the keeps running the rosQueue
private: std::thread rosQueueThread;
```

* 3.At the end of the Load function, add the following.

```C++
// Initialize ros, if it has not already bee initialized.
if (!ros::isInitialized())
{
  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "gazebo_client",
      ros::init_options::NoSigintHandler);
}

// Create our ROS node. This acts in a similar manner to
// the Gazebo node
this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

// Create a named topic, and subscribe to it.
ros::SubscribeOptions so =
  ros::SubscribeOptions::create<std_msgs::Float32>(
      "/" + this->model->GetName() + "/vel_cmd",
      1,
      boost::bind(&VelodynePlugin::OnRosMsg, this, _1),
      ros::VoidPtr(), &this->rosQueue);
this->rosSub = this->rosNode->subscribe(so);

// Spin up the queue helper thread.
this->rosQueueThread =
  std::thread(std::bind(&VelodynePlugin::QueueThread, this));
```

* 4.Two new functions: OnRosMsg and QueueThread

```C++
/// \brief Handle an incoming message from ROS
/// \param[in] _msg A float value that is used to set the velocity
/// of the Velodyne.
public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
{
  this->SetVelocity(_msg->data);
}

/// \brief ROS helper function that processes messages
private: void QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
```

* 5.Edit **CMakeLists.txt**

```bash
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(velodyne_plugin SHARED velodyne_plugin.cc)
target_link_libraries(velodyne_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES})

# Build the stand-alone test program
add_executable(vel vel.cc)

if (${gazebo_VERSION_MAJOR} LESS 6)
  include(FindBoost)
  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
  target_link_libraries(vel ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
else()
  target_link_libraries(vel ${GAZEBO_LIBRARIES})
endif()
```
