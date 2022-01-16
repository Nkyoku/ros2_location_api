# ros2_location_api
ROS2 wrapper package of Windows Location API

## Installation
1. Install Visual Studio 2019 or later.
1. Install ROS2 foxy or later.
1. Clone this repository.
1. Build this repository.  
`colcon build --merge-install --packages-select location_api`

## Usage
1. Allow Apps to access your location.  
   These settings are in `Location` of `Privacy & security` of Settings apps.
1. Source this package.  
   `./install/local_setup.ps1`
1. Run the publisher node.  
   `ros2 run location_api publisher`
1. There are several parameters.
   - interval_in_ms : int  
   Period in milliseconds of Location API's event notification.  
   If interval_in_ms is zero, location sensor's default interval will be used.
   - frame_id : string  
   `header.frame_id` value of the sensor_msgs/NavSatFix message.
   - topic_name : string  
   Topic name that the publisher node publishes.
