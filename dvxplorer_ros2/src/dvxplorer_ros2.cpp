#include "dvxplorer_ros2/dvxplorer_driver.hpp"

namespace dvxplorer_ros2_driver {

// Constructor        
DvXplorer::DvXplorer() :
	Node("dvxplorer_node"), 
    imu_calibration_running_(false) {
	
    // load parameters
    //parameters
    this->declare_parameter("serial_number", std::string(" "));
    this->get_parameter("serial_number", this->device_id_);
    RCLCPP_INFO_STREAM(this->get_logger(), "serial_number: " << this->device_id);
	
    this->declare_parameter("master", true);
    this->get_parameter("master", this->master_);
    RCLCPP_INFO_STREAM(this->get_logger(), "master? " << this->master_);
    
    float reset_timestamps_delay;
    this->declare_parameter("reset_timestamps_delay", -1.0f);
    this->get_parameter("reset_timestamps_delay", reset_timestamps_delay);
    RCLCPP_INFO_STREAM(this->get_logger(), "reset_timestamps_delay: " << reset_timestamps_delay);

    this->declare_parameter("imu_calibration_sample_size", 1000);
    this->get_parameter("imu_calibration_sample_size", reset);
    RCLCPP_INFO_STREAM(this->get_logger(), "imu_calibration_sample_size: " << imu_calibration_sample_size_);

    
	// initialize IMU bias
    this->declare_parameter("imu_bias/ax", 0.0f);
    this->get_parameter("imu_bias/ax", this->bias.linear_acceleration.x);
    RCLCPP_INFO_STREAM(this->get_logger(), "imu_bias/ax: " << this->bias.linear_acceleration.x);

    this->declare_parameter("imu_bias/ay", 0.0f);
    this->get_parameter("imu_bias/ay", this->bias.linear_acceleration.y);
    RCLCPP_INFO_STREAM(this->get_logger(), "imu_bias/ay: " << this->bias.linear_acceleration.y);

    this->declare_parameter("imu_bias/az", 0.0f);
    this->get_parameter("imu_bias/az", this->bias.linear_acceleration.z);
    RCLCPP_INFO_STREAM(this->get_logger(), "imu_bias/az: " << this->bias.linear_acceleration.z);

    this->declare_parameter("imu_bias/wx", 0.0f);
    this->get_parameter("imu_bias/wx", this->bias.angular_velocity.x);
    RCLCPP_INFO_STREAM(this->get_logger(), "imu_bias/wx: " << this->bias.angular_velocity.x);

    this->declare_parameter("imu_bias/wy", 0.0f);
    this->get_parameter("imu_bias/wy", this->bias.angular_velocity.y);
    RCLCPP_INFO_STREAM(this->get_logger(), "imu_bias/wy: " << this->bias.angular_velocity.y);

    this->declare_parameter("imu_bias/wz", 0.0f);
    this->get_parameter("imu_bias/wz", this->bias.angular_velocity.z);
    RCLCPP_INFO_STREAM(this->get_logger(), "imu_bias/wz: " << this->bias.angular_velocity.z);

	// set namespace
	ns = this->get_namespace();
	if (ns == "/") {
		ns = "/dv";
	}

    // Initialise Publishers
    this->event_array_pub_ = this->create_publisher<dvxplorer_interfaces::msg::EventArray>(ns + "/events", 10);
    this->camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(ns + "/camera_info", 1);
	this->imu_pub_ =  this->create_publisher<sensor_msgs::msg::Imu>(ns + "/imu", 10);

	// reset timestamps is publisher as master, subscriber as slave
	if (master_) {
		reset_pub_ = nh_.advertise<std_msgs::Time>((ns + "/reset_timestamps").c_str(), 1);
	}
	else {
		reset_sub_
			= nh_.subscribe((ns + "/reset_timestamps").c_str(), 1, &DvxplorerRosDriver::resetTimestampsCallback, this);
	}

	// Open device.
	caerConnect();
    
    



















	imu_calibration_sub_ = nh_.subscribe((ns + "/calibrate_imu").c_str(), 1, &DvxplorerRosDriver::imuCalibrationCallback, this);

	// start timer to reset timestamps for synchronization
	if (reset_timestamps_delay > 0.0) {
		timestamp_reset_timer_
			= nh_.createTimer(ros::Duration(reset_timestamps_delay), &DvxplorerRosDriver::resetTimerCallback, this);
		ROS_INFO("Started timer to reset timestamps on master DVS for synchronization (delay=%3.2fs).",
			reset_timestamps_delay);
	}
}

}