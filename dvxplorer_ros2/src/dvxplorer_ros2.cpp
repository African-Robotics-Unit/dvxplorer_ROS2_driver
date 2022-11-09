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
		this->reset_pub_ = this->create_publisher<builtin_interfaces::msg::Time>((ns + "/reset_timestamps").c_str(), 1);
	}
	else {
		this->reset_sub_ = this->create_subscription<builtin_interfaces::msg::Time>((ns + "/reset_timestamps").c_str(), 1, std::bind(&DvXplorer::resetTimestampsCallback, this));
	}

	// Open device.
	caerConnect();

	this->imu_calibration_sub_ = this->create_subscription((ns + "/calibrate_imu").c_str(), 1, std::bind(&DvXplorer::imuCalibrationCallback, this));

	// start timer to reset timestamps for synchronization
	if (reset_timestamps_delay > 0.0) {
		timestamp_reset_timer_
			= this-> create_wall_timer(std::chrono::duration<float>(reset_timestamps_delay), std:bind(&DvXplorer::resetTimerCallback, this));
        RCLCPP_INFO_STREAM(this->get_logger(), "Started timer to reset timestamps on master DVS for synchronization (delay=%3.2fs).", reset_timestamps_delay);
	}
}

// Destructor
DvXplorer::~DvXplorer() {
	if (this->running_) {
		RCLCPP_INFO_STREAM(this->get_logger(), "Shutting down threads");
		this->running_ = false;
		this->readout_thread_.join();
		RCLCPP_INFO_STREAM("Threads stopped");
		caerLog(CAER_LOG_ERROR, "Destructor", "Data stop now");
		caerDeviceDataStop(this->dvxplorer_handle_);
		caerDeviceClose(&this->dvxplorer_handle_);
	}
}

void DvXplorer::dataStop() {
	caerLog(CAER_LOG_INFO, "Exiting from driver node", "Executing data stop");
	RCLCPP_INFO_STREAM(this->get_logger(), "Exiting from driver node, executing data stop");
	caerDeviceDataStop(this->dvxplorer_handle_);
	caerDeviceClose(&this->dvxplorer_handle_);
}

void DvXplorer::caerConnect() {
	// start driver
	bool device_is_running = false;
	while (!device_is_running) {
		const char *serial_number_restrict = (this->device_id_.empty()) ? nullptr : this->device_id_.c_str();

		if (serial_number_restrict) {
			RCLCPP_INFO("Requested serial number: %s", this->device_id_.c_str());
		}

		this->dvxplorer_handle_ = caerDeviceOpen(1, CAER_DEVICE_DVXPLORER, 0, 0, serial_number_restrict);

		// was opening successful?
		device_is_running = (this->dvxplorer_handle_ != nullptr);

		if (!device_is_running) {
			RCLCPP_INFO("Could not find DVXplorer. Will retry every second.");

            rclcpp::Rate sleepRate(std::chrono::seconds(1));
            sleepRate.sleep();
            //TO DO: COUNTER TO TRY OPENING DEVICE, ELSE BREAK
            //rclcpp::Executor::spin_node_once(this, )	
			//rclcpp::Executor::spinOnce();
            //ros::spinOnce();
		}

		if (!rclcpp::ok()) {
			return;
		}
	}

	this->dvxplorer_info_ = caerDVXplorerInfoGet(this->dvxplorer_handle_);
	this->device_id_      = "DVXplorer-" + std::string(this->dvxplorer_info_.deviceSerialNumber);

	RCLCPP_INFO(this->get_logger(), "%s --- ID: %d, Master: %d, DVS X: %d, DVS Y: %d, Firmware: %d, Logic: %d.\n",
		this->dvxplorer_info_.deviceString, this->dvxplorer_info_.deviceID, this->dvxplorer_info_.deviceIsMaster,
		this->dvxplorer_info_.dvsSizeX, this->dvxplorer_info_.dvsSizeY, this->dvxplorer_info_.firmwareVersion,
		this->dvxplorer_info_.logicVersion);

	if (this->master_ && !this->dvxplorer_info_.deviceIsMaster) {
		RCLCPP_INFO(this->get_logger(),"Device %s should be master, but is not!", this->device_id_.c_str());
	}

	// Send the default configuration before using the device.
	// No configuration is sent automatically!
	caerDeviceSendDefaultConfig(this->dvxplorer_handle_);

	// spawn threads
	this->running_ = true;
	this->readout_thread_
		= std::shared_ptr<boost::thread>(new boost::thread(std::bind(&DvXplorer::readout, this)));
	
    //TO DO!!!
    // camera info handling
    //ros::NodeHandle nh_ns(ns)
	//camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(nh_ns, device_id_));

	// initialize timestamps
	resetTimestamps();
}


void DvXplorer::onDisconnectUSB(void *driver) {
	RCLCPP_ERROR(this->get_logger(), "USB connection lost with DVS!");
	static_cast<dvxplorer_ros2_driver::DvXplorer *>(driver)->caerConnect();
}

void DvXplorer::resetTimestamps() {
	caerDeviceConfigSet(this->dvxplorer_handle_, DVX_MUX, DVX_MUX_TIMESTAMP_RESET, 1);
	this->reset_time_ = now();

	RCLCPP_INFO_STREAM(this->get_logger(), "Reset timestamps on %s to %.9f.", this->device_id_.c_str(), reset_time_.toSecs());
    //RCLCPP_INFO_STREAM(this->get_logger(), "Reset timestamps on %s to %.9f.", this->device_id_.c_str(), reset_time_.sec);

	// if master, publish reset time to slaves
	if (this->master_) {
		builtin_interfaces::msg::Time reset_msg;
		reset_msg = this->reset_time_;
		this->reset_pub_.publish(reset_msg);
	}
}

void DvXplorer::resetTimestampsCallback(const rclcpp::Time &ts) {
	// if slave, only adjust offset time
	if (!this->dvxplorer_info_.deviceIsMaster) {
		RCLCPP_INFO_STREAM(this->get_logger(), "Adapting reset time of master on slave %s.", this->device_id_.c_str());
		this->reset_time_ = ts;
	}
	// if master, or not single camera configuration, just reset timestamps
	else {
		resetTimestamps();
	}
}

void DvXplorer::imuCalibrationCallback(const std_msgs::Empty::ConstPtr &msg) {
	RCLCPP_INFO_STREAM(this->get_logger(), "Starting IMU calibration with %d samples.", (int) this->imu_calibration_sample_size_);
	this->imu_calibration_running_ = true;
	this->imu_calibration_samples_.clear();
}


void DvXplorer::resetTimerCallback() {
	timestamp_reset_timer_.stop();
	resetTimestamps();
}

void DvXplorer::updateImuBias() {
	this->bias.linear_acceleration.x = 0.0;
	this->bias.linear_acceleration.y = 0.0;
	this->bias.linear_acceleration.z = 0.0;
	this->bias.angular_velocity.x    = 0.0;
	this->bias.angular_velocity.y    = 0.0;
	this->bias.angular_velocity.z    = 0.0;

	for (const auto &m : this->imu_calibration_samples_) {
		this->bias.linear_acceleration.x += m.linear_acceleration.x;
		this->bias.linear_acceleration.y += m.linear_acceleration.y;
		this->bias.linear_acceleration.z += m.linear_acceleration.z;
		this->bias.angular_velocity.x += m.angular_velocity.x;
		this->bias.angular_velocity.y += m.angular_velocity.y;
		this->bias.angular_velocity.z += m.angular_velocity.z;
	}

	this->bias.linear_acceleration.x /= (double) this->imu_calibration_samples_.size();
	this->bias.linear_acceleration.y /= (double) this->imu_calibration_samples_.size();
	this->bias.linear_acceleration.z /= (double) this->imu_calibration_samples_.size();
	this->bias.linear_acceleration.z -= this->STANDARD_GRAVITY * sgn(this->bias.linear_acceleration.z);

	this->bias.angular_velocity.x /= (double) this->imu_calibration_samples_.size();
	this->bias.angular_velocity.y /= (double) this->imu_calibration_samples_.size();
	this->bias.angular_velocity.z /= (double) this->imu_calibration_samples_.size();

	RCLCPP_INFO_STREAM(this->get_logger(), "IMU calibration done.");
	RCLCPP_INFO_STREAM(this->get_logger(), "Acceleration biases: %1.5f %1.5f %1.5f [m/s^2]", this->bias.linear_acceleration.x, this->bias.linear_acceleration.y,
		this->bias.linear_acceleration.z);
	RCLCPP_INFO_STREAM(this->get_logger(), "Gyroscope biases: %1.5f %1.5f %1.5f [rad/s]", this->bias.angular_velocity.x, this->bias.angular_velocity.y,
		this->bias.angular_velocity.z);
}














}

