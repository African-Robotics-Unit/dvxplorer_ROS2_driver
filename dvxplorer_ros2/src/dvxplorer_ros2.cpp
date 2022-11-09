#include "dvxplorer_ros2/dvxplorer_driver.hpp"

namespace dvxplorer_ros2_driver {

// Constructor        
DvXplorer::DvXplorer() :
	Node("dvxplorer_node"), 
    imu_calibration_running_(false) {
	
    this->initCam();


    // Initialise Publishers
    this->event_array_pub_ = this->create_publisher<dvxplorer_interfaces::msg::EventArray>("/dv/events", 10);
    this->cam_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/dv/camera_info", 1);
	this->imu_pub_ =  this->create_publisher<sensor_msgs::msg::Imu>("/dv/imu", 10);

    // Setup camera info manager
    this->cam_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);

	// reset timestamps is publisher as master, subscriber as slave
	if (this->master_) {
		this->reset_pub_ = this->create_publisher<builtin_interfaces::msg::Time>(("/dv/reset_timestamps").c_str(), 1);
	}
	else {
		this->reset_sub_ = this->create_subscription<builtin_interfaces::msg::Time>(("/dv/reset_timestamps").c_str(), 1, std::bind(&DvXplorer::resetTimestampsCallback, this));
	}

	// Open device.
	caerConnect();

	this->imu_calibration_sub_ = this->create_subscription(("/dv/calibrate_imu").c_str(), 1, std::bind(&DvXplorer::imuCalibrationCallback, this));

	// start timer to reset timestamps for synchronization
	if (this->reset_timestamps_delay > 0.0) {
		timestamp_reset_timer_
			= this-> create_wall_timer(std::chrono::duration<float>(this->reset_timestamps_delay), std:bind(&DvXplorer::resetTimerCallback, this));
        RCLCPP_INFO_STREAM(this->get_logger(), "Started timer to reset timestamps on master DVS for synchronization (delay=%3.2fs).", this->reset_timestamps_delay);
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

void initCam() {
    // load parameters
    RCLCPP_INFO(this->get_logger(), "Loading Camera Configuration")

    this->declare_parameter("serial_number", std::string(" "));
    this->get_parameter("serial_number", this->device_id_);
    RCLCPP_INFO_STREAM(this->get_logger(), "serial_number: " << this->device_id);
	
    this->declare_parameter("master", true);
    this->get_parameter("master", this->master_);
    RCLCPP_INFO_STREAM(this->get_logger(), "master? " << this->master_);
    
    this->declare_parameter("reset_timestamps_delay", -1.0f);
    this->get_parameter("reset_timestamps_delay", this->reset_timestamps_delay);
    RCLCPP_INFO_STREAM(this->get_logger(), "reset_timestamps_delay: " << this->reset_timestamps_delay);

    this->declare_parameter("imu_calibration_sample_size", 1000);
    this->get_parameter("imu_calibration_sample_size", this->imu_calibration_sample_size);
    RCLCPP_INFO_STREAM(this->get_logger(), "imu_calibration_sample_size: " << this->imu_calibration_sample_size_);

    
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

    // configuration parameters
    //      -- DVS control parameters --

    //      -- subsampling parameters --

    //      -- polarity control parameters --

    //      -- region of interest parameters --

    //      -- inertial measurement unit parameters --

    //      -- streaming rate parameters --
    


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
			RCLCPP_ERROR("Error: Could not find DVXplorer.");
            device_is_running = false;

            //rclcpp::Rate sleepRate(std::chrono::seconds(1));
            //sleepRate.sleep();
            ///????????
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


void DvxplorerRosDriver::readout() {
	// std::vector<dvs::Event> events;

	caerDeviceConfigSet(this->dvxplorer_handle_, CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);
	caerDeviceDataStart(this->dvxplorer_handle_, nullptr, nullptr, nullptr, &DvXplorer::onDisconnectUSB, this);

	boost::posix_time::ptime next_send_time = boost::posix_time::microsec_clock::local_time();

	dvxplorer_interfaces::msg::EventArray event_array_msg;

	while (this->running_) {
		try {
			caerEventPacketContainer packetContainer = caerDeviceDataGet(this->dvxplorer_handle_);
			if (packetContainer == nullptr) {
				continue; // Skip if nothing there.
			}

			const int32_t packetNum = caerEventPacketContainerGetEventPacketsNumber(packetContainer);

			for (int32_t i = 0; i < packetNum; i++) {
				caerEventPacketHeader packetHeader = caerEventPacketContainerGetEventPacket(packetContainer, i);
				if (packetHeader == nullptr) {
					continue; // Skip if nothing there.
				}

				const int type = caerEventPacketHeaderGetEventType(packetHeader);

				// Packet 0 is always the special events packet for DVS128, while packet is the polarity events packet.
				if (type == POLARITY_EVENT) {
					if (!event_array_msg) {
						event_array_msg         = dvxplorer_interfaces::msg::EventArray(new dvxplorer_interfaces::msg::EventArray());
						event_array_msg->height = this->dvxplorer_info_.dvsSizeY;
						event_array_msg->width  = this->dvxplorer_info_.dvsSizeX;
					}

					caerPolarityEventPacket polarity = (caerPolarityEventPacket) packetHeader;

					const int numEvents = caerEventPacketHeaderGetEventNumber(packetHeader);
					for (int j = 0; j < numEvents; j++) {
						// Get full timestamp and addresses of first event.
						caerPolarityEvent event = caerPolarityEventPacketGetEvent(polarity, j);

						dvxplorer_interfaces::msg::Event e;
						e.x  = caerPolarityEventGetX(event);
						e.y  = caerPolarityEventGetY(event);
						//e.ts = reset_time_
						//	   + ros::Duration().fromNSec(caerPolarityEventGetTimestamp64(event, polarity) * 1000);
                        e.ts = this->reset_time_ + rclcpp::Duration().nanoseconds(caerPolarityEventGetTimestamp64(event, polarity) * 1000);
						e.polarity = caerPolarityEventGetPolarity(event);

						if (j == 0) {
							event_array_msg->header.stamp = e.ts;
						}

						event_array_msg->events.push_back(e);
					}

					int streaming_rate = streaming_rate_;
					int max_events     = max_events_;

					// throttle event messages
					if ((boost::posix_time::microsec_clock::local_time() > next_send_time) || (streaming_rate == 0)
						|| ((max_events != 0) && (event_array_msg->events.size() > max_events))) {
						this->event_array_pub_.publish(event_array_msg);

						if (streaming_rate > 0) {
							next_send_time += this->delta_;
						}

						if ((max_events != 0) && (event_array_msg->events.size() > max_events)) {
							next_send_time = boost::posix_time::microsec_clock::local_time() + this->delta_;
						}

						event_array_msg.reset();
					}

					if (this->cam_info_manager_->isCalibrated()) {
						sensor_msgs::msg::CameraInfo camera_info_msg(
							new sensor_msgs::msg::CameraInfo(this->camera_info_manager_->getCameraInfo()));
						this->cam_info_pub_.publish(camera_info_msg);
					}
				}
				else if (type == IMU6_EVENT) {
					caerIMU6EventPacket imu = (caerIMU6EventPacket) packetHeader;

					const int numEvents = caerEventPacketHeaderGetEventNumber(packetHeader);

					for (int j = 0; j < numEvents; j++) {
						caerIMU6Event event = caerIMU6EventPacketGetEvent(imu, j);

						sensor_msgs::msg::Imu imu_msg;

						// convert from g's to m/s^2 and align axes with camera frame
						imu_msg.linear_acceleration.x = -caerIMU6EventGetAccelX(event) * this->STANDARD_GRAVITY;
						imu_msg.linear_acceleration.y = caerIMU6EventGetAccelY(event) * this->STANDARD_GRAVITY;
						imu_msg.linear_acceleration.z = -caerIMU6EventGetAccelZ(event) * this->STANDARD_GRAVITY;
						// convert from deg/s to rad/s and align axes with camera frame
						imu_msg.angular_velocity.x = -caerIMU6EventGetGyroX(event) / 180.0 * M_PI;
						imu_msg.angular_velocity.y = caerIMU6EventGetGyroY(event) / 180.0 * M_PI;
						imu_msg.angular_velocity.z = -caerIMU6EventGetGyroZ(event) / 180.0 * M_PI;

						// no orientation estimate: http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
						imu_msg.orientation_covariance[0] = -1.0;

						// time
						//msg.header.stamp
						//	= rclcpp::Time(reset_time_ + fromNSec(caerIMU6EventGetTimestamp64(event, imu) * 1000));
                        imu_msg.header.stamp =  this->reset_time_ + rclcpp::Duration().nanoseconds(caerIMU6EventGetTimestamp64(event, imu) * 1000);


						// frame
						imu_msg.header.frame_id = "base_link";

						// IMU calibration
						if (this->imu_calibration_running_) {
							if (this->imu_calibration_samples_.size() < this->imu_calibration_sample_size_) {
								this->imu_calibration_samples_.push_back(imu_msg);
							}
							else {
								this->imu_calibration_running_ = false;
								updateImuBias();
							}
						}

						// bias correction
						imu_msg.linear_acceleration.x -= this->bias.linear_acceleration.x;
						imu_msg.linear_acceleration.y -= this->bias.linear_acceleration.y;
						imu_msg.linear_acceleration.z -= this->bias.linear_acceleration.z;
						imu_msg.angular_velocity.x -= this->bias.angular_velocity.x;
						imu_msg.angular_velocity.y -= this->bias.angular_velocity.y;
						imu_msg.angular_velocity.z -= this->bias.angular_velocity.z;

						this->imu_pub_.publish(imu_msg);
					}
				}
			}

			caerEventPacketContainerFree(packetContainer);

			//ros::spinOnce();
		}
		catch (boost::thread_interrupted &) {
			return;
		}
	}

	caerDeviceDataStop(this->dvxplorer_handle_);
}



} // namespace dvxplorer_ros2_driver

