#include "dvxplorer_ros2/dvxplorer_driver.hpp"

namespace dvxplorer_ros2_driver {

// Constructor        
DvXplorer::DvXplorer() :
	Node("dvxplorer_node"), 
    imu_calibration_running_(false),
	frame_count_(0)
	{
    this->initCam();

    // Initialise Publishers
    this->event_array_pub_ = this->create_publisher<dvxplorer_interfaces::msg::EventArray>("/dv/events", 10);
	this->imu_pub_ =  this->create_publisher<sensor_msgs::msg::Imu>("/dv/imu", 10);

    // only load and publish calib file if it isn't empty, assume camera info is not loaded, Setup camera info manager for calibration
    this->cam_info_loaded_ = false;
    this->cam_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);
    if (this->cam_info_manager_->loadCameraInfo(this->cam_calib_file_)) {
        this->cam_info_loaded_ = true;
    }
    if (this->cam_info_loaded_) {
		this->cam_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/dv/cam_info", 1);

    }

	// reset timestamps is publisher as master, subscriber as slave
	if (this->master_) {
		this->reset_pub_ = this->create_publisher<builtin_interfaces::msg::Time>("/dv/reset_timestamps", 1);
	}
	else {
		this->reset_sub_ = this->create_subscription<builtin_interfaces::msg::Time>("/dv/reset_timestamps", 1, std::bind(&DvXplorer::resetTimestampsCallback, this, std::placeholders::_1));
	}

	// Open device.
	this->caerConnect();

	std::function<void(std::shared_ptr<sensor_msgs::msg::Imu>)> fcn2 = std::bind(&DvXplorer::imuCalibrationCallback, this);
	this->imu_calibration_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/dv/calibrate_imu", 1, fcn2);

	// start timer to reset timestamps for synchronization
	if (this->reset_timestamps_delay_ > 0.0) {
		this->reset_timer_cb_
			= this-> create_wall_timer(std::chrono::duration<float>(this->reset_timestamps_delay_), std::bind(&DvXplorer::resetTimerCallback, this));
        RCLCPP_INFO(this->get_logger(), "Started timer to reset timestamps on master DVS for synchronization (delay=%3.2fs).", this->reset_timestamps_delay_);
	}
}

// Destructor
DvXplorer::~DvXplorer() {
	if (this->running_) {
		RCLCPP_INFO_STREAM(this->get_logger(), "Shutting down threads");
		this->running_ = false;
		this->readout_thread_->join();
		RCLCPP_INFO_STREAM(this->get_logger(), "Threads stopped");
		caerLog(CAER_LOG_ERROR, "Destructor", "Data stop now");
		caerDeviceDataStop(this->dvxplorer_handle_);
		caerDeviceClose(&this->dvxplorer_handle_);
	}
}

void DvXplorer::dataStop() {
	caerLog(CAER_LOG_INFO, "Exiting from driver node", "Executing data stop");
	RCLCPP_INFO(this->get_logger(), "Exiting from driver node, executing data stop");
	caerDeviceDataStop(this->dvxplorer_handle_);
	caerDeviceClose(&this->dvxplorer_handle_);
}

void DvXplorer::caerConnect() {
	// start driver
	bool device_is_running = false;
	while (!device_is_running) {
		const char *serial_number_restrict = (this->device_id_.empty()) ? nullptr : this->device_id_.c_str();

		if (serial_number_restrict) {
			RCLCPP_INFO_STREAM(this->get_logger(),"Requested serial number: " << this->device_id_);
		}

		this->dvxplorer_handle_ = caerDeviceOpen(1, CAER_DEVICE_DVXPLORER, 0, 0, serial_number_restrict);

		// was opening successful?
		device_is_running = (this->dvxplorer_handle_ != nullptr);

		if (!device_is_running) {
			RCLCPP_ERROR(this->get_logger(), "Error: Could not find DVXplorer.");
            device_is_running = false;
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

	//caerDeviceSendDefaultConfig(this->dvxplorer_handle_); //default cam configuration, no configuration is sent automatically 
	
	// Apply parameters to camera
    //      -- Set DVS control  --
	caerDeviceConfigSet(this->dvxplorer_handle_, DVX_DVS, DVX_DVS_RUN, this->dvs_enabled_);

    if (this->bias_sensitivity_< 0 || this->bias_sensitivity_ > 4) {
        // Out of bounds, throw error here
		RCLCPP_ERROR(this->get_logger(), "ERROR: DVS Bias Sensitivity out of bounds!");
        return;
    }
	caerDeviceConfigSet(this->dvxplorer_handle_, DVX_DVS_CHIP_BIAS, DVX_DVS_CHIP_BIAS_SIMPLE, this->bias_sensitivity_);

	//      -- Set subsampling factors  --
	caerDeviceConfigSet(this->dvxplorer_handle_, DVX_DVS_CHIP, DVX_DVS_CHIP_SUBSAMPLE_ENABLE, this->subsampling_enabled_);


	if (!(this->horizontal_subsampling_factor_ != 0 || this->horizontal_subsampling_factor_ != 1 ||
		this->horizontal_subsampling_factor_ != 3 || this->horizontal_subsampling_factor_ != 7 ||
		this->vertical_subsampling_factor_   != 0 || this->vertical_subsampling_factor_   != 1 ||
		this->vertical_subsampling_factor_   != 3 || this->vertical_subsampling_factor_   != 7)) {
        // Out of bounds, throw error here
		RCLCPP_ERROR(this->get_logger(), "ERROR: DVS Subsampling Factors out of bounds!");
        return;
    }
	caerDeviceConfigSet(this->dvxplorer_handle_, DVX_DVS_CHIP, DVX_DVS_CHIP_SUBSAMPLE_HORIZONTAL, this->horizontal_subsampling_factor_);
	caerDeviceConfigSet(this->dvxplorer_handle_, DVX_DVS_CHIP, DVX_DVS_CHIP_SUBSAMPLE_VERTICAL, this->vertical_subsampling_factor_);

	//      -- Set polarity control --
	caerDeviceConfigSet(this->dvxplorer_handle_, DVX_DVS_CHIP, DVX_DVS_CHIP_EVENT_FLATTEN, this->polarity_flatten_);
	caerDeviceConfigSet(this->dvxplorer_handle_, DVX_DVS_CHIP, DVX_DVS_CHIP_EVENT_ON_ONLY, this->polarity_on_only_);
	caerDeviceConfigSet(this->dvxplorer_handle_, DVX_DVS_CHIP, DVX_DVS_CHIP_EVENT_OFF_ONLY, this->polarity_off_only_);

	//      -- Set region of interest --
	caerDeviceConfigSet(this->dvxplorer_handle_, DVX_DVS_CHIP_CROPPER, DVX_DVS_CHIP_CROPPER_ENABLE, this->roi_enabled_);

	   // Check bounds
    if (this->roi_left_ < 0 || this->roi_left_ > this->MAX_WIDTH||
        this->roi_top_ < 0 || this->roi_top_ > this->MAX_HEIGHT ||
        this->roi_width_ < 0 || this->roi_width_ > this->MAX_WIDTH ||
        this->roi_height_ < 0 || this->roi_height_ > this->MAX_HEIGHT ||
        this->roi_left_ + this->roi_width_ > this->MAX_WIDTH ||
        this->roi_top_ + this->roi_height_ > this->MAX_HEIGHT) {

        // Out of bounds, throw error here
		RCLCPP_ERROR(this->get_logger(), "ERROR: DVS ROI out of bounds!");
        return;
    }
	caerDeviceConfigSet(this->dvxplorer_handle_, DVX_DVS_CHIP_CROPPER, DVX_DVS_CHIP_CROPPER_X_START_ADDRESS, this->roi_left_);
	caerDeviceConfigSet(this->dvxplorer_handle_, DVX_DVS_CHIP_CROPPER, DVX_DVS_CHIP_CROPPER_X_END_ADDRESS, this->roi_width_);
	caerDeviceConfigSet(this->dvxplorer_handle_, DVX_DVS_CHIP_CROPPER, DVX_DVS_CHIP_CROPPER_Y_START_ADDRESS, this->roi_top_);
	caerDeviceConfigSet(this->dvxplorer_handle_, DVX_DVS_CHIP_CROPPER, DVX_DVS_CHIP_CROPPER_Y_END_ADDRESS, this->roi_height_);

	//      -- Set IMU   --
	caerDeviceConfigSet(this->dvxplorer_handle_, DVX_IMU, DVX_IMU_RUN_ACCELEROMETER, this->imu_enabled_);
	caerDeviceConfigSet(this->dvxplorer_handle_, DVX_IMU, DVX_IMU_RUN_GYROSCOPE, this->imu_enabled_);
	
	if (this->imu_acc_scale_< 0   || this->imu_acc_scale_ > 3 ||
		this->imu_gyro_scale_ < 0 || this->imu_gyro_scale_ > 4) {
        // Out of bounds, throw error here
		RCLCPP_ERROR(this->get_logger(), "ERROR: DVS IMU Scale out of bounds!");
        return;
    }
	caerDeviceConfigSet(this->dvxplorer_handle_, DVX_IMU, DVX_IMU_ACCEL_RANGE, this->imu_acc_scale_);
	caerDeviceConfigSet(this->dvxplorer_handle_, DVX_IMU, DVX_IMU_GYRO_RANGE, this->imu_gyro_scale_);

	//      -- Set streaming rate   --
	if (this->streaming_rate_ > 0) {
			this->delta_ = boost::posix_time::microseconds(long(1e6 / this->streaming_rate_));
		}


	// spawn threads
	this->running_ = true;
	this->readout_thread_
		= std::shared_ptr<boost::thread>(new boost::thread(std::bind(&DvXplorer::readout, this)));
	
	// initialize timestamps
	this->resetTimestamps();
}


void DvXplorer::onDisconnectUSB(void *ptr) {
	(void) (ptr);
	RCLCPP_ERROR(rclcpp::get_logger("Log"), "USB connection lost with DVS!");
}

void DvXplorer::resetTimestamps() {
	caerDeviceConfigSet(this->dvxplorer_handle_, DVX_MUX, DVX_MUX_TIMESTAMP_RESET, 1);
	this->reset_time_ = now();
	RCLCPP_INFO(this->get_logger(), "Reset timestamps on %s to %.9f.", this->device_id_.c_str(), this->reset_time_);
	// if master, publish reset time to slaves
	if (this->master_) {
		builtin_interfaces::msg::Time reset_msg;
		reset_msg = this->reset_time_;
		this->reset_pub_->publish(reset_msg);
	}
}

void DvXplorer::resetTimestampsCallback(const builtin_interfaces::msg::Time::SharedPtr ts) {
	// if slave, only adjust offset time
	if (!this->dvxplorer_info_.deviceIsMaster) {
		RCLCPP_INFO_STREAM(this->get_logger(), "Adapting reset time of master on slave: " << this->device_id_);
		this->reset_time_ = *ts;
	}
	// if master, or not single camera configuration, just reset timestamps
	else {
		this->resetTimestamps();
	} 
} 

void DvXplorer::imuCalibrationCallback()
{
	RCLCPP_INFO(this->get_logger(), "Starting IMU calibration with %d samples.", (int) this->imu_calibration_sample_size_);
	this->imu_calibration_running_ = true;
	this->imu_calibration_samples_.clear();
}

void DvXplorer::resetTimerCallback() {
	this->reset_timer_cb_->cancel();
	this->resetTimestamps();
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
	this->bias.linear_acceleration.z -= this->STANDARD_GRAVITY * this->sgn(this->bias.linear_acceleration.z);

	this->bias.angular_velocity.x /= (double) this->imu_calibration_samples_.size();
	this->bias.angular_velocity.y /= (double) this->imu_calibration_samples_.size();
	this->bias.angular_velocity.z /= (double) this->imu_calibration_samples_.size();

	RCLCPP_INFO(this->get_logger(), "IMU calibration done.");
	RCLCPP_INFO(this->get_logger(), "Acceleration biases: %1.5f %1.5f %1.5f [m/s^2]", this->bias.linear_acceleration.x, this->bias.linear_acceleration.y,
		this->bias.linear_acceleration.z);
	RCLCPP_INFO(this->get_logger(), "Gyroscope biases: %1.5f %1.5f %1.5f [rad/s]", this->bias.angular_velocity.x, this->bias.angular_velocity.y,
		this->bias.angular_velocity.z);
}


void DvXplorer::readout() {
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
					//if (!event_array_msg) {
					//dvxplorer_interfaces::msg::EventArray event_array_msg = new dvxplorer_interfaces::msg::EventArray;
					dvxplorer_interfaces::msg::EventArray event_array_msg;
					event_array_msg.height = this->dvxplorer_info_.dvsSizeY;
					event_array_msg.width  = this->dvxplorer_info_.dvsSizeX;
					//}

					caerPolarityEventPacket polarity = (caerPolarityEventPacket) packetHeader;

					const int numEvents = caerEventPacketHeaderGetEventNumber(packetHeader);
					for (int j = 0; j < numEvents; j++) {
						// Get full timestamp and addresses of first event.
						caerPolarityEvent event = caerPolarityEventPacketGetEvent(polarity, j);

						dvxplorer_interfaces::msg::Event e;
						e.x  = caerPolarityEventGetX(event);
						e.y  = caerPolarityEventGetY(event);
                        e.ts = rclcpp::Time(this->reset_time_) + rclcpp::Duration((caerPolarityEventGetTimestamp64(event, polarity) * 1000));
						e.polarity = caerPolarityEventGetPolarity(event);

						if (j == 0) {
							event_array_msg.header.stamp = e.ts;
						}

						event_array_msg.events.push_back(e);
					}

					int streaming_rate = this->streaming_rate_;
					int max_events     = this->max_events_;

					// throttle event messages
					if ((boost::posix_time::microsec_clock::local_time() > next_send_time) || (streaming_rate == 0)
						|| ((max_events != 0) && (event_array_msg.events.size() > max_events))) {
						event_array_msg.header.frame_id = std::to_string(this->frame_count_);
						this->event_array_pub_->publish(event_array_msg);
						RCLCPP_INFO(this->get_logger(), "Published Event Array: [%i]", this->frame_count_);

						if (streaming_rate > 0) {
							next_send_time += this->delta_;
						}

						if ((max_events != 0) && (event_array_msg.events.size() > max_events)) {
							next_send_time = boost::posix_time::microsec_clock::local_time() + this->delta_;
						}
						//event_array_msg.reset();
					}				

					if (this->cam_info_loaded_) {
						sensor_msgs::msg::CameraInfo cam_info = this->cam_info_manager_->getCameraInfo();  
						cam_info.header.frame_id = std::to_string(this->frame_count_);
						this->cam_info_pub_->publish(cam_info);
						RCLCPP_INFO(this->get_logger(), "Published Camera Info.");
					}
					++this->frame_count_;
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
						//msg.header.stamp = rclcpp::Time(reset_time_ + fromNSec(caerIMU6EventGetTimestamp64(event, imu) * 1000));
						imu_msg.header.stamp =  rclcpp::Time(this->reset_time_) + rclcpp::Duration(caerIMU6EventGetTimestamp64(event, imu) * 1000);


						// frame
						imu_msg.header.frame_id = "base_link";

						// IMU calibration
						if (this->imu_calibration_running_) {
							if (this->imu_calibration_samples_.size() < this->imu_calibration_sample_size_) {
								this->imu_calibration_samples_.push_back(imu_msg);
								RCLCPP_INFO(this->get_logger(), "TRACE 1");
							}
							else {
								this->imu_calibration_running_ = false;
								this->updateImuBias();
							}
						}

						// bias correction
						imu_msg.linear_acceleration.x -= this->bias.linear_acceleration.x;
						imu_msg.linear_acceleration.y -= this->bias.linear_acceleration.y;
						imu_msg.linear_acceleration.z -= this->bias.linear_acceleration.z;
						imu_msg.angular_velocity.x -= this->bias.angular_velocity.x;
						imu_msg.angular_velocity.y -= this->bias.angular_velocity.y;
						imu_msg.angular_velocity.z -= this->bias.angular_velocity.z;

						this->imu_pub_->publish(imu_msg);
						RCLCPP_INFO(this->get_logger(), "Published IMU.");
					}
				}
			}

			caerEventPacketContainerFree(packetContainer);
		}
		catch (boost::thread_interrupted &) {
			return;
		}
	}

	caerDeviceDataStop(this->dvxplorer_handle_);
}

void DvXplorer::initCam() {
    // load parameters
    RCLCPP_INFO(this->get_logger(), "Loading Camera Configuration");

    this->declare_parameter("serial_number", std::string(" "));
    this->get_parameter("serial_number", this->device_id_);
    RCLCPP_INFO_STREAM(this->get_logger(), "serial_number: " << this->device_id_);
	
    this->declare_parameter("master", true);
    this->get_parameter("master", this->master_);
    RCLCPP_INFO_STREAM(this->get_logger(), "master? " << this->master_);
    
    this->declare_parameter("reset_timestamps_delay", -1.0f);
    this->get_parameter("reset_timestamps_delay", this->reset_timestamps_delay_);
    RCLCPP_INFO_STREAM(this->get_logger(), "reset_timestamps_delay: " << this->reset_timestamps_delay_);

    this->declare_parameter("imu_calibration_sample_size", 1000);
    this->get_parameter("imu_calibration_sample_size", this->imu_calibration_sample_size_);
    RCLCPP_INFO_STREAM(this->get_logger(), "imu_calibration_sample_size: " << this->imu_calibration_sample_size_);

	this->declare_parameter( "calib_file", std::string("INVALID"));
    this->get_parameter("calib_file", this->cam_calib_file_);
    RCLCPP_INFO_STREAM(this->get_logger(), "calibration file: " << this->cam_calib_file_);

    
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
	this->declare_parameter("dvs_enabled", true);
    this->get_parameter("dvs_enabled", this->dvs_enabled_);
    RCLCPP_INFO_STREAM(this->get_logger(), "dvs_enabled: " << this->dvs_enabled_);

	this->declare_parameter("bias_sensitivity", 0);
    this->get_parameter("bias_sensitivity", this->bias_sensitivity_);
    RCLCPP_INFO_STREAM(this->get_logger(), "bias_sensitivity: " << this->bias_sensitivity_);

    //      -- subsampling parameters --
	this->declare_parameter("subsampling_enabled", true);
    this->get_parameter("subsampling_enabled", this->subsampling_enabled_);
    RCLCPP_INFO_STREAM(this->get_logger(), "subsampling_enabled: " << this->subsampling_enabled_);

	this->declare_parameter("vertical_subsampling_factor", 0);
    this->get_parameter("vertical_subsampling_factor", this->vertical_subsampling_factor_);
    RCLCPP_INFO_STREAM(this->get_logger(), "vertical_subsampling_factor: " << this->vertical_subsampling_factor_);

	this->declare_parameter("horizontal_subsampling_factor", 0);
    this->get_parameter("horizontal_subsampling_factor", this->horizontal_subsampling_factor_);
    RCLCPP_INFO_STREAM(this->get_logger(), "horizontal_subsampling_factor: " << this->horizontal_subsampling_factor_);

    //      -- polarity control parameters --
	this->declare_parameter("polarity_on_only", false);
    this->get_parameter("polarity_on_only", this->polarity_on_only_);
    RCLCPP_INFO_STREAM(this->get_logger(), "polarity_on_only: " << this->polarity_on_only_);

	this->declare_parameter("polarity_off_only", false);
    this->get_parameter("polarity_off_only", this->polarity_off_only_);
    RCLCPP_INFO_STREAM(this->get_logger(), "polarity_off_only: " << this->polarity_off_only_);

	this->declare_parameter("polarity_flatten", false);
    this->get_parameter("polarity_flatten", this->polarity_flatten_);
    RCLCPP_INFO_STREAM(this->get_logger(), "polarity_flatten: " << this->polarity_flatten_);

    //      -- region of interest parameters --
	this->declare_parameter("roi_enabled", false);
    this->get_parameter("roi_enabled", this->roi_enabled_);
    RCLCPP_INFO_STREAM(this->get_logger(), "roi_enabled: " << this->roi_enabled_);

	this->declare_parameter("roi_left", -1);
    this->get_parameter("roi_left", this->roi_left_);
    RCLCPP_INFO_STREAM(this->get_logger(), "roi_left: " << this->roi_left_);

	this->declare_parameter("roi_top", -1);
    this->get_parameter("roi_top", this->roi_top_);
    RCLCPP_INFO_STREAM(this->get_logger(), "roi_top: " << this->roi_top_);

	this->declare_parameter("roi_width", -1);
    this->get_parameter("roi_width", this->roi_width_);
    RCLCPP_INFO_STREAM(this->get_logger(), "roi_width: " << this->roi_width_);

	this->declare_parameter("roi_height", -1);
    this->get_parameter("roi_height", this->roi_height_);
    RCLCPP_INFO_STREAM(this->get_logger(), "roi_height: " << this->roi_height_);

    //      -- inertial measurement unit parameters --
	this->declare_parameter("imu_enabled", false);
    this->get_parameter("imu_enabled", this->imu_enabled_);
    RCLCPP_INFO_STREAM(this->get_logger(), "imu_enabled: " << this->imu_enabled_);

	this->declare_parameter("imu_acc_scale", 1);
    this->get_parameter("imu_acc_scale", this->imu_acc_scale_);
    RCLCPP_INFO_STREAM(this->get_logger(), "imu_acc_scale: " << this->imu_acc_scale_);

	this->declare_parameter("imu_gyro_scale", 2);
    this->get_parameter("imu_gyro_scale", this->imu_gyro_scale_);
    RCLCPP_INFO_STREAM(this->get_logger(), "imu_gyro_scale: " << this->imu_gyro_scale_);

    //      -- streaming rate parameters --
	this->declare_parameter("streaming_rate", 30);
    this->get_parameter("streaming_rate", this->streaming_rate_);
    RCLCPP_INFO_STREAM(this->get_logger(), "streaming_rate: " << this->streaming_rate_);
    
	this->declare_parameter("max_events", 2);
    this->get_parameter("max_events", this->max_events_);
    RCLCPP_INFO_STREAM(this->get_logger(), "max_events: " << this->max_events_);

}

} // namespace dvxplorer_ros2_driver

