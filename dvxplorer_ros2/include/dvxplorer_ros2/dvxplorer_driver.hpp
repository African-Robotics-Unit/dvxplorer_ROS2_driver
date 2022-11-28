#pragma once
#ifndef DV_XPLORER_HPP
#define DV_XPLORER_HPP

// C++ Includes
#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <boost/thread/thread_time.hpp>
#include <atomic>

// ROS2 stuff
#include "rclcpp/rclcpp.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "std_msgs/msg/empty.hpp"
#include "dvxplorer_interfaces/msg/event.hpp"
#include "dvxplorer_interfaces/msg/event_array.hpp"
#include "std_msgs/msg/int32.hpp"

// DVXplorer driver
#include <libcaer/devices/dvxplorer.h>
#include <libcaer/libcaer.h>

namespace dvxplorer_ros2_driver
{


class DvXplorer : public rclcpp::Node
{
    public:
        DvXplorer();
        ~DvXplorer();
		static void onDisconnectUSB(void *);
		
    
    private:
        void readout();
	    void resetTimestamps();
	    void caerConnect();
		void initCam();
		void dataStop();

        // ROS publishers
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;                   // Cam info publisher handle
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        rclcpp::Publisher<dvxplorer_interfaces::msg::EventArray>::SharedPtr event_array_pub_;
        
        std::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_manager_;                  // Cam info manager handle 

		//libcaer stuff
        caerDeviceHandle dvxplorer_handle_;
		struct caer_dvx_info dvxplorer_info_;

        // reset stuff
        rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr reset_sub_;
	    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr reset_pub_;
        void resetTimestampsCallback(const builtin_interfaces::msg::Time::SharedPtr ts);
		float reset_timestamps_delay_;
		void resetTimerCallback();
        rclcpp::TimerBase::SharedPtr reset_timer_cb_; 

		builtin_interfaces::msg::Time reset_time_;

		// imu stuff
	    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_calibration_sub_;
		void imuCalibrationCallback();
	    bool imu_calibration_running_;
	    int imu_calibration_sample_size_;
	    std::vector<sensor_msgs::msg::Imu> imu_calibration_samples_;
	    sensor_msgs::msg::Imu bias;
	    void updateImuBias();

	    template <typename T> int sgn(T val) {
		    return (T(0) < val) - (val < T(0));
            }

	    std::shared_ptr<boost::thread> readout_thread_;

		bool running_;
		
	    static constexpr double STANDARD_GRAVITY = 9.81;

		// dvs parameters
		// general
		int frame_count_; 
		bool master_;
	    std::string device_id_;
		std::string cam_calib_file_;
		bool cam_info_loaded_;
		

		// dvs control
		bool dvs_enabled_;
		int bias_sensitivity_;

		// subsampling 
		bool subsampling_enabled_;
    	int vertical_subsampling_factor_;
    	int horizontal_subsampling_factor_;

		// polarity
		bool polarity_on_only_;
    	bool polarity_off_only_;
    	bool polarity_flatten_;

		// roi
		bool roi_enabled_;
    	int roi_left_;                                    
    	int roi_top_;                                    
    	int roi_width_;
		int roi_height_;     

		static constexpr int MAX_WIDTH = 640;
		static constexpr int MAX_HEIGHT = 480;

		// imu
		bool imu_enabled_;
		int imu_acc_scale_;
		int imu_gyro_scale_;

		// streaming                        
		boost::posix_time::time_duration delta_;
	    int streaming_rate_;
	    int max_events_;
			    
};

} //namespace dvxplorer_ros2_driver

#endif  // DV_XPLORER_HPP