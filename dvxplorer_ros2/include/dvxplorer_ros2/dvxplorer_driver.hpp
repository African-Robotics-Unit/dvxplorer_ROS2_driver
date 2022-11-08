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
        void dataStop();
        static void onDisconnectUSB(void *);
    
    private:
        //void callback(dvxplorer_ros_driver::DVXplorer_ROS_DriverConfig &config, uint32_t level);
	    void callback(uint32_t level);
        void readout();
	    void resetTimestamps();
	    void caerConnect();

        // ROS Publishers
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;                   // Cam info publisher handle
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        rclcpp::Publisher<dvxplorer_interfaces::msg::EventArray>::SharedPtr event_array_pub_;
        
        std::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_manager_;                  // Cam info manager handle 

        caerDeviceHandle dvxplorer_handle_;

        // reset stuff
        rclcpp::Subscriber <builtin_interfaces::msg::Time> reset_sub_;
	    rclcpp::Publisher<builtin_interfaces::msg::Time> reset_pub_;
        void resetTimestampsCallback(const rclcpp::Time &ts);
   
        std::string ns;

	    std::atomic<bool> running_;

        // Dynamic reconfigure stuff
	    //boost::shared_ptr<dynamic_reconfigure::Server<dvxplorer_ros_driver::DVXplorer_ROS_DriverConfig>> server_;
	    //dynamic_reconfigure::Server<dvxplorer_ros_driver::DVXplorer_ROS_DriverConfig>::CallbackType dynamic_reconfigure_callback_;

	
	    rclcpp::Subscriber<sensor_msgs::msg::Imu>::SharedPtr imu_calibration_sub_;
	    void imuCalibrationCallback(const std_msgs::msg::Empty::ConstPtr &msg);
	    std::atomic<bool> imu_calibration_running_;
	    int imu_calibration_sample_size_;
	    std::vector<sensor_msgs::msg::Imu> imu_calibration_samples_;
	    sensor_msgs::msg::Imu bias;
	    void updateImuBias();

	    template<typename T>
	    int sgn(T val) {
		    return (T(0) < val) - (val < T(0));
            }

	    std::shared_ptr<boost::thread> readout_thread_;

	    boost::posix_time::time_duration delta_;

	    std::atomic<int> streaming_rate_;
	    std::atomic<int> max_events_;

	    struct caer_dvx_info dvxplorer_info_;
	    bool master_;
	    std::string device_id_;

	    rclcpp::Time reset_time_;

	    static constexpr double STANDARD_GRAVITY = 9.81;
	    
        void resetTimerCallback();
        rclcpp::TimerBase::SharedPtr reset_timer_cb_;  

};

} //namespace dvxplorer_ros2_driver

#endif  // DV_XPLORER_HPP