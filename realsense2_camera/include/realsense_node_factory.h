// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved

#pragma once

#include <yaml_parser.h>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <cv_bridge/cv_bridge.h>
#include <constants.h>
#include <realsense2_camera/Extrinsics.h>
#include <realsense2_camera/IMUInfo.h>
#include <csignal>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <thread>

namespace realsense2_camera
{
    const stream_index_pair COLOR{RS2_STREAM_COLOR, 0};
    const stream_index_pair DEPTH{RS2_STREAM_DEPTH, 0};
    const stream_index_pair INFRA1{RS2_STREAM_INFRARED, 1};
    const stream_index_pair INFRA2{RS2_STREAM_INFRARED, 2};
    const stream_index_pair FISHEYE{RS2_STREAM_FISHEYE, 0};
    const stream_index_pair FISHEYE1{RS2_STREAM_FISHEYE, 1};
    const stream_index_pair FISHEYE2{RS2_STREAM_FISHEYE, 2};
    const stream_index_pair GYRO{RS2_STREAM_GYRO, 0};
    const stream_index_pair ACCEL{RS2_STREAM_ACCEL, 0};
    const stream_index_pair POSE{RS2_STREAM_POSE, 0};
    

    const std::vector<stream_index_pair> IMAGE_STREAMS = {DEPTH, INFRA1, INFRA2,
                                                          COLOR,
                                                          FISHEYE,
                                                          FISHEYE1, FISHEYE2};

    const std::vector<stream_index_pair> HID_STREAMS = {GYRO, ACCEL, POSE};

    class InterfaceRealSenseNode
    {
    public:
        virtual void publishTopics() = 0;
        virtual void registerDynamicReconfigCb(ros::NodeHandle& nh) = 0;
        virtual std::vector<geometry_msgs::TransformStamped> getStaticTransforms() = 0;
        virtual ~InterfaceRealSenseNode() = default;
        virtual void setResetEvent(){triggered_reset = true;};
        virtual bool resetEvent()
        {
          if (triggered_reset)
          {
            triggered_reset = false;
            return true;
          }
          return false;
        }
    private:
        bool triggered_reset = false;
    };

    class RealSenseNodeFactory : public nodelet::Nodelet
    {
    public:
        RealSenseNodeFactory();
        virtual ~RealSenseNodeFactory();

    private:
        void closeDevice();
        void StartDevice(const size_t& count);
        void change_device_callback(rs2::event_information& info);
        void getDevices(rs2::device_list list);
        virtual void onInit() override;
        void tryGetLogSeverity(rs2_log_severity& severity) const;
        static std::string parse_usb_port(std::string line);

        std::vector<rs2::device> _devices;
        std::vector<std::string> _device_names;
        std::vector<bool> _devices_started;
        std::vector<std::unique_ptr<InterfaceRealSenseNode>> _realSenseNodes;
        rs2::context _ctx;
        std::vector<std::string> _user_serial_nums;
        std::vector<bool> _user_initial_resets;
        std::vector<std::string> _serial_nums;
        std::vector<std::string> _usb_port_ids;
        std::vector<std::string> _device_types;
        std::vector<bool> _initial_resets;
        std::vector<std::thread> _query_threads;
        std::vector<std::thread> _reset_threads;
        bool _is_alive;
        tf2_ros::StaticTransformBroadcaster _static_tf_broadcaster;
    };
}//end namespace
