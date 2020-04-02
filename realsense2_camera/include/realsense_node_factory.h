// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved

#pragma once

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/package.h>
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
        virtual ~InterfaceRealSenseNode() = default;
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
        std::vector<std::string> _serial_nums;
        std::vector<std::string> _usb_port_ids;
        std::vector<std::string> _device_types;
        std::vector<bool> _initial_resets;
        std::vector<std::thread> _query_threads;
        bool _is_alive;
    };
}//end namespace
