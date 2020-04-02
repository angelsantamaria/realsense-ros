// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include "../include/realsense_node_factory.h"
#include "../include/base_realsense_node.h"
#include "../include/t265_realsense_node.h"
#include <iostream>
#include <map>
#include <mutex>
#include <condition_variable>
#include <signal.h>
#include <thread>
#include <sys/time.h>
#include <regex>

using namespace realsense2_camera;

#define REALSENSE_ROS_EMBEDDED_VERSION_STR (VAR_ARG_STRING(VERSION: REALSENSE_ROS_MAJOR_VERSION.REALSENSE_ROS_MINOR_VERSION.REALSENSE_ROS_PATCH_VERSION))
constexpr auto realsense_ros_camera_version = REALSENSE_ROS_EMBEDDED_VERSION_STR;

PLUGINLIB_EXPORT_CLASS(realsense2_camera::RealSenseNodeFactory, nodelet::Nodelet)

RealSenseNodeFactory::RealSenseNodeFactory():
	_is_alive(true)
{
	ROS_INFO("RealSense ROS v%s", REALSENSE_ROS_VERSION_STR);
	ROS_INFO("Running with LibRealSense v%s", RS2_API_VERSION_STR);

	auto severity = rs2_log_severity::RS2_LOG_SEVERITY_WARN;
	tryGetLogSeverity(severity);
	if (rs2_log_severity::RS2_LOG_SEVERITY_DEBUG == severity)
		ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

	rs2::log_to_console(severity);
}

RealSenseNodeFactory::~RealSenseNodeFactory()
{
	_is_alive = false;
	if (_query_thread.joinable())
	{
		_query_thread.join();
	}
}

std::string RealSenseNodeFactory::parse_usb_port(std::string line)
{
  std::string port_id;
  std::regex self_regex("(?:[^ ]+/usb[0-9]+[0-9./-]*/){0,1}([0-9.-]+)(:){0,1}[^ ]*", std::regex_constants::ECMAScript);
  std::smatch base_match;
  bool found = std::regex_match(line, base_match, self_regex);
  if (found)
  {
    port_id = base_match[1].str();
    if (base_match[2].str().size() == 0)    //This is libuvc string. Remove counter if exists.
    {
      std::regex end_regex = std::regex(".+(-[0-9]+$)", std::regex_constants::ECMAScript);
      bool found_end = std::regex_match(port_id, base_match, end_regex);
      if (found_end)
      {
        port_id = port_id.substr(0, port_id.size() - base_match[1].str().size());
      }
    }
  }
  return port_id;
}

void RealSenseNodeFactory::getDevice(rs2::device_list list)
{
  if (_devices.empty())
  {
		if (0 == list.size())
		{
			ROS_WARN("No RealSense devices were found!");
		}
		else
		{
      // Allocate for all possible devices in the list
      _devices.resize(list.size());

			std::vector<bool> found(list.size(), false);
      ROS_INFO_STREAM(" ");
			for (size_t count = 0; count < list.size(); count++)
			{
				rs2::device dev;
				try
				{
					dev = list[count];
				}
				catch(const std::exception& ex)
				{
					ROS_WARN_STREAM("Device " << count+1 << "/" << list.size() << " failed with exception: " << ex.what());
					continue;
				}
				auto sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
				ROS_INFO_STREAM("Device with serial number " << sn << " was found."<<std::endl);
				std::string pn = dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);
				std::string name = dev.get_info(RS2_CAMERA_INFO_NAME);
				ROS_INFO_STREAM("Device with physical ID " << pn << " was found.");
				std::vector<std::string> results;
				ROS_INFO_STREAM("Device with name " << name << " was found.");
				std::string port_id = parse_usb_port(pn);
				if (port_id.empty())
				{
					std::stringstream msg;
					msg << "Error extracting usb port from device with physical ID: " << pn << std::endl << "Please report on github issue at https://github.com/IntelRealSense/realsense-ros";
					if (_usb_port_ids[count].empty())
					{
						ROS_WARN_STREAM(msg.str());
					}
					else
					{
						ROS_ERROR_STREAM(msg.str());
						ROS_ERROR_STREAM("Please use serial number instead of usb port.");
					}
				}
				else
				{
					ROS_INFO_STREAM("Device with port number " << port_id << " was found.");					
				}
				bool found_device_type(true);
				if (!_device_types[count].empty())
				{
					std::smatch match_results;
					std::regex device_type_regex(_device_types[count].c_str(), std::regex::icase);
					found_device_type = std::regex_search(name, match_results, device_type_regex);
				}

				if ((_serial_nums[count].empty() || sn == _serial_nums[count]) && (_usb_port_ids[count].empty() || port_id == _usb_port_ids[count]) && found_device_type)
				{
					_devices[count] = dev;
					_serial_nums[count] = sn;
					found[count] = true;
					break;
				}
			}
      for (size_t count = 0; count < list.size(); count++)
      {
  			if (!found[count])
  			{
  				// T265 could be caught by another node.
  				std::string msg ("The requested device with ");
  				bool add_and(false);
  				if (!_serial_nums[count].empty())
  				{
  					msg += "serial number " + _serial_nums[count];
  					add_and = true;
  				}
  				if (!_usb_port_ids[count].empty())
  				{
  					if (add_and)
  					{
  						msg += " and ";
  					}
  					msg += "usb port id " + _usb_port_ids[count];
  					add_and = true;
  				}
  				if (!_device_types[count].empty())
  				{
  					if (add_and)
  					{
  						msg += " and ";
  					}
  					msg += "device name containing " + _device_types[count];
  				}
  				msg += " is NOT found. Will Try again.";
  				ROS_ERROR_STREAM(msg);
  			}
      }
		}
	}

  for (size_t count = 0; count < _devices.size(); count++)
  {
    bool remove_tm2_handle(_devices[count] && RS_T265_PID != std::stoi(_devices[count].get_info(RS2_CAMERA_INFO_PRODUCT_ID), 0, 16));
    if (remove_tm2_handle)
    {
      _ctx.unload_tracking_module();
    }
  
    if (_devices[count] && _initial_resets[count])
    {
      _initial_resets[count] = false;
      try
      {
        ROS_INFO("Resetting device %d ...", (int)count);
        _devices[count].hardware_reset();
        _devices[count] = rs2::device();
        
      }
      catch(const std::exception& ex)
      {
        ROS_WARN_STREAM("An exception has been thrown: " << ex.what());
      }
    }
  }
}

void RealSenseNodeFactory::change_device_callback(rs2::event_information& info)
{
  // FIXME get actual count for each device
  size_t count = 0;
	if (info.was_removed(_devices[count]))
	{
		ROS_ERROR("The device has been disconnected!");
		_realSenseNodes[count].reset(nullptr);
		_devices[count] = rs2::device();
	}
	if (!_devices[count])
	{
		rs2::device_list new_devices = info.get_new_devices();
		if (new_devices.size() > 0)
		{
			ROS_INFO("Checking new devices...");
			getDevice(new_devices);
			if (_devices[count])
			{
				StartDevice(count);
			}
		}
	}
}

void RealSenseNodeFactory::onInit()
{
	try
	{
#ifdef BPDEBUG
		std::cout << "Attach to Process: " << getpid() << std::endl;
		std::cout << "Press <ENTER> key to continue." << std::endl;
		std::cin.get();
#endif
		ros::NodeHandle nh = getNodeHandle();
		auto privateNh = getPrivateNodeHandle();
		
		// std::string rosbag_filename("");
		// privateNh.param("rosbag_filename", rosbag_filename, std::string(""));
		// if (!rosbag_filename.empty())
		// {
		// 	 ROS_INFO_STREAM("publish topics from rosbag file: " << rosbag_filename.c_str());
		// 	 auto pipe = std::make_shared<rs2::pipeline>();
		// 	 rs2::config cfg;
		// 	 cfg.enable_device_from_file(rosbag_filename.c_str(), false);
		// 	 cfg.enable_all_streams();
		// 	 pipe->start(cfg); //File will be opened in read mode at this point
		// 	 _devices = pipe->get_active_profile().get_device();
		// 	 _serial_nums.resize(_devices.size());
    //   for (size_t count = 0; count < _devices.size(); ++count)
    //   {
		//     _serial_nums[count] = _devices[count].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
		// 	  	StartDevice(count);
    //   }
		// }
		// else
		// {
      bool all_params_read = false;
      int id = 0;
      while (!all_params_read)
      {
        std::string cam_name("cam" + std::to_string(id));
        std::string serial("");
        std::string usb_port_id("");
        std::string device_type("");
        bool initial_reset(false);

        if (!privateNh.getParam(cam_name + "/serial_no", serial))
          all_params_read = true;
        if (!privateNh.getParam(cam_name + "/usb_port_id", usb_port_id))
          all_params_read = true;
        if (!privateNh.getParam(cam_name + "/device_type", device_type))
          all_params_read = true;
        if (!privateNh.getParam(cam_name + "/initial_reset", initial_reset))
          initial_reset = false;

        if (!all_params_read)
        {
          _serial_nums.push_back(serial);
          _usb_port_ids.push_back(usb_port_id);
          _device_types.push_back(device_type);
          _initial_resets.push_back(initial_reset);
          id++;
        }
      }

      _devices.resize(_serial_nums.size());
      for (size_t count = 0; count < _serial_nums.size(); ++count)
      {
        _query_thread = std::thread([=]()
          {
            std::chrono::milliseconds timespan(6000);
            while (_is_alive && !_devices[count])
            {
              getDevice(_ctx.query_devices());
              if (_devices[count])
              {
                std::function<void(rs2::event_information&)> change_device_callback_function = [this](rs2::event_information& info){change_device_callback(info);};
                _ctx.set_devices_changed_callback(change_device_callback_function);
                StartDevice(count);
              }
              else
              {
                std::this_thread::sleep_for(timespan);
              }
            }
          });
      }
		// }
	}
	catch(const std::exception& ex)
	{
		ROS_ERROR_STREAM("An exception has been thrown: " << ex.what());
		exit(1);
	}
	catch(...)
	{
		ROS_ERROR_STREAM("Unknown exception has occured!");
		exit(1);
	}
}

void RealSenseNodeFactory::StartDevice(const size_t& count)
{
	ros::NodeHandle nh = getNodeHandle();
	ros::NodeHandle privateNh = getPrivateNodeHandle();

  assert(_devices.size() >= count);

  if (_realSenseNodes.empty())
    _realSenseNodes.resize(_devices.size());
  else
    if (_realSenseNodes.size() >= count)
      _realSenseNodes[count].reset();

	std::string pid_str(_devices[count].get_info(RS2_CAMERA_INFO_PRODUCT_ID));
	uint16_t pid = std::stoi(pid_str, 0, 16);
	switch(pid)
	{
  	case SR300_PID:
  	case SR300v2_PID:
  	case RS400_PID:
  	case RS405_PID:
  	case RS410_PID:
  	case RS460_PID:
  	case RS415_PID:
  	case RS420_PID:
  	case RS420_MM_PID:
  	case RS430_PID:
  	case RS430_MM_PID:
  	case RS430_MM_RGB_PID:
  	case RS435_RGB_PID:
  	case RS435i_RGB_PID:
  	case RS_USB2_PID:
  	case RS_L515_PID:
  		_realSenseNodes[count] = std::unique_ptr<BaseRealSenseNode>(new BaseRealSenseNode(nh, privateNh, _devices[count], _serial_nums[count]));
  		break;
  	case RS_T265_PID:
  		_realSenseNodes[count] = std::unique_ptr<T265RealsenseNode>(new T265RealsenseNode(nh, privateNh, _devices[count], _serial_nums[count]));
  		break;
  	default:
  		ROS_FATAL_STREAM("Unsupported device!" << " Product ID: 0x" << pid_str);
  		ros::shutdown();
  		exit(1);
	}
	assert(_realSenseNodes[count]);
	_realSenseNodes[count]->publishTopics();
}

void RealSenseNodeFactory::tryGetLogSeverity(rs2_log_severity& severity) const
{
	static const char* severity_var_name = "LRS_LOG_LEVEL";
	auto content = getenv(severity_var_name);

	if (content)
	{
		std::string content_str(content);
		std::transform(content_str.begin(), content_str.end(), content_str.begin(), ::toupper);

		for (uint32_t i = 0; i < RS2_LOG_SEVERITY_COUNT; i++)
		{
			auto current = std::string(rs2_log_severity_to_string((rs2_log_severity)i));
			std::transform(current.begin(), current.end(), current.begin(), ::toupper);
			if (content_str == current)
			{
				severity = (rs2_log_severity)i;
				break;
			}
		}
	}
}
