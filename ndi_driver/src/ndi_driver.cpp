// ndi_driver/src/ndi_driver.cpp

#include "ndi_driver/ndi_driver.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace ndi_driver
{

    hardware_interface::CallbackReturn NdiDriver::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SensorInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        // Get parameters from hardware info
        ip_address_ = info_.hardware_parameters.at("ndi_ip");
        num_tools_ = info_.sensors.size();

        // Get tool names from config
        for (const auto &sensor : info_.sensors)
        {
            tool_names_.push_back(sensor.parameters.at("srom"));
            RCLCPP_INFO(
                rclcpp::get_logger("NdiDriver"),
                "Loading tracker %s from %s",
                sensor.name.c_str(), sensor.parameters.at("srom").c_str());
        }

        // Connect to device
        if (api_.connect(ip_address_) != 0)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("NdiDriver"),
                "Failed to connect to device at %s", ip_address_.c_str());
            return CallbackReturn::ERROR;
        }

        // Wait for connection to stabilize
        rclcpp::sleep_for(std::chrono::seconds(1));

        RCLCPP_INFO(
            rclcpp::get_logger("NdiDriver"),
            "Connected to device at %s with firmware %s",
            ip_address_.c_str(),
            api_.getUserParameter("Features.Firmware.Version").c_str());

        // Initialize device
        supports_bx2_ = supportsBx2();
        onError("initialize", api_.initialize());

        // Load and initialize tools
        loadTools();
        initializeTools();

        port_handles_ = api_.portHandleSearchRequest(PortHandleSearchRequestOption::Enabled);

        if (port_handles_.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger("NdiDriver"), "No tools enabled!");
            return CallbackReturn::ERROR;
        }

        // Initialize enabled tools
        for (const auto &handle : port_handles_)
        {
            ToolData tool;
            tool.transform.toolHandle = api_.stringToInt(handle.getPortHandle());
            tool.toolInfo = getToolInfo(handle.getPortHandle());
            enabled_tools_.push_back(tool);
        }

        // Initialize poses matrix
        tracker_poses_.resize(info_.sensors.size(),
                              std::vector<double>(7, std::numeric_limits<double>::quiet_NaN()));

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>
    NdiDriver::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (uint i = 0; i < info_.sensors.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.sensors[i].name, "pose.position.x", &tracker_poses_[i][0]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.sensors[i].name, "pose.position.y", &tracker_poses_[i][1]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.sensors[i].name, "pose.position.z", &tracker_poses_[i][2]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.sensors[i].name, "pose.orientation.x", &tracker_poses_[i][3]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.sensors[i].name, "pose.orientation.y", &tracker_poses_[i][4]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.sensors[i].name, "pose.orientation.z", &tracker_poses_[i][5]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.sensors[i].name, "pose.orientation.w", &tracker_poses_[i][6]));
        }

        return state_interfaces;
    }

    hardware_interface::CallbackReturn NdiDriver::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        int tracking_return_code = api_.startTracking();
        onError("startTracking", tracking_return_code);
        return (tracking_return_code < 0 ? CallbackReturn::ERROR : CallbackReturn::SUCCESS);
    }

    hardware_interface::CallbackReturn NdiDriver::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        int ret = api_.stopTracking();
        onError("stopTracking", ret);
        return (ret < 0 ? CallbackReturn::ERROR : CallbackReturn::SUCCESS);
    }

    hardware_interface::return_type NdiDriver::read(
        const rclcpp::Time & /*time*/,
        const rclcpp::Duration & /*period*/)
    {
        auto tool_data = supports_bx2_
                             ? api_.getTrackingDataBX2("--6d=tools --3d=tools --sensor=none --1d=none")
                             : api_.getTrackingDataBX(TrackingReplyOption::TransformData |
                                                      TrackingReplyOption::AllTransforms);

        // Update enabled tools with new data
        for (size_t t = 0; t < enabled_tools_.size(); t++)
        {
            for (const auto &new_data : tool_data)
            {
                if (enabled_tools_[t].transform.toolHandle == new_data.transform.toolHandle)
                {
                    // Copy new data while preserving tool info
                    // new_data.toolInfo = enabled_tools_[t].toolInfo;
                    // enabled_tools_[t] = new_data;
                    ToolData updated_tool = new_data;
                    updated_tool.toolInfo = enabled_tools_[t].toolInfo;
                    enabled_tools_[t] = updated_tool;
                    break;
                }
            }
        }

        // Update tracker poses
        for (size_t t = 0; t < enabled_tools_.size(); t++)
        {
            enabled_tools_[t].dataIsNew = false;

            // Convert mm to meters for positions
            tracker_poses_[t][0] = enabled_tools_[t].transform.tx / 1000.0;
            tracker_poses_[t][1] = enabled_tools_[t].transform.ty / 1000.0;
            tracker_poses_[t][2] = enabled_tools_[t].transform.tz / 1000.0;

            // Copy quaternion
            tracker_poses_[t][3] = enabled_tools_[t].transform.qx;
            tracker_poses_[t][4] = enabled_tools_[t].transform.qy;
            tracker_poses_[t][5] = enabled_tools_[t].transform.qz;
            tracker_poses_[t][6] = enabled_tools_[t].transform.q0;
        }

        return hardware_interface::return_type::OK;
    }

    void NdiDriver::loadTools()
    {
        for (const auto &srom_file : tool_names_)
        {
            std::string full_path = findSromPath(srom_file);
            if (full_path.empty())
            {
                RCLCPP_ERROR(rclcpp::get_logger("NdiDriver"),
                             "Could not find SROM file: %s", srom_file.c_str());
                continue;
            }

            // Request port handle
            int port_handle = api_.portHandleRequest();
            onError("portHandleRequest", port_handle);

            // Load SROM file
            api_.loadSromToPort(full_path.c_str(), port_handle);
        }
    }

    void NdiDriver::initializeTools()
    {
        auto handles = api_.portHandleSearchRequest(PortHandleSearchRequestOption::NotInit);

        for (const auto &handle : handles)
        {
            onError("portHandleInitialize",
                    api_.portHandleInitialize(handle.getPortHandle()));
            onError("portHandleEnable",
                    api_.portHandleEnable(handle.getPortHandle()));
        }
    }

    bool NdiDriver::supportsBx2()
    {
        std::string revision = api_.getApiRevision();
        char device_family = revision[0];
        int major_version = api_.stringToInt(revision.substr(2, 3));

        // Only Vega (Polaris with API v3+) supports BX2
        return (device_family == 'G' && major_version >= 3);
    }

    void NdiDriver::onError(const std::string &method, int code)
    {
        if (code < 0)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("NdiDriver"),
                "%s failed: %s", method.c_str(), api_.errorToString(code).c_str());
        }
    }

    std::string NdiDriver::getToolInfo(const std::string &handle)
    {
        PortHandleInfo info = api_.portHandleInfo(handle);
        return info.getToolId() + " s/n:" + info.getSerialNumber();
    }

    std::string NdiDriver::findSromPath(const std::string &srom_name)
    {
        try
        {
            std::string package_path =
                ament_index_cpp::get_package_share_directory("ndi_description");
            return package_path + "/srom/" + srom_name;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("NdiDriver"),
                "Failed to find package path: %s", e.what());
            return "";
        }
    }

} // namespace ndi_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ndi_driver::NdiDriver, hardware_interface::SensorInterface)