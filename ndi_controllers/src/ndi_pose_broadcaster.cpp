// ndi_controllers/src/ndi_pose_broadcaster.cpp
#include "ndi_controllers/ndi_pose_broadcaster.hpp"

#include <memory>
#include <string>
#include <vector>

namespace ndi_controllers
{

    NdiPoseBroadcaster::NdiPoseBroadcaster() : controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn NdiPoseBroadcaster::on_init()
    {
        try
        {
            // Get parameters from ROS
            auto_declare<std::vector<std::string>>("sensor_names", std::vector<std::string>());
            auto_declare<std::vector<int64_t>>("sensor_ids", std::vector<int64_t>());
            auto_declare<std::string>("world_frame", "ndi_frame");
            auto_declare<int>("state_publish_rate", 100);
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn NdiPoseBroadcaster::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        sensor_names_ = get_node()->get_parameter("sensor_names").as_string_array();
        sensor_ids_ = get_node()->get_parameter("sensor_ids").as_integer_array();
        frame_id_ = get_node()->get_parameter("world_frame").as_string();

        if (sensor_names_.empty() || sensor_ids_.empty() ||
            sensor_names_.size() != sensor_ids_.size())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Invalid sensor configuration");
            return controller_interface::CallbackReturn::ERROR;
        }

        tracker_states_.resize(sensor_names_.size());

        // Create publisher
        rigid_pose_publisher_ = get_node()->create_publisher<ndi_msgs::msg::RigidArray>(
            "~/rigid_poses", rclcpp::SystemDefaultsQoS());

        realtime_rigid_pose_publisher_ = std::make_shared<
            realtime_tools::RealtimePublisher<ndi_msgs::msg::RigidArray>>(rigid_pose_publisher_);

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration
    NdiPoseBroadcaster::command_interface_configuration() const
    {
        return controller_interface::InterfaceConfiguration{
            controller_interface::interface_configuration_type::NONE};
    }

    controller_interface::InterfaceConfiguration
    NdiPoseBroadcaster::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        for (const auto &name : sensor_names_)
        {
            config.names.push_back(name + "/pose.position.x");
            config.names.push_back(name + "/pose.position.y");
            config.names.push_back(name + "/pose.position.z");
            config.names.push_back(name + "/pose.orientation.x");
            config.names.push_back(name + "/pose.orientation.y");
            config.names.push_back(name + "/pose.orientation.z");
            config.names.push_back(name + "/pose.orientation.w");
        }

        return config;
    }

    controller_interface::CallbackReturn NdiPoseBroadcaster::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn NdiPoseBroadcaster::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type NdiPoseBroadcaster::update(
        const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
    {
        if (realtime_rigid_pose_publisher_ && realtime_rigid_pose_publisher_->trylock())
        {
            auto &msg = realtime_rigid_pose_publisher_->msg_;

            msg.header.stamp = time;
            msg.header.frame_id = frame_id_;

            msg.poses.clear();
            msg.ids.clear();
            msg.frames.clear();
            msg.inbound.clear();

            // Get latest state from interfaces
            for (size_t i = 0; i < sensor_names_.size(); ++i)
            {
                const auto base_name = sensor_names_[i] + "/pose";
                size_t interface_idx = i * 7; // 7 values per tracker

                // Get position and orientation
                TrackerState &state = tracker_states_[i];
                state.x = state_interfaces_[interface_idx + 0].get_value();
                state.y = state_interfaces_[interface_idx + 1].get_value();
                state.z = state_interfaces_[interface_idx + 2].get_value();
                state.qx = state_interfaces_[interface_idx + 3].get_value();
                state.qy = state_interfaces_[interface_idx + 4].get_value();
                state.qz = state_interfaces_[interface_idx + 5].get_value();
                state.qw = state_interfaces_[interface_idx + 6].get_value();

                // Create pose message
                geometry_msgs::msg::Pose pose;
                pose.position.x = state.x;
                pose.position.y = state.y;
                pose.position.z = state.z;
                pose.orientation.x = state.qx;
                pose.orientation.y = state.qy;
                pose.orientation.z = state.qz;
                pose.orientation.w = state.qw;

                msg.poses.push_back(pose);
                msg.ids.push_back(sensor_ids_[i]);
                msg.frames.push_back(sensor_names_[i]);
                // Consider tracker inbound if position is valid (not NaN)
                msg.inbound.push_back(!std::isnan(state.x));
            }

            realtime_rigid_pose_publisher_->unlockAndPublish();
        }

        return controller_interface::return_type::OK;
    }

} // namespace ndi_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ndi_controllers::NdiPoseBroadcaster,
    controller_interface::ControllerInterface)