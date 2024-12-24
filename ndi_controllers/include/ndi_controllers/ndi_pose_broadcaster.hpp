// ndi_controllers/include/ndi_controllers/ndi_pose_broadcaster.hpp
#ifndef NDI_CONTROLLERS__NDI_POSE_BROADCASTER_HPP_
#define NDI_CONTROLLERS__NDI_POSE_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "ndi_controllers/visibility_control.h"
#include "ndi_msgs/msg/rigid_array.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_publisher.h"

namespace ndi_controllers
{

    class NdiPoseBroadcaster : public controller_interface::ControllerInterface
    {
    public:
        NDI_CONTROLLERS_PUBLIC
        NdiPoseBroadcaster();

        NDI_CONTROLLERS_PUBLIC
        controller_interface::CallbackReturn on_init() override;

        NDI_CONTROLLERS_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        NDI_CONTROLLERS_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        NDI_CONTROLLERS_PUBLIC
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        NDI_CONTROLLERS_PUBLIC
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        NDI_CONTROLLERS_PUBLIC
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        NDI_CONTROLLERS_PUBLIC
        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    protected:
        std::shared_ptr<rclcpp::Publisher<ndi_msgs::msg::RigidArray>> rigid_pose_publisher_;
        std::shared_ptr<realtime_tools::RealtimePublisher<ndi_msgs::msg::RigidArray>>
            realtime_rigid_pose_publisher_;

        std::vector<std::string> sensor_names_;
        std::vector<int64_t> sensor_ids_;
        std::string frame_id_;

        struct TrackerState
        {
            double x{0.0};
            double y{0.0};
            double z{0.0};
            double qx{0.0};
            double qy{0.0};
            double qz{0.0};
            double qw{1.0};
        };
        std::vector<TrackerState> tracker_states_;
    };

} // namespace ndi_controllers

#endif // NDI_CONTROLLERS__NDI_POSE_BROADCASTER_HPP_
