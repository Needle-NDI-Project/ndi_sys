// ndi_driver/include/ndi_driver/ndi_driver.hpp
#ifndef NDI_DRIVER__NDI_DRIVER_HPP_
#define NDI_DRIVER__NDI_DRIVER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/macros.hpp"
#include <rclcpp/rclcpp.hpp>

#include "CombinedApi.h"
#include "PortHandleInfo.h"
#include "ToolData.h"
#include "ndi_driver/visibility_control.h"

namespace ndi_driver
{

    class NdiDriver : public hardware_interface::SensorInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(NdiDriver)

        NDI_DRIVER_PUBLIC
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        NDI_DRIVER_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        NDI_DRIVER_PUBLIC
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        NDI_DRIVER_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        NDI_DRIVER_PUBLIC
        hardware_interface::return_type read(
            const rclcpp::Time &time,
            const rclcpp::Duration &period) override;

    private:
        void loadTools();
        void initializeTools();
        bool supportsBx2();
        void onError(const std::string &method, int code);
        std::string getToolInfo(const std::string &handle);
        std::string findSromPath(const std::string &srom_name);

        // Configuration
        std::string ip_address_;
        std::vector<std::string> tool_names_;
        size_t num_tools_;

        // State
        std::vector<std::vector<double>> tracker_poses_;
        std::vector<ToolData> enabled_tools_;
        std::vector<PortHandleInfo> port_handles_;

        // NDI API
        bool supports_bx2_{false};
        CombinedApi api_;
    };

} // namespace ndi_driver

#endif // NDI_DRIVER__NDI_DRIVER_HPP_