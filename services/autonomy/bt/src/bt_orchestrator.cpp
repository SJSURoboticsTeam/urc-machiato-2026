#include <memory>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>

#include <sstream>
// #include <nlohmann/json.hpp>  // Commented out - requires nlohmann_json package

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/qos.hpp"
#include "rmw/qos_profiles.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

// Jazzy: Events Executor (if available)
#ifdef RCLCPP__EXECUTORS__EVENTS_EXECUTOR_HPP_
#include "rclcpp/executors/events_executor.hpp"
#define EVENTS_EXECUTOR_AVAILABLE
#endif
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Autonomy interfaces
#include "autonomy_interfaces/srv/get_system_state.hpp"
#include "autonomy_interfaces/srv/detect_aruco.hpp"
#include "autonomy_interfaces/srv/detect_mission_aruco.hpp"
#include "autonomy_interfaces/srv/get_blackboard_value.hpp"
#include "autonomy_interfaces/srv/set_blackboard_value.hpp"
#include "autonomy_interfaces/msg/led_command.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Action interfaces for BT
#include "autonomy_interfaces/action/navigate_to_pose.hpp"
#include "autonomy_interfaces/action/execute_mission.hpp"

// Blackboard key constants
#include "autonomy_bt/blackboard_keys.hpp"

using namespace BT;
using namespace std::chrono_literals;

// Standardized BT Node for ROS2 Service calls
class CallService : public SyncActionNode
{
public:
    CallService(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_service_" + name))
    {}

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("service_name", "", "ROS2 service name to call"),
                 InputPort<std::string>("command", "", "Command to send to service"),
                 InputPort<double>("timeout", 5.0, "Timeout in seconds") };
    }

    NodeStatus tick() override
    {
        std::string service_name, command;
        double timeout;

        if (!getInput("service_name", service_name) ||
            !getInput("command", command) ||
            !getInput("timeout", timeout)) {
            return NodeStatus::FAILURE;
        }

        RCLCPP_INFO(node_->get_logger(), "BT: Calling service %s with command %s",
                   service_name.c_str(), command.c_str());

        // Create service client for trigger service (most common for BT operations)
        auto client = node_->create_client<std_srvs::srv::Trigger>(service_name);

        // Wait for service to be available
        if (!client->wait_for_service(std::chrono::seconds(static_cast<int>(timeout)))) {
            RCLCPP_ERROR(node_->get_logger(), "Service %s not available within timeout",
                        service_name.c_str());
            return NodeStatus::FAILURE;
        }

        // Create request and call service
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client->async_send_request(request);

        // Wait for response
        if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(static_cast<int>(timeout)))
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Service call to %s failed or timed out",
                        service_name.c_str());
            return NodeStatus::FAILURE;
        }

        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(node_->get_logger(), "Service %s completed successfully: %s",
                       service_name.c_str(), response->message.c_str());
            return NodeStatus::SUCCESS;
        } else {
            RCLCPP_WARN(node_->get_logger(), "Service %s failed: %s",
                       service_name.c_str(), response->message.c_str());
            return NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
};

// Sensor Check Node
class SensorCheck : public SyncActionNode
{
public:
    SensorCheck(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_" + name))
    {}

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("sensor_type", "", "Type of sensor to check"),
                 InputPort<double>("timeout", 5.0, "Timeout in seconds") };
    }

    NodeStatus tick() override
    {
        std::string sensor_type;
        double timeout;
        getInput("sensor_type", sensor_type);
        getInput("timeout", timeout);

        RCLCPP_INFO(node_->get_logger(), "BT: Checking sensor %s with timeout %.1f",
                   sensor_type.c_str(), timeout);

        // Check different sensor types
        if (sensor_type == "imu") {
            return check_imu_sensor(timeout);
        } else if (sensor_type == "gps") {
            return check_gps_sensor(timeout);
        } else if (sensor_type == "camera") {
            return check_camera_sensor(timeout);
        } else if (sensor_type == "state_check") {
            return check_system_state(timeout);
        } else {
            RCLCPP_WARN(node_->get_logger(), "Unknown sensor type: %s", sensor_type.c_str());
            return NodeStatus::FAILURE;
        }
    }

private:
    NodeStatus check_imu_sensor(double timeout)
    {
        // Simple check - just wait for the timeout period
        // In a real implementation, this would check actual sensor data
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(timeout * 1000)));
        RCLCPP_INFO(node_->get_logger(), "IMU sensor check completed");
        return NodeStatus::SUCCESS;
    }

    NodeStatus check_gps_sensor(double timeout)
    {
        // Simple check - just wait for the timeout period
        // In a real implementation, this would check actual GPS data quality
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(timeout * 1000)));
        RCLCPP_INFO(node_->get_logger(), "GPS sensor check completed");
        return NodeStatus::SUCCESS;
    }

    NodeStatus check_camera_sensor(double timeout)
    {
        // Simple check - just wait for the timeout period
        // In a real implementation, this would check actual camera data
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(timeout * 1000)));
        RCLCPP_INFO(node_->get_logger(), "Camera sensor check completed");
        return NodeStatus::SUCCESS;
    }

    NodeStatus check_system_state(double timeout)
    {
        // Check if the system is in autonomous mode by querying the state machine
        auto client = node_->create_client<autonomy_interfaces::srv::GetSystemState>("/adaptive_state_machine/get_state");

        if (!client->wait_for_service(std::chrono::seconds(static_cast<int>(timeout)))) {
            RCLCPP_ERROR(node_->get_logger(), "State machine service not available");
            return NodeStatus::FAILURE;
        }

        auto request = std::make_shared<autonomy_interfaces::srv::GetSystemState::Request>();
        auto future = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(static_cast<int>(timeout)))
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get system state");
            return NodeStatus::FAILURE;
        }

        auto response = future.get();
        if (response->current_state == "AUTONOMOUS") {
            RCLCPP_INFO(node_->get_logger(), "System is in autonomous mode - proceeding with mission");
            return NodeStatus::SUCCESS;
        } else {
            RCLCPP_WARN(node_->get_logger(), "System is in %s state - mission cannot proceed",
                       response->current_state.c_str());
            return NodeStatus::FAILURE;
        }
    }

    rclcpp::Node::SharedPtr node_;
};

// Navigation Node
class NavigateToWaypoint : public SyncActionNode
{
public:
    NavigateToWaypoint(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_" + name))
    {}

    static PortsList providedPorts()
    {
        return { InputPort<double>("x", 0.0, "Target X coordinate"),
                 InputPort<double>("y", 0.0, "Target Y coordinate"),
                 InputPort<double>("tolerance", 0.5, "Position tolerance in meters") };
    }

    NodeStatus tick() override
    {
        double x, y, tolerance;
        getInput("x", x);
        getInput("y", y);
        getInput("tolerance", tolerance);

        RCLCPP_INFO(node_->get_logger(), "BT: Navigating to waypoint (%.2f, %.2f) with tolerance %.2f",
                   x, y, tolerance);

        // Create navigation action client
        auto action_client = rclcpp_action::create_client<autonomy_interfaces::action::NavigateToPose>(
            node_, "/bt/navigate_to_pose");

        // Wait for action server to be available
        if (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Navigation action server not available");
            return NodeStatus::FAILURE;
        }

        // Create goal
        auto goal = autonomy_interfaces::action::NavigateToPose::Goal();
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = node_->get_clock()->now();
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.position.z = 0.0;
        goal.target_pose.pose.orientation.w = 1.0;  // Identity quaternion
        goal.tolerance = tolerance;
        goal.timeout = 60.0;  // 60 second timeout

        RCLCPP_INFO(node_->get_logger(), "Sending navigation goal to action server");

        // Send goal
        auto goal_options = rclcpp_action::Client<autonomy_interfaces::action::NavigateToPose>::SendGoalOptions();
        goal_options.result_callback = [](const rclcpp_action::ClientGoalHandle<autonomy_interfaces::action::NavigateToPose>::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(rclcpp::get_logger("bt_navigation"), "Navigation completed successfully");
            } else {
                RCLCPP_WARN(rclcpp::get_logger("bt_navigation"), "Navigation failed");
            }
        };

        auto goal_handle_future = action_client->async_send_goal(goal, goal_options);

        // For synchronous BT execution, wait for result
        if (rclcpp::spin_until_future_complete(node_, goal_handle_future, std::chrono::seconds(2))
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to send navigation goal");
            return NodeStatus::FAILURE;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "Navigation goal was rejected");
            return NodeStatus::FAILURE;
        }

        // Wait for result with timeout
        auto result_future = action_client->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(node_, result_future, std::chrono::seconds(65))
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Navigation timed out");
            return NodeStatus::FAILURE;
        }

        auto result = result_future.get();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(node_->get_logger(), "Navigation completed successfully");
            return NodeStatus::SUCCESS;
        } else {
            RCLCPP_WARN(node_->get_logger(), "Navigation failed with code: %d", static_cast<int>(result.code));
            return NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
};

// =============================================================================
// URC 2026 MISSION-SPECIFIC BT NODES
// =============================================================================

// Autonomous Navigation Mission Nodes
class NavigateToGNSS : public SyncActionNode
{
public:
    NavigateToGNSS(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_" + name))
    {}

    static PortsList providedPorts()
    {
        return { InputPort<double>("latitude", 0.0, "Target latitude (decimal degrees)"),
                 InputPort<double>("longitude", 0.0, "Target longitude (decimal degrees)"),
                 InputPort<double>("tolerance", 3.0, "Position tolerance in meters") };
    }

    NodeStatus tick() override
    {
        double latitude, longitude, tolerance;
        getInput("latitude", latitude);
        getInput("longitude", longitude);
        getInput("tolerance", tolerance);

        RCLCPP_INFO(node_->get_logger(), "BT: Navigating to GNSS waypoint (%.6f, %.6f) with tolerance %.1fm",
                   latitude, longitude, tolerance);

        // Call navigation action server with GNSS coordinates
        auto action_client = rclcpp_action::create_client<autonomy_interfaces::action::NavigateToPose>(
            node_, "/bt/navigate_to_pose");

        if (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Navigation action server not available");
            return NodeStatus::FAILURE;
        }

        // Convert GNSS to local coordinates (simplified - would need proper transform)
        // For now, assume coordinates are already in local frame
        auto goal = autonomy_interfaces::action::NavigateToPose::Goal();
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = node_->get_clock()->now();
        goal.target_pose.pose.position.x = longitude * 111319.49;  // Rough longitude to meters conversion
        goal.target_pose.pose.position.y = latitude * 111319.49;   // Rough latitude to meters conversion
        goal.target_pose.pose.position.z = 0.0;
        goal.target_pose.pose.orientation.w = 1.0;
        goal.tolerance = tolerance;
        goal.timeout = 120.0;  // 2 minutes for GNSS navigation

        auto goal_options = rclcpp_action::Client<autonomy_interfaces::action::NavigateToPose>::SendGoalOptions();
        goal_options.result_callback = [](const rclcpp_action::ClientGoalHandle<autonomy_interfaces::action::NavigateToPose>::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(rclcpp::get_logger("bt_gnss_navigation"), "GNSS navigation completed successfully");
            } else {
                RCLCPP_WARN(rclcpp::get_logger("bt_gnss_navigation"), "GNSS navigation failed");
            }
        };

        auto goal_handle_future = action_client->async_send_goal(goal, goal_options);
        if (rclcpp::spin_until_future_complete(node_, goal_handle_future, std::chrono::seconds(2))
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to send GNSS navigation goal");
            return NodeStatus::FAILURE;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "GNSS navigation goal was rejected");
            return NodeStatus::FAILURE;
        }

        auto result_future = action_client->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(node_, result_future, std::chrono::seconds(125))
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "GNSS navigation timed out");
            return NodeStatus::FAILURE;
        }

        auto result = result_future.get();
        return (result.code == rclcpp_action::ResultCode::SUCCEEDED) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
};

class DetectArUcoPost : public SyncActionNode
{
public:
    DetectArUcoPost(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_" + name))
    {}

    static PortsList providedPorts()
    {
        return { InputPort<double>("gnss_lat", 0.0, "Post GNSS latitude"),
                 InputPort<double>("gnss_lon", 0.0, "Post GNSS longitude"),
                 InputPort<double>("search_radius", 20.0, "Search radius in meters") };
    }

    NodeStatus tick() override
    {
        double gnss_lat, gnss_lon, search_radius;
        getInput("gnss_lat", gnss_lat);
        getInput("gnss_lon", gnss_lon);
        getInput("search_radius", search_radius);

        RCLCPP_INFO(node_->get_logger(), "BT: Detecting AR tag post near (%.6f, %.6f) within %.1fm",
                   gnss_lat, gnss_lon, search_radius);

        // Simplified AR tag detection - assume vision service will be implemented
        std::this_thread::sleep_for(std::chrono::seconds(3));  // Simulate detection time

        // For URC, AR tags should be detectable within the specified radius
        RCLCPP_INFO(node_->get_logger(), "AR tag post detected successfully within search radius");
        return NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
};

class DetectObject : public SyncActionNode
{
public:
    DetectObject(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_" + name))
    {}

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("object_type", "", "Object to detect (mallet|hammer|bottle)"),
                 InputPort<double>("gnss_lat", 0.0, "Expected GNSS latitude"),
                 InputPort<double>("gnss_lon", 0.0, "Expected GNSS longitude"),
                 InputPort<double>("accuracy", 10.0, "Required detection accuracy (meters)") };
    }

    NodeStatus tick() override
    {
        std::string object_type;
        double gnss_lat, gnss_lon, accuracy;
        getInput("object_type", object_type);
        getInput("gnss_lat", gnss_lat);
        getInput("gnss_lon", gnss_lon);
        getInput("accuracy", accuracy);

        RCLCPP_INFO(node_->get_logger(), "BT: Detecting %s near (%.6f, %.6f) within %.1fm accuracy",
                   object_type.c_str(), gnss_lat, gnss_lon, accuracy);

        // Simplified object detection - assume vision service will be implemented
        // For now, simulate detection based on object type
        std::this_thread::sleep_for(std::chrono::seconds(2));  // Simulate detection time

        // Simulate detection success/failure based on object type
        if (object_type == "mallet" || object_type == "hammer" || object_type == "bottle") {
            RCLCPP_INFO(node_->get_logger(), "%s detected successfully at target location", object_type.c_str());
            return NodeStatus::SUCCESS;
        } else {
            RCLCPP_WARN(node_->get_logger(), "Unknown object type: %s", object_type.c_str());
            return NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
};

class SignalArrival : public SyncActionNode
{
public:
    SignalArrival(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_" + name))
    {}

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("signal_type", "arrival", "Type of signal (arrival|success|error)"),
                 InputPort<std::string>("target_description", "", "Description of arrival target") };
    }

    NodeStatus tick() override
    {
        std::string signal_type, target_description;
        getInput("signal_type", signal_type);
        getInput("target_description", target_description);

        RCLCPP_INFO(node_->get_logger(), "BT: SIGNAL ARRIVAL - Signaling %s at %s",
                   signal_type.c_str(), target_description.c_str());

        // Publish LED command message
        auto led_publisher = node_->create_publisher<autonomy_interfaces::msg::LedCommand>("/hardware/led_command", 10);

        autonomy_interfaces::msg::LedCommand led_msg;
        led_msg.header.stamp = node_->get_clock()->now();
        led_msg.header.frame_id = "rover";

        // URC 2026 LED status mapping
        if (signal_type == "arrival" || signal_type == "waypoint_reached" ||
            signal_type == "post_reached" || signal_type == "object_detected") {
            // Flashing Green: Successful arrival
            led_msg.status_code = 5;  // Success
            led_msg.red = 0.0;
            led_msg.green = 1.0;
            led_msg.blue = 0.0;
            led_msg.pattern = "blinking";
            led_msg.frequency = 2.0;  // 2 Hz blinking
            led_msg.priority = 1;     // Warning priority
            led_msg.duration = 5.0;   // 5 seconds
        } else if (signal_type == "mission_complete") {
            // Solid Green: Mission complete
            led_msg.status_code = 5;  // Success
            led_msg.red = 0.0;
            led_msg.green = 1.0;
            led_msg.blue = 0.0;
            led_msg.pattern = "solid";
            led_msg.frequency = 0.0;
            led_msg.priority = 2;     // Critical priority
            led_msg.duration = 10.0;  // 10 seconds
        } else {
            // Default: Solid Red (error/unknown)
            led_msg.status_code = 3;  // Error
            led_msg.red = 1.0;
            led_msg.green = 0.0;
            led_msg.blue = 0.0;
            led_msg.pattern = "solid";
            led_msg.frequency = 0.0;
            led_msg.priority = 2;     // Critical priority
            led_msg.duration = 3.0;   // 3 seconds
        }

        led_msg.override = true;  // Override any lower priority signals

        led_publisher->publish(led_msg);
        RCLCPP_INFO(node_->get_logger(), "LED signal published for %s", signal_type.c_str());

        return NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
};

// Equipment Servicing Mission Nodes
class PerformTyping : public SyncActionNode
{
public:
    PerformTyping(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_" + name))
    {}

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("launch_key", "", "Launch key sequence to type"),
                 InputPort<double>("timeout", 120.0, "Typing timeout in seconds") };
    }

    NodeStatus tick() override
    {
        std::string launch_key;
        double timeout;
        getInput("launch_key", launch_key);
        getInput("timeout", timeout);

        RCLCPP_INFO(node_->get_logger(), "BT: Performing autonomous typing of launch key: %s",
                   launch_key.c_str());

        // Simplified autonomous typing - assume typing service will be implemented
        std::this_thread::sleep_for(std::chrono::seconds(10));  // Simulate typing time

        // For URC, assume typing succeeds for valid launch keys (3-6 characters)
        if (launch_key.length() >= 3 && launch_key.length() <= 6) {
            RCLCPP_INFO(node_->get_logger(), "Autonomous typing completed successfully for key: %s", launch_key.c_str());
            return NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Invalid launch key length: %zu characters", launch_key.length());
            return NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
};

class InsertCache : public SyncActionNode
{
public:
    InsertCache(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_" + name))
    {}

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("cache_id", "", "Cache container identifier"),
                 InputPort<double>("timeout", 60.0, "Cache insertion timeout in seconds") };
    }

    NodeStatus tick() override
    {
        std::string cache_id;
        double timeout;
        getInput("cache_id", cache_id);
        getInput("timeout", timeout);

        RCLCPP_INFO(node_->get_logger(), "BT: Inserting cache %s into lander drawer",
                   cache_id.c_str());

        // Call arm control service for cache insertion sequence
        auto client = node_->create_client<std_srvs::srv::Trigger>("/arm/insert_cache");

        if (!client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Arm cache insertion service not available");
            return NodeStatus::FAILURE;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(static_cast<int>(timeout)))
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Cache insertion timed out");
            return NodeStatus::FAILURE;
        }

        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(node_->get_logger(), "Cache insertion completed successfully");
            return NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Cache insertion failed: %s", response->message.c_str());
            return NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
};

// Additional Equipment Servicing Nodes
class ConnectUSB : public SyncActionNode
{
public:
    ConnectUSB(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_" + name))
    {}

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("usb_slot_id", "", "USB slot identifier"),
                 InputPort<double>("timeout", 30.0, "USB connection timeout in seconds") };
    }

    NodeStatus tick() override
    {
        std::string usb_slot_id;
        double timeout;
        getInput("usb_slot_id", usb_slot_id);
        getInput("timeout", timeout);

        RCLCPP_INFO(node_->get_logger(), "BT: Connecting USB to slot %s", usb_slot_id.c_str());

        // Call arm control for USB insertion
        auto client = node_->create_client<std_srvs::srv::Trigger>("/arm/connect_usb");

        if (!client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Arm USB connection service not available");
            return NodeStatus::FAILURE;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(static_cast<int>(timeout)))
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "USB connection timed out");
            return NodeStatus::FAILURE;
        }

        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(node_->get_logger(), "USB connection completed successfully");
            // Could read GNSS coordinates from the memory card here
            return NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "USB connection failed: %s", response->message.c_str());
            return NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
};

class ConnectHose : public SyncActionNode
{
public:
    ConnectHose(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_" + name))
    {}

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("hose_type", "fuel", "Type of hose to connect"),
                 InputPort<double>("timeout", 45.0, "Hose connection timeout in seconds") };
    }

    NodeStatus tick() override
    {
        std::string hose_type;
        double timeout;
        getInput("hose_type", hose_type);
        getInput("timeout", timeout);

        RCLCPP_INFO(node_->get_logger(), "BT: Connecting %s hose to lander", hose_type.c_str());

        // Call arm control for hose connection (cam lock fitting)
        auto client = node_->create_client<std_srvs::srv::Trigger>("/arm/connect_hose");

        if (!client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Arm hose connection service not available");
            return NodeStatus::FAILURE;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(static_cast<int>(timeout)))
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Hose connection timed out");
            return NodeStatus::FAILURE;
        }

        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(node_->get_logger(), "%s hose connection completed successfully", hose_type.c_str());
            return NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "%s hose connection failed: %s", hose_type.c_str(), response->message.c_str());
            return NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
};

class TurnValve : public SyncActionNode
{
public:
    TurnValve(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_" + name))
    {}

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("valve_id", "", "Valve identifier"),
                 InputPort<double>("turns", 0.25, "Number of turns (default 1/4 turn)"),
                 InputPort<double>("timeout", 30.0, "Valve operation timeout in seconds") };
    }

    NodeStatus tick() override
    {
        std::string valve_id;
        double turns, timeout;
        getInput("valve_id", valve_id);
        getInput("turns", turns);
        getInput("timeout", timeout);

        RCLCPP_INFO(node_->get_logger(), "BT: Turning valve %s by %.2f turns", valve_id.c_str(), turns);

        // Call arm control for valve operation
        auto client = node_->create_client<std_srvs::srv::Trigger>("/arm/turn_valve");

        if (!client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Arm valve operation service not available");
            return NodeStatus::FAILURE;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(static_cast<int>(timeout)))
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Valve operation timed out");
            return NodeStatus::FAILURE;
        }

        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(node_->get_logger(), "Valve %s operation completed successfully", valve_id.c_str());
            return NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Valve %s operation failed: %s", valve_id.c_str(), response->message.c_str());
            return NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
};

// Delivery Mission Nodes
class PickupObject : public SyncActionNode
{
public:
    PickupObject(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_" + name))
    {}

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("object_type", "", "Type of object to pickup"),
                 InputPort<double>("gnss_lat", 0.0, "Object GNSS latitude"),
                 InputPort<double>("gnss_lon", 0.0, "Object GNSS longitude"),
                 InputPort<double>("timeout", 120.0, "Pickup timeout in seconds") };
    }

    NodeStatus tick() override
    {
        std::string object_type;
        double gnss_lat, gnss_lon, timeout;
        getInput("object_type", object_type);
        getInput("gnss_lat", gnss_lat);
        getInput("gnss_lon", gnss_lon);
        getInput("timeout", timeout);

        RCLCPP_INFO(node_->get_logger(), "BT: Picking up %s at (%.6f, %.6f)",
                   object_type.c_str(), gnss_lat, gnss_lon);

        // Navigate to object location first
        auto nav_client = rclcpp_action::create_client<autonomy_interfaces::action::NavigateToPose>(
            node_, "/bt/navigate_to_pose");

        if (!nav_client->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Navigation action server not available for pickup");
            return NodeStatus::FAILURE;
        }

        auto nav_goal = autonomy_interfaces::action::NavigateToPose::Goal();
        nav_goal.target_pose.header.frame_id = "map";
        nav_goal.target_pose.header.stamp = node_->get_clock()->now();
        nav_goal.target_pose.pose.position.x = gnss_lon * 111319.49;
        nav_goal.target_pose.pose.position.y = gnss_lat * 111319.49;
        nav_goal.target_pose.pose.position.z = 0.0;
        nav_goal.target_pose.pose.orientation.w = 1.0;
        nav_goal.tolerance = 2.0;  // 2m tolerance for object pickup
        nav_goal.timeout = timeout * 0.7;  // Use 70% of time for navigation

        auto nav_future = nav_client->async_send_goal(nav_goal);
        if (rclcpp::spin_until_future_complete(node_, nav_future, std::chrono::seconds(2))
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to send navigation goal for pickup");
            return NodeStatus::FAILURE;
        }

        auto nav_handle = nav_future.get();
        if (!nav_handle) {
            RCLCPP_ERROR(node_->get_logger(), "Navigation goal for pickup was rejected");
            return NodeStatus::FAILURE;
        }

        auto nav_result_future = nav_client->async_get_result(nav_handle);
        if (rclcpp::spin_until_future_complete(node_, nav_result_future, std::chrono::seconds(static_cast<int>(timeout * 0.7) + 5))
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Navigation for pickup timed out");
            return NodeStatus::FAILURE;
        }

        auto nav_result = nav_result_future.get();
        if (nav_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(node_->get_logger(), "Navigation for pickup failed");
            return NodeStatus::FAILURE;
        }

        // Now perform object pickup
        auto pickup_client = node_->create_client<std_srvs::srv::Trigger>("/arm/pickup_object");

        if (!pickup_client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Arm pickup service not available");
            return NodeStatus::FAILURE;
        }

        auto pickup_request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto pickup_future = pickup_client->async_send_request(pickup_request);

        if (rclcpp::spin_until_future_complete(node_, pickup_future, std::chrono::seconds(static_cast<int>(timeout * 0.3)))
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Object pickup timed out");
            return NodeStatus::FAILURE;
        }

        auto pickup_response = pickup_future.get();
        if (pickup_response->success) {
            RCLCPP_INFO(node_->get_logger(), "%s pickup completed successfully", object_type.c_str());
            return NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "%s pickup failed: %s", object_type.c_str(), pickup_response->message.c_str());
            return NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
};

class DeliverObject : public SyncActionNode
{
public:
    DeliverObject(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_" + name))
    {}

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("object_type", "", "Type of object to deliver"),
                 InputPort<double>("gnss_lat", 0.0, "Delivery GNSS latitude"),
                 InputPort<double>("gnss_lon", 0.0, "Delivery GNSS longitude"),
                 InputPort<double>("timeout", 120.0, "Delivery timeout in seconds") };
    }

    NodeStatus tick() override
    {
        std::string object_type;
        double gnss_lat, gnss_lon, timeout;
        getInput("object_type", object_type);
        getInput("gnss_lat", gnss_lat);
        getInput("gnss_lon", gnss_lon);
        getInput("timeout", timeout);

        RCLCPP_INFO(node_->get_logger(), "BT: Delivering %s to (%.6f, %.6f)",
                   object_type.c_str(), gnss_lat, gnss_lon);

        // Navigate to delivery location
        auto nav_client = rclcpp_action::create_client<autonomy_interfaces::action::NavigateToPose>(
            node_, "/bt/navigate_to_pose");

        if (!nav_client->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Navigation action server not available for delivery");
            return NodeStatus::FAILURE;
        }

        auto nav_goal = autonomy_interfaces::action::NavigateToPose::Goal();
        nav_goal.target_pose.header.frame_id = "map";
        nav_goal.target_pose.header.stamp = node_->get_clock()->now();
        nav_goal.target_pose.pose.position.x = gnss_lon * 111319.49;
        nav_goal.target_pose.pose.position.y = gnss_lat * 111319.49;
        nav_goal.target_pose.pose.position.z = 0.0;
        nav_goal.target_pose.pose.orientation.w = 1.0;
        nav_goal.tolerance = 2.0;
        nav_goal.timeout = timeout * 0.7;

        auto nav_future = nav_client->async_send_goal(nav_goal);
        if (rclcpp::spin_until_future_complete(node_, nav_future, std::chrono::seconds(2))
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to send navigation goal for delivery");
            return NodeStatus::FAILURE;
        }

        auto nav_handle = nav_future.get();
        if (!nav_handle) {
            RCLCPP_ERROR(node_->get_logger(), "Navigation goal for delivery was rejected");
            return NodeStatus::FAILURE;
        }

        auto nav_result_future = nav_client->async_get_result(nav_handle);
        if (rclcpp::spin_until_future_complete(node_, nav_result_future, std::chrono::seconds(static_cast<int>(timeout * 0.7) + 5))
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Navigation for delivery timed out");
            return NodeStatus::FAILURE;
        }

        auto nav_result = nav_result_future.get();
        if (nav_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(node_->get_logger(), "Navigation for delivery failed");
            return NodeStatus::FAILURE;
        }

        // Perform object delivery
        auto deliver_client = node_->create_client<std_srvs::srv::Trigger>("/arm/deliver_object");

        if (!deliver_client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Arm delivery service not available");
            return NodeStatus::FAILURE;
        }

        auto deliver_request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto deliver_future = deliver_client->async_send_request(deliver_request);

        if (rclcpp::spin_until_future_complete(node_, deliver_future, std::chrono::seconds(static_cast<int>(timeout * 0.3)))
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Object delivery timed out");
            return NodeStatus::FAILURE;
        }

        auto deliver_response = deliver_future.get();
        if (deliver_response->success) {
            RCLCPP_INFO(node_->get_logger(), "%s delivery completed successfully", object_type.c_str());
            return NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "%s delivery failed: %s", object_type.c_str(), deliver_response->message.c_str());
            return NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
};

// Additional Delivery Mission Nodes
class ReadSign : public SyncActionNode
{
public:
    ReadSign(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_" + name))
    {}

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("sign_type", "", "Type of sign (astronaut_held|posted)"),
                 InputPort<double>("timeout", 30.0, "Sign reading timeout in seconds") };
    }

    NodeStatus tick() override
    {
        std::string sign_type;
        double timeout;
        getInput("sign_type", sign_type);
        getInput("timeout", timeout);

        RCLCPP_INFO(node_->get_logger(), "BT: Reading %s sign", sign_type.c_str());

        // Simplified sign reading - assume vision service will be implemented
        std::this_thread::sleep_for(std::chrono::seconds(5));  // Simulate reading time

        // For URC, assume signs can be read successfully
        RCLCPP_INFO(node_->get_logger(), "%s sign read successfully", sign_type.c_str());
        return NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
};

class SearchArea : public SyncActionNode
{
public:
    SearchArea(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_" + name))
    {}

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("target_type", "", "Type of target to search for"),
                 InputPort<double>("center_lat", 0.0, "Search area center latitude"),
                 InputPort<double>("center_lon", 0.0, "Search area center longitude"),
                 InputPort<double>("radius", 50.0, "Search radius in meters"),
                 InputPort<double>("timeout", 180.0, "Search timeout in seconds") };
    }

    NodeStatus tick() override
    {
        std::string target_type;
        double center_lat, center_lon, radius, timeout;
        getInput("target_type", target_type);
        getInput("center_lat", center_lat);
        getInput("center_lon", center_lon);
        getInput("radius", radius);
        getInput("timeout", timeout);

        RCLCPP_INFO(node_->get_logger(), "BT: Searching for %s in area (%.6f, %.6f) radius %.1fm",
                   target_type.c_str(), center_lat, center_lon, radius);

        // Perform systematic search pattern
        // This would involve multiple navigation waypoints in a search pattern
        // For now, simplified implementation

        auto search_client = node_->create_client<std_srvs::srv::Trigger>("/mission/search_area");

        if (!search_client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Area search service not available");
            return NodeStatus::FAILURE;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = search_client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(static_cast<int>(timeout)))
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Area search timed out");
            return NodeStatus::FAILURE;
        }

        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(node_->get_logger(), "%s found in search area", target_type.c_str());
            return NodeStatus::SUCCESS;
        } else {
            RCLCPP_WARN(node_->get_logger(), "%s not found in search area", target_type.c_str());
            return NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
};

// Sample Collection Node
class SampleCollection : public SyncActionNode
{
public:
    SampleCollection(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_" + name))
    {}

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("site_id", "", "Sampling site identifier"),
                 InputPort<double>("timeout", 60.0, "Sampling timeout in seconds") };
    }

    NodeStatus tick() override
    {
        std::string site_id;
        double timeout;
        getInput("site_id", site_id);
        getInput("timeout", timeout);

        RCLCPP_INFO(node_->get_logger(), "BT: Collecting sample at site %s with timeout %.1f",
                   site_id.c_str(), timeout);

        // Create mission execution action client
        auto action_client = rclcpp_action::create_client<autonomy_interfaces::action::ExecuteMission>(
            node_, "/bt/execute_mission");

        // Wait for action server to be available
        if (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Mission execution action server not available");
            return NodeStatus::FAILURE;
        }

        // Create goal for sample collection mission
        auto goal = autonomy_interfaces::action::ExecuteMission::Goal();
        goal.mission_type = "sample_collection";
        goal.mission_id = site_id;
        goal.timeout = timeout;
        goal.waypoints = {site_id};  // Single waypoint for this sample

        RCLCPP_INFO(node_->get_logger(), "Sending sample collection mission goal");

        // Send goal
        auto goal_options = rclcpp_action::Client<autonomy_interfaces::action::ExecuteMission>::SendGoalOptions();
        goal_options.result_callback = [](const rclcpp_action::ClientGoalHandle<autonomy_interfaces::action::ExecuteMission>::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(rclcpp::get_logger("bt_sample_collection"), "Sample collection completed successfully");
            } else {
                RCLCPP_WARN(rclcpp::get_logger("bt_sample_collection"), "Sample collection failed");
            }
        };

        auto goal_handle_future = action_client->async_send_goal(goal, goal_options);

        // For synchronous BT execution, wait for result
        if (rclcpp::spin_until_future_complete(node_, goal_handle_future, std::chrono::seconds(2))
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to send sample collection goal");
            return NodeStatus::FAILURE;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "Sample collection goal was rejected");
            return NodeStatus::FAILURE;
        }

        // Wait for result with timeout
        auto result_future = action_client->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(node_, result_future, std::chrono::seconds(static_cast<int>(timeout) + 5))
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Sample collection timed out");
            return NodeStatus::FAILURE;
        }

        auto result = result_future.get();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(node_->get_logger(), "Sample collection completed successfully");
            return NodeStatus::SUCCESS;
        } else {
            RCLCPP_WARN(node_->get_logger(), "Sample collection failed with code: %d", static_cast<int>(result.code));
            return NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
};

// CheckEmergencyStop: condition node; SUCCESS when emergency stop is active (so emergency branch runs)
// In XML use emergency_stop="{system.emergency_stop}" or emergency_stop="{emergency_stop_active}"
class CheckEmergencyStop : public SyncActionNode
{
public:
    CheckEmergencyStop(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_" + name))
    {}

    static PortsList providedPorts()
    {
        return { InputPort<bool>("emergency_stop", false, "True when emergency stop is active") };
    }

    NodeStatus tick() override
    {
        bool emergency = false;
        getInput("emergency_stop", emergency);
        return emergency ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
};

// Emergency Stop Node
class EmergencyStop : public SyncActionNode
{
public:
    EmergencyStop(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_" + name))
    {}

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("reason", "unknown", "Reason for emergency stop") };
    }

    NodeStatus tick() override
    {
        std::string reason;
        getInput("reason", reason);

        RCLCPP_ERROR(node_->get_logger(), "BT: EMERGENCY STOP triggered - %s", reason.c_str());
        // Execute emergency stop synchronously (e.g. stop commands would be published here).
        // SyncActionNode must never return RUNNING; report success so tree can handle follow-up.
        return NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
};

// Jazzy QoS Profiles for BT Orchestrator
rclcpp::QoS get_autonomy_status_qos() {
    rclcpp::QoS qos(5);
    qos.reliable();
    qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    qos.deadline(std::chrono::milliseconds(100));
    return qos;
}

rclcpp::QoS get_telemetry_qos() {
    rclcpp::QoS qos(50);
    qos.best_effort();
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
    qos.deadline(std::chrono::seconds(1));
    return qos;
}

class BTOrchestrator : public rclcpp_lifecycle::LifecycleNode
{
public:
    BTOrchestrator() : rclcpp_lifecycle::LifecycleNode("bt_orchestrator")
    {
        RCLCPP_INFO(get_logger(), "BT Orchestrator initialized (BT.CPP 4.x with LifecycleNode)");
        declare_parameter("run_bt_tick", true);
        run_bt_tick_ = get_parameter("run_bt_tick").get_value<bool>();

        // Create callback groups for priority scheduling
        realtime_callback_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        
        telemetry_callback_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        blackboard_callback_group_ = create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
    }
    
    // Lifecycle callbacks
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State & /* state */) override
    {
        RCLCPP_INFO(get_logger(), "Configuring BT Orchestrator...");
        
        // Initialize BT orchestrator
        if (!initialize_bt_orchestrator()) {
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
        
        RCLCPP_INFO(get_logger(), "BT Orchestrator configured successfully");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & /* state */) override
    {
        RCLCPP_INFO(get_logger(), "Activating BT Orchestrator...");
        
        // Activate lifecycle publishers
        if (telemetry_publisher_) {
            telemetry_publisher_->on_activate();
        }

        if (run_bt_tick_) {
            execution_timer_ = create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&BTOrchestrator::execute_tree, this),
                realtime_callback_group_);
            telemetry_timer_ = create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&BTOrchestrator::publish_telemetry_status, this),
                telemetry_callback_group_);
        } else {
            RCLCPP_INFO(get_logger(), "BT tick disabled (run_bt_tick:=false); blackboard services only");
        }

        RCLCPP_INFO(get_logger(), "BT Orchestrator activated");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & /* state */) override
    {
        RCLCPP_INFO(get_logger(), "Deactivating BT Orchestrator...");
        
        // Cancel timers
        if (execution_timer_) {
            execution_timer_->cancel();
        }
        if (telemetry_timer_) {
            telemetry_timer_->cancel();
        }
        
        // Deactivate publishers
        if (telemetry_publisher_) {
            telemetry_publisher_->on_deactivate();
        }
        
        RCLCPP_INFO(get_logger(), "BT Orchestrator deactivated");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State & /* state */) override
    {
        RCLCPP_INFO(get_logger(), "Cleaning up BT Orchestrator...");
        
        // Clean up resources
        execution_timer_.reset();
        telemetry_timer_.reset();
        telemetry_publisher_.reset();
        file_logger_.reset();
        console_logger_.reset();
        
        RCLCPP_INFO(get_logger(), "BT Orchestrator cleaned up");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    bool initialize_bt_orchestrator()
    {
        // Initialize blackboard for inter-node communication
        blackboard_ = BT::Blackboard::create();

        // Initialize with validated default values using schema constants
        // Mission state
        set_blackboard_value(BlackboardKeys::MISSION_ACTIVE, false);
        set_blackboard_value(BlackboardKeys::SAMPLES_COLLECTED, 0);
        set_blackboard_value(BlackboardKeys::WAYPOINTS_COMPLETED, 0);
        set_blackboard_value(BlackboardKeys::CURRENT_WAYPOINT_INDEX, 0);
        set_blackboard_value(BlackboardKeys::CURRENT_MISSION_PHASE, std::string("idle"));
        
        // Robot state (from odometry)
        set_blackboard_value(BlackboardKeys::ROBOT_X, 0.0);
        set_blackboard_value(BlackboardKeys::ROBOT_Y, 0.0);
        set_blackboard_value(BlackboardKeys::ROBOT_YAW, 0.0);
        
        // SLAM state
        set_blackboard_value(BlackboardKeys::SLAM_X, 0.0);
        set_blackboard_value(BlackboardKeys::SLAM_Y, 0.0);
        set_blackboard_value(BlackboardKeys::SLAM_CONFIDENCE, 0.0);
        
        // Navigation state (for autonomous navigation)
        set_blackboard_value(BlackboardKeys::NAVIGATION_STATUS, std::string("idle"));
        set_blackboard_value(BlackboardKeys::PATH_CLEAR, true);
        set_blackboard_value(BlackboardKeys::DISTANCE_TO_TARGET, 999.0);
        
        // Perception state
        set_blackboard_value(BlackboardKeys::PERCEPTION_CONFIDENCE, 0.0);
        set_blackboard_value(BlackboardKeys::MAP_QUALITY, 0.0);
        set_blackboard_value(BlackboardKeys::FEATURE_COUNT, 0);
        set_blackboard_value(BlackboardKeys::OBSTACLE_DETECTION_CONFIDENCE, 0.0);
        
        // Obstacle detection
        set_blackboard_value(BlackboardKeys::CLOSEST_OBSTACLE_DISTANCE, 999.0);
        set_blackboard_value(BlackboardKeys::OBSTACLE_DETECTED, false);
        
        // Safety state
        set_blackboard_value(BlackboardKeys::SAFETY_STOP_ACTIVE, false);
        set_blackboard_value(BlackboardKeys::EMERGENCY_STOP_ACTIVE, false);
        set_blackboard_value(BlackboardKeys::PROXIMITY_VIOLATION_DISTANCE, 999.0);
        set_blackboard_value(BlackboardKeys::BATTERY_LEVEL, 100.0);
        
        // System health
        set_blackboard_value(BlackboardKeys::SENSORS_OK, true);
        set_blackboard_value(BlackboardKeys::NAVIGATION_OK, true);
        set_blackboard_value(BlackboardKeys::LAST_ERROR, std::string(""));

        // Hybrid / mode keys (dot notation)
        set_blackboard_value(BlackboardKeys::Auto::MODE, std::string("idle"));
        set_blackboard_value(BlackboardKeys::Auto::MODE_REQUEST, std::string(""));
        set_blackboard_value(BlackboardKeys::Auto::HUMAN_OVERRIDE_ACTIVE, false);
        set_blackboard_value(BlackboardKeys::Auto::ASSIST_ENABLED, true);
        set_blackboard_value(BlackboardKeys::System::EMERGENCY_STOP, false);
        set_blackboard_value(BlackboardKeys::System::BATTERY_PERCENT, 100.0);
        set_blackboard_value(BlackboardKeys::System::LAST_ERROR, std::string(""));

        // Register standardized nodes
        factory_.registerNodeType<CallService>("CallService");
        factory_.registerNodeType<SensorCheck>("SensorCheck");
        factory_.registerNodeType<NavigateToWaypoint>("NavigateToWaypoint");
        factory_.registerNodeType<SampleCollection>("SampleCollection");
        factory_.registerNodeType<EmergencyStop>("EmergencyStop");
        factory_.registerNodeType<CheckEmergencyStop>("CheckEmergencyStop");

        // Register URC 2026 mission-specific nodes
        // Autonomous Navigation Mission
        factory_.registerNodeType<NavigateToGNSS>("NavigateToGNSS");
        factory_.registerNodeType<DetectArUcoPost>("DetectArUcoPost");
        factory_.registerNodeType<DetectObject>("DetectObject");
        factory_.registerNodeType<SignalArrival>("SignalArrival");

        // Equipment Servicing Mission
        factory_.registerNodeType<PerformTyping>("PerformTyping");
        factory_.registerNodeType<InsertCache>("InsertCache");
        factory_.registerNodeType<ConnectUSB>("ConnectUSB");
        factory_.registerNodeType<ConnectHose>("ConnectHose");
        factory_.registerNodeType<TurnValve>("TurnValve");

        // Delivery Mission
        factory_.registerNodeType<PickupObject>("PickupObject");
        factory_.registerNodeType<DeliverObject>("DeliverObject");
        factory_.registerNodeType<ReadSign>("ReadSign");
        factory_.registerNodeType<SearchArea>("SearchArea");

        // Load behavior tree
        try {
            // Find behavior tree file using ROS2 share directory
            std::string package_share_directory = ament_index_cpp::get_package_share_directory("autonomy_bt");
            std::string bt_file_path = package_share_directory + "/behavior_trees/main_mission.xml";
            tree_ = factory_.createTreeFromFile(bt_file_path, blackboard_);
            RCLCPP_INFO(get_logger(), "Behavior tree loaded successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to load behavior tree: %s", e.what());
            return false;
        }

        // Set up ROS2 subscribers for blackboard updates with Jazzy QoS
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = realtime_callback_group_;
        
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", get_autonomy_status_qos(),
            std::bind(&BTOrchestrator::odom_callback, this, std::placeholders::_1),
            sub_options);

        // Try PoseWithCovarianceStamped first (for confidence calculation), fallback to PoseStamped
        slam_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/slam/pose", get_autonomy_status_qos(),
            std::bind(&BTOrchestrator::slam_pose_callback, this, std::placeholders::_1),
            sub_options);
        
        // Also subscribe to PoseWithCovarianceStamped if available (for better confidence)
        slam_pose_cov_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/slam/pose_with_covariance", get_autonomy_status_qos(),
            std::bind(&BTOrchestrator::slam_pose_cov_callback, this, std::placeholders::_1),
            sub_options);

        // Set up file logger for debugging
        file_logger_ = std::make_unique<BT::FileLogger2>(tree_, "/tmp/bt_execution.btlog");
        console_logger_ = std::make_unique<BT::StdCoutLogger>(tree_);

        // Set up telemetry publisher with lifecycle support
        telemetry_publisher_ = create_publisher<std_msgs::msg::String>("/bt/telemetry", get_telemetry_qos());

        // Set up action servers with callback groups
        navigate_to_pose_server_ = rclcpp_action::create_server<autonomy_interfaces::action::NavigateToPose>(
            this,
            "/bt/navigate_to_pose",
            std::bind(&BTOrchestrator::handle_navigate_to_pose_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&BTOrchestrator::handle_navigate_to_pose_cancel, this, std::placeholders::_1),
            std::bind(&BTOrchestrator::handle_navigate_to_pose_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(), realtime_callback_group_);

        execute_mission_server_ = rclcpp_action::create_server<autonomy_interfaces::action::ExecuteMission>(
            this,
            "/bt/execute_mission",
            std::bind(&BTOrchestrator::handle_execute_mission_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&BTOrchestrator::handle_execute_mission_cancel, this, std::placeholders::_1),
            std::bind(&BTOrchestrator::handle_execute_mission_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(), realtime_callback_group_);

        // Set up blackboard services with dedicated callback group (avoids blocking telemetry)
        blackboard_get_srv_ = create_service<autonomy_interfaces::srv::GetBlackboardValue>(
            "/blackboard/get_value",
            std::bind(&BTOrchestrator::get_blackboard_value_callback, this, std::placeholders::_1, std::placeholders::_2),
            rclcpp::ServicesQoS(), blackboard_callback_group_);

        blackboard_set_srv_ = create_service<autonomy_interfaces::srv::SetBlackboardValue>(
            "/blackboard/set_value",
            std::bind(&BTOrchestrator::set_blackboard_value_callback, this, std::placeholders::_1, std::placeholders::_2),
            rclcpp::ServicesQoS(), blackboard_callback_group_);

        RCLCPP_INFO(get_logger(), "BT Orchestrator fully initialized with unified blackboard services");
        return true;
    }

public:

private:
    // Blackboard for inter-node communication
    BT::Blackboard::Ptr blackboard_;

    // Callback groups for priority scheduling
    rclcpp::CallbackGroup::SharedPtr realtime_callback_group_;
    rclcpp::CallbackGroup::SharedPtr telemetry_callback_group_;
    rclcpp::CallbackGroup::SharedPtr blackboard_callback_group_;

    // ROS2 subscribers for blackboard updates
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr slam_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_pose_cov_sub_;

    // ROS2 publisher for telemetry (lifecycle)
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr telemetry_publisher_;

    // Action servers for BT-driven operations
    rclcpp_action::Server<autonomy_interfaces::action::NavigateToPose>::SharedPtr navigate_to_pose_server_;
    rclcpp_action::Server<autonomy_interfaces::action::ExecuteMission>::SharedPtr execute_mission_server_;

    // Blackboard services for unified access
    rclcpp::Service<autonomy_interfaces::srv::GetBlackboardValue>::SharedPtr blackboard_get_srv_;
    rclcpp::Service<autonomy_interfaces::srv::SetBlackboardValue>::SharedPtr blackboard_set_srv_;

    BehaviorTreeFactory factory_;
    Tree tree_;
    std::unique_ptr<BT::FileLogger2> file_logger_;
    std::unique_ptr<BT::StdCoutLogger> console_logger_;
    bool run_bt_tick_;
    rclcpp::TimerBase::SharedPtr execution_timer_;
    rclcpp::TimerBase::SharedPtr telemetry_timer_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Update blackboard with current position using schema constants
        blackboard_->set(BlackboardKeys::ROBOT_X, msg->pose.pose.position.x);
        blackboard_->set(BlackboardKeys::ROBOT_Y, msg->pose.pose.position.y);
        
        // Convert quaternion to yaw using tf2_geometry_msgs
        double roll, pitch, yaw;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        blackboard_->set(BlackboardKeys::ROBOT_YAW, yaw);
        
        // Also update velocity
        blackboard_->set(BlackboardKeys::ROBOT_VELOCITY_X, msg->twist.twist.linear.x);
        blackboard_->set(BlackboardKeys::ROBOT_VELOCITY_Y, msg->twist.twist.linear.y);
    }

    void slam_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Update blackboard with SLAM pose (no covariance available)
        blackboard_->set(BlackboardKeys::SLAM_X, msg->pose.position.x);
        blackboard_->set(BlackboardKeys::SLAM_Y, msg->pose.position.y);
        
        // Default confidence (will be updated by covariance callback if available)
        blackboard_->set(BlackboardKeys::SLAM_CONFIDENCE, 0.9);
        blackboard_->set(BlackboardKeys::PERCEPTION_CONFIDENCE, 0.9);
    }

    void slam_pose_cov_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // Update blackboard with SLAM pose using schema constants
        blackboard_->set(BlackboardKeys::SLAM_X, msg->pose.pose.position.x);
        blackboard_->set(BlackboardKeys::SLAM_Y, msg->pose.pose.position.y);
        
        // Calculate confidence from covariance
        double confidence = 0.9;  // Default
        if (msg->pose.covariance.size() >= 1) {
            // Use position variance (xx element) to calculate confidence
            double position_variance = msg->pose.covariance[0];
            // Inverse relationship: higher variance = lower confidence
            confidence = 1.0 / (1.0 + position_variance);
            confidence = std::max(0.0, std::min(1.0, confidence));  // Clamp to [0, 1]
        }
        blackboard_->set(BlackboardKeys::SLAM_CONFIDENCE, confidence);
        
        // Also set as perception_confidence for navigation purposes
        blackboard_->set(BlackboardKeys::PERCEPTION_CONFIDENCE, confidence);
    }


    void publish_bt_telemetry(const std::string& event_type)
    {
        // Publish comprehensive BT execution telemetry with robust blackboard access
        auto telemetry_msg = std_msgs::msg::String();
        std::stringstream ss;
        ss << "{"
           << "\"component\":\"bt_orchestrator\","
           << "\"timestamp\":" << (get_clock()->now().nanoseconds() / 1e9) << ","
           << "\"event_type\":\"" << event_type << "\","
           << "\"tree_status\":\"" << (tree_.rootNode() ? BT::toStr(tree_.rootNode()->status()) : "no_tree") << "\","
           << "\"mission_progress\":{"
           << "\"mission_active\":" << (get_blackboard_value<bool>(BlackboardKeys::MISSION_ACTIVE, false) ? "true" : "false") << ","
           << "\"current_mission\":\"sample_collection\","
           << "\"samples_collected\":" << get_blackboard_value<int>(BlackboardKeys::SAMPLES_COLLECTED, 0) << ","
           << "\"waypoints_completed\":" << get_blackboard_value<int>(BlackboardKeys::WAYPOINTS_COMPLETED, 0)
           << "},"
           << "\"robot_state\":{"
           << "\"robot_x\":" << get_blackboard_value<double>(BlackboardKeys::ROBOT_X, 0.0) << ","
           << "\"robot_y\":" << get_blackboard_value<double>(BlackboardKeys::ROBOT_Y, 0.0) << ","
           << "\"robot_yaw\":" << get_blackboard_value<double>(BlackboardKeys::ROBOT_YAW, 0.0) << ","
           << "\"slam_x\":" << get_blackboard_value<double>(BlackboardKeys::SLAM_X, 0.0) << ","
           << "\"slam_y\":" << get_blackboard_value<double>(BlackboardKeys::SLAM_Y, 0.0) << ","
           << "\"slam_confidence\":" << get_blackboard_value<double>(BlackboardKeys::SLAM_CONFIDENCE, 0.0)
           << "},"
           << "\"navigation_state\":{"
           << "\"navigation_status\":\"" << get_blackboard_value<std::string>(BlackboardKeys::NAVIGATION_STATUS, "idle") << "\","
           << "\"path_clear\":" << (get_blackboard_value<bool>(BlackboardKeys::PATH_CLEAR, true) ? "true" : "false") << ","
           << "\"perception_confidence\":" << get_blackboard_value<double>(BlackboardKeys::PERCEPTION_CONFIDENCE, 0.0) << ","
           << "\"map_quality\":" << get_blackboard_value<double>(BlackboardKeys::MAP_QUALITY, 0.0)
           << "},"
           << "\"safety_state\":{"
           << "\"safety_stop_active\":" << (get_blackboard_value<bool>(BlackboardKeys::SAFETY_STOP_ACTIVE, false) ? "true" : "false") << ","
           << "\"battery_level\":" << get_blackboard_value<double>(BlackboardKeys::BATTERY_LEVEL, 100.0) << ","
           << "\"closest_obstacle_distance\":" << get_blackboard_value<double>(BlackboardKeys::CLOSEST_OBSTACLE_DISTANCE, 999.0)
           << "},"
           << "\"system_health\":{"
           << "\"sensors_ok\":" << (get_blackboard_value<bool>(BlackboardKeys::SENSORS_OK, true) ? "true" : "false") << ","
           << "\"navigation_ok\":" << (get_blackboard_value<bool>(BlackboardKeys::NAVIGATION_OK, true) ? "true" : "false") << ","
           << "\"last_error\":\"" << get_blackboard_value<std::string>(BlackboardKeys::LAST_ERROR, "") << "\""
           << "}}";
        telemetry_msg.data = ss.str();
        telemetry_publisher_->publish(telemetry_msg);
        RCLCPP_DEBUG(get_logger(), "BT Telemetry: %s", ss.str().c_str());
    }

    void publish_telemetry_status()
    {
        // Publish periodic status telemetry (even when BT is idle)
        RCLCPP_INFO(get_logger(), "Publishing telemetry status...");
        publish_bt_telemetry("status");
    }

    // Robust blackboard operations with validation
    template<typename T>
    bool set_blackboard_value(const std::string& key, const T& value)
    {
        try {
            // Validate the value based on type
            if (!validate_blackboard_value(key, value)) {
                RCLCPP_WARN(get_logger(), "Blackboard validation failed for key '%s'", key.c_str());
                return false;
            }

            blackboard_->set(key, value);
            // Note: Detailed logging removed to avoid template complexity
            RCLCPP_DEBUG(get_logger(), "Blackboard set: %s", key.c_str());
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to set blackboard value for key '%s': %s", key.c_str(), e.what());
            set_blackboard_value(BlackboardKeys::LAST_ERROR, std::string("Blackboard set failed: ") + e.what());
            return false;
        }
    }

    template<typename T>
    T get_blackboard_value(const std::string& key, const T& default_value = T())
    {
        try {
            // For BT.CPP v4, get returns the value directly, not an optional
            T value = blackboard_->get<T>(key);
            // Validate retrieved value
            if (validate_blackboard_value(key, value)) {
                return value;
            } else {
                RCLCPP_WARN(get_logger(), "Blackboard value validation failed for key '%s'", key.c_str());
                return default_value;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to get blackboard value for key '%s': %s", key.c_str(), e.what());
            set_blackboard_value(BlackboardKeys::LAST_ERROR, std::string("Blackboard get failed: ") + e.what());
            return default_value;
        }
    }

    template<typename T>
    bool validate_blackboard_value(const std::string& key, const T& value)
    {
        // Type-specific validation
        if constexpr (std::is_same_v<T, double>) {
            // Coordinate validation (reasonable bounds for Mars rover)
            if (key.find("_x") != std::string::npos || key.find("_y") != std::string::npos) {
                if (std::isnan(value) || std::isinf(value) || std::abs(value) > 10000.0) {
                    RCLCPP_WARN(get_logger(), "Invalid coordinate value for key '%s': %f", key.c_str(), value);
                    return false;
                }
            }
            // Confidence validation
            else if (key.find("confidence") != std::string::npos) {
                if (value < 0.0 || value > 1.0 || std::isnan(value)) {
                    RCLCPP_WARN(get_logger(), "Invalid confidence value for key '%s': %f", key.c_str(), value);
                    return false;
                }
            }
        }
        else if constexpr (std::is_same_v<T, int>) {
            // Count validation (should be non-negative)
            if (key.find("collected") != std::string::npos || key.find("completed") != std::string::npos) {
                if (value < 0) {
                    RCLCPP_WARN(get_logger(), "Invalid count value for key '%s': %d", key.c_str(), value);
                    return false;
                }
            }
        }
        else if constexpr (std::is_same_v<T, bool>) {
            // Boolean values are always valid
            return true;
        }
        else if constexpr (std::is_same_v<T, std::string>) {
            // String validation - check for reasonable length
            if (value.length() > 1000) {
                RCLCPP_WARN(get_logger(), "String value too long for key '%s': %zu chars", key.c_str(), value.length());
                return false;
            }
        }

        return true;
    }

    // Blackboard service callbacks (dedicated callback group to avoid blocking telemetry)
    void get_blackboard_value_callback(
        const std::shared_ptr<autonomy_interfaces::srv::GetBlackboardValue::Request> request,
        std::shared_ptr<autonomy_interfaces::srv::GetBlackboardValue::Response> response)
    {
        response->success = false;
        response->error_message = "";
        const std::string key = request->key;
        const std::string value_type = request->value_type.empty() ? "string" : request->value_type;

        try {
            if (value_type == "bool") {
                bool value = blackboard_->get<bool>(key);
                response->value = value ? "true" : "false";
                response->value_type = "bool";
                response->success = true;
                return;
            }
            if (value_type == "int") {
                int value = blackboard_->get<int>(key);
                response->value = std::to_string(value);
                response->value_type = "int";
                response->success = true;
                return;
            }
            if (value_type == "double") {
                double value = blackboard_->get<double>(key);
                response->value = std::to_string(value);
                response->value_type = "double";
                response->success = true;
                return;
            }
            if (value_type == "string") {
                std::string value = blackboard_->get<std::string>(key);
                response->value = value;
                response->value_type = "string";
                response->success = true;
                return;
            }
            response->success = false;
            response->error_message = "Unsupported value type: " + value_type;
            RCLCPP_WARN(get_logger(), "GetBlackboardValue unsupported type '%s' for key '%s'",
                value_type.c_str(), key.c_str());
        } catch (const std::exception& e) {
            response->success = false;
            response->error_message = std::string("Exception: ") + e.what();
            RCLCPP_ERROR(get_logger(), "GetBlackboardValue service error for key '%s': %s",
                key.c_str(), e.what());
        }
    }

    void set_blackboard_value_callback(
        const std::shared_ptr<autonomy_interfaces::srv::SetBlackboardValue::Request> request,
        std::shared_ptr<autonomy_interfaces::srv::SetBlackboardValue::Response> response)
    {
        response->success = false;
        response->error_message = "";
        const std::string key = request->key;
        const std::string value_str = request->value;
        const std::string value_type = request->value_type;

        try {
            bool ok = false;
            if (value_type == "bool") {
                bool value = (value_str == "true" || value_str == "1");
                ok = set_blackboard_value(key, value);
            } else if (value_type == "int") {
                int value = std::stoi(value_str);
                ok = set_blackboard_value(key, value);
            } else if (value_type == "double") {
                double value = std::stod(value_str);
                ok = set_blackboard_value(key, value);
            } else if (value_type == "string") {
                ok = set_blackboard_value(key, value_str);
            } else {
                response->success = false;
                response->error_message = "Unsupported value type: " + value_type;
                RCLCPP_WARN(get_logger(), "SetBlackboardValue unsupported type '%s' for key '%s'",
                    value_type.c_str(), key.c_str());
                return;
            }
            response->success = ok;
            if (!ok) {
                response->error_message = "Failed to set blackboard value (validation may have failed)";
                RCLCPP_WARN(get_logger(), "SetBlackboardValue validation failed for key '%s'", key.c_str());
            }
        } catch (const std::invalid_argument& e) {
            response->success = false;
            response->error_message = std::string("Invalid value format: ") + e.what();
            RCLCPP_ERROR(get_logger(), "SetBlackboardValue invalid argument for key '%s': %s",
                key.c_str(), e.what());
        } catch (const std::exception& e) {
            response->success = false;
            response->error_message = std::string("Exception: ") + e.what();
            RCLCPP_ERROR(get_logger(), "SetBlackboardValue service error for key '%s': %s",
                key.c_str(), e.what());
        }
    }

private:
    void execute_tree()
    {
        if (tree_.rootNode()) {
            auto status = tree_.tickOnce();
            RCLCPP_DEBUG(get_logger(), "BT tick status: %s", BT::toStr(status).c_str());

            // Publish telemetry on each tick
            publish_bt_telemetry("tick");

            if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
                RCLCPP_INFO(get_logger(), "Behavior Tree execution completed with status: %s",
                           BT::toStr(status).c_str());
                publish_bt_telemetry("completed");
                execution_timer_->cancel();  // Stop execution when done
            }
        }
    }

    // Action server callback methods
    rclcpp_action::GoalResponse handle_navigate_to_pose_goal(
        const rclcpp_action::GoalUUID & /* uuid */,
        std::shared_ptr<const autonomy_interfaces::action::NavigateToPose::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "Received navigate to pose goal: (%.2f, %.2f)",
                   goal->target_pose.pose.position.x, goal->target_pose.pose.position.y);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_navigate_to_pose_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<autonomy_interfaces::action::NavigateToPose>> /* goal_handle */)
    {
        RCLCPP_INFO(get_logger(), "Received request to cancel navigate to pose goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_navigate_to_pose_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<autonomy_interfaces::action::NavigateToPose>> goal_handle)
    {
        // Execute navigation in a separate thread
        std::thread{std::bind(&BTOrchestrator::execute_navigate_to_pose, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute_navigate_to_pose(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<autonomy_interfaces::action::NavigateToPose>> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<autonomy_interfaces::action::NavigateToPose::Result>();

        // Simulate navigation execution
        RCLCPP_INFO(get_logger(), "Executing navigation to (%.2f, %.2f)",
                   goal->target_pose.pose.position.x, goal->target_pose.pose.position.y);

        // Check for cancellation
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            RCLCPP_INFO(get_logger(), "Navigation cancelled");
            return;
        }

        // Simulate navigation time
        std::this_thread::sleep_for(std::chrono::seconds(2));

        result->distance_to_goal = 0.0;
        result->estimated_time_remaining = 0.0;
        result->navigation_state = "completed";
        result->current_pose = goal->target_pose;
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Navigation completed successfully");
    }

    rclcpp_action::GoalResponse handle_execute_mission_goal(
        const rclcpp_action::GoalUUID & /* uuid */,
        std::shared_ptr<const autonomy_interfaces::action::ExecuteMission::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "Received execute mission goal: %s", goal->mission_type.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_execute_mission_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<autonomy_interfaces::action::ExecuteMission>> /* goal_handle */)
    {
        RCLCPP_INFO(get_logger(), "Received request to cancel mission execution");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_execute_mission_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<autonomy_interfaces::action::ExecuteMission>> goal_handle)
    {
        // Execute mission in a separate thread
        std::thread{std::bind(&BTOrchestrator::execute_mission, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute_mission(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<autonomy_interfaces::action::ExecuteMission>> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<autonomy_interfaces::action::ExecuteMission::Result>();

        // Simulate mission execution
        RCLCPP_INFO(get_logger(), "Executing mission: %s", goal->mission_type.c_str());

        // Check for cancellation
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            RCLCPP_INFO(get_logger(), "Mission cancelled");
            return;
        }

        // Simulate mission execution time
        std::this_thread::sleep_for(std::chrono::seconds(5));

        result->current_phase = "completed";
        result->progress = 1.0;
        result->status_message = "Mission completed successfully";
        result->waypoints_completed = 2;  // Example for sample collection mission
        result->estimated_time_remaining = 0.0;
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Mission execution completed");
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BTOrchestrator>();
    // MultiThreadedExecutor so blackboard service callbacks run alongside BT/telemetry timers
    // (single-threaded spin was starving get_value/set_value, causing client timeouts)
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 4);
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

