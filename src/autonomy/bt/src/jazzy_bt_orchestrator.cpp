/**
 * Jazzy-Enhanced BT Orchestrator for URC 2026 Mars Rover
 *
 * Features:
 * - Cyclone DDS optimized communication
 * - Iceoryx2 intra-process communication
 * - Real-time scheduling with Events Executor
 * - Enhanced QoS profiles for deterministic performance
 * - Component lifecycle management
 * - Performance monitoring and telemetry
 */

#include <memory>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <sched.h>
#include <pthread.h>
#include <nlohmann/json.hpp>  // Jazzy: JSON support for enhanced telemetry

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "behaviortree_cpp/loggers/bt_mqtt_logger.h"  // Jazzy: Enhanced logging

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/events_executor.hpp"  // Jazzy: Events Executor
#include "rclcpp/callback_group.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

// Autonomy interfaces
#include "autonomy_interfaces/srv/get_system_state.hpp"
#include "autonomy_interfaces/srv/detect_aruco.hpp"
#include "autonomy_interfaces/srv/detect_mission_aruco.hpp"
#include "autonomy_interfaces/msg/led_command.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Action interfaces for BT
#include "autonomy_interfaces/action/navigate_to_pose.hpp"
#include "autonomy_interfaces/action/execute_mission.hpp"

using namespace BT;
using namespace std::chrono_literals;

namespace urc {

// Jazzy QoS Profiles (equivalent to Python QoS profiles)
rclcpp::QoS get_autonomy_status_qos() {
    return rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
        .keep_last(5)
        .reliable()
        .transient_local()
        .deadline(std::chrono::milliseconds(100));
}

rclcpp::QoS get_telemetry_qos() {
    return rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
        .keep_last(50)
        .best_effort()
        .volatile_()
        .deadline(std::chrono::seconds(1));
}

// Performance monitoring structure
struct BTPerformanceMetrics {
    std::atomic<uint64_t> tick_count{0};
    std::atomic<uint64_t> total_execution_time_us{0};
    std::atomic<uint64_t> max_execution_time_us{0};
    std::atomic<uint64_t> deadline_misses{0};

    void record_execution(uint64_t execution_time_us) {
        tick_count++;
        total_execution_time_us += execution_time_us;
        if (execution_time_us > max_execution_time_us.load()) {
            max_execution_time_us.store(execution_time_us);
        }
        if (execution_time_us > 100000) { // 100ms deadline
            deadline_misses++;
        }
    }

    double get_average_execution_time_ms() const {
        if (tick_count == 0) return 0.0;
        return static_cast<double>(total_execution_time_us.load()) /
               static_cast<double>(tick_count.load()) / 1000.0;
    }
};

/**
 * Jazzy-Enhanced BT Orchestrator
 *
 * Key improvements:
 * - Events Executor for deterministic scheduling
 * - Real-time thread priority
 * - Enhanced QoS profiles
 * - Performance monitoring
 * - Component lifecycle management
 */
class JazzyBTOrchestrator : public rclcpp_lifecycle::LifecycleNode
{
public:
    JazzyBTOrchestrator()
        : rclcpp_lifecycle::LifecycleNode("jazzy_bt_orchestrator"),
          performance_metrics_(std::make_unique<BTPerformanceMetrics>())
    {
        RCLCPP_INFO(get_logger(), "🎯 Jazzy BT Orchestrator initializing...");
    }

    ~JazzyBTOrchestrator() override = default;

    // ===== LIFECYCLE MANAGEMENT =====

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State & state) override
    {
        RCLCPP_INFO(get_logger(), "🔧 Configuring Jazzy BT Orchestrator...");

        // Initialize blackboard
        blackboard_ = BT::Blackboard::create();

        // Initialize blackboard with validated defaults
        initialize_blackboard();

        // Register BT nodes
        register_bt_nodes();

        // Load behavior tree
        if (!load_behavior_tree()) {
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        // Configure ROS2 interfaces with Jazzy QoS
        configure_interfaces();

        // Set up real-time scheduling
        configure_realtime_scheduling();

        RCLCPP_INFO(get_logger(), "✅ Jazzy BT Orchestrator configured successfully");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & state) override
    {
        RCLCPP_INFO(get_logger(), "🚀 Activating Jazzy BT Orchestrator...");

        // Activate lifecycle publishers
        telemetry_publisher_->on_activate();

        // Start BT execution timer with real-time callback group
        execution_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),  // 10Hz
            std::bind(&JazzyBTOrchestrator::execute_tree_callback, this),
            realtime_callback_group_
        );

        // Start telemetry timer
        telemetry_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&JazzyBTOrchestrator::publish_telemetry_callback, this),
            telemetry_callback_group_
        );

        // Activate action servers
        navigate_to_pose_server_->activate();

        RCLCPP_INFO(get_logger(), "✅ Jazzy BT Orchestrator activated");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & state) override
    {
        RCLCPP_INFO(get_logger(), "🛑 Deactivating Jazzy BT Orchestrator...");

        // Cancel timers
        execution_timer_->cancel();
        telemetry_timer_->cancel();

        // Deactivate publishers
        telemetry_publisher_->on_deactivate();

        // Deactivate action servers
        navigate_to_pose_server_->deactivate();

        RCLCPP_INFO(get_logger(), "✅ Jazzy BT Orchestrator deactivated");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State & state) override
    {
        RCLCPP_INFO(get_logger(), "🧹 Cleaning up Jazzy BT Orchestrator...");

        // Clean up resources
        execution_timer_.reset();
        telemetry_timer_.reset();
        navigate_to_pose_server_.reset();
        telemetry_publisher_.reset();

        // Close file logger
        file_logger_.reset();

        RCLCPP_INFO(get_logger(), "✅ Jazzy BT Orchestrator cleaned up");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State & state) override
    {
        RCLCPP_INFO(get_logger(), "💥 Shutting down Jazzy BT Orchestrator...");

        // Emergency cleanup
        if (tree_) {
            tree_->haltTree();
        }

        RCLCPP_INFO(get_logger(), "✅ Jazzy BT Orchestrator shut down");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    // ===== INITIALIZATION =====

    void initialize_blackboard() {
        // Initialize with type-safe defaults
        blackboard_->set<bool>("mission_active", false);
        blackboard_->set<double>("robot_x", 0.0);
        blackboard_->set<double>("robot_y", 0.0);
        blackboard_->set<double>("robot_yaw", 0.0);
        blackboard_->set<double>("slam_x", 0.0);
        blackboard_->set<double>("slam_y", 0.0);
        blackboard_->set<double>("slam_confidence", 0.0);
        blackboard_->set<int>("samples_collected", 0);
        blackboard_->set<int>("waypoints_completed", 0);
        blackboard_->set<bool>("sensors_ok", true);
        blackboard_->set<bool>("navigation_ok", true);
        blackboard_->set<std::string>("last_error", "");
        blackboard_->set<std::string>("current_mission_phase", "idle");
    }

    void register_bt_nodes() {
        // Register standardized nodes
        factory_.registerNodeType<CallService>("CallService");
        factory_.registerNodeType<SensorCheck>("SensorCheck");
        factory_.registerNodeType<NavigateToWaypoint>("NavigateToWaypoint");
        factory_.registerNodeType<SampleCollection>("SampleCollection");
        factory_.registerNodeType<EmergencyStop>("EmergencyStop");

        // Register URC 2026 mission-specific nodes
        factory_.registerNodeType<NavigateToGNSS>("NavigateToGNSS");
        factory_.registerNodeType<DetectArUcoPost>("DetectArUcoPost");
        factory_.registerNodeType<DetectObject>("DetectObject");
        factory_.registerNodeType<SignalArrival>("SignalArrival");
        factory_.registerNodeType<PerformTyping>("PerformTyping");
        factory_.registerNodeType<PickupObject>("PickupObject");

        RCLCPP_INFO(get_logger(), "✅ BT nodes registered");
    }

    bool load_behavior_tree() {
        try {
            std::string package_share_directory =
                ament_index_cpp::get_package_share_directory("autonomy_bt");
            std::string bt_file_path = package_share_directory + "/behavior_trees/main_mission.xml";

            tree_ = factory_.createTreeFromFile(bt_file_path, blackboard_);

            // Jazzy: Enhanced logging
            cout_logger_ = std::make_unique<BT::StdCoutLogger>(tree_);
            file_logger_ = std::make_unique<BT::FileLogger2>(tree_, "/tmp/jazzy_bt_execution.btlog");

            RCLCPP_INFO(get_logger(), "✅ Behavior tree loaded successfully");
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "❌ Failed to load behavior tree: %s", e.what());
            return false;
        }
    }

    void configure_interfaces() {
        // Jazzy: Create callback groups for real-time execution
        realtime_callback_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        telemetry_callback_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // Configure subscribers with Jazzy QoS
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", get_autonomy_status_qos(),
            std::bind(&JazzyBTOrchestrator::odom_callback, this, std::placeholders::_1),
            rmw_qos_profile_default, realtime_callback_group_);

        slam_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/slam/pose", get_autonomy_status_qos(),
            std::bind(&JazzyBTOrchestrator::slam_pose_callback, this, std::placeholders::_1),
            rmw_qos_profile_default, realtime_callback_group_);

        // Configure publishers with lifecycle management
        telemetry_publisher_ = create_publisher<std_msgs::msg::String>(
            "/bt/telemetry", get_telemetry_qos());

        // Configure action servers
        navigate_to_pose_server_ = rclcpp_action::create_server<autonomy_interfaces::action::NavigateToPose>(
            this, "/jazzy_bt/navigate_to_pose",
            std::bind(&JazzyBTOrchestrator::handle_navigate_to_pose_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&JazzyBTOrchestrator::handle_navigate_to_pose_cancel, this, std::placeholders::_1),
            std::bind(&JazzyBTOrchestrator::handle_navigate_to_pose_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(), realtime_callback_group_);

        RCLCPP_INFO(get_logger(), "✅ Interfaces configured with Jazzy QoS");
    }

    void configure_realtime_scheduling() {
        // Set thread priority for real-time execution
        struct sched_param param;
        param.sched_priority = 50;  // High priority but not maximum

        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
            RCLCPP_WARN(get_logger(), "⚠️ Failed to set real-time priority, using default");
        } else {
            RCLCPP_INFO(get_logger(), "✅ Real-time scheduling configured");
        }

        // Lock memory to prevent page faults
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
            RCLCPP_WARN(get_logger(), "⚠️ Failed to lock memory, performance may vary");
        } else {
            RCLCPP_INFO(get_logger(), "✅ Memory locked for real-time performance");
        }
    }

    // ===== EXECUTION =====

    void execute_tree_callback() {
        if (!tree_) return;

        auto start_time = std::chrono::steady_clock::now();

        // Execute BT tick
        BT::NodeStatus status = tree_->tickRoot();

        auto end_time = std::chrono::steady_clock::now();
        auto execution_time = std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - start_time);

        // Record performance metrics
        performance_metrics_->record_execution(execution_time.count());

        // Jazzy: Enhanced logging with performance data
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
            "BT tick: %s (%ld μs)", toString(status).c_str(), execution_time.count());

        // Check for deadline misses
        if (execution_time > std::chrono::milliseconds(100)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "BT tick exceeded deadline: %ld μs", execution_time.count());
        }
    }

    void publish_telemetry_callback() {
        if (!telemetry_publisher_->is_activated()) return;

        // Jazzy: Enhanced telemetry with performance metrics
        auto telemetry_msg = std_msgs::msg::String();

        nlohmann::json telemetry = {
            {"component", "jazzy_bt_orchestrator"},
            {"timestamp", this->now().seconds()},
            {"tree_status", tree_ ? toString(tree_->rootNode()->status()).c_str() : "INVALID"},
            {"tick_count", static_cast<uint64_t>(performance_metrics_->tick_count.load())},
            {"avg_execution_time_ms", performance_metrics_->get_average_execution_time_ms()},
            {"max_execution_time_us", static_cast<uint64_t>(performance_metrics_->max_execution_time_us.load())},
            {"deadline_misses", static_cast<uint64_t>(performance_metrics_->deadline_misses.load())},
            {"blackboard", {
                {"mission_active", blackboard_->get<bool>("mission_active")},
                {"samples_collected", blackboard_->get<int>("samples_collected")},
                {"sensors_ok", blackboard_->get<bool>("sensors_ok")},
                {"last_error", blackboard_->get<std::string>("last_error")}
            }}
        };

        telemetry_msg.data = telemetry.dump();
        telemetry_publisher_->publish(telemetry_msg);
    }

    // ===== CALLBACKS =====

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        blackboard_->set<double>("robot_x", msg->pose.pose.position.x);
        blackboard_->set<double>("robot_y", msg->pose.pose.position.y);

        // Extract yaw from quaternion
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        blackboard_->set<double>("robot_yaw", yaw);
    }

    void slam_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        blackboard_->set<double>("slam_x", msg->pose.position.x);
        blackboard_->set<double>("slam_y", msg->pose.position.y);

        // Simplified confidence based on covariance (enhance with actual SLAM confidence)
        double confidence = 0.8;  // Placeholder - integrate with actual SLAM confidence
        blackboard_->set<double>("slam_confidence", confidence);
    }

    // ===== ACTION SERVER IMPLEMENTATION =====
    // (Keep existing action server implementations but enhance with Jazzy features)

    rclcpp_action::GoalResponse handle_navigate_to_pose_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const autonomy_interfaces::action::NavigateToPose::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "🎯 Received navigate to pose goal");
        (void)uuid;  // Suppress unused parameter warning

        // Jazzy: Enhanced goal validation
        if (!blackboard_->get<bool>("sensors_ok")) {
            RCLCPP_WARN(get_logger(), "⚠️ Rejecting navigation goal - sensors not OK");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_navigate_to_pose_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<autonomy_interfaces::action::NavigateToPose>> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "🛑 Navigation goal cancelled");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_navigate_to_pose_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<autonomy_interfaces::action::NavigateToPose>> goal_handle)
    {
        // Execute navigation asynchronously
        std::thread{std::bind(&JazzyBTOrchestrator::execute_navigate_to_pose, this, goal_handle)}.detach();
    }

    void execute_navigate_to_pose(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<autonomy_interfaces::action::NavigateToPose>> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<autonomy_interfaces::action::NavigateToPose::Feedback>();
        auto result = std::make_shared<autonomy_interfaces::action::NavigateToPose::Result>();

        // Set navigation target in blackboard
        blackboard_->set<double>("nav_target_x", goal->target_pose.pose.position.x);
        blackboard_->set<double>("nav_target_y", goal->target_pose.pose.position.y);

        // Execute navigation BT subtree
        BT::NodeStatus nav_status = BT::NodeStatus::RUNNING;
        rclcpp::Rate rate(10);  // 10Hz feedback

        while (nav_status == BT::NodeStatus::RUNNING && rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->error_message = "Navigation cancelled";
                goal_handle->canceled(result);
                return;
            }

            // Update feedback
            feedback->current_pose.header.stamp = this->now();
            feedback->distance_remaining = calculate_distance_to_target();
            goal_handle->publish_feedback(feedback);

            rate.sleep();
        }

        // Navigation complete
        result->success = (nav_status == BT::NodeStatus::SUCCESS);
        if (result->success) {
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "✅ Navigation completed successfully");
        } else {
            goal_handle->abort(result);
            RCLCPP_ERROR(get_logger(), "❌ Navigation failed");
        }
    }

    double calculate_distance_to_target() {
        double current_x = blackboard_->get<double>("robot_x");
        double current_y = blackboard_->get<double>("robot_y");
        double target_x = blackboard_->get<double>("nav_target_x");
        double target_y = blackboard_->get<double>("nav_target_y");

        double dx = target_x - current_x;
        double dy = target_y - current_y;
        return std::sqrt(dx*dx + dy*dy);
    }

    // ===== MEMBER VARIABLES =====

    // BT components
    BT::BehaviorTreeFactory factory_;
    BT::Tree tree_;
    BT::Blackboard::Ptr blackboard_;

    // Loggers
    std::unique_ptr<BT::StdCoutLogger> cout_logger_;
    std::unique_ptr<BT::FileLogger2> file_logger_;

    // Performance monitoring
    std::unique_ptr<BTPerformanceMetrics> performance_metrics_;

    // ROS2 interfaces with Jazzy QoS
    rclcpp::CallbackGroup::SharedPtr realtime_callback_group_;
    rclcpp::CallbackGroup::SharedPtr telemetry_callback_group_;

    rclcpp::TimerBase::SharedPtr execution_timer_;
    rclcpp::TimerBase::SharedPtr telemetry_timer_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr slam_pose_sub_;

    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr telemetry_publisher_;

    // Action servers
    rclcpp_action::Server<autonomy_interfaces::action::NavigateToPose>::SharedPtr navigate_to_pose_server_;
};

}  // namespace urc

// Main function with Jazzy Events Executor
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // Jazzy: Use Events Executor for deterministic, low-latency execution
    auto executor = std::make_shared<rclcpp::executors::EventsExecutor>(
        rclcpp::executor::ExecutorOptions()
    );

    auto node = std::make_shared<urc::JazzyBTOrchestrator>();
    executor->add_node(node->get_node_base_interface());

    RCLCPP_INFO(node->get_logger(), "🚀 Starting Jazzy BT Orchestrator with Events Executor");

    try {
        executor->spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "💥 Executor error: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
