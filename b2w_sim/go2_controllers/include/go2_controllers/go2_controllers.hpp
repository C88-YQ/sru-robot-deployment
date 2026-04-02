#ifndef GO2_CONTROLLERS_HPP
#define GO2_CONTROLLERS_HPP

#include <array>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <onnxruntime_cxx_api.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class Go2Controllers : public rclcpp::Node {
public:
    explicit Go2Controllers(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    static constexpr int kNumJoints = 12;
    static constexpr int kInputSize = 45;

    struct EnvironmentConfig {
        std::vector<double> default_joint_positions;
        std::vector<double> base_position_xyz;
        std::vector<double> base_orientation_rpy;
    };

    template <typename T>
    T getOrDeclare(const std::string &name, const T &default_value);

    void loadParameters();
    void resolvePolicyPath();
    void initializeOnnxSession();
    void initializeJointNamesAndTopics();
    void setupSubscribers();
    void setupPublishers();
    void setupTimers();

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void processOdometry();

    void inference();
    void processActions();
    void publishDebugData();
    void publishJointCommands();

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr action_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr network_input_debug_pub_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> joint_pubs_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_pub_;

    std::vector<std::string> joint_names_;
    std::vector<std::string> joint_topics_;
    std::vector<int> reorder_indices_;
    std::vector<double> action_scales_;

    std::map<std::string, double> joint_commands_;
    std::vector<double> joint_command_buffer_;

    Ort::Env env_;
    Ort::AllocatorWithDefaultOptions allocator_;
    std::unique_ptr<Ort::Session> session_;
    std::vector<std::string> input_node_name_storage_;
    std::vector<std::string> output_node_name_storage_;
    std::vector<const char *> input_node_names_;
    std::vector<const char *> output_node_names_;

    geometry_msgs::msg::Vector3 base_ang_vel_;
    Eigen::Vector3d projected_gravity_;
    geometry_msgs::msg::Twist cmd_vel_;

    Eigen::VectorXd joint_positions_;
    Eigen::VectorXd joint_velocities_;
    Eigen::VectorXd default_joint_positions_;
    Eigen::VectorXd last_actions_;
    Eigen::VectorXd scaled_actions_;

    Eigen::Matrix3d rotation_matrix_;
    bool odometry_received_;
    bool odometry_warned_;
    std::mutex odometry_mutex_;
    std::mutex joint_state_mutex_;
    std::mutex command_mutex_;

    std::array<float, kInputSize> input_buffer_;

    std::string policy_package_;
    std::string policy_relative_path_;
    std::string policy_override_path_;
    std::filesystem::path policy_path_;

    std::string odometry_topic_;
    std::string velocity_topic_;
    std::string joint_state_topic_;

    double base_ang_vel_scale_;
    double joint_pos_scale_;
    double joint_vel_scale_;
    double command_scale_;
    double action_observation_scale_;

    std::string environment_profile_;
    EnvironmentConfig environment_config_;

    bool publish_joint_array_;
};

#endif
