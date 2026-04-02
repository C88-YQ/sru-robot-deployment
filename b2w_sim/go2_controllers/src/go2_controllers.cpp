#include "go2_controllers/go2_controllers.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include <rclcpp/executors/multi_threaded_executor.hpp>

namespace {
constexpr std::chrono::milliseconds kInferencePeriod{20};
constexpr std::chrono::milliseconds kPublishPeriod{5};
}

template <typename T>
T Go2Controllers::getOrDeclare(const std::string &name, const T &default_value) {
    rclcpp::Parameter param;
    if (this->get_parameter(name, param)) {
        return param.get_parameter_value().get<T>();
    }
    return this->declare_parameter<T>(name, default_value);
}

Go2Controllers::Go2Controllers(const rclcpp::NodeOptions &options)
    : Node("go2_controllers", options),
      env_(ORT_LOGGING_LEVEL_WARNING, "go2_onnx_policy"),
      allocator_(),
      session_(nullptr),
      joint_positions_(kNumJoints),
      joint_velocities_(kNumJoints),
      default_joint_positions_(kNumJoints),
      last_actions_(kNumJoints),
      scaled_actions_(kNumJoints),
      rotation_matrix_(Eigen::Matrix3d::Identity()),
      odometry_received_(false),
      odometry_warned_(false) {
    const bool use_sim_time = getOrDeclare<bool>("use_sim_time", true);
    this->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));

    loadParameters();
    resolvePolicyPath();
    initializeOnnxSession();

    if (environment_config_.default_joint_positions.size() != kNumJoints) {
        throw std::runtime_error("Environment configuration default_joint_positions size mismatch");
    }

    default_joint_positions_ = Eigen::Map<Eigen::VectorXd>(
        environment_config_.default_joint_positions.data(),
        environment_config_.default_joint_positions.size());

    base_ang_vel_ = geometry_msgs::msg::Vector3();
    cmd_vel_ = geometry_msgs::msg::Twist();
    projected_gravity_ = Eigen::Vector3d(0.0, 0.0, -1.0);

    joint_positions_ = Eigen::VectorXd::Zero(kNumJoints);
    joint_velocities_ = Eigen::VectorXd::Zero(kNumJoints);
    last_actions_ = Eigen::VectorXd::Zero(kNumJoints);
    scaled_actions_ = default_joint_positions_;
    input_buffer_.fill(0.0f);

    initializeJointNamesAndTopics();
    setupSubscribers();
    setupPublishers();
    setupTimers();

    RCLCPP_INFO(
        this->get_logger(),
        "Go2 controller ready (environment profile: %s, policy: %s)",
        environment_profile_.c_str(),
        policy_path_.string().c_str());
}

void Go2Controllers::loadParameters() {
    const std::vector<std::string> default_joint_names = {
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};

    const std::vector<std::string> default_joint_topics = {
        "/FR_hip_joint_position_cmd", "/FR_thigh_joint_position_cmd", "/FR_calf_joint_position_cmd",
        "/FL_hip_joint_position_cmd", "/FL_thigh_joint_position_cmd", "/FL_calf_joint_position_cmd",
        "/RR_hip_joint_position_cmd", "/RR_thigh_joint_position_cmd", "/RR_calf_joint_position_cmd",
        "/RL_hip_joint_position_cmd", "/RL_thigh_joint_position_cmd", "/RL_calf_joint_position_cmd"};

    const std::vector<int64_t> default_reorder = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
    const std::vector<double> default_joint_positions = {
        0.0, 0.8, -1.5,
        0.0, 0.8, -1.5,
        0.0, 0.8, -1.5,
        0.0, 0.8, -1.5};
    const std::vector<double> default_action_scales = {
        0.125, 0.25, 0.25,
        0.125, 0.25, 0.25,
        0.125, 0.25, 0.25,
        0.125, 0.25, 0.25};

    policy_package_ = getOrDeclare<std::string>("policy.package", "go2_controllers");
    policy_relative_path_ = getOrDeclare<std::string>("policy.relative_path", "policy/policy.onnx");
    policy_override_path_ = getOrDeclare<std::string>("policy.path", "");

    odometry_topic_ = getOrDeclare<std::string>("topics.odometry", "/dlio/odom_node/odom");
    velocity_topic_ = getOrDeclare<std::string>("topics.cmd_vel", "/path_manager/path_manager_ros/nav_vel");
    joint_state_topic_ = getOrDeclare<std::string>("topics.joint_state", "/joint_states");

    joint_names_ = getOrDeclare<std::vector<std::string>>("joint_model.names", default_joint_names);
    joint_topics_ = getOrDeclare<std::vector<std::string>>("joint_model.command_topics", default_joint_topics);
    action_scales_ = getOrDeclare<std::vector<double>>("joint_model.action_scales", default_action_scales);

    auto reorder_param = getOrDeclare<std::vector<int64_t>>("joint_model.reorder_indices", default_reorder);
    auto default_positions_param =
        getOrDeclare<std::vector<double>>("joint_model.default_positions", default_joint_positions);

    if (joint_names_.size() != kNumJoints || joint_topics_.size() != kNumJoints ||
        action_scales_.size() != kNumJoints || reorder_param.size() != kNumJoints ||
        default_positions_param.size() != kNumJoints) {
        throw std::runtime_error("Go2 joint configuration parameter length mismatch");
    }

    reorder_indices_.assign(reorder_param.begin(), reorder_param.end());

    environment_profile_ = getOrDeclare<std::string>("environment.profile", "default");

    std::vector<double> base_position_default =
        getOrDeclare<std::vector<double>>("environment.base_position", std::vector<double>{0.0, 0.0, 0.0});
    std::vector<double> base_orientation_default =
        getOrDeclare<std::vector<double>>("environment.base_orientation_rpy", std::vector<double>{0.0, 0.0, 0.0});

    if (base_position_default.size() != 3 || base_orientation_default.size() != 3) {
        throw std::runtime_error("Environment base pose parameters must have exactly three elements");
    }

    std::vector<double> joint_position_override = default_positions_param;
    const std::string env_prefix = "environment.profiles." + environment_profile_;

    if (this->has_parameter(env_prefix + ".default_joint_positions")) {
        auto override = this->get_parameter(env_prefix + ".default_joint_positions").as_double_array();
        if (override.size() != kNumJoints) {
            throw std::runtime_error("Environment profile default_joint_positions size mismatch");
        }
        joint_position_override.assign(override.begin(), override.end());
    }

    std::vector<double> base_position_override = base_position_default;
    if (this->has_parameter(env_prefix + ".base_position")) {
        auto override = this->get_parameter(env_prefix + ".base_position").as_double_array();
        if (override.size() != 3) {
            throw std::runtime_error("Environment profile base_position size must be 3");
        }
        base_position_override.assign(override.begin(), override.end());
    }

    std::vector<double> base_orientation_override = base_orientation_default;
    if (this->has_parameter(env_prefix + ".base_orientation_rpy")) {
        auto override = this->get_parameter(env_prefix + ".base_orientation_rpy").as_double_array();
        if (override.size() != 3) {
            throw std::runtime_error("Environment profile base_orientation_rpy size must be 3");
        }
        base_orientation_override.assign(override.begin(), override.end());
    }

    environment_config_.default_joint_positions = joint_position_override;
    environment_config_.base_position_xyz = base_position_override;
    environment_config_.base_orientation_rpy = base_orientation_override;

    base_ang_vel_scale_ = getOrDeclare<double>("observation_scales.base_ang_vel", 0.25);
    joint_pos_scale_ = getOrDeclare<double>("observation_scales.joint_pos", 1.0);
    joint_vel_scale_ = getOrDeclare<double>("observation_scales.joint_vel", 0.05);
    command_scale_ = getOrDeclare<double>("observation_scales.velocity_commands", 1.0);
    action_observation_scale_ = getOrDeclare<double>("observation_scales.actions", 1.0);

    publish_joint_array_ = getOrDeclare<bool>("joint_model.publish_joint_array", true);
}

void Go2Controllers::resolvePolicyPath() {
    if (!policy_override_path_.empty()) {
        policy_path_ = std::filesystem::path(policy_override_path_);
    } else {
        std::string package_share;
        try {
            package_share = ament_index_cpp::get_package_share_directory(policy_package_);
        } catch (const std::exception &e) {
            std::ostringstream oss;
            oss << "Unable to find package '" << policy_package_ << "' for policy lookup: " << e.what();
            throw std::runtime_error(oss.str());
        }
        policy_path_ = std::filesystem::path(package_share) / policy_relative_path_;
    }

    if (!std::filesystem::exists(policy_path_)) {
        std::ostringstream oss;
        oss << "Policy file not found: " << policy_path_.string();
        throw std::runtime_error(oss.str());
    }
}

void Go2Controllers::initializeOnnxSession() {
    try {
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(1);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session_ = std::make_unique<Ort::Session>(env_, policy_path_.c_str(), session_options);
    } catch (const Ort::Exception &e) {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize ONNX session: %s", e.what());
        throw;
    }

    const size_t input_count = session_->GetInputCount();
    const size_t output_count = session_->GetOutputCount();
    if (input_count < 1 || output_count < 1) {
        throw std::runtime_error("ONNX model must have at least one input and one output");
    }

    input_node_name_storage_.clear();
    output_node_name_storage_.clear();
    input_node_names_.clear();
    output_node_names_.clear();

    for (size_t i = 0; i < input_count; ++i) {
        auto name = session_->GetInputNameAllocated(i, allocator_);
        input_node_name_storage_.emplace_back(name.get());
    }
    for (size_t i = 0; i < output_count; ++i) {
        auto name = session_->GetOutputNameAllocated(i, allocator_);
        output_node_name_storage_.emplace_back(name.get());
    }
    for (const auto &name : input_node_name_storage_) {
        input_node_names_.push_back(name.c_str());
    }
    for (const auto &name : output_node_name_storage_) {
        output_node_names_.push_back(name.c_str());
    }

    const auto input_shape = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    const auto output_shape = session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (input_shape.empty() || output_shape.empty()) {
        throw std::runtime_error("ONNX model input/output shape must not be empty");
    }

    const int64_t input_dim = input_shape.back();
    const int64_t output_dim = output_shape.back();
    if (input_dim != kInputSize) {
        std::ostringstream oss;
        oss << "Expected ONNX input last dimension " << kInputSize << ", got " << input_dim;
        throw std::runtime_error(oss.str());
    }
    if (output_dim != kNumJoints) {
        std::ostringstream oss;
        oss << "Expected ONNX output last dimension " << kNumJoints << ", got " << output_dim;
        throw std::runtime_error(oss.str());
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Loaded ONNX policy from %s (input=%s, output=%s)",
        policy_path_.string().c_str(),
        input_node_name_storage_.front().c_str(),
        output_node_name_storage_.front().c_str());
}

void Go2Controllers::initializeJointNamesAndTopics() {
    if (joint_names_.size() != joint_topics_.size()) {
        throw std::runtime_error("joint_names and joint_topics size mismatch");
    }

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        joint_pubs_[joint_names_[i]] =
            this->create_publisher<std_msgs::msg::Float64>(joint_topics_[i], rclcpp::QoS(10));
        joint_commands_[joint_names_[i]] = environment_config_.default_joint_positions[i];
    }
    joint_command_buffer_ = environment_config_.default_joint_positions;
}

void Go2Controllers::setupSubscribers() {
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic_, rclcpp::QoS(10),
        std::bind(&Go2Controllers::odometryCallback, this, std::placeholders::_1));

    velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        velocity_topic_, rclcpp::QoS(10),
        std::bind(&Go2Controllers::velocityCallback, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        joint_state_topic_, rclcpp::QoS(50),
        std::bind(&Go2Controllers::jointStateCallback, this, std::placeholders::_1));
}

void Go2Controllers::setupPublishers() {
    action_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_commands", 10);
    network_input_debug_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/network_input_debug", 10);
}

void Go2Controllers::setupTimers() {
    timer_ = this->create_wall_timer(kInferencePeriod, std::bind(&Go2Controllers::inference, this));
    timer_pub_ = this->create_wall_timer(kPublishPeriod, std::bind(&Go2Controllers::publishJointCommands, this));
}

void Go2Controllers::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(odometry_mutex_);
        base_ang_vel_ = msg->twist.twist.angular;

        Eigen::Quaterniond quat(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);
        rotation_matrix_ = quat.toRotationMatrix();
    }

    if (!odometry_received_) {
        odometry_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Odometry data received, starting inference.");
    }

    processOdometry();
}

void Go2Controllers::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(odometry_mutex_);
    cmd_vel_ = *msg;
}

void Go2Controllers::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.size() < kNumJoints || msg->velocity.size() < kNumJoints) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Received joint state with insufficient data");
        return;
    }

    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    for (int i = 0; i < kNumJoints; ++i) {
        const int src_index = reorder_indices_[i];
        if (src_index < 0 || src_index >= static_cast<int>(msg->position.size()) ||
            src_index >= static_cast<int>(msg->velocity.size())) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "Reorder index %d out of range", src_index);
            return;
        }
        joint_positions_[i] = msg->position[src_index];
        joint_velocities_[i] = msg->velocity[src_index];
    }
}

void Go2Controllers::processOdometry() {
    if (!odometry_received_) {
        return;
    }

    const Eigen::Vector3d gravity(0.0, 0.0, -1.0);
    std::lock_guard<std::mutex> lock(odometry_mutex_);
    projected_gravity_ = rotation_matrix_.transpose() * gravity;
}

void Go2Controllers::inference() {
    if (!odometry_received_) {
        if (!odometry_warned_) {
            RCLCPP_WARN(this->get_logger(), "No odometry data received yet.");
            odometry_warned_ = true;
        }
        return;
    }

    geometry_msgs::msg::Vector3 base_ang_vel_local;
    Eigen::Vector3d projected_gravity_local;
    geometry_msgs::msg::Twist cmd_vel_local;
    {
        std::lock_guard<std::mutex> lock(odometry_mutex_);
        base_ang_vel_local = base_ang_vel_;
        projected_gravity_local = projected_gravity_;
        cmd_vel_local = cmd_vel_;
    }

    Eigen::VectorXd joint_positions_local;
    Eigen::VectorXd joint_velocities_local;
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        joint_positions_local = joint_positions_;
        joint_velocities_local = joint_velocities_;
    }

    input_buffer_[0] = static_cast<float>(base_ang_vel_local.x * base_ang_vel_scale_);
    input_buffer_[1] = static_cast<float>(base_ang_vel_local.y * base_ang_vel_scale_);
    input_buffer_[2] = static_cast<float>(base_ang_vel_local.z * base_ang_vel_scale_);

    input_buffer_[3] = static_cast<float>(projected_gravity_local.x());
    input_buffer_[4] = static_cast<float>(projected_gravity_local.y());
    input_buffer_[5] = static_cast<float>(projected_gravity_local.z());

    input_buffer_[6] = static_cast<float>(cmd_vel_local.linear.x * command_scale_);
    input_buffer_[7] = static_cast<float>(cmd_vel_local.linear.y * command_scale_);
    input_buffer_[8] = static_cast<float>(cmd_vel_local.angular.z * command_scale_);

    for (Eigen::Index i = 0; i < joint_positions_local.size(); ++i) {
        input_buffer_[9 + i] =
            static_cast<float>((joint_positions_local[i] - default_joint_positions_[i]) * joint_pos_scale_);
        input_buffer_[21 + i] = static_cast<float>(joint_velocities_local[i] * joint_vel_scale_);
        input_buffer_[33 + i] = static_cast<float>(last_actions_[i] * action_observation_scale_);
    }

    std::array<int64_t, 2> obs_shape = {1, kInputSize};
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    auto obs_tensor = Ort::Value::CreateTensor<float>(
        memory_info, input_buffer_.data(), input_buffer_.size(), obs_shape.data(), obs_shape.size());

    std::vector<Ort::Value> input_tensors;
    input_tensors.push_back(std::move(obs_tensor));

    try {
        auto output_tensors = session_->Run(
            Ort::RunOptions{nullptr},
            input_node_names_.data(),
            input_tensors.data(),
            1,
            output_node_names_.data(),
            1);

        float *actions_data = output_tensors.front().GetTensorMutableData<float>();
        Eigen::Map<Eigen::VectorXf> actions_map(actions_data, kNumJoints);
        last_actions_ = actions_map.cast<double>();
    } catch (const Ort::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "ONNX Runtime error: %s", e.what());
        return;
    }

    processActions();
    publishDebugData();
}

void Go2Controllers::processActions() {
    scaled_actions_ = last_actions_;
    for (int i = 0; i < kNumJoints; ++i) {
        scaled_actions_[i] = scaled_actions_[i] * action_scales_[i] + default_joint_positions_[i];
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        joint_commands_[joint_names_[i]] = scaled_actions_[i];
        joint_command_buffer_[i] = scaled_actions_[i];
    }
}

void Go2Controllers::publishDebugData() {
    if (network_input_debug_pub_->get_subscription_count() == 0 &&
        network_input_debug_pub_->get_intra_process_subscription_count() == 0) {
        return;
    }

    std_msgs::msg::Float64MultiArray debug_msg;
    debug_msg.data.reserve(input_buffer_.size());
    for (const float value : input_buffer_) {
        debug_msg.data.push_back(static_cast<double>(value));
    }
    network_input_debug_pub_->publish(debug_msg);
}

void Go2Controllers::publishJointCommands() {
    std::vector<double> joint_snapshot;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        joint_snapshot = joint_command_buffer_;
    }

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        std_msgs::msg::Float64 msg;
        msg.data = joint_snapshot[i];
        joint_pubs_[joint_names_[i]]->publish(msg);
    }

    if (publish_joint_array_ &&
        (action_pub_->get_subscription_count() > 0 ||
         action_pub_->get_intra_process_subscription_count() > 0)) {
        std_msgs::msg::Float64MultiArray aggregate_msg;
        aggregate_msg.data.assign(joint_snapshot.begin(), joint_snapshot.end());
        action_pub_->publish(aggregate_msg);
    }
}

int main(int argc, char **argv) {
    try {
        rclcpp::init(argc, argv);
        auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
        auto node = std::make_shared<Go2Controllers>(options);

        rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
        exec.add_node(node);
        exec.spin();
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
}
