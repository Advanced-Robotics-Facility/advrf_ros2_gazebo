#include <advrf_ros2_interfaces/msg/motor_tx_pdo.hpp>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <unordered_set>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace advrf_ros2_gazebo
{
class EcatSystem final : public gz::sim::System,
                         public gz::sim::ISystemConfigure,
                         public gz::sim::ISystemPreUpdate,
                         public gz::sim::ISystemPostUpdate
{
public:
  ~EcatSystem() override
  {
    if (executor_) executor_->cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
  }

  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(entity);
    if (!model_.Valid(ecm))
        throw std::runtime_error("ADVRF Gazebo backend must be attached to a model");

    // Params defined in .xacro file
    namespace_ = sdf->Get<std::string>("ros_namespace", "/advrf/miranda/ecat").first;
    command_timeout_ = std::chrono::milliseconds(sdf->Get<unsigned int>("command_timeout_ms", 100U).first);
    feedback_period_ = std::chrono::milliseconds(sdf->Get<unsigned int>("feedback_period_ms", 4U).first);
    auto_activate_ = sdf->Get<bool>("active_on_first_command", false).first;
    startup_hold_ = sdf->Get<bool>("hold_initial_pose", false).first;

    // Parse .xacro joints definition/params and save into Joint datastruct
    std::unordered_set<std::string> names;
    auto joint = sdf->FindElement("joint");
    while (joint) {
      Joint item;
      item.name = joint->Get<std::string>("name");
      item.id = joint->Get<int32_t>("id");
      item.stiffness = joint->Get<double>("stiffness");
      item.damping = joint->Get<double>("damping");
      item.hold_when_uncommanded = joint->Get<bool>("hold_when_uncommanded", false).first;
      item.effort_limit = joint->Get<double>("effort_limit", 0.0).first;
      if (!std::isfinite(item.stiffness) || item.stiffness < 0 ||
          !std::isfinite(item.damping) || item.damping < 0 ||
          !std::isfinite(item.effort_limit) || item.effort_limit < 0)
        throw std::runtime_error("Invalid startup gains or effort limit for Gazebo joint: " + item.name);
      item.entity = model_.JointByName(ecm, item.name);
      if (item.entity == gz::sim::kNullEntity)
        throw std::runtime_error("Gazebo model is missing joint: " + item.name);
      if (!names.insert(item.name).second || by_id_.count(item.id))
        throw std::runtime_error("Duplicate joint name or EtherCAT ID in Gazebo plugin");
      by_id_[item.id] = joints_.size();
      joints_.push_back(std::move(item));
      joint = joint->GetNextElement("joint");
    }
    if (joints_.empty()) throw std::runtime_error("ADVRF Gazebo backend has no configured joints");

    // Initialize actuator components
    for (const auto &item : joints_) {
      ecm.CreateComponent(item.entity, gz::sim::components::JointPosition());
      ecm.CreateComponent(item.entity, gz::sim::components::JointVelocity());
      ecm.CreateComponent(item.entity, gz::sim::components::JointForceCmd({0.0}));
    }

    // ROS2 Init
    if (!rclcpp::ok()) {
      int argc = 0;
      char **argv = nullptr;
      rclcpp::init(argc, argv);
    }

    // ROS2 Node
    node_ = std::make_shared<rclcpp::Node>("gazebo_ecat_backend", namespace_);

    // ROS2 Pub/Sub
    command_sub_ = node_->create_subscription<advrf_ros2_interfaces::msg::MotorTxPdo>(
      "tx/motors",
      rclcpp::QoS(1).best_effort(),
      [this](advrf_ros2_interfaces::msg::MotorTxPdo::ConstSharedPtr msg)
      {
        command(*msg);
      }
    );
    joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      "rx/joint_states",
      rclcpp::QoS(5).reliable()
    );
    active_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "active",
      rclcpp::QoS(1).reliable().transient_local()
    );
    active_service_ = node_->create_service<std_srvs::srv::SetBool>(
      "set_active_tx",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
             std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto_activate_ = false;
        have_command_ = false;
        for (auto &joint : joints_)
          joint.hold_position.reset();
        if (!request->data) startup_hold_ = false;
        active_.store(request->data);
        publish_active();
        response->success = true;
        response->message = request->data ? "Gazebo motor commands enabled" : "Gazebo motor commands disabled";
      }
    );
    publish_active();

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
    RCLCPP_INFO(node_->get_logger(), "Gazebo EtherCAT backend ready for %zu joints", joints_.size());
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override
  {
    if (info.paused)
        return;

    std::lock_guard<std::mutex> lock(mutex_);

    // simulation-reset safety guard
    if (info.dt < decltype(info.dt)::zero()) {
      have_command_ = false;
      for (auto &joint : joints_)
        joint.hold_position.reset();
    }

    // Watchdog
    const bool drive_enabled = active_.load() && have_command_ &&
      (std::chrono::steady_clock::now() - last_command_ <= command_timeout_);

    for (auto &joint : joints_) {
      if (!startup_hold_ && !drive_enabled)
        joint.hold_position.reset();

      // Read simulated joint state from Gazebo
      const auto position = scalar<gz::sim::components::JointPosition>(ecm, joint.entity);
      const auto velocity = scalar<gz::sim::components::JointVelocity>(ecm, joint.entity);
      double effort = 0.0;
      if (position && velocity) {
        if (startup_hold_ || (drive_enabled && !joint.commanded && joint.hold_when_uncommanded)) {
          if (!joint.hold_position)
            joint.hold_position = *position;
          effort = joint.stiffness * (*joint.hold_position - *position) - joint.damping * *velocity;
        } else if (drive_enabled && joint.commanded) {
          effort = joint.torque + joint.kp * (joint.position - *position) + joint.kd * (joint.velocity - *velocity);
        }
        const double clamped = std::clamp(effort, -joint.effort_limit, joint.effort_limit);
        if (effort != clamped) {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), diagnostic_clock_, 2000,
            "Effort limit reached for %s: requested %.3f, commanded %.3f",
            joint.name.c_str(), effort, clamped);
        }
        effort = clamped;
      }
      joint.applied_effort = effort;

      // Send torque command to Gazebo's physics solver
      auto component = ecm.Component<gz::sim::components::JointForceCmd>(joint.entity);
      if (component)
        component->Data()[0] = effort;
      else
        ecm.CreateComponent(joint.entity, gz::sim::components::JointForceCmd({effort}));
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override
  {
    if (info.dt < decltype(info.dt)::zero())
        next_feedback_time_ = info.simTime;

    if (info.paused || !joint_pub_ || info.simTime < next_feedback_time_)
        return;
    next_feedback_time_ = info.simTime + feedback_period_;

    sensor_msgs::msg::JointState state;
    const auto nanoseconds = info.simTime.count();
    state.header.stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    state.header.stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
    state.name.reserve(joints_.size());
    state.position.reserve(joints_.size());
    state.velocity.reserve(joints_.size());
    state.effort.reserve(joints_.size());
    for (const auto &joint : joints_) {
      const auto position = scalar<gz::sim::components::JointPosition>(ecm, joint.entity);
      const auto velocity = scalar<gz::sim::components::JointVelocity>(ecm, joint.entity);
      if (!position || !velocity) return;
      state.name.push_back(joint.name);
      state.position.push_back(*position);
      state.velocity.push_back(*velocity);
      state.effort.push_back(joint.applied_effort);
    }
    joint_pub_->publish(state);
  }

private:
  struct Joint {
    std::string name;
    int32_t id{0};
    gz::sim::Entity entity{gz::sim::kNullEntity};
    double effort_limit{0.0};
    double position{0.0};
    double velocity{0.0};
    double torque{0.0};

    double stiffness{0.0};
    double damping{0.0};
    bool hold_when_uncommanded{false};
    double kp{0.0};
    double kd{0.0};

    bool commanded{false};
    double applied_effort{0.0};
    std::optional<double> hold_position;
  };

  template<class Component>
  static std::optional<double> scalar(const gz::sim::EntityComponentManager &ecm,
                                      gz::sim::Entity entity)
  {
    const auto component = ecm.Component<Component>(entity);
    if (!component || component->Data().empty() || !std::isfinite(component->Data()[0]))
      return std::nullopt;
    return component->Data()[0];
  }

  void command(const advrf_ros2_interfaces::msg::MotorTxPdo &msg)
  {
    const auto size = msg.ecat_id.size();
    if (size == 0 || msg.pos_ref.size() != size || msg.vel_ref.size() != size ||
        msg.tor_ref.size() != size || msg.gain_0.size() != size || msg.gain_1.size() != size)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Rejected malformed MotorTxPdo");
      return;
    }

    std::unordered_set<int32_t> seen;
    for (std::size_t i = 0; i < size; ++i) {
      if (!by_id_.count(msg.ecat_id[i]) || !seen.insert(msg.ecat_id[i]).second ||
          !std::isfinite(msg.pos_ref[i]) || !std::isfinite(msg.vel_ref[i]) ||
          !std::isfinite(msg.tor_ref[i]) || !std::isfinite(msg.gain_0[i]) ||
          !std::isfinite(msg.gain_1[i]) || msg.gain_0[i] < 0 || msg.gain_1[i] < 0)
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Rejected invalid motor reference, gain or EtherCAT ID");
        return;
      }
    }

    // Lock so PreUpdate cannot read half-updated targets mid loop
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_.load() && !auto_activate_)
        return;
    startup_hold_ = false;

    const auto received = std::chrono::steady_clock::now();
    const bool command_expired = have_command_ && received - last_command_ > command_timeout_;
    for (auto &joint : joints_) {
      joint.commanded = false;
      if (command_expired)
        joint.hold_position.reset();
    }

    for (std::size_t i = 0; i < size; ++i) {
      auto &joint = joints_[by_id_.at(msg.ecat_id[i])];
      joint.position = msg.pos_ref[i];
      joint.velocity = msg.vel_ref[i];
      joint.torque = msg.tor_ref[i];
      joint.kp = msg.gain_0[i];
      joint.kd = msg.gain_1[i];
      joint.commanded = true;
      joint.hold_position.reset();
    }

    if (!active_.exchange(true))
        publish_active();
    last_command_ = received;
    have_command_ = true;
  }

  void publish_active()
  {
    if (!active_pub_) return;
    std_msgs::msg::Bool msg;
    msg.data = active_.load();
    active_pub_->publish(msg);
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  std::string namespace_;
  std::vector<Joint> joints_;
  std::unordered_map<int32_t, std::size_t> by_id_;
  std::mutex mutex_;
  rclcpp::Clock diagnostic_clock_{RCL_STEADY_TIME};
  std::chrono::steady_clock::time_point last_command_;
  std::chrono::milliseconds command_timeout_{100};
  std::chrono::milliseconds feedback_period_{4};
  std::chrono::steady_clock::duration next_feedback_time_{0};
  bool have_command_{false};
  std::atomic_bool active_{false};
  bool auto_activate_{false};
  bool startup_hold_{false};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<advrf_ros2_interfaces::msg::MotorTxPdo>::SharedPtr command_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr active_service_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
};
}

GZ_ADD_PLUGIN(advrf_ros2_gazebo::EcatSystem,
              gz::sim::System,
              advrf_ros2_gazebo::EcatSystem::ISystemConfigure,
              advrf_ros2_gazebo::EcatSystem::ISystemPreUpdate,
              advrf_ros2_gazebo::EcatSystem::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(advrf_ros2_gazebo::EcatSystem,
                    "advrf_ros2_gazebo::EcatSystem")
