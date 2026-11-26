#include <memory>
#include <mutex>
#include <rclcpp/executors.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <gz/msgs/double.pb.h>
#include <gz/transport/Node.hh>
#include <gz/transport/TopicUtils.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "advrf_ros2_interfaces/msg/motor_tx_pdo.hpp"

// https://gazebosim.org/api/sim/8/createsystemplugins.html
// https://gazebosim.org/api/sim/8/usingcomponents.html

using MotorCommand = advrf_ros2_interfaces::msg::MotorTxPdo;
using JointState = sensor_msgs::msg::JointState;

namespace advrf_ros2_gazebo {

class EcatSystem : public gz::sim::System,
                   public gz::sim::ISystemConfigure,
                   public gz::sim::ISystemPreUpdate,
                   public gz::sim::ISystemPostUpdate
{
public:
    // Called once when Gazebo attaches this plugin to a model
    // Parse SDF, Find Joints, Create ROS2 pub/sub
    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &/*_eventMgr*/) override
    {
        std::cout  << "Configure" << std::endl;

        model_ = gz::sim::Model(_entity);
        if (!model_.Valid(_ecm)) {
            std::cerr << "ADVRF Gazebo backend must be attached to a model" << std::endl;
            return;
        }
        std::cout  << "EcatSystem loaded for model [" << model_.Name(_ecm) << "]" << std::endl;

        if (!_sdf->HasElement("joint")) {
            std::cerr << "EcatSystem requires <joint>." << std::endl;
            return;
        }

        sdf::ElementPtr sdfCopy = _sdf->Clone();
        auto jointElem = sdfCopy->GetElement("joint");

        while (jointElem) {
            if (!jointElem->HasElement("name")) {
                std::cerr << "Each <joint> require a field <name> in xacro file" << std::endl;
                return;
            }

            Joints joint;
            joint.name = jointElem->Get<std::string>("name");
            joint.id = jointElem->Get<int32_t>("id");
            joint.effort_limit = jointElem->Get<double>("effort_limit");
            joint.entity = model_.JointByName(_ecm, joint.name);
            if (joint.entity == gz::sim::kNullEntity) 
                return;
            std::cout  << "Found joint: " << joint.name << std::endl;

            const auto topic = gz::transport::TopicUtils::AsValidTopic("/model/" + model_.Name(_ecm) + "/joint/" + joint.name  + "/cmd_force");
            if (topic.empty()) {
                std::cerr << "Failed to subscribe to joint force topic [" << topic << "]." << std::endl;
                return;
            }

            const bool subscribed =  gz_node_.Subscribe<gz::msgs::Double>(
                topic, 
                [this, jointName = joint.name](const gz::msgs::Double &msg)
                {
                    this->OnCmdForce(jointName, msg);
                }
            );
            if (!subscribed) {
                std::cerr << "Failed to subscribe to [" << topic << "]." << std::endl;
                return;
            }
            
            std::cout  << "EcatSystem loaded for joint [" << joint.name 
                << "], topic [" << topic << "]." << std::endl;
            
            joints_.push_back(std::move(joint));
            jointElem = jointElem->GetNextElement("joint");
        }

        if (!rclcpp::ok()) {
            int argc = 0;
            char **argv = nullptr;
            rclcpp::init(argc, argv);
        }
        ros_node_ = std::make_shared<rclcpp::Node>("gazebo_ecat_system", "/advrf/miranda/ecat");
        // cmd_sub_ = ros_node_->create_subscription<MotorCommand>(
        //     "tx/motors",
        //     rclcpp::QoS(1).reliable(),
        //     [this](MotorCommand::ConstSharedPtr msg) 
        //     {
        //         std::lock_guard<std::mutex> lock(cmd_mutex_);
        //         latest_cmd_ = *msg;
        //         cmd_recv_ = true;
        //     }
        // );
        state_pub_ = ros_node_->create_publisher<JointState>(
            "rx/joint_state", 
            rclcpp::SensorDataQoS()
        );
    }

    // Called before each simulation update
    // Process ROS2 cmds, read latest measured state, compute & write cmd
    void PreUpdate(const gz::sim::UpdateInfo &_info,
                   gz::sim::EntityComponentManager &_ecm) override
    {
        if (_info.paused)
            return;

        rclcpp::spin_some(ros_node_);

        std::lock_guard<std::mutex> lock(command_mutex_);

        for (const auto &joint : joints_) {
            const auto it = joint_force_cmd_.find(joint.name);
            double command;
            if (it != joint_force_cmd_.end()) {
                command = std::clamp(it->second, -joint.effort_limit, joint.effort_limit);
            }
            else {
                command = 0.0;
            }
        
            auto *force = _ecm.ComponentDefault<gz::sim::components::JointForceCmd>(joint.entity, std::vector<double>{0.0});
            force->Data()[0] = command;
        }
    }

    // Called after each simulation update
    // Read new data, publish jointstate
    void PostUpdate(const gz::sim::UpdateInfo &_info,
                    const gz::sim::EntityComponentManager &_ecm) override
    {
    }

private:
    void OnCmdForce(const std::string &jointName, const gz::msgs::Double &_msg) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        joint_force_cmd_[jointName] = _msg.data();
        std::cout << "Received force command for [" << jointName << "]: " << _msg.data() << std::endl;
    }

    struct Joints {
        std::string name;
        int32_t id;
        gz::sim::Entity entity{gz::sim::kNullEntity};

        double pos{0.0};
        double vel{0.0};
        double eff{0.0};

        double pos_ref{0.0};
        double vel_ref{0.0};
        double tor_ref{0.0};

        double stiffness{0.0};
        double damping{0.0};
        double kp{0.0};
        double ki{0.0};
        double kd{0.0};

        double effort_limit{0.0};
    };

    gz::sim::Model model_{gz::sim::kNullEntity};
    gz::transport::Node gz_node_;

    std::vector<Joints> joints_;
    std::unordered_map<std::string, double> joint_force_cmd_;

    std::mutex command_mutex_;

    rclcpp::Node::SharedPtr ros_node_;
    //rclcpp::Subscription<MotorCommand>::SharedPtr cmd_sub_;
    rclcpp::Publisher<JointState>::SharedPtr state_pub_;
    // MotorCommand latest_cmd_;
    // bool cmd_recv_{false};
};

}  // namespace advrf_ros2_gazebo

// Expose the class and its callbacks to Gazebo's plugin loader
GZ_ADD_PLUGIN(advrf_ros2_gazebo::EcatSystem,
              gz::sim::System,
              advrf_ros2_gazebo::EcatSystem::ISystemConfigure,
              advrf_ros2_gazebo::EcatSystem::ISystemPreUpdate,
              advrf_ros2_gazebo::EcatSystem::ISystemPostUpdate)
