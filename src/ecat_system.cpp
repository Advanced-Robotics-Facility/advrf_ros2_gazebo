#include <memory>
#include <mutex>
#include <string>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <gz/msgs/double.pb.h>
#include <gz/transport/Node.hh>
#include <gz/transport/TopicUtils.hh>

// https://gazebosim.org/api/sim/8/createsystemplugins.html
// https://gazebosim.org/api/sim/8/usingcomponents.html

namespace advrf_ros2_gazebo {

class EcatSystem : public gz::sim::System,
                      public gz::sim::ISystemConfigure,
                      public gz::sim::ISystemPreUpdate
                     // public gz::sim::ISystemPostUpdate
{
public:
    // Called once when Gazebo attaches this plugin to a model
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

        if (!_sdf->HasElement("joint_name")) {
            std::cerr << "EcatSystem requires <joint_name>." << std::endl;
            return;
        }
        joint_name_ = _sdf->Get<std::string>("joint_name");

        const auto topic = gz::transport::TopicUtils::AsValidTopic(
            "/model/" + model_.Name(_ecm) + "/joint/" + joint_name_ + "/cmd_force");
        if (topic.empty() || !node_.Subscribe(topic, &EcatSystem::OnCmdForce, this)) {
            std::cerr << "Failed to subscribe to joint force topic [" << topic << "]." << std::endl;
            return;
        }
        
        std::cout  << "EcatSystem loaded for model [" << model_.Name(_ecm)
              << "], joint [" << joint_name_
              << "], topic [" << topic << "]." << std::endl;
    }

    void PreUpdate(const gz::sim::UpdateInfo &_info,
                   gz::sim::EntityComponentManager &_ecm) override
    {
        if (_info.paused)
            return;

        if (joint_entity_ == gz::sim::kNullEntity) {
            joint_entity_ = model_.JointByName(_ecm, joint_name_);
            if (joint_entity_ == gz::sim::kNullEntity)
                return;
            std::cout  << "Found joint: " << joint_name_ << std::endl;
        }

        double command;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            command = joint_force_cmd_;
        }
        auto force = _ecm.ComponentDefault<gz::sim::components::JointForceCmd>(joint_entity_, {0.0});
        force->Data()[0] += command;
    }

    // Called after each simulation update
    // void PostUpdate(const gz::sim::UpdateInfo &_info,
    //                 const gz::sim::EntityComponentManager &_ecm) override
    // {
    // }

private:
    void OnCmdForce(const gz::msgs::Double &_msg) {
        std::cout << "Received force command: " << _msg.data() << std::endl;

        std::lock_guard<std::mutex> lock(command_mutex_);
        joint_force_cmd_ = _msg.data();
    }

    gz::sim::Model model_{gz::sim::kNullEntity};
    gz::sim::Entity joint_entity_{gz::sim::kNullEntity};
    gz::transport::Node node_;

    std::string joint_name_;
    double joint_force_cmd_{0.0};

    std::mutex command_mutex_;
};

}  // namespace advrf_ros2_gazebo

// Expose the class and its callbacks to Gazebo's plugin loader
GZ_ADD_PLUGIN(advrf_ros2_gazebo::EcatSystem,
              gz::sim::System,
              advrf_ros2_gazebo::EcatSystem::ISystemConfigure,
              advrf_ros2_gazebo::EcatSystem::ISystemPreUpdate)
              //advrf_ros2_gazebo::EcatSystem::ISystemPostUpdate)
