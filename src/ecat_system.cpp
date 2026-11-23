#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>

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

            const bool subscribed =  node_.Subscribe<gz::msgs::Double>(
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
    }

    void PreUpdate(const gz::sim::UpdateInfo &_info,
                   gz::sim::EntityComponentManager &_ecm) override
    {
        if (_info.paused)
            return;

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
    // void PostUpdate(const gz::sim::UpdateInfo &_info,
    //                 const gz::sim::EntityComponentManager &_ecm) override
    // {
    // }

private:
    void OnCmdForce(const std::string &jointName, const gz::msgs::Double &_msg) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        joint_force_cmd_[jointName] = _msg.data();
        std::cout << "Received force command for [" << jointName << "]: " << _msg.data() << std::endl;
    }

    struct Joints {
        std::string name;
        int32_t id {0};
        double effort_limit {0.0};
        gz::sim::Entity entity{gz::sim::kNullEntity};
    };

    gz::sim::Model model_{gz::sim::kNullEntity};
    gz::transport::Node node_;

    std::vector<Joints> joints_;
    std::unordered_map<std::string, double> joint_force_cmd_;

    std::mutex command_mutex_;
};

}  // namespace advrf_ros2_gazebo

// Expose the class and its callbacks to Gazebo's plugin loader
GZ_ADD_PLUGIN(advrf_ros2_gazebo::EcatSystem,
              gz::sim::System,
              advrf_ros2_gazebo::EcatSystem::ISystemConfigure,
              advrf_ros2_gazebo::EcatSystem::ISystemPreUpdate)
              //advrf_ros2_gazebo::EcatSystem::ISystemPostUpdate)
