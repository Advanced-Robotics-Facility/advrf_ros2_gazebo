#include <memory>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

namespace gazebo {

class JointFeedback : public gz::sim::System,
                      public gz::sim::ISystemConfigure,
                      public gz::sim::ISystemPostUpdate
{
public:
    // Called once when Gazebo attaches this plugin to a model
    void Configure(const gz::sim::Entity& entity,
                   const std::shared_ptr<const sdf::Element>& sdf,
                   gz::sim::EntityComponentManager& ecm,
                   gz::sim::EventManager&) override
    {
        const gz::sim::Model model(entity);
        if (!model.Valid(ecm)) {
            gzerr << "JointFeedback must be attached to a model.\n";
            return;
        }
        gzmsg << "JointFeedback loaded for model: " << model.Name(ecm) << std::endl;

        if (!sdf->HasElement("joint_name")) {
            gzerr << "JointFeedback requires <joint_name>." << std::endl;
            return;
        }
        const auto joint_name = sdf->Get<std::string>("joint_name");
        joint_entity_ = model.JointByName(ecm, joint_name);
        if (joint_entity_ == gz::sim::kNullEntity) {
            gzerr << "Joint not found: " << joint_name << std::endl;
            return;
        }
        gzmsg << "Found joint: " << joint_name << std::endl;
    }

    // Called after each simulation update
    void PostUpdate(const gz::sim::UpdateInfo&,
                    const gz::sim::EntityComponentManager&) override
    {
    }

private:
    gz::sim::Entity joint_entity_{gz::sim::kNullEntity};
};

}  // namespace gazebo

// Expose the class and its callbacks to Gazebo's plugin loader
GZ_ADD_PLUGIN(gazebo::JointFeedback,
              gz::sim::System,
              gazebo::JointFeedback::ISystemConfigure,
              gazebo::JointFeedback::ISystemPostUpdate)
