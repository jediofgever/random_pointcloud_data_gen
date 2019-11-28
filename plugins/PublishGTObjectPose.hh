/**
 * @author Fetullah Atas
 * @email fetulahatas1@gmail.com
 * @create date 2019-11-27 16:25:11
 * @modify date 2019-11-27 16:25:11
 * @desc [description]
 */
#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math3/ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <Eigen/Core>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

namespace gazebo
{

class PublishGTObjectPose : public WorldPlugin
{

    // Pointer to the model
private:
    physics::WorldPtr world;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    bool flag = true;

    ros::Publisher objects_tobe_picked_pub_;
    ros::NodeHandle *nh_;
    ros::Publisher gtBBX_pub_;
    tf::TransformListener *listener_;

public:
    PublishGTObjectPose();
    ~PublishGTObjectPose();
    void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/);
    void OnUpdate();
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(PublishGTObjectPose);
} // namespace gazebo