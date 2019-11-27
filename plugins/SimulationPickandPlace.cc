/**
 * @author Fetullah Atas
 * @email fetulahatas1@gmail.com
 * @create date 2019-11-27 16:26:02
 * @modify date 2019-11-27 16:26:02
 * @desc [description]
 */
#include "SimulationPickandPlace.hh"

using namespace gazebo;
SimulationPickandPlace::SimulationPickandPlace()
{

    nh_ = new ros::NodeHandle();
    listener_ = new tf::TransformListener();
    objects_tobe_picked_pub_ = nh_->advertise<geometry_msgs::PoseArray>("/objects_tobe_picked", 10);
    gtBBX_pub_ = nh_->advertise<jsk_recognition_msgs::BoundingBoxArray>("/gt_labels", 10);
}

SimulationPickandPlace::~SimulationPickandPlace()
{
}

void SimulationPickandPlace::Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
{
    // Store the pointer to the model
    this->world = _parent;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SimulationPickandPlace::OnUpdate, this));
}

void SimulationPickandPlace::OnUpdate()
{
    geometry_msgs::PoseArray objects_tobe_picked_array;

    physics::Model_V objects_tobe_picked;

    physics::Model_V models_vector = world->Models();
    for (int i = 0; i < models_vector.size(); i++)
    {
        std::string current_model_name = models_vector[i]->GetName();
        physics::ModelPtr current_model = models_vector[i];

        if (current_model_name.substr(0, 6) == "pulley")
        {
            objects_tobe_picked.push_back(current_model);
        }
    }
    jsk_recognition_msgs::BoundingBoxArray gt_box_array;
    gt_box_array.header.frame_id = "camera_link";
    gt_box_array.header.stamp = ros::Time::now();
    ROS_INFO("objects_tobe_picked.size() %d", objects_tobe_picked.size());
    for (int o = 0; o < objects_tobe_picked.size(); o++)
    {
        physics::ModelPtr current_object_tobe_picked = objects_tobe_picked[o];

        // publish ground truth 3d boxes for dataset creation
        ignition::math::Box box = current_object_tobe_picked->BoundingBox();

        ignition::math::Quaterniond rot(current_object_tobe_picked->WorldPose().Rot());
        ignition::math::Vector3d center;
        center = box.Center();

        geometry_msgs::Pose pose_in_world_frame;
        center = current_object_tobe_picked->WorldPose().Pos();
        pose_in_world_frame.position.x = center[0];
        pose_in_world_frame.position.y = center[1];
        pose_in_world_frame.position.z = center[2];
        pose_in_world_frame.orientation.x = rot.X();
        pose_in_world_frame.orientation.y = rot.Y();
        pose_in_world_frame.orientation.z = rot.Z();
        pose_in_world_frame.orientation.w = rot.W();

        tf::Transform pose_in_world_frame_tf;
        tf::poseMsgToTF(pose_in_world_frame, pose_in_world_frame_tf);

        tf::StampedTransform world_to_camera_link_transform;
        // lookup transform (this should be cached, since it’s probably static)
        try
        {
            listener_->lookupTransform("camera_link", "world", ros::Time(0.0f), world_to_camera_link_transform);
        }
        catch (tf::TransformException ex)
        {
            //ROS_ERROR("%s", ex.what());
            return;
            ros::Duration(1.0).sleep();
        }

        tf::Transform pose_in_camera_frame_tf;
        pose_in_camera_frame_tf = world_to_camera_link_transform * pose_in_world_frame_tf;

        geometry_msgs::Pose pose_in_camera_frame;
        tf::poseTFToMsg(pose_in_camera_frame_tf, pose_in_camera_frame);

        jsk_recognition_msgs::BoundingBox jsk_box_msg;

        jsk_box_msg.header.frame_id = "camera_link";
        jsk_box_msg.header.stamp = ros::Time::now();
        jsk_box_msg.label = o;
        jsk_box_msg.pose.position.x = pose_in_camera_frame.position.x;
        jsk_box_msg.pose.position.y = pose_in_camera_frame.position.y;
        jsk_box_msg.pose.position.z = pose_in_camera_frame.position.z;

        jsk_box_msg.pose.orientation = pose_in_camera_frame.orientation;

        jsk_box_msg.dimensions.x = 0.1120;
        jsk_box_msg.dimensions.y = 0.112;
        jsk_box_msg.dimensions.z = 0.05;
        gt_box_array.boxes.push_back(jsk_box_msg);
    }
    gtBBX_pub_.publish(gt_box_array);
    objects_tobe_picked_pub_.publish(objects_tobe_picked_array);
}
