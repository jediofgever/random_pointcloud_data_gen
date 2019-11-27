/**
 * @author Fetullah Atas
 * @email fetulahatas1@gmail.com
 * @create date 2019-11-27 16:25:40
 * @modify date 2019-11-27 16:25:40
 * @desc [description]
 */
#include <random_pointcloud_data_gen/RandomPCLGenerator.h>

RandomPCLGenerator::RandomPCLGenerator(/* args */)
{
    nh_ = new ros::NodeHandle();

    pcl_sub.subscribe(*nh_, "/camera/depth/points", 1);
    lbl_sub.subscribe(*nh_, "/gt_labels", 1);
    labeled_cloud_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("labeled_cloud", 1);

    //register the callback
    syncseg.reset(new SyncSeg(SyncPolicyforPCLSegmentation(50), pcl_sub, lbl_sub));
    syncseg->registerCallback(boost::bind(&RandomPCLGenerator::generateDataset, this, _1, _2));
    corrected_cloud_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("corrected_pointcloud", 1);
    base_path_to_artifical_dataset = "/home/atas/artifical_pcl_dataset/";
    ROS_INFO("Random generator INitilazed");
}

RandomPCLGenerator::~RandomPCLGenerator()
{
    delete nh_;
}

sensor_msgs::PointCloud2 *RandomPCLGenerator::recorrectPointcloud(sensor_msgs::PointCloud2 *msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

    pcl::PointCloud<pcl::PointXYZ> temp_cloud_in_ros_frames;
    pcl::PCLPointCloud2 pcl2in_ros_frames;

    for (int i = 0; i < temp_cloud->points.size(); i++)
    {

        pcl::PointXYZ point_to_ROS_frames;
        point_to_ROS_frames.x = temp_cloud->points.at(i).z;
        point_to_ROS_frames.y = -temp_cloud->points.at(i).x;
        point_to_ROS_frames.z = -temp_cloud->points.at(i).y;
        temp_cloud_in_ros_frames.points.push_back(point_to_ROS_frames);
    }

    pcl::toPCLPointCloud2(temp_cloud_in_ros_frames, pcl2in_ros_frames);
    pcl_conversions::fromPCL(pcl2in_ros_frames, corrected_cloud);
    corrected_cloud.header.frame_id = "camera_link";
    corrected_cloud.header.stamp = ros::Time::now();

    return &corrected_cloud;
}

void RandomPCLGenerator::generateDataset(const sensor_msgs::PointCloud2ConstPtr &pcl, const jsk_recognition_msgs::BoundingBoxArrayConstPtr &lbl)
{

    ROS_INFO("insode callback");
    jsk_recognition_msgs::BoundingBoxArray box_msg_ = *lbl;

    // prepare and publish RGB colored Lidar scan
    sensor_msgs::PointCloud2 cloud_msg;
    //

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*pcl, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromPCLPointCloud2(pcl_pc2, *in_cloud);

    pcl::toROSMsg(*in_cloud, cloud_msg);

    addGaussiaNoisetoCloud(&cloud_msg, &cloud_msg, 0.001);

    cloud_msg.header = pcl->header;
    sensor_msgs::PointCloud2 *corrected_cloud_msg = recorrectPointcloud(&cloud_msg);

    writeDataset(corrected_cloud_msg, *lbl, 10000);

    corrected_cloud_pub_.publish(*corrected_cloud_msg);
}

void RandomPCLGenerator::addGaussiaNoisetoCloud(sensor_msgs::PointCloud2 *input, sensor_msgs::PointCloud2 *output,
                                                double standard_deviation)
{
    TicToc tt;
    tt.tic();
    PointCloud<PointXYZRGB>::Ptr xyz_cloud(new pcl::PointCloud<PointXYZRGB>());
    PointCloud<PointXYZRGB>::Ptr output_cloud_pcl(new pcl::PointCloud<PointXYZRGB>());
    fromROSMsg(*input, *xyz_cloud);
    PointCloud<PointXYZRGB>::Ptr xyz_cloud_filtered(new PointCloud<PointXYZRGB>());
    xyz_cloud_filtered->points.resize(xyz_cloud->points.size());
    xyz_cloud_filtered->header = xyz_cloud->header;
    xyz_cloud_filtered->width = xyz_cloud->width;
    xyz_cloud_filtered->height = xyz_cloud->height;
    boost::mt19937 rng;
    rng.seed(static_cast<unsigned int>(time(0)));
    boost::normal_distribution<> nd(0, standard_deviation);
    boost::variate_generator<boost::mt19937 &, boost::normal_distribution<>> var_nor(rng, nd);
    for (size_t point_i = 0; point_i < xyz_cloud->points.size(); ++point_i)
    {
        xyz_cloud_filtered->points[point_i].x = xyz_cloud->points[point_i].x + static_cast<float>(var_nor());
        xyz_cloud_filtered->points[point_i].y = xyz_cloud->points[point_i].y + static_cast<float>(var_nor());
        xyz_cloud_filtered->points[point_i].z = xyz_cloud->points[point_i].z + static_cast<float>(var_nor());
        xyz_cloud_filtered->points[point_i].r = xyz_cloud->points[point_i].r;
        xyz_cloud_filtered->points[point_i].g = xyz_cloud->points[point_i].g;
        xyz_cloud_filtered->points[point_i].b = xyz_cloud->points[point_i].b;
    }
    sensor_msgs::PointCloud2 input_xyz_filtered;
    concatenateFields(*xyz_cloud, *xyz_cloud_filtered, *output_cloud_pcl);
    toROSMsg(*output_cloud_pcl, *output);
}

void RandomPCLGenerator::saveCloudasPLY(const std::string &filename, const pcl::PCLPointCloud2 &cloud, bool binary, bool use_camera)
{
    TicToc tt;
    tt.tic();
    pcl::PLYWriter writer;
    writer.write(filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), binary, use_camera);
}

void RandomPCLGenerator::writeDataset(sensor_msgs::PointCloud2 *pcl, jsk_recognition_msgs::BoundingBoxArray box_msg, int num_samples)
{
    // if specified number of samples achieved DO not write anymore .ply files
    if (counter > num_samples)
    {
        return;
    }

    pcl::PCLPointCloud2 pcl_type_pc2;
    pcl_conversions::toPCL(*pcl, pcl_type_pc2);
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr temp_cloud_to_recieve_raw_cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);

    pcl::fromPCLPointCloud2(pcl_type_pc2, *temp_cloud_to_recieve_raw_cloud);

    pcl::PointCloud<pcl::PointXYZRGBL> temp_cloud_for_artificial_dataset;
    pcl::PCLPointCloud2::Ptr pcl_type_pc2_for_artificial_dataset(new pcl::PCLPointCloud2());

    //Find Points of each ground truth object and lable the points for semantic segmenation of point cloud
    for (int j = 0; j < box_msg.boxes.size(); j++)
    {
        jsk_recognition_msgs::BoundingBox current_gt_box = box_msg.boxes[j];
        double bound_x_min, bound_x_max, bound_y_min, bound_y_max, bound_z_min, bound_z_max;

        // define the boundries of each ground truth object
        bound_x_min = current_gt_box.pose.position.x - (current_gt_box.dimensions.z / 2);
        bound_x_max = current_gt_box.pose.position.x + (current_gt_box.dimensions.z / 2);
        bound_y_min = current_gt_box.pose.position.y - (current_gt_box.dimensions.y / 2);
        bound_y_max = current_gt_box.pose.position.y + (current_gt_box.dimensions.y / 2);
        bound_z_min = current_gt_box.pose.position.z - (current_gt_box.dimensions.x / 2);
        bound_z_max = current_gt_box.pose.position.z + (current_gt_box.dimensions.x / 2);

        // walk through each point and see which of them  are in the boundries of this object(current_gt_box)
        for (int i = 0; i < temp_cloud_to_recieve_raw_cloud->points.size(); i++)
        {
            // get xyz but initially lablel their colors as blank(0)
            pcl::PointXYZRGBL point_for_artificial_dataset;
            point_for_artificial_dataset.x = temp_cloud_to_recieve_raw_cloud->points.at(i).x;
            point_for_artificial_dataset.y = temp_cloud_to_recieve_raw_cloud->points.at(i).y;
            point_for_artificial_dataset.z = temp_cloud_to_recieve_raw_cloud->points.at(i).z;
            point_for_artificial_dataset.r = 0;
            point_for_artificial_dataset.g = 0;
            point_for_artificial_dataset.b = 0;

            // see if the bound is in defined object box
            if (point_for_artificial_dataset.x > bound_x_min && point_for_artificial_dataset.x < bound_x_max)
            {
                if (point_for_artificial_dataset.y > bound_y_min && point_for_artificial_dataset.y < bound_y_max)
                {
                    if (point_for_artificial_dataset.z > bound_z_min && point_for_artificial_dataset.z < bound_z_max)
                    {
                        // this point is in the defined object bounding box, so label this point
                        point_for_artificial_dataset.r = 255;
                        point_for_artificial_dataset.g = 255;
                        point_for_artificial_dataset.b = 255;
                    }
                }
            }
            //add all points to cloud
            temp_cloud_for_artificial_dataset.points.push_back(point_for_artificial_dataset);
        }
    }

    pcl::toPCLPointCloud2(temp_cloud_for_artificial_dataset, *pcl_type_pc2_for_artificial_dataset);

    downsamplePCL(pcl_type_pc2_for_artificial_dataset, pcl_type_pc2_for_artificial_dataset);
    //Downsample the cloud because pointnet accepts 4096 points
    pcl::PointCloud<pcl::_PointXYZRGBL>::Ptr cloud_after_downsample(new pcl::PointCloud<pcl::_PointXYZRGBL>);
    pcl::fromPCLPointCloud2(*pcl_type_pc2_for_artificial_dataset, *cloud_after_downsample);

    // convert labeled cloud to ros type and publish for visualiztion
    sensor_msgs::PointCloud2 labeled_cloud_msg;
    labeled_cloud_msg.header = pcl->header;

    // convert it to ROS message type
    pcl::toROSMsg(*cloud_after_downsample, labeled_cloud_msg);
    labeled_cloud_msg.header.frame_id = "camera_link";
    labeled_cloud_msg.header.stamp = ros::Time::now();

    //publish labeled cloud
    labeled_cloud_pub_.publish(labeled_cloud_msg);
    ROS_INFO("NUMBER OF POINTS %d", cloud_after_downsample->points.size());
    if (cloud_after_downsample->points.size() > 4096)
    {

        std::string this_ply_file_path(base_path_to_artifical_dataset + std::to_string(counter) + ".ply");

        // if after the sownsample the number of points are graeter than 4096 , remove from the behind
        while (cloud_after_downsample->points.size() > 4096)
        {
            cloud_after_downsample->points.pop_back();
        }

        cloud_after_downsample->width = 1;
        cloud_after_downsample->height = cloud_after_downsample->points.size();

        pcl::toPCLPointCloud2(*cloud_after_downsample, *pcl_type_pc2_for_artificial_dataset);

        //save this frame to disk for POINTNET TRAINING
        saveCloudasPLY(this_ply_file_path, *pcl_type_pc2_for_artificial_dataset, true, false);

        ROS_INFO("Wrote %d Frames to %s ", counter, base_path_to_artifical_dataset.c_str());

        // increase counter
        counter++;
    }
}

void RandomPCLGenerator::downsamplePCL(pcl::PCLPointCloud2::Ptr cloud, pcl::PCLPointCloud2::Ptr cloud_filtered)
{
    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.004f, 0.004f, 0.004f);
    sor.filter(*cloud_filtered);
}