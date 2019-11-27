/**
 * @author Fetullah Atas
 * @email fetulahatas1@gmail.com
 * @create date 2019-11-27 16:25:50
 * @modify date 2019-11-27 16:25:50
 * @desc [description]
 */
#include <random_pointcloud_data_gen/RandomPCLGenerator.h>

#include <ros/ros.h>
#include <sstream>
#include <iomanip>
#include <iostream>
/**
 * @brief 
 * Main Node resposible for the control actions of robot, including both control from GUI and Automatic
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "generator_node");
    ros::NodeHandle node_handle;

    RandomPCLGenerator random_pcl_data_gen_;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok())
    {
        int node_loop_rate;
        node_handle.param<int>("execution_frequency", node_loop_rate, 100);
        ros::Rate loop_rate(node_loop_rate);

        loop_rate.sleep();

        ros::spinOnce();
    }
    return 0;
}
