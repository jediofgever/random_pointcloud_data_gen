/**
 * @author Fetullah Atas
 * @email fetulahatas1@gmail.com
 * @create date 2019-11-27 16:26:46
 * @modify date 2019-11-27 16:26:46
 * @desc [description]
 */
#include <vector>
#include </usr/include/eigen3/Eigen/Geometry>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <sensor_msgs/Image.h>
#include <jsoncpp/json/json.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <sensor_msgs/PointCloud2.h>

//for Gaussian noise to pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/thread/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>
#include <boost/date_time/gregorian/gregorian_types.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/thread/thread.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <algorithm>

using namespace message_filters;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

class RandomPCLGenerator
{
private:
    int counter = 0;
    std::string base_path_to_artifical_dataset;
    ros::NodeHandle *nh_;

    ros::Publisher labeled_cloud_pub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub;
    message_filters::Subscriber<jsk_recognition_msgs::BoundingBoxArray> lbl_sub;

    //APPROXIMATE Time Policy for callback to subcribe raw point cloud and segented image from dope node
    // aim is to create segmented point cloud
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, jsk_recognition_msgs::BoundingBoxArray> SyncPolicyforPCLSegmentation;
    typedef Synchronizer<SyncPolicyforPCLSegmentation> SyncSeg;
    boost::shared_ptr<SyncSeg> syncseg;
    sensor_msgs::PointCloud2 corrected_cloud;
    ros::Publisher corrected_cloud_pub_;

public:
    RandomPCLGenerator(/* args */);
    ~RandomPCLGenerator();
    sensor_msgs::PointCloud2 *recorrectPointcloud(sensor_msgs::PointCloud2 *msg);

    void generateDataset(const sensor_msgs::PointCloud2ConstPtr &pcl, const jsk_recognition_msgs::BoundingBoxArrayConstPtr &lbl);
    void addGaussiaNoisetoCloud(sensor_msgs::PointCloud2 *input, sensor_msgs::PointCloud2 *output,
                                double standard_deviation);
    void saveCloudasPLY(const std::string &filename, const pcl::PCLPointCloud2 &cloud, bool binary, bool use_camera);

    void writeDataset(sensor_msgs::PointCloud2 *pcl, jsk_recognition_msgs::BoundingBoxArray box_msg, int num_samples);
    void downsamplePCL(pcl::PCLPointCloud2::Ptr cloud, pcl::PCLPointCloud2::Ptr cloud_filtered);
};
