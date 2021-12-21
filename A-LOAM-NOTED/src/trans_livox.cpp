#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <sensor_msgs/PointCloud2.h>
#include <stdlib.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <Eigen/Core>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
// #include <pcl/visualization/pcl_visualizer.h>

ros::Publisher pubCloud2; //话题发布拼接点云
double tana = 0;
typedef pcl::PointXYZI PointType;
pcl::PointCloud<PointType> cloud;

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    double time_cloud = laserCloudMsg->header.stamp.toSec();
    pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
    pcl::PointCloud<pcl::PointXYZI> laserCloudOut;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);

    pcl::copyPointCloud(laserCloudIn,laserCloudOut);

    pcl::PointIndices::Ptr indices(new pcl::PointIndices());
    // for(auto &point:laserCloudIn)
    for (int i = 0; i < laserCloudOut.points.size(); i++)
    {
        PointType point = laserCloudOut.points[i];
        if (pow(point.x * tana, 2) > (pow(point.y, 2) + pow(point.z, 2)) && point.x >= 0)
        {
            // laserCloudOut.points.push_back(point);
            indices->indices.push_back(i);
        }
    }

    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(laserCloudOut.makeShared());
    extract.setIndices(indices);
    extract.setNegative(false);//false 是保存自己的indice的点
    pcl::PointCloud<PointType> result_map2;
    extract.filter(result_map2);

    sensor_msgs::PointCloud2 pub_cloud2;
    pcl::toROSMsg(result_map2, pub_cloud2);
    pub_cloud2.header.stamp = ros::Time().fromSec(time_cloud);
    pub_cloud2.header.frame_id = "/livox_frame";
    pubCloud2.publish(pub_cloud2);
}

int main(int argc, char **argv)
{
    double angle = 20. / 180 * M_PI;
    tana = tan(angle);
    ros::init(argc, argv, "trans_cloud");
    ros::NodeHandle n;
    ros::Subscriber LaserCloud_sub = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, laserCloudHandler);
    pubCloud2 = n.advertise<sensor_msgs::PointCloud2>("/livox/lidar", 20);

    ros::spin();

    return 0;
}
