//
// Created by steve on 4/1/16.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/registration/ia_ransac.h>
#include  <pcl/registration/icp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl_ros/point_cloud.h>




ros::Publisher pub;


pcl::PointCloud<pcl::PointXYZ>::Ptr last_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::Normal>::Ptr last_normal_ptr(new pcl::PointCloud<pcl::Normal>);

//pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::Normal> normal_estimation;

Eigen::Matrix4f c_TransformMatrix;//(Eigen::Matrix4f::Identity());

//transform point cloud,use method in pcl
void transformCallback(const sensor_msgs::PointCloud2::ConstPtr& pc_msg_ptr)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr src_normal_ptr(new pcl::PointCloud<pcl::Normal>);
    pcl::fromROSMsg(*pc_msg_ptr,*src_ptr);
    //std::cout <<"1"<<std::endl;
    if(last_pc_ptr->size() < 10)
    {
        std::cout <<"2"<<std::endl;
        c_TransformMatrix=Eigen::Matrix4f::Identity();

        *last_pc_ptr = *src_ptr;


        return;
    }
    //normal_estimation.setInputCloud(src_ptr);
    //normal_estimation.compute(*src_normal_ptr);
   // std::cout <<"3"<<std::endl;
    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;

    icp.setInputSource(src_ptr);
    if(src_ptr->size()<10) return;
    icp.setInputTarget(last_pc_ptr);
    std::cout <<"4"<<std::endl;

    std::cout << src_ptr->size()<< ":"<<last_pc_ptr->size()<<std::endl;
    pcl::PointCloud<pcl::PointXYZ> tp;

    icp.align(tp);
    std::cout <<"5"<<std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr tr_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    c_TransformMatrix =  c_TransformMatrix * icp.getFinalTransformation() ;
    pcl::transformPointCloud(*src_ptr,*tr_pc_ptr,c_TransformMatrix);
    *last_pc_ptr = *src_ptr;


    sensor_msgs::PointCloud2 pub_msg;
    pcl::toROSMsg(*tr_pc_ptr,pub_msg);

    std::cout << std::endl;
    std::cout << c_TransformMatrix <<std::endl;

    pub_msg.header.stamp=pc_msg_ptr->header.stamp;
    pub_msg.header.frame_id = pc_msg_ptr->header.frame_id;

    pub.publish(pub_msg);



}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"M_SLAM");

    ros::NodeHandle n_;

    c_TransformMatrix = Eigen::Matrix4f::Identity();

    pub = n_.advertise<sensor_msgs::PointCloud2>("pc_t",1);

    ros::Subscriber sub= n_.subscribe("pc",1,transformCallback);
    ros::spin();
    return 0;


}