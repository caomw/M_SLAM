//
// Created by steve on 3/31/16.
//

#include  <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher PointCloud_pub;

double fx(525.0),fy(525.0),cx(319.5),cy(239.5),factor(1);
void t_Callback(const sensor_msgs::Image::ConstPtr& img_msg)
{
    cv_bridge::CvImagePtr cv_img;
    cv_img = cv_bridge::toCvCopy(img_msg);
    pcl::PointCloud<pcl::PointXYZ> tmp_PointCloud;

    double rows = cv_img->image.rows;
    double cols = cv_img->image.cols;
    tmp_PointCloud.resize(rows * cols);
    cv::Mat img(cv_img->image);

    for(int i(0);i<rows;++i)
    {
        for(int j(0);j<cols;++j)
        {
            double px,py,pz;

            pz =  img.at<float>(i,j);

            px = (j - cx) * pz /fx;
            py = (i - cy) * pz /fy;

            tmp_PointCloud.at(i * cols + j) = pcl::PointXYZ(px, py, pz);
        }
    }


/************************************************ /
    fx = 525.0  # focal length x
    fy = 525.0  # focal length y
    cx = 319.5  # optical center x
    cy = 239.5  # optical center y

    factor = 5000 # for the 16-bit PNG files
# OR: factor = 1 # for the 32-bit float images in the ROS bag files

    for v in range(depth_image.height): j
    for u in range(depth_image.width): i
    Z = depth_image[v,u] / factor;
    X = (u - cx) * Z / fx;
    Y = (v - cy) * Z / fy;
    /******************************************/

    sensor_msgs::PointCloud2 PointCloud_msg;
    pcl::toROSMsg(tmp_PointCloud, PointCloud_msg);

    PointCloud_msg.header.stamp = cv_img->header.stamp;
    PointCloud_msg.header.frame_id = "/camera";
    PointCloud_pub.publish(PointCloud_msg);
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"Img2Pc");
    ros::NodeHandle n;

    PointCloud_pub = n.advertise<sensor_msgs::PointCloud2>("pc",1);

    ros::Subscriber sub=n.subscribe("/camera/depth/image",1,t_Callback);

    ros::spin();


    return 0;
}
