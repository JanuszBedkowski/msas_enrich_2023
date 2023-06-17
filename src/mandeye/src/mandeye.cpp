#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mandeye");
    ros::NodeHandle nh;
    ros::Publisher pc_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI> > ("cloud", 1);

    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr msg(new pcl::PointCloud<pcl::PointXYZI>);

        size_t point_count = 100;

        msg->header.frame_id = "odom";
        msg->height = 1;
        msg->width = point_count;
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);

        for (size_t i=0;i<point_count;++i)
        {
            pcl::PointXYZI p;
            p.x = -1.5  + 3 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
            p.y = - 0.5 + 1 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
            p.z = - 1 + 2 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
            p.intensity = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
            msg->points.push_back(p);
        }

        pc_pub.publish(*msg);


        ros::spinOnce();

        loop_rate.sleep();
    }


  return 0;
}