#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <pointmatcher/PointMatcher.h>
#include <geometry_msgs/TransformStamped.h>
#include <fstream>

typedef PointMatcher<float> PM;

std::unique_ptr<tf2_ros::Buffer> tfBuffer;
sensor_msgs::PointCloud2 cloud;
std::atomic_bool newCloud;
//std::shared_ptr<PM::DataPointsFilters> inputFilters;

void cloudCallback(const sensor_msgs::PointCloud2& msg)
{
    cloud = msg;
    newCloud.store(true);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bounding_box_node");
    ros::NodeHandle nodeHandle;
    ros::NodeHandle privateNodeHandle("~");

    newCloud.store(false);

    std::string inputFiltersConfigFilePath;
    nodeHandle.getParam("/bounding_box_node/input_filters_config", inputFiltersConfigFilePath);

    std::ifstream ifs(inputFiltersConfigFilePath);
    PM::DataPointsFilters inputFilters = PM::DataPointsFilters(ifs);
    ifs.close();

    ros::Publisher cloudPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("cloud_out", 1);

    ros::Subscriber CloudSubscriber = nodeHandle.subscribe("cloud_in", 1, cloudCallback);

    ros::Rate loopRate(20);
    while(ros::ok())
    {
        if(newCloud.load())
        {
            newCloud.store(false);
            ros::Time lookupTime;
            lookupTime = cloud.header.stamp;
            try
            {
                PM::DataPoints cloudPM = PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(cloud);
                inputFilters.apply(cloudPM);
                std::string cloudFrame;
                cloudFrame = cloud.header.frame_id;
                sensor_msgs::PointCloud2 filteredCloud = PointMatcher_ROS::pointMatcherCloudToRosMsg<float>(cloudPM, cloudFrame, lookupTime);
                cloudPublisher.publish(filteredCloud);
            }
            catch(const tf2::TransformException& ex)
            {
                ROS_WARN("%s", ex.what());
            }
        }

        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
