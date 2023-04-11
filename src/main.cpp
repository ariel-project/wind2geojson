#include <wind2geojson/ros_handler.hpp>

int main (int argc, char* argv[])
{
    ros::init(argc, argv, "wind2geojson");

    ros::NodeHandle nh("~");

    RosHandler rh(nh);

    ros::spin();

    return 0;
}
