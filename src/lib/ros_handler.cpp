#include <wind2geojson/ros_handler.hpp>

RosHandler::RosHandler(ros::NodeHandle &nh) :
    GeojsonHandler("../logs/"),
    nh(nh),
    nav_sat_sum(Eigen::Vector3d::Zero()),
    vel_sum(Eigen::Vector3d::Zero()),
    num_of_samples(0),
    period_secs(2.0)
{
    std::string temp_str;
    std::string node_name = ros::this_node::getName();

    if (!nh.getParam(node_name + "/full_geojson_filepath", temp_str) || temp_str.empty())
        throw std::runtime_error(BOLD_RED "[ ERROR] parameter full_geojson_filepath doesn't exist."
                                          "Please, set the parameter in congig.yaml");
    setNewFilepath(temp_str);
    temp_str.clear();

    if (!nh.getParam(node_name + "/navsat_subscriber", temp_str) || temp_str.empty())
        throw std::runtime_error(BOLD_RED "[ ERROR] parameter navsat_subscriber doesn't exist."
                                          "Please, set the parameter in congig.yaml");
    navsat_sub.subscribe(nh, temp_str, 5);
    temp_str.clear();

    if (!nh.getParam(node_name + "/wind_velocity_subscriber", temp_str) || temp_str.empty())
        throw std::runtime_error(BOLD_RED "[ ERROR] parameter wind_velocity_subscriber doesn't exist."
                                          "Please, set the parameter in congig.yaml");
    wind_velocity_sub.subscribe(nh, temp_str, 5);
    temp_str.clear();

    _sync_ptr.reset(new Sync(MySyncPolicy(5), navsat_sub, wind_velocity_sub));
    _sync_ptr->registerCallback(boost::bind(&RosHandler::mainCallback, this, _1, _2));

    std::cout << BOLD_YELLOW "[ WARN] all topics configured." << std::endl;
}

void RosHandler::mainCallback(const sensor_msgs::NavSatFix::ConstPtr &nav_sat,
                              const geometry_msgs::Vector3Stamped::ConstPtr &wind_vel)
{
    boost::chrono::milliseconds dt = boost::chrono::duration_cast<boost::chrono::milliseconds>(boost::chrono::steady_clock::now() - last_sample_time);
    double dt_secs = dt.count()/1000.0;

    if ( dt_secs > period_secs )
    {
        nav_sat_sum /= double(num_of_samples);
        vel_sum /= double(num_of_samples);

        addPoint({nav_sat_sum[0], nav_sat_sum[1], nav_sat_sum[2]},
                 {vel_sum[0], vel_sum[1], vel_sum[2]},
                 wind_vel->header.stamp.toBoost());

        num_of_samples = 0;
        nav_sat_sum = Eigen::Vector3d::Zero();
        vel_sum = Eigen::Vector3d::Zero();
        last_sample_time = boost::chrono::steady_clock::now();
    }
    else
    {
        num_of_samples++;
        nav_sat_sum += Eigen::Vector3d({nav_sat->longitude, nav_sat->latitude, nav_sat->altitude});
        vel_sum += Eigen::Vector3d({wind_vel->vector.x, wind_vel->vector.y, wind_vel->vector.z});
    }
}

RosHandler::~RosHandler()
{
    writeToFile();
    std::cerr << BOLD_YELLOW "[ WARN] shutting down node!" << std::endl;
}
