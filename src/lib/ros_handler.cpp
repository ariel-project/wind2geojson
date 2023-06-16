#include <wind2geojson/ros_handler.hpp>

RosHandler::RosHandler(ros::NodeHandle &nh) :
    GeojsonHandler("../logs/"),
    nh(nh),
    nav_sat_sum(Eigen::Vector3d::Zero()),
    vel_sum(Eigen::Vector3d::Zero()),
    cov_matrix(Eigen::Matrix3d::Zero()),
    num_of_samples(0),
    period_secs(60.0),
    stop_time(300),
    is_saving(false)
{
    std::string temp_str;
    std::string node_name = ros::this_node::getName();

    if (!nh.getParam(node_name + "/start_service_name", temp_str))
        throw std::runtime_error(BOLD_RED "[ ERROR] parameter start_service_name doesn't exist."
                                          "Please, set the parameter in config.yaml");
    start_saving = nh.advertiseService(temp_str, &RosHandler::startStopCallback, this);

    if (!nh.getParam(node_name + "/period_secs", period_secs))
        throw std::runtime_error(BOLD_RED "[ ERROR] parameter period_secs doesn't exist."
                                          "Please, set the parameter in congig.yaml");

    if (!nh.getParam(node_name + "/full_geojson_filepath", temp_str) || temp_str.empty())
        throw std::runtime_error(BOLD_RED "[ ERROR] parameter full_geojson_filepath doesn't exist."
                                          "Please, set the parameter in congig.yaml");
    setNewFilepath(temp_str);
    temp_str.clear();

    _sync_ptr.reset(new Sync(MySyncPolicy(5), navsat_sub, wind_velocity_sub));
    _sync_ptr->registerCallback(boost::bind(&RosHandler::mainCallback, this, _1, _2));
}

void RosHandler::mainCallback(const sensor_msgs::NavSatFix::ConstPtr &nav_sat,
                              const geometry_msgs::Vector3Stamped::ConstPtr &wind_vel)
{
    boost::chrono::milliseconds stop_duration = boost::chrono::duration_cast<boost::chrono::milliseconds>(boost::chrono::steady_clock::now() - stop_tp);
    double stop_duration_secs = stop_duration.count()/1000.0;

    if (stop_duration_secs > stop_time)
        stopRecording();
    else
    {
        boost::chrono::milliseconds period_duration= boost::chrono::duration_cast<boost::chrono::milliseconds>(boost::chrono::steady_clock::now() - period_tp);
        double period_duration_secs = period_duration.count()/1000.0;

        if ( period_duration_secs > period_secs )
        {
            nav_sat_sum /= double(num_of_samples);
            vel_sum /= double(num_of_samples);

            addPoint({nav_sat_sum[0], nav_sat_sum[1], nav_sat_sum[2]},
                     {vel_sum[0], vel_sum[1], vel_sum[2]},
                     boost::posix_time::second_clock::local_time());

            num_of_samples = 0;
            nav_sat_sum = Eigen::Vector3d::Zero();
            vel_sum = Eigen::Vector3d::Zero();
            period_tp = boost::chrono::steady_clock::now();
        }
        else
        {
            num_of_samples++;
            nav_sat_sum += Eigen::Vector3d({nav_sat->longitude, nav_sat->latitude, nav_sat->altitude});
            vel_sum += Eigen::Vector3d({wind_vel->vector.x, wind_vel->vector.y, wind_vel->vector.z});
        }
    }
}

bool RosHandler::startStopCallback(wind2geojson::start_stop_recordingRequest &req,
                                   wind2geojson::start_stop_recordingResponse &res)
{
    period_secs = req.period_secs;
    stop_time = req.stop_time;

    if ( !is_saving )
        startRecording();
    else
        stopRecording();

    res.success = is_saving;

    return true;
}

void RosHandler::startRecording()
{
    is_saving = true;

    std::string temp_str;
    std::string node_name = ros::this_node::getName();

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

    period_tp = stop_tp = boost::chrono::steady_clock::now();

    std::cout << BOLD_YELLOW "[ WARN] all topics configured." << std::endl;

    num_of_samples = 0;
    nav_sat_sum = Eigen::Vector3d::Zero();
    vel_sum = Eigen::Vector3d::Zero();
}

void RosHandler::stopRecording()
{
    is_saving = false;

    navsat_sub.unsubscribe();
    wind_velocity_sub.unsubscribe();

    std::cerr << BOLD_YELLOW "[ WARN] writting geojson file." << std::endl;
    writeToFile();
}

RosHandler::~RosHandler()
{
    std::cerr << BOLD_YELLOW "[ WARN] shutting down node!" << std::endl;
}
