#ifndef ROS_HANDLER_HPP
#define ROS_HANDLER_HPP

#define BOLD_RED "\033[1;31m"
#define BOLD_GREEN "\033[1;32m"
#define BOLD_YELLOW "\033[1;33m"

#include <wind2geojson/geojson_handler.hpp>

#include <ros/ros.h>

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_srvs/Empty.h>
#include <wind2geojson/start_stop_recording.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/bind.hpp>
#include <boost/chrono.hpp>

#include <eigen3/Eigen/Dense>

class RosHandler : public GeojsonHandler
{
    private:
        ros::NodeHandle &nh;
        ros::ServiceServer start_saving;

        message_filters::Subscriber<sensor_msgs::NavSatFix> navsat_sub;
        message_filters::Subscriber<geometry_msgs::Vector3Stamped> wind_velocity_sub;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix,
        geometry_msgs::Vector3Stamped> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        typedef boost::chrono::steady_clock::duration Duration;

        boost::shared_ptr<Sync> _sync_ptr;
        boost::chrono::steady_clock::time_point period_tp;
        boost::chrono::steady_clock::time_point stop_tp;

        Eigen::Vector3d nav_sat_sum;
        Eigen::Vector3d vel_sum;
        Eigen::Matrix3d cov_matrix;

        int num_of_samples;
        double period_secs;
        double stop_time;
        bool is_saving;

    public:
        RosHandler(ros::NodeHandle &nh);
        ~RosHandler();

        void mainCallback(const sensor_msgs::NavSatFix::ConstPtr &nav_sat,
                          const geometry_msgs::Vector3Stamped::ConstPtr &wind_vel);
        bool startStopCallback(wind2geojson::start_stop_recordingRequest &req,
                               wind2geojson::start_stop_recordingResponse &res);

        void startRecording();
        void stopRecording();
};

#endif // ROS_HANDLER_HPP
