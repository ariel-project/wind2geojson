#ifndef GEOJSON_HANDLER_HPP
#define GEOJSON_HANDLER_HPP

#include <iostream>
#include <fstream>

#include <vector>

#include <jsoncpp/json/json.h>

#include <boost/date_time.hpp>
#include <boost/chrono.hpp>

#include <iomanip>
#include <ctime>
#include <sstream>

class GeojsonHandler
{
    typedef boost::posix_time::ptime Time;

    private:
        Json::StreamWriterBuilder builder;
        
        Json::Value feature_collection;
        Json::Value feature_array;
        unsigned int feature_index;

        std::string file_path;
    
    public:
        GeojsonHandler(std::string path) :
        feature_index(0),
        file_path(path)
        {
            feature_collection["type"] = "FeatureCollection";
            builder["indentation"] = "  ";
        }

        ~GeojsonHandler(){}

        void addPoint(std::vector<double> nav_sat,
                      std::vector<double> wind_vel,
                      boost::posix_time::ptime time_stamp)
        {
            Json::Value point;
            point[0] = nav_sat[0];
            point[1] = nav_sat[1];
            point[2] = nav_sat[2];

            Json::Value wind;
            wind[0] = wind_vel[0];
            wind[1] = wind_vel[1];
            wind[2] = wind_vel[2];

            Json::Value geometry;
            geometry["type"] = "Point";
            geometry["coordinates"] = point;
            geometry["wind"] = wind;

            Json::Value feature;
            feature["type"] = "Feature";
            feature["properties"] = Json::Value();
            feature["geometry"] = geometry;
            feature["stamp"] = boost::posix_time::to_iso_extended_string(time_stamp);

            feature_array[feature_index++] = feature;
        }

        void writeToFile()
        {
            feature_collection["features"] = feature_array;
            const std::string json_file = Json::writeString(builder, feature_collection);

            std::ofstream output_file(file_path +
                                      boost::posix_time::to_iso_extended_string(boost::posix_time::second_clock::local_time()) +
                                      ".geojson");
            output_file << json_file;
            output_file.close();
        }

        void setNewFilepath(std::string path)
        {
            file_path = path;
        }
};

#endif
