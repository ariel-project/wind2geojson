#ifndef GEOJSON_HANDLER_HPP
#define GEOJSON_HANDLER_HPP

#include <iostream>
#include <fstream>

#include <vector>

#include <jsoncpp/json/json.h>

#include <iomanip>
#include <ctime>
#include <sstream>

class GeojsonHandler
{
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
                      std::vector<double> wind_vel)
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

            Json::Value feature;
            feature["type"] = "Feature";
            feature["properties"] = Json::Value();
            feature["geometry"] = geometry;
            feature["wind"] = wind;
            feature["stamp"] = getCurrentTime().str();

            feature_array[feature_index++] = feature;
        }

        void writeToFile()
        {
            feature_collection["features"] = feature_array;
            const std::string json_file = Json::writeString(builder, feature_collection);

            std::ofstream output_file(file_path + getCurrentTime().str() + ".geojson");
            output_file << json_file;
            output_file.close();
        }

        std::ostringstream getCurrentTime()
        {
            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);

            std::ostringstream oss;
            oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");

            return oss;
        }

        void setNewFilepath(std::string path)
        {
            file_path = path;
        }
};

#endif
