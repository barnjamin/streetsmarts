#include <Open3D/Open3D.h>
#include "config.h"
#include "fragments.hpp"

#include "osrm/match_parameters.hpp"
#include "osrm/nearest_parameters.hpp"
#include "osrm/route_parameters.hpp"
#include "osrm/table_parameters.hpp"
#include "osrm/trip_parameters.hpp"

#include "osrm/coordinate.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/json_container.hpp"

#include "osrm/osrm.hpp"
#include "osrm/status.hpp"

#include <cmath>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <exception>
#include <iostream>
#include <string>
#include <utility>
#include <fstream>

#include <cstdlib>

#include "config.h"
#include "utils.h"

using namespace std;
using namespace open3d;
using namespace open3d::utility;
using namespace osrm;
using namespace GeographicLib;

struct pos {
    double x,y,z;
};
struct section {
    pos start, stop;
    double distance;
};


section createSection(json::Object& step, LocalCartesian& proj){
    double lat, lon;
    double x,y,z;

    std::cout << "adsf1"<< std::endl;

    auto &geom = step.values["geometry"].get<json::Object>();

    std::cout << "adsf1"<< std::endl;

    auto &coords = geom.values["coordinates"].get<json::Array>(); 

    std::cout << "adsf1"<< std::endl;
    auto &start = coords.values.at(0).get<json::Array>();
    lon = start.values.at(0).get<json::Number>().value;
    lat = start.values.at(1).get<json::Number>().value;
    proj.Forward(lat, lon, 0, x, y, z);

    std::cout << "adsf1"<< std::endl;
    pos startPos{x,y,z};

    auto &stop = coords.values.at(1).get<json::Array>();
    lon = start.values.at(0).get<json::Number>().value;
    lat = start.values.at(1).get<json::Number>().value;
    proj.Forward(lat, lon, 0, x, y, z);
    pos stopPos{x,y,z};

    return section{startPos,stopPos,0};
}

void setCoords(const string& path, MatchParameters& params) {
    fstream fin;
    fin.open(path, ios::in);
    vector<string> row;
    string line, word, temp;
    while (fin >> line) {
        row.clear();

        stringstream s(line);
        while (getline(s, word, ',')) {
            row.push_back(word);
        }

        if(row.size()<5){
            continue;
        }

        unsigned long ts     = stoul(row[0]);
        double lat = (double)stof(row[1]);
        double lon = (double)stof(row[2]);
        double h   = (double)stof(row[3]);
        int dop    = stoi(row[4]);

        params.coordinates.push_back({util::FloatLongitude{(float)lon}, util::FloatLatitude{(float)lat}});
        params.timestamps.push_back(ts);
    }
}

int main(int argc, char * argv[])
{
    Config conf;
    // Assume json
    if(argc==2){
        std::string config_path = argv[1];
        if(!open3d::io::ReadIJsonConvertible(config_path, conf)) {
            open3d::utility::PrintError("Failed to read config\n");
            return 1;
        }
    }else{
        conf = Config(argc, argv);
    }

    EngineConfig config;

    config.storage_config = {"/home/ben/maps/ny/new-york.osrm"};
    config.use_shared_memory = false;
    config.algorithm = EngineConfig::Algorithm::MLD;


    const OSRM osrm{config};

    // The following shows how to use the Route service; configure this service
    MatchParameters params;
    params.steps = true;
    params.geometries = RouteParameters::GeometriesType::GeoJSON;

    setCoords(conf.GPSFile(), params);

    //Initialize local cartesian
    auto origin = params.coordinates[0];
    auto lat = util::toFloating(origin.lat);
    auto lon = util::toFloating(origin.lon);
    LocalCartesian proj(static_cast<double>(lat), static_cast<double>(lon));

    json::Object result;
    const auto status = osrm.Match(params, result);

    if (status != Status::Ok){
        PrintError("Failed to match gps");
        return 1;
    }
    
    std::cout << "adsf"<< std::endl;

    auto &matchings = result.values["matchings"].get<json::Array>();
    auto &route = matchings.values.at(0).get<json::Object>();
    auto &legs = route.values["legs"].get<json::Array>();

    std::cout << "adsf"<< std::endl;
    std::vector<section> sections;
    for(int i=0; i<legs.values.size(); i++){
        std::cout << i << std::endl;
        auto &leg = legs.values.at(i).get<json::Object>();
        std::cout << "asdf" << std::endl;

        auto &steps = leg.values["steps"].get<json::Array>(); 
        std::cout << "Step count: " << steps.values.size() << std::endl;

        //Just pull the first one for now
        auto &step  = steps.values.at(0).get<json::Object>();

        section s = createSection(step, proj);
        s.distance = leg.values["distance"].get<json::Number>().value;
        sections.push_back(s);
    }
    std::cout << "adsf"<< std::endl;

    for(auto const& s: sections){
        std::cout << " start: " << s.start.x  <<":"<< s.start.y;
        std::cout << " stop: " << s.stop.x  <<":"<< s.stop.y << std::endl;;
    }

    // RGBDOdom _every_ frame
    //MakeFullPoseGraph(conf);

    // Create Road Segments using sub 15m legs where street is the same, no intersections allowed
    // Accumulate Transformations until we hit leg distance
    // ChunkPoseGraph(sections, conf);


    //for(int i=0; i<sections.size(); i++){
    //    // Add UTM Coords to posegraph start/end with high(?low?) covariance prior to optimization
    //    OptimizePoseGraphForFragment(i, conf);
    //    
    //    // Generate fragment as usual 
    //    IntegrateForFragment(i, conf);
    //}
    
    // Registration as normal along Street ONLY
    // Add UTM Coords to start/end of posegraph with high? covariance

    // Integration as usual

    return EXIT_SUCCESS;
}

