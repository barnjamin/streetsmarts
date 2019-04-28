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

        int ts     = stoi(row[0]);
        double lat = (double)stof(row[1]);
        double lon = (double)stof(row[2]);
        double h   = (double)stof(row[3]);
        int dop    = stoi(row[4]);

        params.coordinates.push_back({util::FloatLongitude{(float)lon}, util::FloatLatitude{(float)lat}});
        params.timestamps.push_back((unsigned)ts);
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

    LocalCartesian proj;

    const OSRM osrm{config};

    // The following shows how to use the Route service; configure this service
    MatchParameters params;
    params.steps = true;

    setCoords(conf.GPSFile(), params);

    // Response is in JSON format
    json::Object result;

    // Execute routing request, this does the heavy lifting
    const auto status = osrm.Match(params, result);

    if (status != Status::Ok){
        PrintError("Failed to match gps");
        return 1;
    }

    auto &matchings = result.values["matchings"].get<json::Array>();

    // Let's just use the first route
    auto &route = matchings.values.at(0).get<json::Object>();

    auto &legs = route.values["legs"].get<json::Array>();

    std::cout << steps.values.size() << std::endl;


    for(int i=0; i<legs.values.size(); i++){
        auto &leg = legs.values.at(i).get<json::Object>();

        auto &steps = leg.values["step"].get<json::Array>(); 
        std::cout << "Step count: " << steps.values.size() << std::endl;

        //Just pull the first one for now
        auto &step  = steps.values.at(0).get<json::Object>();

        //double x,y,z;
        //proj.forward(lat,lng,alt,x,y,z);

        // RGBDOdom _every_ frame
        
        // Create Road Segments using sub 15m legs where street is the same, no intersections allowed
        // Accumulate Transformations until we hit leg distance
        
        // Add UTM Coords to posegraph start/end with high(?low?) covariance prior to optimization
        // Generate fragment as usual 
        
        // Registration as normal along Street ONLY
        // Add UTM Coords to start/end of posegraph with high? covariance

        // Integration as usual
       
        

        //for(int j=0; j<steps.values.size(); j++){ }
        // std::cout << "Distance: " << leg.values["distance"].get<json::Number>().value
        //           << "\tDuration: "<< leg.values["duration"].get<json::Number>().value
        //           << "\tSummary: " << leg.values["summary"].get<json::String>().value << std::endl;
    }

    for (int i = 0; i < fragments; ++i) {
        PrintInfo("Processing fragment %d / %d\n", i, fragments);

        MakePoseGraphForFragment(i, conf);
        OptimizePoseGraphForFragment(i, conf);
        IntegrateForFragment(i, conf);
    }

    timer.Stop();
    PrintInfo("MakeFragment takes %.3f s\n", timer.GetDuration() / 1000.0f);

    return EXIT_SUCCESS;
}

