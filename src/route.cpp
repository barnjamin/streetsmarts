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

using namespace osrm;

void setCoords(const std::string& path, osrm::MatchParameters& params) {
    using namespace std;
    using namespace GeographicLib;

    fstream fin;

    Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());

    fin.open(path, ios::in);

    vector<string> row;
    string line, word, temp;

    int base =  1556453679;
    while (fin >> line) {

        row.clear();

        stringstream s(line);
        while (getline(s, word, ',')) {
            row.push_back(word);
        }

        if(row.size()<6){
            continue;
        }

        int ts    = stoi(row[0]);
        int dop   = stoi(row[6]);

        double X     = ((double)stoi(row[1]))/100.0;
        double Y     = ((double)stoi(row[2]))/100.0;
        double Z     = ((double)stoi(row[3]))/100.0;



        double lat, lon, h;
        earth.Reverse(X, Y, Z, lat, lon, h);

        //std::cout << ts <<","<< lat <<","<< lon <<","<< h <<","<< dop << std::endl;
        params.coordinates.push_back({util::FloatLongitude{(float)lon}, util::FloatLatitude{(float)lat}});
        params.timestamps.push_back((unsigned)ts+base);
    }
}


int main(int argc, char *argv[])
{

    Config conf(argc, argv);
    EngineConfig config;

    config.storage_config = {"/home/ben/maps/ny/new-york.osrm"};
    config.use_shared_memory = false;
    config.algorithm = EngineConfig::Algorithm::MLD;

    const OSRM osrm{config};

    // The following shows how to use the Route service; configure this service
    MatchParameters params;
    params.steps = true;

    setCoords(conf.GPSFile(), params);

    // Response is in JSON format
    json::Object result;

    // Execute routing request, this does the heavy lifting
    const auto status = osrm.Match(params, result);

    if (status == Status::Ok)
    {
        auto &matchings = result.values["matchings"].get<json::Array>();


        // Let's just use the first route
        auto &route = matchings.values.at(0).get<json::Object>();


        auto &steps = route.values["legs"].get<json::Array>();

        std::cout << steps.values.size() << std::endl;

        for(int i=0; i<steps.values.size(); i++){
            auto &leg = steps.values.at(i).get<json::Object>();
            std::cout << "Distance: " << leg.values["distance"].get<json::Number>().value
                      << "\tDuration: "<< leg.values["duration"].get<json::Number>().value
                      << "\tSummary: " << leg.values["summary"].get<json::String>().value << std::endl;
        }

        return EXIT_SUCCESS;
    }
    else if (status == Status::Error)
    {
        const auto code = result.values["code"].get<json::String>().value;
        const auto message = result.values["message"].get<json::String>().value;

        std::cout << "Code: " << code << "\n";
        std::cout << "Message: " << code << "\n";
        return EXIT_FAILURE;
    }
}
