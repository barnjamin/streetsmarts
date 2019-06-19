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


std::shared_ptr<geometry::LineSet> MakeLineSetFromSections(std::vector<section> sections){
    std::shared_ptr<geometry::LineSet> secline = std::make_shared<geometry::LineSet>();

    for(int i=0; i<sections.size(); i++){
        section s = sections[i];
        secline->points_.push_back(Eigen::Vector3d(s.start.x, s.start.y, s.start.z));
        secline->points_.push_back(Eigen::Vector3d(s.stop.x, s.stop.y, s.stop.z));

        secline->lines_.push_back(Eigen::Vector2i(i*2, (i*2)+1));

        double red   = (((double) rand() / (RAND_MAX)) ) ;
        double blue  = (((double) rand() / (RAND_MAX)) ) ;
        double green = (((double) rand() / (RAND_MAX)) ) ;

        secline->colors_.push_back(Eigen::Vector3d(red, blue, green));
    }

    return secline;
}

section createSection(json::Array& steps, LocalCartesian& proj){
    double lat, lon;
    double x,y,z;

    //TODO:: we're only using the first and second step for now, for intersections there are 3 steps 
    //for(int i=0;i<steps.values.size();i++) { }
    
    auto &step_start  = steps.values.at(0).get<json::Object>();
    auto &man_start = step_start.values["maneuver"].get<json::Object>();
    auto &coord_start = man_start.values["location"].get<json::Array>(); 
    lon = coord_start.values.at(0).get<json::Number>().value;
    lat = coord_start.values.at(1).get<json::Number>().value;
    proj.Forward(lat, lon, 0, x, y, z);
    pos startPos{x,y,z};

    auto &step_stop  = steps.values.at(1).get<json::Object>();
    auto &man_stop = step_stop.values["maneuver"].get<json::Object>();
    auto &coord_stop = man_stop.values["location"].get<json::Array>(); 
    lon = coord_stop.values.at(0).get<json::Number>().value;
    lat = coord_stop.values.at(1).get<json::Number>().value;
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
        float lat = stof(row[1]);
        float lon = stof(row[2]);
        //double h   = (double)stof(row[3]);
        //int dop    = stoi(row[4]);

        params.coordinates.push_back({util::FloatLongitude{(float)lon}, util::FloatLatitude{(float)lat}});
        params.timestamps.push_back(ts);
    }
}

void ChunkPoseGraph(std::vector<section> sections, Config& conf) {
    // Read in full posegraph
    PoseGraph full_pose_graph;
    ReadPoseGraph(conf.PoseFile(-1), full_pose_graph);

    // find distance for each section
    double dist = 0;
    //int sidx = 0;
    //int s = sections[sidx];
    for(int x=0;x<full_pose_graph.edges_.size();x++){
        Eigen::Vector3d t = full_pose_graph.edges_[x].transformation_.topRightCorner(3,1);
        std::cout << t.norm() << std::endl;
        dist += t.norm(); 
    }
    std::cout << "dist: "<< dist<< std::endl;


    Eigen::Vector3d totalt = full_pose_graph.nodes_[full_pose_graph.nodes_.size()-1].pose_.topRightCorner(3,1);
    std::cout << "total: " << totalt.norm() << std::endl;

    /* Pin start/end via posegraph to utm coords 
     * Run optimizer
     * Pin midpoints using same method, run optimizer
     * 
     *
     */
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
    params.tidy = true;
    params.geometries = RouteParameters::GeometriesType::GeoJSON;

    setCoords(conf.GPSFile(), params);


    json::Object result;
    const auto status = osrm.Match(params, result);

    if (status != Status::Ok){
        PrintError("Failed to match gps\n");
        return 1;
    }
    
    auto &matchings = result.values["matchings"].get<json::Array>();
    auto &route = matchings.values.at(0).get<json::Object>();
    auto &legs = route.values["legs"].get<json::Array>();
    
    //Initialize local cartesian
    auto &tracepts = result.values["tracepoints"].get<json::Array>();
    auto &origin = tracepts.values.at(0).get<json::Object>();
    auto &origcoords = origin.values["location"].get<json::Array>();
    double lat = origcoords.values.at(1).get<json::Number>().value;
    double lon = origcoords.values.at(0).get<json::Number>().value;
    LocalCartesian proj(lat, lon);

    std::vector<section> sections;
    for(int i=0; i<legs.values.size(); i++){
        auto &leg = legs.values.at(i).get<json::Object>();

        auto &steps = leg.values["steps"].get<json::Array>(); 

        section s = createSection(steps, proj);
        if(s.start.x == s.stop.x && s.start.y == s.stop.y){
            continue ;
        }
        s.distance = leg.values["distance"].get<json::Number>().value;
        sections.push_back(s);
    }

    std::ofstream segment_file;
    segment_file.open(conf.RoadSegmentFile());
    for(auto &section: sections){
        segment_file << section.start.x << "," << section.start.y << "," << section.start.z 
            << "," << section.stop.x << "," << section.stop.y << "," << section.stop.z  << std::endl;
    }
    segment_file.close();

    //auto lineset = MakeLineSetFromSections(sections);

    //auto pcd = io::CreatePointCloudFromFile(conf.SceneMeshFile());

    ////Flip it over
    //Eigen::AngleAxis<double> fa(-M_PI, Eigen::Vector3d(0,0,1));
    //Eigen::Transform<double,3,Eigen::Affine> flip = Eigen::Translation3d(0,0,0) * fa;
    //pcd->Transform(flip.matrix());

    //////Rotate to align with road section
    //Eigen::AngleAxis<double> sa(M_PI/2, Eigen::Vector3d(0,1,0));
    //Eigen::Transform<double,3,Eigen::Affine> spin = Eigen::Translation3d(0,0,0) * sa;
    //pcd->Transform(spin.matrix());

    ////Flatten it
    //pcd->Transform(Flatten(*pcd));

    //visualization::DrawGeometries({lineset, pcd});

    return 0;

    // RGBDOdom _every_ frame
    MakeFullPoseGraph(conf);

    // Create Road Segments sections where street is the same, no intersections yet
    // Accumulate Transformations until we hit section distance
    //ChunkPoseGraph(sections, conf);


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

