#include <iostream>
#include <Visualization/Visualization.h>
#include <Core/Core.h>
#include <IO/IO.h>

#include <Cuda/Registration/RegistrationCuda.h>
#include <Cuda/Registration/ColoredICPCuda.h>
#include <Cuda/Registration/FastGlobalRegistrationCuda.h>
#include <Core/Registration/PoseGraph.h>
#include <Core/Registration/GlobalOptimization.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include "config.h"
#include "utils.h"

std::shared_ptr<open3d::PointCloud> readKittiVelodyne(std::string& fileName){
    std::shared_ptr<open3d::PointCloud> pc = std::make_shared<open3d::PointCloud>();

    std::ifstream input(fileName.c_str(), std::ios_base::binary);
    if(!input.good()){
        std::cerr<<"Cannot open file : "<<fileName<<std::endl;
        return pc;
    }

    float bbmax = 30.0;
    for (int iter=0; input.good() && !input.eof(); iter++) {
        float x,y,z;
        float i;

        input.read((char *) &x, sizeof(float));
        input.read((char *) &y, sizeof(float));
        input.read((char *) &z, sizeof(float));
        input.read((char *) &i, sizeof(float));


        if(abs(x)>bbmax || abs(y)>bbmax || abs(z)>bbmax){
            continue; 
        }

        pc->points_.push_back(Eigen::Vector3d(x,y,z));
        pc->colors_.push_back(Eigen::Vector3d(i, i, i));
    }
    input.close();

    EstimateNormals(*pc, open3d::KDTreeSearchParamKNN());

    return pc;

    //auto result = RemoveStatisticalOutliers(*pc, 50, 0.3);
    //return std::get<0>(result);
}

std::tuple<double, double, double> get_lat_lng(std::string gps_file) 
{
    //Read entire file (only 1 line)
    std::ifstream ifs(gps_file.c_str());
    std::string line( (std::istreambuf_iterator<char>(ifs)),
                       (std::istreambuf_iterator<char>()));

    std::stringstream sline(line);
    std::string segment;

    std::getline(sline, segment, ' ');
    double lat = std::stod(segment);

    std::getline(sline, segment, ' ');
    double lng = std::stod(segment);

    std::getline(sline, segment, ' ');
    double alt = std::stod(segment);

    return std::make_tuple(lat, lng, alt);
}

int main(int argc, char ** argv) 
{

    Config config(argc, argv);


    std::string gps_dir = "/home/ben/kitti-sync/oxts/data";
    std::vector<std::string> gps;
    open3d::filesystem::ListFilesInDirectory(gps_dir, gps);
    sort(gps.begin(), gps.end());

    Eigen::Matrix4d trans_odometry = Eigen::Matrix4d::Identity();
    open3d::PoseGraph pose_graph;
    pose_graph.nodes_.emplace_back(open3d::PoseGraphNode(trans_odometry));

    const GeographicLib::Geocentric& earth = GeographicLib::Geocentric::WGS84();
    GeographicLib::LocalCartesian proj;

    Eigen::Quaterniond q(1,0,0,0);

    Eigen::Matrix6d information;
    information << 
        614817.25, -2948.681884765625, 65493.2734375, 0.0, -393870.875, -2794.27001953125,
        -2948.681884765625, 675245.5625, 8518.068359375, 393870.875, 0.0, 27101.2109375,
        65493.2734375, 8518.068359375, 200426.1875, 2794.26806640625, -27101.2109375, 0.0,
        0.0, 393870.875, 2794.26806640625, 288427.0, 0.0, 0.0,
        -393870.875, 0.0, -27101.2109375, 0.0, 288427.0, 0.0,
        -2794.27001953125, 27101.2109375, 0.0, 0.0, 0.0, 288427.0 ;


    double lat, lng, alt;
    double x, y, z;
    double x_last, y_last, z_last;
    int idx = 0;
    for(std::string line: gps)
    {
        std::tie(lat, lng, alt) = get_lat_lng(line);
        //open3d::PrintInfo("%.6f %.6f %.6f\n", lat, lng, alt);

        if(idx == 0){ 
            proj.Reset(lat, lng, alt);   
            idx++; 
            continue; 
        }

        proj.Forward(lat, lng, alt, x, y, z);

        Eigen::Translation3d trans_to_world(Eigen::Vector3d(x, y, z));
        Eigen::Transform<double, 3, Eigen::Projective>  world_trans = trans_to_world * q.normalized().toRotationMatrix();


        // Create transform between last 2
        Eigen::Translation3d translation_diff(Eigen::Vector3d(x-x_last, y-y_last, z-z_last));
        Eigen::Transform<double, 3, Eigen::Projective> local_trans = translation_diff * q.normalized().toRotationMatrix();


        
        pose_graph.nodes_.emplace_back(open3d::PoseGraphNode(world_trans.matrix().inverse()));
        pose_graph.edges_.emplace_back(open3d::PoseGraphEdge(idx - 1, idx, local_trans.matrix(), information, false));

        x_last = x;
        y_last = y;
        z_last = z;

        idx++;
    }

    WritePoseGraph("/home/ben/kitti-sync/gps_pg.json", pose_graph);

    //Update PoseGraph

    std::shared_ptr<open3d::PointCloud> src;
    std::shared_ptr<open3d::PointCloud> tgt;

    std::string bin_dir = "/home/ben/kitti-sync/velodyne_points/data";
    std::vector<std::string> bins;
    open3d::filesystem::ListFilesInDirectory(bin_dir, bins);
    sort(bins.begin(), bins.end());

    for(int i=0; i<bins.size(); i++){
        src = readKittiVelodyne(bins[i]);
        if(i == 0) { 
            tgt = src;
            continue; 
        }


        //open3d::PrintInfo("PointCloud %d to %d\n", i-1, i);
        //VisualizeRegistration(*src, *tgt, Eigen::Matrix4d::Identity());

        open3d::cuda::RegistrationCuda registration(open3d::TransformationEstimationType::PointToPoint);

        registration.Initialize(*src, *tgt, 0.9, pose_graph.nodes_[i].pose_);

        registration.ComputeICP();

        auto imat = registration.ComputeInformationMatrix();

        src->Transform(registration.transform_source_to_target_);

        src->PaintUniformColor(Eigen::Vector3d(1.0,0,0));
        tgt->PaintUniformColor(Eigen::Vector3d(0,0,1.0));

        open3d::DrawGeometries({src,tgt});

        tgt = src;
    }

}
