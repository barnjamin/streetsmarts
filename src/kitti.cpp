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


std::tuple<Eigen::Matrix4d, Eigen::Matrix6d>
    MultiScaleICP(const open3d::PointCloud &source, const open3d::PointCloud& target,
                    const Eigen::Matrix4d &init_trans,
                    const float voxel_size,
                    const std::vector<int> &iters = {50, 30, 14},
                    const std::vector<float> &voxel_factors = {1.0, 2.0, 4.0}) {

    using namespace open3d;

    assert(iters.size() == voxel_factors.size());

    Eigen::Matrix4d transformation = init_trans;
    Eigen::Matrix6d information = Eigen::Matrix6d::Identity();

    for (int i = 0; i < iters.size(); i++) {
        float voxel_size_level = voxel_size / voxel_factors[i];
        auto source_down = VoxelDownSample(source, voxel_size_level);
        auto target_down = VoxelDownSample(target, voxel_size_level);

        cuda::RegistrationCuda registration(TransformationEstimationType::PointToPoint);
        registration.Initialize(*source_down, *target_down, voxel_size_level * 1.4f, transformation);
        registration.ComputeICP(iters[i]);

        transformation = registration.transform_source_to_target_;

        if (i == iters.size() - 1) {
            information = registration.ComputeInformationMatrix();
        }
    }

    return std::make_tuple(transformation, information);
}

std::shared_ptr<open3d::PointCloud> readKittiVelodyne(std::string& fileName){
    std::shared_ptr<open3d::PointCloud> pc = std::make_shared<open3d::PointCloud>();

    std::ifstream input(fileName.c_str(), std::ios_base::binary);
    if(!input.good()){
        std::cerr<<"Cannot open file : "<<fileName<<std::endl;
        return pc;
    }

    float bbmax = 20.0;
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

    auto result = RemoveStatisticalOutliers(*pc, 50, 0.2);

    return std::get<0>(result);
}

std::tuple<double, double, double, Eigen::Quaterniond> get_lat_lng_quat(std::string gps_file) 
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

    std::getline(sline, segment, ' ');
    double roll = std::stod(segment);

    std::getline(sline, segment, ' ');
    double pitch = std::stod(segment);

    std::getline(sline, segment, ' ');
    double yaw = std::stod(segment);

    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX())
     *  Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
     *  Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ());

    return std::make_tuple(lat, lng, alt, q);
}

int main(int argc, char ** argv) 
{

    Config config(argc, argv);

    //std::string base_dir = "/home/ben/kitti-sync";
    std::string base_dir = "/home/ben/Documents/2011_09_26_drive_0015_sync";

    std::string gps_dir = base_dir + "/oxts/data";
    std::vector<std::string> gps;
    open3d::filesystem::ListFilesInDirectory(gps_dir, gps);
    sort(gps.begin(), gps.end());

    Eigen::Matrix4d trans_odometry = Eigen::Matrix4d::Identity();
    open3d::PoseGraph pose_graph;
    pose_graph.nodes_.emplace_back(open3d::PoseGraphNode(trans_odometry));

    const GeographicLib::Geocentric& earth = GeographicLib::Geocentric::WGS84();
    GeographicLib::LocalCartesian proj;

    Eigen::Matrix6d information;
    information << 
        614817.25, -2948.681884765625, 65493.2734375, 0.0, -393870.875, -2794.27001953125,
        -2948.681884765625, 675245.5625, 8518.068359375, 393870.875, 0.0, 27101.2109375,
        65493.2734375, 8518.068359375, 200426.1875, 2794.26806640625, -27101.2109375, 0.0,
        0.0, 393870.875, 2794.26806640625, 288427.0, 0.0, 0.0,
        -393870.875, 0.0, -27101.2109375, 0.0, 288427.0, 0.0,
        -2794.27001953125, 27101.2109375, 0.0, 0.0, 0.0, 288427.0 ;


    double lat, lng, alt;

    Eigen::Quaterniond q;
    Eigen::Quaterniond q_last;

    double x, y, z;
    double x_last, y_last, z_last;

    int idx = 0;
    for(std::string line: gps)
    {
        std::tie(lat, lng, alt, q) = get_lat_lng_quat(line);

        if(idx == 0){ 
            proj.Reset(lat, lng, alt);   
            q_last = q;
            idx++; 
            continue; 
        }

        proj.Forward(lat, lng, alt, x, y, z);

        Eigen::Translation3d trans_to_world(Eigen::Vector3d(x, y, z));
        Eigen::Transform<double, 3, Eigen::Affine>  world_trans = trans_to_world * q.normalized().toRotationMatrix();

        // Create transform between last 2
        Eigen::Translation3d translation_diff(Eigen::Vector3d(x-x_last, y-y_last, z-z_last));
        Eigen::Quaterniond q_diff = (q * q_last.inverse());
        Eigen::Transform<double, 3, Eigen::Affine> local_trans = translation_diff * q_diff.normalized().toRotationMatrix();
        
        pose_graph.nodes_.emplace_back(open3d::PoseGraphNode(world_trans.matrix().inverse()));
        pose_graph.edges_.emplace_back(open3d::PoseGraphEdge(idx - 1, idx, local_trans.matrix(), information, false));

        x_last = x;
        y_last = y;
        z_last = z;
        q_last = q;

        idx++;
    }

    WritePoseGraph(base_dir + "/gps_pg.json", pose_graph);

    std::shared_ptr<open3d::PointCloud> src;
    std::shared_ptr<open3d::PointCloud> tgt;

    std::string bin_dir = base_dir + "/velodyne_points/data";
    std::vector<std::string> bins;
    open3d::filesystem::ListFilesInDirectory(bin_dir, bins);
    sort(bins.begin(), bins.end());

    // Init vars 
    Eigen::Matrix4d trans_s_to_t;
    Eigen::Matrix6d info;
    Eigen::Matrix4d trans_world = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond nada(1.0, 0.0, 0.0, 0.0);
    std::shared_ptr<open3d::PointCloud>  pcd = std::make_shared<open3d::PointCloud>();

    // Create pg, add initial identity
    open3d::PoseGraph pg;
    pg.nodes_.emplace_back(open3d::PoseGraphNode(trans_world));

    for(int i=1; i<bins.size(); i++){
        src = readKittiVelodyne(bins[i-1]);
        tgt = readKittiVelodyne(bins[i]);

        open3d::PrintInfo("PointCloud %d to %d\n", i-1, i);

        // Get the world pose from gps posegraph
        Eigen::Transform<double, 3, Eigen::Affine> world_frame_pose(
                pose_graph.nodes_[i-1].pose_.inverse());

        // Get the local transform
        Eigen::Transform<double, 3, Eigen::Affine> world_frame_transform(
                pose_graph.edges_[i].transformation_);

        // Isolate the translation
        Eigen::Vector3d world_translation(world_frame_transform.translation());

        // Rotate the translation to the local frame 
        Eigen::Translation3d newtrans(
                world_frame_pose.rotation().inverse() * world_translation);

        // Construct seed transform in world 
        Eigen::Transform<double, 3, Eigen::Affine> seed = 
            newtrans * nada.normalized().toRotationMatrix();

        // Compute MultiScaleICP
        std::tie(trans_s_to_t, info) =  
                MultiScaleICP(*src, *tgt, seed.matrix(), 0.5);

        // Add inverse of source to target to local transfrom accumulator
        trans_world = trans_world * trans_s_to_t.inverse();

        // Add transforms to posegraph
        pg.nodes_.emplace_back(open3d::PoseGraphNode(trans_world.inverse()));
        pg.edges_.emplace_back(open3d::PoseGraphEdge(i-1, i, trans_s_to_t, info, false));

        // Apply local transform
        src->Transform(trans_world.inverse());

        // Add to pointcloud
        *pcd += *src;
    }
    
    open3d::WritePoseGraph("registered.json", pg);

    open3d::WritePointCloud("combined.pcd", *pcd);

    auto ds = open3d::VoxelDownSample(*pcd, 0.05);
    open3d::WritePointCloud("downsampled.pcd", *ds);
}
