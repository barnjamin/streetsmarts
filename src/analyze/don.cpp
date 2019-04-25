#include <Open3D/Open3D.h>
#include <vector>
#include <math.h>
#include "../config.h"
#include "../utils.h"

using namespace open3d;
using namespace open3d::io;
using namespace open3d::geometry;
using namespace open3d::visualization;
using namespace open3d::utility;

enum FilterType {WITHIN_RANGE, OUTSIDE_RANGE};
enum GroupType {EUCLIDIAN, GAUSSIAN};

std::shared_ptr<PointCloud> DifferenceOfNorm(PointCloud& pc, float smallr, float bigr, float low_thresh, float high_thresh, FilterType dt);
std::vector<std::shared_ptr<PointCloud>> Group(PointCloud& pc, GroupType gt);
std::shared_ptr<PointCloud> TrimLongitudinalAxis(PointCloud& pc, float width, int slices);

int main(int argc, char ** argv) 
{

    Config conf(argc, argv);

    auto pcd = std::make_shared<PointCloud>();
    if (ReadPointCloud(argv[1], *pcd)) {
        PrintWarning("Successfully read %s\n", argv[1]);
    } else {
        PrintError("Failed to read %s\n\n", argv[1]);
        return 1;
    }
    


    std::vector<std::shared_ptr<const Geometry>> geoms;

    //auto result = RemoveStatisticalOutliers(*pcd, 20, 0.05);
    //pcd = std::get<0>(result);

    pcd = VoxelDownSample(*pcd, conf.don_downsample);

    //auto trimmed_pc = TrimLongitudinalAxis(*pcd, 0.0, 200);
    //WritePointCloud("trimmed.pcd", *trimmed_pc);

    //Diff of Norms 
    auto DoN = DifferenceOfNorm(*pcd, 
            conf.don_small, conf.don_large, 
            conf.threshold_min, conf.threshold_max, 
            WITHIN_RANGE);

    geoms.push_back(DoN);

    Eigen::Matrix4d side_by_side;
    side_by_side << 1,0,0,10,
                    0,1,0,0,
                    0,0,1,0,
                    0,0,0,1;
    pcd->Transform(side_by_side);
    geoms.push_back(pcd);

    DrawGeometries(geoms);
    WritePointCloud("diff_of_norm.pcd", *DoN, false, true);

    return 0;
}

std::shared_ptr<PointCloud> DifferenceOfNorm(
    PointCloud& pc, float smallr, float bigr, float low_thresh, float high_thresh, FilterType dt) 
{
    auto small_pc = PointCloud(pc);
    auto big_pc = PointCloud(pc);

    EstimateNormals(small_pc, KDTreeSearchParamRadius(smallr));
    EstimateNormals(big_pc, KDTreeSearchParamRadius(bigr));

    small_pc.NormalizeNormals();
    big_pc.NormalizeNormals();

    SetGlobalColorMap(ColorMap::ColorMapOption::Jet);
    auto color_map_ptr = GetGlobalColorMap();

    //For each element in both normal arrays 
    std::vector<size_t> indicies;
    std::map<double, int> histo;

    Eigen::Vector3d avg_small;
    Eigen::Vector3d avg_large;

    for(size_t i=0; i<pc.points_.size(); i++){
        Eigen::Vector3d diff = (big_pc.normals_[i] - small_pc.normals_[i]) / 2.0;

        avg_small += small_pc.normals_[i];
        avg_large += big_pc.normals_[i];

        //auto delta  = pow(2, 10*diff.norm());
        auto delta  = diff.norm();
        double bucket = floorf(delta * 100)/100;
        if(histo.find(bucket) == histo.end()){
            histo.insert(std::make_pair(bucket, 0));
        }

        histo[bucket]++;

        if (i%100000 == 0 ) {
            std::cout << "Diff: " << delta << std::endl;
        }


        //big_pc.colors_[i] =  color_map_ptr->GetColor((delta+0.25)/2);
        big_pc.colors_[i] =  color_map_ptr->GetColor((delta+0.15));
    
        if(delta > low_thresh && delta < high_thresh) 
            indicies.push_back(i);
    }

    avg_small /= pc.points_.size();
    avg_large /= pc.points_.size();

    std::cout << avg_small << std::endl;
    std::cout << avg_large << std::endl;

    std::map<double, int>::iterator it = histo.begin();
    while(it != histo.end()){
        std::cout<<it->first<<" :: "<<it->second<<std::endl;
        it++;
    }
    if(dt == WITHIN_RANGE){
        return SelectDownSample(big_pc, indicies, false);
    }else{
        return SelectDownSample(big_pc, indicies, true);
    }
}

std::vector<std::shared_ptr<PointCloud>> Group(PointCloud& pc, GroupType gt) 
{
    //for (auto &c : cloud_ptr->colors_) {
    //    c = color_map_ptr->GetColor(index);
    //}

    return std::vector<std::shared_ptr<PointCloud>>();
}




std::shared_ptr<PointCloud> TrimLongitudinalAxis(PointCloud& pcd, float width, int slices) 
{

    Eigen::Vector3d max = pcd.GetMaxBound();
    Eigen::Vector3d min = pcd.GetMinBound();

    auto bb = LineSetFromBBox(min, max);

    //geoms.push_back(bb);

    double max_val = 0;
    int max_idx = 0;
    for(int i=0; i<3; i++){
        double delta = abs(max[i] - min[i]);
        if(delta > max_val){
            max_val = delta;
            max_idx = i;
        } 
    }

    std::cout << "Longitudinal axis distance: "<<max_val<< " idx: " <<max_idx<<std::endl;

    std::vector<std::shared_ptr<PointCloud>> pc_chunks;

    double diff = max_val/double(slices);
    for(int i=0; i<slices; i++) {

        Eigen::Vector3d chunk_min(min[0], min[1], min[2]);
        chunk_min[max_idx] = min[max_idx] + double(i) * diff ;

        Eigen::Vector3d chunk_max(max[0], max[1], max[2]);
        chunk_max[max_idx] = min[max_idx] + double(i+1)*diff;

        auto chunk_bb = LineSetFromBBox(chunk_min, chunk_max);

        pc_chunks.push_back(CropPointCloud(pcd, chunk_min, chunk_max));
        
        //geoms.push_back(chunk_bb);
    }
    
    std::shared_ptr<PointCloud> trimmed_pc = std::make_shared<PointCloud>();
    for(int i=0; i<pc_chunks.size(); i++){
        auto pc = pc_chunks[i];
        Eigen::Vector3d max = pc->GetMaxBound();
        Eigen::Vector3d min = pc->GetMinBound();

        auto gg = LineSetFromBBox(min, max);

        double max_val = 0;
        int max_idx = 0;
        for(int i=0; i<3; i++){
            double delta = abs(max[i] - min[i]);
            if(delta > max_val){
                max_val = delta;
                max_idx = i;
            } 
        }

        Eigen::Vector3d trim_min(min[0], min[1], min[2]);
        trim_min[max_idx] = min[max_idx] + width;

        Eigen::Vector3d trim_max(max[0], max[1], max[2]);
        trim_max[max_idx] = max[max_idx] - width;

        pc = CropPointCloud(*pc, trim_min, trim_max);
        *trimmed_pc += *pc;
    }

    return trimmed_pc;


}
