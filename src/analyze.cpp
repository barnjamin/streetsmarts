#include <Core/Core.h>
#include <IO/IO.h>
#include <Visualization/Visualization.h>
#include <vector>
#include <math.h>

using namespace open3d;

enum FilterType {WITHIN_RANGE, OUTSIDE_RANGE};
enum GroupType {EUCLIDIAN, GAUSSIAN};

std::shared_ptr<PointCloud> DifferenceOfNorm(PointCloud& pc, float smallr, float bigr, float low_thresh, float high_thresh, FilterType dt);
std::vector<std::shared_ptr<PointCloud>> Group(PointCloud& pc, GroupType gt);
std::shared_ptr<PointCloud> LoadPointCloud(std::string& filename) ;
std::shared_ptr<LineSet> LineSetFromBBox(Eigen::Vector3d min, Eigen::Vector3d max);
void Visualize(std::shared_ptr<PointCloud> mesh) ;

int main(int argc, char ** argv) 
{
    auto pcd = std::make_shared<PointCloud>();
    if (ReadPointCloud(argv[1], *pcd)) {
        PrintWarning("Successfully read %s\n", argv[1]);
    } else {
        PrintError("Failed to read %s\n\n", argv[1]);
        return 1;
    }
    

    std::vector<std::shared_ptr<const Geometry>> geoms;

    pcd = VoxelDownSample(*pcd, 0.02);

    geoms.push_back(pcd);

    Eigen::Vector3d max = pcd->GetMaxBound();
    Eigen::Vector3d min = pcd->GetMinBound();

    auto bb = LineSetFromBBox(min, max);

    geoms.push_back(bb);

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

    int chunks = 500;
    double diff = max_val/double(chunks);
    for(int i=0; i<chunks; i++) {

        Eigen::Vector3d chunk_min(min[0], min[1], min[2]);
        chunk_min[max_idx] = min[max_idx] + double(i) * diff ;

        Eigen::Vector3d chunk_max(max[0], max[1], max[2]);
        chunk_max[max_idx] = min[max_idx] + double(i+1)*diff;

        auto chunk_bb = LineSetFromBBox(chunk_min, chunk_max);

        pc_chunks.push_back(CropPointCloud(*pcd, chunk_min, chunk_max));
        
        geoms.push_back(chunk_bb);
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
        trim_min[max_idx] = min[max_idx] + 0.8;

        Eigen::Vector3d trim_max(max[0], max[1], max[2]);
        trim_max[max_idx] = max[max_idx] - 0.8;

        pc = CropPointCloud(*pc, trim_min, trim_max);
        *trimmed_pc += *pc;
    }

    std::cout << "Finished trimming"<<std::endl;

    //DrawGeometries(geoms);

    //Diff of Norms 
    auto DoN = DifferenceOfNorm(*trimmed_pc, 0.03, 0.5, 0.025, 0.2, WITHIN_RANGE);


    Eigen::Matrix4d side_by_side;
    side_by_side << 1,0,0,10,
                    0,1,0,0,
                    0,0,1,0,
                    0,0,0,1;
    trimmed_pc->Transform(side_by_side);

    WritePointCloud("downsampled.pcd", *DoN);

    DrawGeometries({trimmed_pc, DoN});

    //Group points
    std::vector<std::shared_ptr<PointCloud>> objs = Group(*DoN, EUCLIDIAN);

    //PointNet - Learn Features of Potholes and Cracks
    //Annotate Locations with Transformation to center of shape (Sphere, Cube) + Type

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

        big_pc.colors_[i] =  color_map_ptr->GetColor(delta*10);
    
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

std::shared_ptr<PointCloud> LoadPointCloud(std::string& filename) 
{
    auto pc = std::make_shared<PointCloud>();
    ReadPointCloud(filename, *pc, "pcd");
    return pc;
}


//void Visualize(PointCloud &mesh, std::vector<std::shared_ptr<PointCloud>> pcs) 
void Visualize(std::shared_ptr<PointCloud> mesh) 
{
}

std::shared_ptr<LineSet> LineSetFromBBox(Eigen::Vector3d min, Eigen::Vector3d max){
    double maxx = max[0];
    double maxy = max[1];
    double maxz = max[2];

    double minx = min[0];
    double miny = min[1];
    double minz = min[2];

    std::shared_ptr<LineSet> bb = std::make_shared<LineSet>();

    bb->points_.push_back(Eigen::Vector3d(minx,miny,minz));
    bb->points_.push_back(Eigen::Vector3d(maxx,miny,minz));
    bb->points_.push_back(Eigen::Vector3d(minx,maxy,minz));
    bb->points_.push_back(Eigen::Vector3d(maxx,maxy,minz));
    bb->points_.push_back(Eigen::Vector3d(minx,miny,maxz));
    bb->points_.push_back(Eigen::Vector3d(maxx,miny,maxz));
    bb->points_.push_back(Eigen::Vector3d(minx,maxy,maxz));
    bb->points_.push_back(Eigen::Vector3d(maxx,maxy,maxz));
    
    bb->lines_.push_back(Eigen::Vector2i(0,1));
    bb->lines_.push_back(Eigen::Vector2i(0,2));
    bb->lines_.push_back(Eigen::Vector2i(1,3));
    bb->lines_.push_back(Eigen::Vector2i(2,3));
    bb->lines_.push_back(Eigen::Vector2i(4,5));
    bb->lines_.push_back(Eigen::Vector2i(4,6));
    bb->lines_.push_back(Eigen::Vector2i(5,7));
    bb->lines_.push_back(Eigen::Vector2i(6,7));
    bb->lines_.push_back(Eigen::Vector2i(0,4));
    bb->lines_.push_back(Eigen::Vector2i(1,5));
    bb->lines_.push_back(Eigen::Vector2i(2,6));
    bb->lines_.push_back(Eigen::Vector2i(3,7));

    for(int i=0; i<bb->lines_.size(); i++) {
        bb->colors_.push_back(Eigen::Vector3d(255,0,0));
    }

    return bb;
}
