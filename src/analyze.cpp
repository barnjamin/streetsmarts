#include <Core/Core.h>
#include <IO/IO.h>
#include <Visualization/Visualization.h>
#include <vector>


using namespace open3d;

enum FilterType { FLATNESS, ROUGHNESS };
enum GroupType {EUCLIDIAN, GAUSSIAN};

std::shared_ptr<PointCloud> DifferenceOfNorm(PointCloud& pc, float smallr, float bigr, float thresh, FilterType dt);
std::vector<std::shared_ptr<PointCloud>> Group(PointCloud& pc, GroupType gt);
std::shared_ptr<PointCloud> LoadPointCloud(std::string& filename) ;
void Visualize(std::shared_ptr<PointCloud> mesh) ;

int main(int argc, char ** argv) 
{
    //Load combined mesh or Stream
    std::string meshfile = "fragment-0.ply";

    //Load PCD
    auto pcd = LoadPointCloud(meshfile);

    //Diff of Norms 
    auto DoN = DifferenceOfNorm(*pcd, 0.03, 0.10, 0.99, ROUGHNESS);

    //Visualize the Downsampled points
    Visualize(DoN);

    //Group points
    std::vector<std::shared_ptr<PointCloud>> objs = Group(*DoN, EUCLIDIAN);

    //PointNet - Learn Features of Potholes and Cracks
    //Annotate Locations with Transformation to center of shape (Sphere, Cube) + Type

    return 0;
}

std::shared_ptr<PointCloud> DifferenceOfNorm(
    PointCloud& pc, float smallr, float bigr, float threshold, FilterType dt) 
{
    auto small_pc = PointCloud(pc);
    auto big_pc = PointCloud(pc);

    EstimateNormals(small_pc, KDTreeSearchParamRadius(smallr));

    EstimateNormals(big_pc, KDTreeSearchParamRadius(bigr));

    //For each element in both normal arrays 
    std::vector<size_t> indicies;
    for(size_t i=0; i<pc.points_.size(); i++){
        if(dt == FLATNESS && abs(small_pc.normals_[i].dot(big_pc.normals_[i])) > threshold){
            indicies.push_back(i);
        }else if(dt == ROUGHNESS && abs(small_pc.normals_[i].dot(big_pc.normals_[i])) < threshold) {
            indicies.push_back(i);
        }

    }

    return SelectDownSample(pc, indicies, false);
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
    ReadPointCloud(filename, *pc, "ply");
    return pc;
}


//void Visualize(PointCloud &mesh, std::vector<std::shared_ptr<PointCloud>> pcs) 
void Visualize(std::shared_ptr<PointCloud> mesh) 
{
    DrawGeometries({mesh});
}

