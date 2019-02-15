#include <Core/Core.h>
#include <IO/IO.h>
#include <Visualization/Visualization.h>
#include <vector>
#include <math.h>


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
    std::string pcd_file = "/home/ben/data.pcd";

    //Load PCD
    auto pcd = LoadPointCloud(pcd_file);

    pcd = VoxelDownSample(*pcd, 0.02);

    //Diff of Norms 
    auto DoN = DifferenceOfNorm(*pcd, 0.03, 0.15, 0.85, FLATNESS);

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

    small_pc.NormalizeNormals();
    big_pc.NormalizeNormals();

    //For each element in both normal arrays 
    std::vector<size_t> indicies;
    for(size_t i=0; i<pc.points_.size(); i++){
        Eigen::Vector3d diff = (small_pc.normals_[i] - big_pc.normals_[i]) / 2.0;

        std::cout<< diff.norm() << std::endl;

        //pc.colors_[i] = Eigen::Vector3d(diff[0], diff[1], diff[2]); 

        if(dt == FLATNESS &&  diff.norm() > threshold){
            indicies.push_back(i);
        }else if(dt == ROUGHNESS && diff.norm() < threshold) {
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
    ReadPointCloud(filename, *pc, "pcd");
    return pc;
}


//void Visualize(PointCloud &mesh, std::vector<std::shared_ptr<PointCloud>> pcs) 
void Visualize(std::shared_ptr<PointCloud> mesh) 
{
    DrawGeometries({mesh});
}

