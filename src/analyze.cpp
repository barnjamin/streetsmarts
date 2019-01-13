#include <Core/Core.h>
#include <IO/IO.h>
#include <Visualization/Visualization.h>
#include <vector>


using namespace open3d;

enum FilterType { FLATNESS, ROUGHNESS };
enum GroupType {EUCLIDIAN, GAUSSIAN};


std::shared_ptr<PointCloud> DifferenceOfNorm(PointCloud& pc, int smallr, int bigr, FilterType dt);
std::vector<std::shared_ptr<PointCloud>> Group(PointCloud& pc, GroupType gt);
std::shared_ptr<PointCloud> LoadPointCloud(std::string& filename) ;
void Visualize(std::shared_ptr<PointCloud> mesh) ;
//void Visualize(PointCloud &mesh, std::vector<std::shared_ptr<PointCloud>> pcs) ;

int main(int argc, char ** argv) 
{
    //Load combined mesh or Stream
    std::string meshfile = "fragment-0.ply";

    auto pcd = LoadPointCloud(meshfile);

    Visualize(pcd); //, std::vector<std::shared_ptr<PointCloud>>());
    

    //Diff of Norms 
    auto DoN = DifferenceOfNorm(*pcd, 0.01, 0.05, 0.5, FLATNESS);

    //Group points
    std::vector<std::shared_ptr<PointCloud>> objs = Group(*DoN, EUCLIDIAN);

    //PointNet - Learn Features of Potholes and Cracks

    //Annotate Locations with Transformation to center of shape (Sphere, Cube) + Type

    return 0;
}

std::shared_ptr<PointCloud> DifferenceOfNorm(
    PointCloud& pc, float smallr, float bigr, float threshold, DifferenceType dt) 
{
    auto small_pc = pc.Clone();
    auto big_pc = pc.Clone();

    EstimateNormals(small_pc, KDTreeSearchParamRadius(smallr));
    EstimateNormals(big_pc, KDTreeSearchParamRadius(bigr));

    //For each element in both normal arrays 
    std::vector<size_t> indicies;
    for(size_t i=0; i<pc.points_.size(); i++){
        //TODO:: think the normals are 1x3 vecs? need euclid dist not just sub
        // Need dot product
        if(dt == FLATNESS && small_pc.normals[i] - big_pc.normals[i] > threshold){
            indicies.push_back(i);
        }else if(dt == ROUGHNESS && small_pc.normals[i] - big_pc.normals[i] < threshold) {
            indicies.push_back(i);
        }
    }

    auto pc_out = std::make_shared<PointCloud>();

    return pc_out;
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
    //TODO:: Combine? 
    auto pc = std::make_shared<PointCloud>();

    ReadPointCloud(filename, *pc, "ply");

    return pc;
}


//void Visualize(PointCloud &mesh, std::vector<std::shared_ptr<PointCloud>> pcs) 
void Visualize(std::shared_ptr<PointCloud> mesh) 
{
    DrawGeometries({mesh});
}

