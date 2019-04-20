#include <vector>
#include <string>
#include <Open3D/Open3D.h>


int main(int argc, char **argv)
{

    using namespace open3d;


    int verbose = utility::GetProgramOptionAsInt(argc, argv, "--verbose", 2);
    utility::SetVerbosityLevel((utility::VerbosityLevel)verbose);
    double voxel_size =
            utility::GetProgramOptionAsDouble(argc, argv, "--voxel_size", -1.0);
    bool with_dialog =
            !utility::ProgramOptionExists(argc, argv, "--without_dialog");

    visualization::VisualizerWithEditing vis(
            voxel_size, with_dialog,
            utility::filesystem::GetFileParentDirectory(argv[1]));

    vis.CreateVisualizerWindow("Crop Point Cloud", 1920, 1080, 100, 100);

    if (utility::ProgramOptionExists(argc, argv, "--pointcloud")) {
        auto pcd_ptr = io::CreatePointCloudFromFile(argv[2]);
        if (pcd_ptr->IsEmpty()) {
            utility::PrintWarning("Failed to read the point cloud.\n");
            return 0;
        }
        vis.AddGeometry(pcd_ptr);
        if (pcd_ptr->points_.size() > 5000000) {
            vis.GetRenderOption().point_size_ = 1.0;
        }
    } else if (utility::ProgramOptionExists(argc, argv, "--mesh")) {
        auto mesh_ptr = io::CreateMeshFromFile(argv[2]);
        if (mesh_ptr->IsEmpty()) {
            utility::PrintWarning("Failed to read the mesh.\n");
            return 0;
        }
        vis.AddGeometry(mesh_ptr);
    }
    vis.Run();
    vis.DestroyVisualizerWindow();
    return 1;
}
