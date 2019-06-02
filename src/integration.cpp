#include <vector>
#include <string>

#include <Open3D/Open3D.h>
#include <Open3D/Cuda/Open3DCuda.h>

#include "config.h"
#include "utils.h"
#include "integrate.hpp"

int main(int argc, char * argv[]) {
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

    //IntegrateScene(conf);
    IntegrateSceneFromFullPG(conf);
    //IntegrateSceneCPU(conf);
    //MakePointCloudForScene(conf);
}
