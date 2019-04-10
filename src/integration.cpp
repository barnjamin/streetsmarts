#include <vector>
#include <string>

#include <Open3D/Open3D.h>
#include <Cuda/Open3DCuda.h>

#include "config.h"
#include "utils.h"
#include "integrate.hpp"

using namespace open3d;
using namespace open3d::cuda;
using namespace open3d::geometry;
using namespace open3d::integration;
using namespace open3d::registration;
using namespace open3d::utility;
using namespace open3d::camera;
using namespace open3d::io;

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

    IntegrateScene(conf);
}
