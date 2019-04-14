#include <Open3D/Open3D.h>
#include "config.h"
#include "fragments.hpp"

using namespace open3d;
using namespace open3d::utility;

int main(int argc, char * argv[])
{
    Config conf;
    
    if(argc==2){
        // Assume json
        std::string config_path = argv[1];
        if(!open3d::io::ReadIJsonConvertible(config_path, conf)) {
            open3d::utility::PrintError("Failed to read config\n");
            return 1;
        }
    }else{
        conf = Config(argc, argv);
    }

    Timer timer;
    timer.Start();


    MakePoseGraphForSession(conf);

    OptimizePoseGraphForSession(conf);

    //IntegrateForFragment(i, conf);

    timer.Stop();
    PrintInfo("MakeFragment takes %.3f s\n", timer.GetDuration() / 1000.0f);
}
