#include <Open3D/Open3D.h>
#include "config.h"
#include "fragments.hpp"

using namespace open3d;
using namespace open3d::utility;

int main(int argc, char * argv[])
{
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

    Timer timer;
    timer.Start();

    int fragments = conf.GetFragmentCount();
    for (int i = 0; i < fragments; ++i) {
        PrintInfo("Processing fragment %d / %d\n", i, fragments);

        //MakePoseGraphForFragment(i, conf);
        //OptimizePoseGraphForFragment(i, conf);
        MakePointCloudForFragment(i, conf);
        //IntegrateForFragment(i, conf);
    }

    timer.Stop();
    PrintInfo("MakeFragment takes %.3f s\n", timer.GetDuration() / 1000.0f);
}
