#include <Open3D/Open3D.h>
#include "config.h"
#include "fragments.hpp"

using namespace open3d;
using namespace open3d::utility;

int main(int argc, char * argv[])
{
    Config conf(argc, argv);

    Timer timer;
    timer.Start();

    for (int i = 0; i < conf.fragments; ++i) {
        PrintInfo("Processing fragment %d / %d\n", i, conf.fragments - 1);

        MakePoseGraphForFragmentTester(i, conf);
        OptimizePoseGraphForFragment(i, conf);
        IntegrateForFragment(i, conf);

    }

    timer.Stop();
    PrintInfo("MakeFragment takes %.3f s\n", timer.GetDuration() / 1000.0f);
}
