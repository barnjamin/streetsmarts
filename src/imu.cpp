#include <string>
#include <fstream>
#include <iostream>

#include "utils.h"

int main(int argc, char * argv[])
{

    Config conf;
    conf.parseArgs(argc, argv);



    std::string str;
    std::fstream f;
    f.open(conf.imu_src);
    while (f >> str)
    {
        std::cout << str;
    }
    return 0;
}

