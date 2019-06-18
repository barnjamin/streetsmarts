#include "gps.h"
#include <unistd.h>
#include "config.h"

int main(int argc, char **argv){
    Config conf(argc, argv);
    conf.CreateLocalSession();
    GPS gps(conf);
    gps.Start();
    usleep(10000000);
    gps.Stop();
}
