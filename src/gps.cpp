#include "session.h"
#include <boost/asio.hpp>
#include "config.h"
#include "utils.h"

int main(int argc, char ** argv){

    Config conf(argc, argv);

    boost::asio::io_service io;
    Session session(io, conf);
    if (!session.start()) {
        std::cout << "Failed to connect" << std::endl;
        return 1;
    }

    io.run();

    //std::thread gps_thread;
    //gps_thread = std::thread([&](){
    //        io.run();
    //});

}

