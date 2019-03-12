// Define frame callback
// The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
// Therefore any modification to common memory should be done under lock

#include <librealsense2/rs.hpp> 
#include <iostream>
#include <map>
#include <chrono>
#include <mutex>
#include <thread>


class SaveToDisk {
public:
    static std::mutex mtx;
    std::map<int, int> counters;
    std::map<int, std::string> stream_names;

    SaveToDisk() {}
    ~SaveToDisk() {}

    void AddStream(int sid, std::string sn){
        std::lock_guard<std::mutex> lock(mtx);
        stream_names[sid] = sn; 
    }

    void PrintState() {
        std::lock_guard<std::mutex> lock(mtx);
        for (auto p : counters)
        {
            std::cout << stream_names[p.first] << "[" << p.first << "]: " << p.second << " [frames] || " << std::endl;
        }
    }

    void operator()(const rs2::frame& frame) {
        std::lock_guard<std::mutex> lock(mtx);
        if (rs2::frameset fs = frame.as<rs2::frameset>())
        {

            // With callbacks, all synchronized stream will arrive in a single frameset
            for (const rs2::frame& f : fs)
                counters[f.get_profile().unique_id()]++;

        } else {

            // Stream that bypass synchronization (such as IMU) will produce single frames
            counters[frame.get_profile().unique_id()]++;
        }
    }
};

std::mutex SaveToDisk::mtx;
