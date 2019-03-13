// Define frame callback
// The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
// Therefore any modification to common memory should be done under lock

#include <librealsense2/rs.hpp> 
#include <iostream>
#include <map>
#include <chrono>
#include <mutex>
#include <thread>

using FrameCallback = std::function<void(rs2::frame)>; 

class Record {
public:
    std::mutex * mtx;
    std::map<int, int> counters;
    std::map<int, std::string> stream_names;

    Record(){
        std::mutex mutex; 
        mtx = &mutex;
        std::cout<<"empty constructor called"<<std::endl;
    }

    Record(std::mutex* mtx): mtx(mtx) {
        std::cout<<"constructor called"<<std::endl;
    }

    Record(const Record &r) {
        std::cout<<"copy constructor called"<<std::endl;
        mtx = r.mtx;
        counters = r.counters;
        stream_names = r.stream_names;
    }

    ~Record() {}

    void AddStream(int sid, std::string sn){
        mtx->lock();
        stream_names[sid] = sn; 
        mtx->unlock();
    }

    void PrintState() {
        mtx->lock();
        for (auto p : counters)
        {
            std::cout << stream_names[p.first] 
                << "[" << p.first << "]: " << p.second << " [frames] || " ;
        }
        std::cout << std::endl;
        
        mtx->unlock();
    }
};

FrameCallback RecordCallBack(Record & r) {
    return [&](const rs2::frame& frame){
        r.mtx->lock();
        if (rs2::frameset fs = frame.as<rs2::frameset>())
        {

            // With callbacks, all synchronized stream will arrive in a single frameset
            for (const rs2::frame& f : fs)
                r.counters[f.get_profile().unique_id()]++;

        } else {

            // Stream that bypass synchronization (such as IMU) will produce single frames
            r.counters[frame.get_profile().unique_id()]++;
        }
        r.mtx->unlock();
    };
}
