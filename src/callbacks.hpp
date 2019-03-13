#include <librealsense2/rs.hpp> 
#include <iostream>
#include <map>
#include <chrono>
#include <mutex>
#include <thread>

using FrameCallback = std::function<void(rs2::frame)>; 

class Context {
    public:
    virtual void Lock() = 0;
    virtual void Unlock() = 0;
    virtual void AddStream(int sid, std::string sn) = 0;
    virtual void PrintState() = 0;
    virtual void HandleFrame(const rs2::frame& frame) = 0;
};

FrameCallback CallBack(Context & ctx) {
    return [&](const rs2::frame& frame){
        ctx.Lock();
        if (rs2::frameset fs = frame.as<rs2::frameset>())
        {
            for (const rs2::frame& f : fs) ctx.HandleFrame(f);
        } else { 
            ctx.HandleFrame(frame); 
        }
        ctx.Unlock();
    };
}

class RecordContext : public Context {
    std::mutex * mtx;

public:
    std::map<int, int> counters;
    std::map<int, std::string> stream_names;

    RecordContext(){
        std::mutex mutex; 
        mtx = &mutex;
    }

    RecordContext(std::mutex* mtx): mtx(mtx) { }

    RecordContext(const RecordContext &r) {
        mtx = r.mtx;
        counters = r.counters;
        stream_names = r.stream_names;
    }

    ~RecordContext() {}

    void Lock() { mtx->lock(); }
    void Unlock() { mtx->unlock(); }

    void AddStream(int sid, std::string sn){
        Lock();
        stream_names[sid] = sn; 
        Unlock();
    }

    void PrintState() {
        Lock();
        for (auto p : counters)
        {
            std::cout << stream_names[p.first] 
                << "[" << p.first << "]: " << p.second << " [frames] || " ;
        }
        std::cout << std::endl;
        Unlock();
    }

    void HandleFrame(const rs2::frame & frame) {
        counters[frame.get_profile().unique_id()]++;
    }
};


