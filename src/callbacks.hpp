#include <librealsense2/rs.hpp> 
#include <iostream>
#include <map>
#include <chrono>
#include <mutex>
#include <thread>
#include "config.h"

#include <Core/Core.h>
#include <IO/IO.h>

using FrameCallback = std::function<void(rs2::frame)>; 

class Context {
    public:
    virtual void Lock() = 0;
    virtual void Unlock() = 0;
    virtual void PrintState() = 0;
    virtual void HandleIMU(const rs2::frame& frame) = 0;
    virtual void HandleImages(const rs2::frameset& frame) = 0;
};

FrameCallback CallBack(Context & ctx) {
    rs2::align align(RS2_STREAM_COLOR);
    int idx;
    return [&](const rs2::frame& frame){
        ctx.Lock();

        if (rs2::frameset fs = frame.as<rs2::frameset>()) {
            if(idx>30){
                //fs = align.process(fs); //Segfaults
                ctx.HandleImages(fs);
            }
            idx++;
        } else { 
            ctx.HandleIMU(frame); 
        }
        ctx.Unlock();
    };
}

class RecordContext : public Context {
    std::mutex * mtx;
    Config conf;
    int img_idx;

    
public:
    RecordContext(){
        std::mutex mutex; 
        mtx = &mutex;
    }

    RecordContext(std::mutex* mtx, Config & conf): mtx(mtx), conf(conf) { }

    RecordContext(const RecordContext &r) {
        mtx = r.mtx;
        conf = r.conf;
    }

    ~RecordContext() {}

    void Lock() { mtx->lock(); }
    void Unlock() { mtx->unlock(); }

    void PrintState() {
        Lock();
        Unlock();
    }

    //Gyro or Accel frame
    void HandleIMU(const rs2::frame & frame) {

        auto stype = frame.get_profile().stream_type();
        auto mvec = frame.as<rs2::motion_frame>().get_motion_data();

        if(stype == RS2_STREAM_GYRO){
            //TODO: Update pose
        }else{
            //TODO: Update pose
        }
    }

    // Color/Depth frames
    void HandleImages(const rs2::frameset & fs) {
        auto depth_image = std::make_shared<open3d::Image>();
        auto color_image = std::make_shared<open3d::Image>();

        depth_image->PrepareImage(conf.width, conf.height, 1, 2);
        color_image->PrepareImage(conf.width, conf.height, 3, 1);

        auto color_frame = fs.first(RS2_STREAM_COLOR);
        auto depth_frame = fs.get_depth_frame();	       

        if (!depth_frame || !color_frame) { return; }

        if(conf.use_filter){ depth_frame = conf.Filter(depth_frame); }

        memcpy(depth_image->data_.data(), depth_frame.get_data(), conf.width * conf.height * 2);
        memcpy(color_image->data_.data(), color_frame.get_data(), conf.width * conf.height * 3);

        open3d::WriteImage(conf.DepthFile(img_idx), *depth_image);
        open3d::WriteImage(conf.ColorFile(img_idx), *color_image);

        img_idx++;
    }
};


