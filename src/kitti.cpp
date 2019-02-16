#include <iostream>
#include <Visualization/Visualization.h>
#include <Core/Core.h>
#include <IO/IO.h>

std::shared_ptr<open3d::PointCloud> readKittiVelodyne(std::string& fileName){
    std::shared_ptr<open3d::PointCloud> pc = std::make_shared<open3d::PointCloud>();

    std::ifstream input(fileName.c_str(), std::ios_base::binary);
    if(!input.good()){
        std::cerr<<"Cannot open file : "<<fileName<<std::endl;
        return pc;
    }

    for (int iter=0; input.good() && !input.eof(); iter++) {
        float x,y,z;
        float i;

        input.read((char *) &x, sizeof(float));
        input.read((char *) &y, sizeof(float));
        input.read((char *) &z, sizeof(float));
        input.read((char *) &i, sizeof(float));

        pc->points_.push_back(Eigen::Vector3d(x,y,z));
        pc->colors_.push_back(Eigen::Vector3d(i, i, i));
    }
    input.close();

    return pc;
}

inline std::tuple<float, float, float> Project3DPointAndGetUVDepth(
        const Eigen::Vector3d P,
        const open3d::PinholeCameraIntrinsic& intr, 
        const Eigen::Matrix4d& extr) {

    std::pair<double, double> f = intr.GetFocalLength();
    std::pair<double, double> p = intr.GetPrincipalPoint();

    Eigen::Vector4d Vt = extr * Eigen::Vector4d(P(0), P(1), P(2), 1);


    float u = float((Vt(0) * f.first) / Vt(2) + p.first);
    float v = float((Vt(1) * f.second) / Vt(2) + p.second);
    float z = float(Vt(2));

    return std::make_tuple(u, v, z);
}

std::shared_ptr<open3d::Image> PointCloudToImage(open3d::PointCloud& pcd) {

    auto img = std::make_shared<open3d::Image>(); 
    img->PrepareImage(1241, 376, 1, 1);

    open3d::PinholeCameraIntrinsic kintr(1241, 376, 978, 971, 690, 249);

    Eigen::Matrix4d extr;
    extr << 4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -1.198459927713e-02,
            -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, -5.403984729748e-02,
            9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01,
            0, 0, 0, 1;

    //extr << 7.027555e-03, -9.999753e-01, 2.599616e-05, -7.137748e-03,
    //        -2.254837e-03, -4.184312e-05, -9.999975e-01, -7.482656e-02,
    //        9.999728e-01, 7.027479e-03, -2.255075e-03, -3.336324e-01,
    //        0,0,0,1;
            

    float u,v,z;
    int max = 0;
    int min = 100;
    std::vector<uint8_t> vals(1241*376);
    for(auto const& p: pcd.points_) {

        std::tie(u,v,z) = Project3DPointAndGetUVDepth(p, kintr, extr.transpose());     
        //If its outside the cameras viewport, skip it
        if( u<= -690 || v <= -249 || z <= 0 || u >= 690 || v >= 249 ){
            continue;
        }
        //u = 690
        

        //std::cout << "u " << u << " v " << v << " z " << z << std::endl;

        int idx = int(u) * int(v);
        uint8_t col = uint8_t(z*10);
        if(col>max){ max = col;}
        if(col<min){ min = col;}

        img->data_[idx] = int(z/11)*255;
        //vals[idx] = col;
    }

    std::cout << max << " " << min << std::endl;
     
    //for(int i=0; i<vals.size(); i++){
    //    if(vals[i]>0){
    //        auto v = ((float(vals[i]-min)/float(max))/10.0);
    //        //std::cout<< v << std::endl;
    //        img->data_[i] = int(v);
    //    }
    //}

    return img;
}

int main(int argc, char ** argv) 
{
    std::string binf = "/home/ben/kitti/velodata/sequences/00/velodyne/000239.bin";

    auto pcd = readKittiVelodyne(binf);

    Eigen::Matrix4d extr;

    open3d::DrawGeometries({pcd});

    auto img = PointCloudToImage(*pcd);

    open3d::WriteImage("blah.png", *img, 10);

    //open3d::DrawGeometries({img});
}
