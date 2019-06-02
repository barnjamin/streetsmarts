
Mat K = (Mat_<double>(3, 3) << 610.0023193359375, 0.0, 425.36004638671875, 0.0, 609.85760498046875, 237.9273681640625, 0.0, 0.0, 1.0);

double sqr(double x) { return x * x; }

Eigen::Vector3d FastEigen3x3(const Eigen::Matrix3d &A) {
    // Based on:
    // https://en.wikipedia.org/wiki/Eigenvalue_algorithm#3.C3.973_matrices
    double p1 = sqr(A(0, 1)) + sqr(A(0, 2)) + sqr(A(1, 2));
    Eigen::Vector3d eigenvalues;
    if (p1 == 0.0) {
        eigenvalues(2) = std::min(A(0, 0), std::min(A(1, 1), A(2, 2)));
        eigenvalues(0) = std::max(A(0, 0), std::max(A(1, 1), A(2, 2)));
        eigenvalues(1) = A.trace() - eigenvalues(0) - eigenvalues(2);
    } else {
        double q = A.trace() / 3.0;
        double p2 = sqr((A(0, 0) - q)) + sqr(A(1, 1) - q) + sqr(A(2, 2) - q) +
                    2 * p1;
        double p = sqrt(p2 / 6.0);
        Eigen::Matrix3d B = (1.0 / p) * (A - q * Eigen::Matrix3d::Identity());
        double r = B.determinant() / 2.0;
        double phi;
        if (r <= -1) {
            phi = M_PI / 3.0;
        } else if (r >= 1) {
            phi = 0.0;
        } else {
            phi = std::acos(r) / 3.0;
        }
        eigenvalues(0) = q + 2.0 * p * std::cos(phi);
        eigenvalues(2) = q + 2.0 * p * std::cos(phi + 2.0 * M_PI / 3.0);
        eigenvalues(1) = q * 3.0 - eigenvalues(0) - eigenvalues(2);
    }

    Eigen::Vector3d eigenvector =
            (A - Eigen::Matrix3d::Identity() * eigenvalues(0)) *
            (A.col(0) - Eigen::Vector3d(eigenvalues(1), 0.0, 0.0));
    double len = eigenvector.norm();
    if (len == 0.0) {
        return Eigen::Vector3d::Zero();
    } else {
        return eigenvector.normalized();
    }
}

Eigen::Vector3d ComputeNormal(const Mat &depth, const std::vector<std::tuple<int,int>> &indices) {
    if (indices.size() == 0) {
        return Eigen::Vector3d::Zero();
    }

    Eigen::Matrix3d covariance;
    Eigen::Matrix<double, 9, 1> cumulants;
    cumulants.setZero();
    for (size_t i = 0; i < indices.size(); i++) {
        const Eigen::Vector3d &point = Eigen::Vector3d( (double) std::get<0>(indices[i]), (double)std::get<1>(indices[i]), depth.at<double>(std::get<0>(indices[i]), std::get<1>(indices[i])));
        cumulants(0) += point(0);
        cumulants(1) += point(1);
        cumulants(2) += point(2);
        cumulants(3) += point(0) * point(0);
        cumulants(4) += point(0) * point(1);
        cumulants(5) += point(0) * point(2);
        cumulants(6) += point(1) * point(1);
        cumulants(7) += point(1) * point(2);
        cumulants(8) += point(2) * point(2);
    }
    cumulants /= (double)indices.size();
    covariance(0, 0) = cumulants(3) - cumulants(0) * cumulants(0);
    covariance(1, 1) = cumulants(6) - cumulants(1) * cumulants(1);
    covariance(2, 2) = cumulants(8) - cumulants(2) * cumulants(2);
    covariance(0, 1) = cumulants(4) - cumulants(0) * cumulants(1);
    covariance(1, 0) = covariance(0, 1);
    covariance(0, 2) = cumulants(5) - cumulants(0) * cumulants(2);
    covariance(2, 0) = covariance(0, 2);
    covariance(1, 2) = cumulants(7) - cumulants(1) * cumulants(2);
    covariance(2, 1) = covariance(1, 2);

    return FastEigen3x3(covariance);
}

Mat EstimateDepthNormals(Mat depth) {
    Mat normals(depth.size(), CV_32FC3);
    int radius = 1;
    for(int x = 0; x < depth.rows; ++x) {
        for(int y = 0; y < depth.cols; ++y) {
            std::vector<std::tuple<int,int>> indices;

            Eigen::Vector3d normal;
            if (x > radius && depth.rows-x > radius && y > radius && depth.cols - y > radius) {
                for(int i = 1; i<=radius; i++){
                    indices.push_back(std::make_tuple(x-i,y));
                    indices.push_back(std::make_tuple(x+i,y));
                    indices.push_back(std::make_tuple(x,y-i));
                    indices.push_back(std::make_tuple(x,y+i));
                }

                normal = ComputeNormal(depth, indices);

                if (normal.norm() == 0.0) {
                    //normals.at<Vec3f>(x,y) = Vec3f(0,0,1);
                }

                normals.at<Vec3f>(x,y) = normalize(Vec3f(normal[0], normal[1], normal[2]));
            } else {
                //normals.at<Vec3f>(x,y) = Vec3f(0,0,1);
            }
        }
    }

    return normals;
}





