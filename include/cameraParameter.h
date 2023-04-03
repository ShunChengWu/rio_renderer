//
// Created by sc on 3/31/21.
//
#ifndef _H_PSLAM_CAMERA_PARAMETERS
#define _H_PSLAM_CAMERA_PARAMETERS
#ifdef COMPILE_WITH_EIGEN
#include <Eigen/Core>
#endif
#ifdef COMPILE_WITH_OPENCV
#include <opencv2/core.hpp>
#endif
#include <fstream>

namespace PSLAM {
    struct CameraParameters {
        float cx, cy;
        float fx, fy;
        unsigned short width, height;
        float scale=1.f;
        std::vector<float> distortions;

#ifdef COMPILE_WITH_EIGEN
        Eigen::Vector4f camParam;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
        void SetK(float fx_,float fy_,float cx_,float cy_){
            this->fx = fx_;
            this->fy = fy_;
            this->cx = cx_;
            this->cy = cy_;
#ifdef COMPILE_WITH_EIGEN
            camParam << fx_, fy_, cx_, cy_;
#endif
        }

        void Set(unsigned int width_, unsigned int height_, float fx_, float fy_,
                 float cx_, float cy_, float scale_=1.f, const std::vector<float> *distortion=nullptr) {
            SetK(fx_,fy_,cx_,cy_);
            this->width = width_;
            this->height = height_;
            this->scale = scale_;
            if(distortion) {
                distortions = *distortion;
            }
        }

        [[maybe_unused]] [[nodiscard]] Eigen::Matrix3f GetK() const {
            Eigen::Matrix3f K;
            K(0,0) = fx;
            K(1,1) = fy;
            K(0,2) = cx;
            K(1,2) = cy;
            K(2,2) = 1;
            return K;
        }

#ifdef COMPILE_WITH_OPENCV
        [[nodiscard]] cv::Mat GetcvK() const {
            cv::Mat K = cv::Mat::eye(3,3,CV_32F);
            K.at<float>(0,0) = fx;
            K.at<float>(1,1) = fy;
            K.at<float>(0,2) = cx;
            K.at<float>(1,2) = cy;
            return K;
        }
        [[nodiscard]] cv::Mat GetcvDistCoef() const {
            cv::Mat DistCoef(distortions.size(),1,CV_32F);
            for(size_t d=0;d<distortions.size();++d)
                DistCoef.at<float>(d) = distortions[d];
            return DistCoef;
        }
#endif


        void SaveParams(const std::string &path) const {
            std::fstream file (path, std::ios::out);
            if(!file.is_open()){
                char buffer[1024];
                sprintf(buffer, "Unable to open a file for saving camera parameters  at location %s\n",path.c_str());
                throw std::runtime_error(buffer);
            }
            file << this->width << " " << this->height << std::endl;
            file << this->fx << " " << this->fy << std::endl;
            file << this->cx << " " << this->cy << std::endl;
            file << this->scale << std::endl;
            file << distortions.size();
            for (auto d : distortions)
                file << " " << d;
        }

        void LoadParams(const std::string &path) {
            std::fstream file (path, std::ios::in);
            if(!file.is_open()){
                char buffer[1024];
                sprintf(buffer, "Unable to open a file for saving camera parameters  at location %s\n",path.c_str());
                throw std::runtime_error(buffer);
            }
            file >> this->width >> this->height;
            file >> this->fx >> this->fy;
            file >> this->cx >> this->cy;
            file >> this->scale;
            int size;
            file >> size;
            distortions.resize(size);
            for(int i=0;i<size;++i)
                file >> distortions[i];
        }

        friend std::ostream & operator << (std::ostream &os, const CameraParameters &param) {
            os << "width height: " << param.width << " " << param.height << "\n";
            os << "intrinsics: " << param.fx << " " << param.fy << " " << param.cx << " " << param.cy << "\n";
            os << "distortion coeff.:";
            for (auto d : param.distortions)
                os << " " << d;
            os << "\n";
            os << "scale: " << param.scale;
            return os;
        }
    };
}
#endif
