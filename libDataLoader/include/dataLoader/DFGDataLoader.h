#pragma once

#include <cstdio>
#include <cmath>
#include <memory>
#include <string>
#include <sstream>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <PathTool.hpp>
#include <utility>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <Logging.h>
#include <cameraParameter.h>

namespace PSLAM {
    enum MULTICON_DATATYPES {
        MULTICON_UCHAR,
        MULTICON_UCHAR3,
        MULTICON_USHORT
    };

    class DataBuffer {
    public:
        DataBuffer(MULTICON_DATATYPES datatypes, unsigned long size):m_dtype(datatypes), m_size(size){
            switch (m_dtype){
                case MULTICON_UCHAR:
                    buffer = new unsigned char[size];
                    break;
                case MULTICON_UCHAR3:
                    buffer = new unsigned char[size*3];
                    break;
                case MULTICON_USHORT:
                    buffer = new unsigned short[size];
                    break;
                default:
                    throw std::runtime_error("unimplemented\n");
            }
        }
        ~DataBuffer(){
            switch (m_dtype){
                case MULTICON_UCHAR:
                    delete static_cast<unsigned char*>(buffer);
                    break;
                case MULTICON_UCHAR3:
                    delete static_cast<unsigned char*>(buffer);
                    break;
                case MULTICON_USHORT:
                    delete static_cast<unsigned short*>(buffer);
                    break;
                default:
                    ;
                    //throw std::runtime_error("unimplemented\n");
            }
        }

        void* data(){return buffer;}

        unsigned long GetDataSize(){return m_size;}
        unsigned long GetByteSize(){
            switch (m_dtype){
                case MULTICON_UCHAR:
                    return sizeof(unsigned char)*m_size;
                case MULTICON_UCHAR3:
                    return sizeof(unsigned char)*m_size*3;
                    break;
                case MULTICON_USHORT:
                    return sizeof(unsigned short)*m_size;
                    break;
                default:
                    throw std::runtime_error("unimplemented\n");
            }
        }

    private:
        void *buffer;
        MULTICON_DATATYPES m_dtype;
        unsigned long m_size;
    };

    class MultiConLoader{
        struct Pose {
            int idx;
            double time;
            bool occluded;
            float t[3];
            float r[4];

            friend  bool operator <(const Pose &left, const Pose &right){
                return left.idx < right.idx;
            }
        };
    public:
        explicit MultiConLoader(std::string folder, const std::vector<std::string> &logNames = {"lola_vision"})
        : m_ImageCounter(0), m_folder(std::move(folder)), m_logNames(std::move(logNames)){
            m_imageFolder = m_folder + _m_image_folder;
            m_depthFolder = m_imageFolder + _m_depthFolderName;
            m_rgbFolder = m_imageFolder + _m_rgbFolderName;
            m_logFolder = m_folder + _m_log_folder;
            // Check folder exist
            auto checkFolderExistance=[](const std::string &pth){
                if(!tools::PathTool::checkfolderexist(pth))
                    SCLOG(ERROR) << "input folder does not exist " << pth;
            };
            checkFolderExistance(m_folder);
            checkFolderExistance(m_imageFolder);
            checkFolderExistance(m_depthFolder);
            checkFolderExistance(m_rgbFolder);
            checkFolderExistance(m_logFolder);

            loadIntrinsics(m_depthParam, m_imageFolder + "cparam_d.txt");
            loadIntrinsics(m_rgbParam, m_imageFolder + "cparam_rgb.txt");
            // create buffers
            m_depthBuffer = std::make_unique<DataBuffer>(MULTICON_USHORT,m_depthParam.width*m_depthParam.height);
            m_rgbBuffer = std::make_unique<DataBuffer>(MULTICON_UCHAR3,m_rgbParam.width*m_rgbParam.height);

            // Check image numbers are the same
            m_depthImages = tools::PathTool::get_files_in_folder(m_depthFolder,"",true,true);
            m_rgbImages   = tools::PathTool::get_files_in_folder(m_rgbFolder,"",true,true);
            if(m_depthImages.size()!=m_rgbImages.size())
                SCLOG(ERROR) << "the number of dpeth and rgb images does not match";

            if(m_depthImages.empty() || m_rgbImages.empty())
                throw std::runtime_error("Cannot find any images.\n");

            ReadBinary(m_depthBuffer->data(),m_depthImages.front(), m_depthBuffer->GetByteSize());
            ReadBinary(m_rgbBuffer->data(),m_rgbImages.front(), m_rgbBuffer->GetByteSize());
            cv::Mat depthImg(m_depthParam.height,m_depthParam.width, CV_16U, m_depthBuffer->data());
            cv::Mat rgbImg(m_rgbParam.height, m_rgbParam.width, CV_8UC3, m_rgbBuffer->data());
            const std::pair<int,int> depthSize = {depthImg.cols, depthImg.rows};
            const std::pair<int,int> rgbSize = {rgbImg.cols, rgbImg.rows};

            if(depthSize.first != m_depthParam.width || depthSize.second != m_depthParam.height ||
               rgbSize.first != m_rgbParam.width || rgbSize.second != m_rgbParam.height){
                SCLOG(ERROR) << "the dimension of depth and rgb images mismatch.";
            }

            // Check log files
            mbHasPose=true;
            for (const std::string &s : m_logNames) {
                auto &log = m_logs[s];
                if(!loadLogFile(log,m_logFolder + s + ".txt"))
                    mbHasPose=false;
            }
        }

        int Next() {
            return m_ImageCounter+1 < m_depthImages.size()? (int)m_ImageCounter++ : -1;
        }

        [[maybe_unused]] void* GetDepthImageRaw(int idx) {
            ReadBinary(m_depthBuffer->data(),m_depthImages[idx], m_depthBuffer->GetByteSize());
            return m_depthBuffer->data();
        }

        [[maybe_unused]] void* GetRGBImageRaw(int idx) {
            ReadBinary(m_rgbBuffer->data(),m_rgbImages[idx], m_rgbBuffer->GetByteSize());
            return m_rgbBuffer->data();
        }

        [[maybe_unused]] cv::Mat GetDepthImage(int idx) {
            GetDepthImageRaw(idx);
            return cv::Mat (m_depthParam.height,m_depthParam.width, CV_16U, m_depthBuffer->data());
        }
        [[maybe_unused]] cv::Mat GetRGBImage(int idx) {
            GetRGBImageRaw(idx);
            return cv::Mat (m_rgbParam.height, m_rgbParam.width, CV_8UC3, m_rgbBuffer->data());
        }

        [[maybe_unused]] const float* GetPose(int idx, const std::string &name = "lola_vision"){
            std::vector<std::shared_ptr<Pose>> *log = nullptr;
            try {
                log = &m_logs.at(name);
            } catch (const std::out_of_range &error) {
                std::stringstream ss;
                ss << "Try to load log with name " << name << " but it did not exist. lognames: ";
                for (const auto &pair: m_logs)
                    ss << pair.first << ", ";
                SCLOG(ERROR) << ss.str();
            }

            if(log->empty()) return nullptr;
            std::shared_ptr<Pose> pose = log->at(idx);
            if (pose->occluded) return nullptr;
            Eigen::Quaternionf quaternionf;
            quaternionf.x() = pose->r[0];
            quaternionf.y() = pose->r[1];
            quaternionf.z() = pose->r[2];
            quaternionf.w() = pose->r[3];

            Eigen::Matrix4f rotMat = Eigen::Matrix4f::Identity();
            rotMat.topLeftCorner<3,3>() = quaternionf.toRotationMatrix();
            rotMat.topRightCorner<3,1>() = Eigen::Vector3f(pose->t[0],pose->t[1],pose->t[2]);
            memcpy(m_poseBuffer,rotMat.data(), sizeof(float)*16);
            return m_poseBuffer;
        }

        [[maybe_unused]] std::pair<int,int> GetDepthSize(){return {m_depthParam.width, m_depthParam.height};}
        [[maybe_unused]] std::pair<int,int> GetRGBSize(){return {m_rgbParam.width, m_rgbParam.height};}

        [[maybe_unused]] void GetDepthParam(float &fx, float &fy, float &px, float &py, float &scale){
            getParam(m_depthParam,fx,fy,px,py, scale);
        }

        [[maybe_unused]] void GetRGBParam(float &fx, float &fy, float &px, float &py, float &scale){
            getParam(m_rgbParam,fx,fy,px,py,scale);
        }

        [[maybe_unused]] void GetDepthParam(int &width,int &height, float &fx, float &fy, float &px, float &py, float &scale){
            getParam(m_depthParam,width,height,fx,fy,px,py,scale);
        }

        [[maybe_unused]] void GetRGBParam(int &width,int &height, float &fx, float &fy, float &px, float &py, float &scale){
            getParam(m_rgbParam,width,height,fx,fy,px,py,scale);
        }

        [[maybe_unused]] float GetDepthScale(){return m_depthScale;}

        int GetImageNumber(){
            return m_depthImages.size();
        }

        std::vector<std::string> m_depthImages;
    private:
        size_t m_ImageCounter;
        std::string m_folder;
        const std::string _m_image_folder = "/images/";
        const std::string _m_log_folder = "/logs/";
        const std::string _m_depthFolderName = "depth/";
        const std::string _m_rgbFolderName = "rgb/";
        const std::vector<std::string> m_logNames;
        std::string m_imageFolder, m_depthFolder, m_rgbFolder, m_logFolder;
        std::vector<std::string> m_rgbImages;
//        std::vector<std::shared_ptr<Pose>> m_poses;
//        std::vector<  > m_logs;
        std::map<std::string, std::vector<std::shared_ptr<Pose>>> m_logs;
        bool mbHasPose;

        float m_poseBuffer[16];
        CameraParameters m_depthParam, m_rgbParam;
        float m_depthScale;
        std::unique_ptr<DataBuffer> m_depthBuffer, m_rgbBuffer, m_fe1Buffer, m_fe2Buffer;


        static bool loadLogFile(std::vector<std::shared_ptr<Pose>> &container, const std::string &pth){
            std::fstream file(pth,std::ios::in);
            if(!file.is_open()) {
                SCLOG(ERROR) << "cannot open file " << pth;
//                return false;
            }

            std::vector<std::string> headers;
            std::string line;
            getline(file,line);
            headers = tools::PathTool::splitLine(line, ' ');

//            for(const auto &s : headers)
//                printf("%s ",s.c_str());
//            printf("\n");

            while(getline(file,line)){
                std::shared_ptr<Pose> pose (new Pose);
                auto tokens = tools::PathTool::splitLine(line, ' ');
                if(tokens.size() != 10) throw std::runtime_error("wrong format");
                pose->idx = std::stoi(tokens[0]);
                pose->time = std::stof(tokens[1]);
                pose->occluded = std::stoi(tokens[2]);
                pose->t[0] = std::stof(tokens[3]);
                pose->t[1] = std::stof(tokens[4]);
                pose->t[2] = std::stof(tokens[5]);
                pose->r[0] = std::stof(tokens[6]);
                pose->r[1] = std::stof(tokens[7]);
                pose->r[2] = std::stof(tokens[8]);
                pose->r[3] = std::stof(tokens[9]);
                container.push_back(pose);
            }
            return true;
        }

        static void loadIntrinsics(CameraParameters & params, const std::string &path){
            std::fstream file(path, std::ios::in);
            if(!file.is_open())
                SCLOG(ERROR) << "cannot open file";//throw MultiConLoaderException(ERR_FILE_CANNOT_OPEN);
            auto getParam = [] (std::fstream &file, CameraParameters &params){
                if(!(file >> params.width)) throw std::runtime_error("Wrong file format.\n");
                if(!(file >> params.height)) throw std::runtime_error("Wrong file format.\n");
                if(!(file >> params.fx)) throw std::runtime_error("Wrong file format.\n");
                if(!(file >> params.fy)) throw std::runtime_error("Wrong file format.\n");
                if(!(file >> params.cx)) throw std::runtime_error("Wrong file format.\n");
                if(!(file >> params.cy)) throw std::runtime_error("Wrong file format.\n");
                if(!(file >> params.scale))  throw std::runtime_error("Wrong file format.\n");;
            };
            getParam(file, params);
        }

        static inline void getParam(const CameraParameters &param, int &width, int &height, float &fx,float &fy, float &px, float &py, float &scale){
            width = param.width;
            height = param.height;
            getParam(param,fx,fy,px,py,scale);
        }

        static inline void getParam(const CameraParameters &param, float &fx,float &fy, float &px, float &py, float &s){
            fx = param.fx;
            fy = param.fy;
            px = param.cx;
            py = param.cy;
            s  = param.scale;
        }

        static inline void ReadBinary(void *buffer, const std::string &path, unsigned long size){
            std::fstream file(path,std::ios::in | std::ios::binary);
            if(!file.is_open())throw std::runtime_error("");
            file.read(static_cast<char *>(buffer), size);
        }
    };
    typedef std::shared_ptr<MultiConLoader> MultiConLoaderPtr;
    static inline MultiConLoaderPtr MakeMultiConLoader(const std::string &path, const std::vector<std::string> &logNames = {"lola_vision"}) {
        return std::make_shared<MultiConLoader>(path,logNames);
    }
}
