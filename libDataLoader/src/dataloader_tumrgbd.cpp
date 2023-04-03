//
// Created by sc on 4/29/21.
//

#include "../include/dataLoader/dataloader_TUMRGBD.h"
#include <Eigen/Dense>
#include <fstream>
#include "Logging.h"
using namespace PSLAM;

DatasetLoader_TUMRGBD::DatasetLoader_TUMRGBD(std::shared_ptr<DatasetDefinitionBase> dataset):
        DatasetLoader_base(std::move(dataset)){
    int width,height;
    float fx,fy,cx,cy;
    float scale;
    std::vector<float> distortions;
    if(m_dataset->folder.find("freiburg1") != std::string::npos) {
        fx=517.306408;fy=	516.469215;cx=	318.643040;cy=	255.313989;
        distortions = {
                0.262383,
                -0.953104,
                -0.005358,
                0.002628,
                1.163314};
        scale = 1/5000.f;
    } else if(m_dataset->folder.find("freiburg2") != std::string::npos) {
        fx=520.908620;fy=	521.007327;cx=	325.141442;cy=	249.701764;
        distortions = {
                0.231222,
                -0.784899,
                -0.003257,
                -0.000105,
                0.917205
        };
        scale = 1/5208.0f;
    } else if(m_dataset->folder.find("freiburg3") != std::string::npos) {
        fx=535.4;fy=	539.2;cx=	320.1;cy=	247.6;
        distortions = {
                0,
                0,
                0,
                0,
        };
        scale = 1/5000.f;
    } else {
        throw std::runtime_error("unable to detect intrinsics");
    }
    width = 640; height = 480;
    m_cam_param_d.Set(width,height,fx,fy,cx,cy,scale);
    m_cam_param_rgb.Set(width,height,fx,fy,cx,cy,scale);
    m_cam_param_d.distortions = distortions;
    m_cam_param_rgb.distortions = distortions;


    this->Reset();
}

static bool ReadAssociatedFile(const std::string &filename, std::vector<std::string>& result_rgb, std::vector<std::string>& result_depth,
                        std::vector<double>& timestamps_rgb, std::vector<double>& timestamps_d) {
    std::ifstream fileDepthList(filename, std::ios::in);
    if (!fileDepthList.is_open()) return false;
    result_depth.clear();
    result_rgb.clear();
    timestamps_rgb.clear();
    timestamps_d.clear();
    std::string dump;
    while (fileDepthList.good())
    {
        double timestamp_rgb, timestamp_depth;
        std::string name_rgb,name_d;
        fileDepthList >> timestamp_rgb >> name_rgb  >> timestamp_depth >> name_d;
        if (name_rgb.empty() || name_d.empty()) break;
        timestamps_d.push_back(timestamp_depth);
        timestamps_rgb.push_back(timestamp_rgb);
        result_rgb.push_back(name_rgb);
        result_depth.push_back(name_d);
    }
    fileDepthList.close();
    return true;
}

static bool ReadTrajectoryFile(const std::string& filename,
                        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>& result,
                        std::vector<double>& timestamps)
{
    std::ifstream file(filename, std::ios::in);
    if (!file.is_open()) return false;
    result.clear();
    std::string dump;
    std::getline(file, dump);
    std::getline(file, dump);
    std::getline(file, dump);

    while (file.good())
    {
        double timestamp;
        file >> timestamp;
        Eigen::Vector3f translation;
        file >> translation.x() >> translation.y() >> translation.z();
        Eigen::Quaternionf rot;
        file >> rot.x() >> rot.y() >> rot.z() >> rot.w();

        rot.normalize();
        Eigen::Matrix4f transf;
        transf.setIdentity();
        transf.block<3, 3>(0, 0) = rot.toRotationMatrix();
        transf.block<3, 1>(0, 3) = translation;

        if (rot.norm() == 0) break;

//        transf = transf.inverse().eval();

        timestamps.push_back(timestamp);
        result.push_back(transf);
    }
    file.close();
    return true;
}

void DatasetLoader_TUMRGBD::Reset() {
    ReadTrajectoryFile(m_dataset->folder + NameGT,m_trajectories,m_timeTraj);
    ReadAssociatedFile(m_dataset->folder + NameAssociation, m_colorPaths, m_depthPaths,
                       m_timeColor, m_timeDepth);

    // Find valid pose association
    // find transformation (simple nearest neighbor, linear search)
    m_validFrameIdx.clear();
    m_validPoseIdx.clear();
    m_validFrameIdx.reserve(m_timeDepth.size());
    m_validPoseIdx.reserve(m_timeDepth.size());
    for (size_t i=0;i<m_timeDepth.size();++i) {
        auto timestamp = m_timeDepth.at(i);
        double min = std::numeric_limits<double>::max();
        size_t idx = 0;
        for (size_t j = 0; j < m_trajectories.size(); ++j)
        {
            double d = fabs(m_timeTraj[j] - timestamp);
            if (min > d)
            {
                min = d;
                idx = j;
            }
        }
        if ( min > 1e-2) continue;
        m_validFrameIdx.push_back(i);
        m_validPoseIdx.push_back(idx);
    }
    frame_index = 0;
    frame_index_max = m_validPoseIdx.size();
}

int DatasetLoader_TUMRGBD::Retrieve() {
    if(frame_index >= frame_index_max) return -1;
    int cur_idx = frame_index;
    auto frame_idx = m_validFrameIdx.at(frame_index);
    auto pose_idx = m_validPoseIdx.at(frame_index);
    m_d = cv::imread(m_dataset->folder+"/"+m_depthPaths.at(frame_idx), cv::IMREAD_UNCHANGED);
    m_rgb = cv::imread(m_dataset->folder+"/"+m_colorPaths.at(frame_idx), cv::IMREAD_UNCHANGED);
    if(m_rgb.empty()) throw std::runtime_error("empty!");
//    cv::cvtColor(m_rgb,m_rgb, cv::COLOR_RGB2BGR);
//    for(size_t c=0;c<(size_t)m_d.cols;++c){
//        for(size_t r=0;r<(size_t)m_d.rows;++r){
//            m_d.at<unsigned short>(r,c) = float(m_d.at<unsigned short>(r,c)) * m_cam_param_d.scale;
////            std::cout << m_d.at<unsigned short>(r,c)<< "\n";
//            if(m_d.at<unsigned short>(r,c)>=m_dataset->max_depth)
//                m_d.at<unsigned short>(r,c) = 0;
//        }
//    }

    m_pose = m_trajectories.at(pose_idx);

    //SCLOG(DEBUG) << "m_pose: " << m_pose;
    //m_pose.topRightCorner<3,1>()*=1e3;
    frame_index += m_dataset->frame_index_counter;
    return cur_idx;
}

double DatasetLoader_TUMRGBD::GetTime(int idx) {
    if(idx >= frame_index_max) return -1;
    auto pose_idx = m_validPoseIdx.at(idx);
    return m_timeTraj.at(pose_idx);
}
