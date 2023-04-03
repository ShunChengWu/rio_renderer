//
// Created by sc on 4/29/21.
//

#include "../include/dataLoader/dataloader_ICLNUIM.h"
#include <Eigen/Dense>
#include <fstream>
using namespace PSLAM;

DatasetLoader_ICLNUIM::DatasetLoader_ICLNUIM(std::shared_ptr<DatasetDefinitionBase> dataset):
        DatasetLoader_base(std::move(dataset)){
    int width,height;
    float fx,fy,cx,cy;
    fx=481.2;fy=	-480.00;cx=	319.5;cy=	239.5;
    width = 640; height = 480;
    m_cam_param_d.Set(width,height,fx,fy,cx,cy);
    m_cam_param_rgb.Set(width,height,fx,fy,cx,cy);

    this->Reset();
}

bool ReadAssociatedFile(const std::string &filename, std::vector<std::string>& result_rgb, std::vector<std::string>& result_depth,
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
        fileDepthList >> timestamp_depth >> name_d  >> timestamp_rgb >> name_rgb;
        if (filename == "") break;
        timestamps_d.push_back(timestamp_depth);
        timestamps_rgb.push_back(timestamp_rgb);
        result_rgb.push_back(name_rgb);
        result_depth.push_back(name_d);
    }
    fileDepthList.close();
    return true;
}

bool ReadTrajectoryFile(const std::string& filename,
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

void DatasetLoader_ICLNUIM::Reset() {

    ReadTrajectoryFile(m_dataset->folder + "/livingRoom0.gt.freiburg",m_trajectories,m_timeTraj);
    ReadAssociatedFile(m_dataset->folder + "/associations.txt", m_colorPaths, m_depthPaths,
                       m_timeColor, m_timeDepth);

    // Find valid pose association
    // find transformation (simple nearest neighbor, linear search)
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

int DatasetLoader_ICLNUIM::Retrieve() {
    if(frame_index >= frame_index_max) return false;
    int cur_idx = frame_index;
    auto frame_idx = m_validFrameIdx.at(frame_index);
    auto pose_idx = m_validPoseIdx.at(frame_index);
    m_d = cv::imread(m_dataset->folder+"/"+m_depthPaths.at(frame_idx), -1);
    m_rgb = cv::imread(m_dataset->folder+"/"+m_colorPaths.at(frame_idx), cv::IMREAD_ANYCOLOR);
    cv::cvtColor(m_rgb,m_rgb, cv::COLOR_RGB2BGR);
    for(size_t c=0;c<(size_t)m_d.cols;++c){
        for(size_t r=0;r<(size_t)m_d.rows;++r){
            m_d.at<unsigned short>(r,c) = float(m_d.at<unsigned short>(r,c)) * 1/5;
//            std::cout << m_d.at<unsigned short>(r,c)<< "\n";
            if(m_d.at<unsigned short>(r,c)>=m_dataset->max_depth)
                m_d.at<unsigned short>(r,c) = 0;
        }
    }

    m_pose = m_trajectories.at(pose_idx);
    m_pose.topRightCorner<3,1>()*=1e3;
    frame_index += m_dataset->frame_index_counter;

    if(1)
    {
        Eigen::Matrix4f mat4f;
        mat4f.setIdentity();

        Eigen::Matrix3f mat3f;
        mat3f = Eigen::AngleAxisf(0.5 * EIGEN_PI, Eigen::Vector3f::UnitX());
        mat4f.topLeftCorner<3, 3>() = mat3f;
        m_pose = mat4f * m_pose;
//        mat3f = Eigen::AngleAxisf(-0.5 * EIGEN_PI, Eigen::Vector3f::UnitY());
//        mat4f.topLeftCorner<3, 3>() = mat3f;
//        m_pose = mat4f * m_pose;
    }

    return cur_idx;
}