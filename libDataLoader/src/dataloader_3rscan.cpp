//
// Created by sc on 1/13/21.
//
#include "../include/dataLoader/dataloader_3rscan.h"
#include "../include/dataLoader/util.h"
#include "../include/dataLoader/Scan3R_json_loader.h"
#include <Logging.h>

#include <utility>
#include <dataLoader/dataset3RScan.h>
#include <PathTool.hpp>

using namespace PSLAM;

static const std::vector<std::string> split(const std::string s, const std::string delim) {
    std::vector<std::string> list;
    auto start = 0U;
    auto end = s.find(delim);
    while (true) {
        list.push_back(s.substr(start, end - start));
        if (end == std::string::npos)
            break;
        start = end + delim.length();
        end = s.find(delim, start);
    }
    return list;
}

static bool LoadInfoIntrinsics(const std::string& filename,
                                  const bool depth_intrinsics,
                               CameraParameters& intrinsics) {
    const std::string search_tag = depth_intrinsics ? "m_calibrationDepthIntrinsic" : "m_calibrationColorIntrinsic";
    const std::string search_tag_w = depth_intrinsics? "m_depthWidth":"m_colorWidth";
    const std::string search_tag_h = depth_intrinsics? "m_depthHeight":"m_colorHeight";
    std::string line{""};
    std::ifstream file(filename);
    int width,height;
    float fx,fy,cx,cy;
    if (file.is_open()) {
        while (std::getline(file,line)) {
            if (line.rfind(search_tag_w, 0) == 0)
                width = std::stoi(line.substr(line.find("= ")+2, std::string::npos));
            else if (line.rfind(search_tag_h, 0) == 0)
                height = std::stoi(line.substr(line.find("= ")+2, std::string::npos));
            else if (line.rfind(search_tag, 0) == 0) {
                const std::string model = line.substr(line.find("= ")+2, std::string::npos);
                const auto parts = split(model, " ");
                fx = std::stof(parts[0]);
                fy = std::stof(parts[5]);
                cx = std::stof(parts[2]);
                cy = std::stof(parts[6]);
            }
        }
        file.close();
        intrinsics.Set(width,height,fx,fy,cx,cy,1.f);
        intrinsics.distortions = {0,0,0,0};
        return true;
    }
    return false;
}

DatasetLoader_3RScan::DatasetLoader_3RScan(std::shared_ptr<DatasetDefinitionBase> dataset):
        DatasetLoader_base(std::move(dataset)) {
    if(!LoadInfoIntrinsics(m_dataset->folder+"/_info.txt",true,m_cam_param_d))
        throw std::runtime_error("unable to open _info file");
    if(!LoadInfoIntrinsics(m_dataset->folder+"/_info.txt",false,m_cam_param_rgb))
        throw std::runtime_error("unable to open _info file");
    m_cam_param_d.scale = m_cam_param_rgb.scale = 1e-3;
    m_poseTransform.setIdentity();
    if(reinterpret_cast<PSLAM::Scan3RDataset*>(m_dataset.get())->use_aligned_pose) {
        auto seq_folder = tools::PathTool::find_parent_folder(m_dataset->folder,1);
        auto seq_name = tools::PathTool::getFileName(seq_folder);
        auto data_folder = tools::PathTool::find_parent_folder(seq_folder,1);
        auto scan3rLoader = PSLAM::io::Scan3RLoader(data_folder+"/3RScan.json");
        if(scan3rLoader.IsRescan(seq_name)) {
            // find ref scan ID
            auto ref_id = scan3rLoader.rescanToReference.at(seq_name);
            m_poseTransform = scan3rLoader.scaninfos.at(ref_id)->rescans.at(seq_name)->transformation;
            //m_poseTransform.topRightCorner<3,1>()*=1e3;
        }
    }
    std::vector<std::string> filesWIthName;
    tools::PathTool::get_files_include_name(m_dataset->folder,".pose.txt",filesWIthName,false,false);
    size_t counter=0;
    for(const auto& n : filesWIthName) {
        if (n.find(".align.pose.txt") != std::string::npos)continue;
        counter++;
    }

    frame_index_max = counter;

    Retrieve();
    Reset();
}

const std::string DatasetLoader_3RScan::GetFileName(const std::string& folder,
                                                    const std::string& subfolder,
                                                    const std::string& prefix,
                                                    const std::string& suffix,
                                                    int number_length = -1) const {
    std::stringstream filename;
    const std::string path = (folder == "/" ? "" : folder) +
                             (subfolder == "/" ? "" : subfolder) +
                             (prefix == "/" ? "" : prefix);
    if (number_length < 0)
        filename << path << (suffix == "/" ? "" : suffix);
    else
        filename << path << std::setfill('0') << std::setw(number_length) << frame_index << (suffix == "/" ? "" : suffix);
    std::string s(filename.str());
    return s;
}

bool DatasetLoader_3RScan::IsV2() const {
    return  m_dataset->rotate_pose_img;
}

int DatasetLoader_3RScan::Retrieve() {
    if(frame_index>=frame_index_max) return -1;
    mPathDepth = GetFileName(m_dataset->folder,
                                            m_dataset->folder_depth,
                                            m_dataset->prefix_depth,
                                            m_dataset->suffix_depth,
                                            m_dataset->number_length);
    mPathRGB = GetFileName(m_dataset->folder,
                                            m_dataset->folder_rgb,
                                            m_dataset->prefix_rgb,
                                            m_dataset->suffix_rgb,
                                            m_dataset->number_length);
    mPathPose = GetFileName(m_dataset->folder,
                                  m_dataset->folder_pose,
                                  m_dataset->prefix_pose,
                                  m_dataset->suffix_pose,
                                  m_dataset->number_length);
    bool isExist = isFileExist(mPathDepth);
    if (!isExist) {
        //frame_index = 0;
        SCLOG(VERBOSE) << "Cannot find path:\n" << mPathDepth << "\n" << mPathRGB;
        return -1;
    }
    m_d = cv::imread(mPathDepth, -1);
    // mask depth
    for(size_t c=0;c<(size_t)m_d.cols;++c){
        for(size_t r=0;r<(size_t)m_d.rows;++r){
            if(m_d.at<unsigned short>(r,c)>=m_dataset->max_depth)
                m_d.at<unsigned short>(r,c) = 0;
        }
    }

    if (isFileExist(mPathRGB.c_str())) {
        m_rgb = cv::imread(mPathRGB, -1);
    }
    if (m_dataset->rotate_pose_img) {
        cv::rotate(m_d, m_d, cv::ROTATE_90_COUNTERCLOCKWISE);
    }
    LoadPose(m_pose, mPathPose,m_dataset->rotate_pose_img);
    m_pose = m_poseTransform * m_pose;
    int cur_idx = frame_index;
    frame_index += m_dataset->frame_index_counter;
    return cur_idx;
}



void DatasetLoader_3RScan::Reset() {
    frame_index = 0;
}

Eigen::Matrix<float,4,4> DatasetLoader_3RScan::rotation_matrix_Z(const float rot) {
    Eigen::Matrix<float,4,4> res = Eigen::Matrix<float,4,4>::Identity();
    res << cos(rot), -sin(rot), 0, 0,
            sin(rot), cos(rot), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    return res;
}