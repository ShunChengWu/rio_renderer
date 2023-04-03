//
// Created by sc on 1/13/21.
//

#ifndef LIBSURFELRECONSTRUCTION_DATALOADER_IMG_H
#define LIBSURFELRECONSTRUCTION_DATALOADER_IMG_H
#include "dataset_loader.h"
#include <fstream>
#include <PathTool.hpp>
namespace PSLAM {
    class DatasetLoader_IMG : public DatasetLoader_base {
    public:
        explicit DatasetLoader_IMG(const std::shared_ptr<DatasetDefinitionBase>& dataset)
        : DatasetLoader_base(dataset){
            // load pose
            std::fstream  file(dataset->folder+"/CameraTrajectory.txt", std::ios::in);
            float data[9];
            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            while(file >> data[0] >> data[1] >> data[2] >> data[3]
            >> data[4] >> data[5] >> data[6] >>data[7]
            >> data[8] >> data[9] >> data[10] >> data[11] ) {
                pose <<
                data[0] , data[1] , data[2] , data[3] ,
                data[4] , data[5] , data[6] , data[7] ,
                data[8] , data[9] , data[10] , data[11],
                0, 0, 0, 1;
                m_poses.push_back(pose);
            }
            SCLOG(DEBUG) << m_poses.size();


            m_depths = tools::PathTool::get_files_in_folder(dataset->folder+"/Depth","",true);
            m_rgbs = tools::PathTool::get_files_in_folder(dataset->folder+"/RGB","",true);
            frame_index_max = m_rgbs.size();
            Retrieve();
            Reset();
            /*
             * [[458.9553651696184, 0.0, 308.15745544433594],
                                       [0.0, 458.9553651696184, 262.75226974487305],
                                       [0.0, 0.0, 1.0]]
             * */
            m_cam_param_d.Set(m_d.cols,m_d.rows,458.9553651696184,458.9553651696184,308.15745544433594,262.75226974487305, 1.0);
            m_cam_param_rgb.Set(m_rgb.cols,m_rgb.rows,458.9553651696184,458.9553651696184,308.15745544433594,262.75226974487305, 1.0);
        }
        void Reset() override {
            frame_index = 0;

        }
        int Retrieve() override {
            if(frame_index >= frame_index_max) return false;
            m_d = cv::imread(m_depths.at(frame_index), -1);
            m_rgb = cv::imread(m_rgbs.at(frame_index), -1);
            m_pose = m_poses.at(frame_index);
            m_pose.topRightCorner<3,1>() *= 1e3;
            int idx = frame_index;
            frame_index += m_dataset->frame_index_counter;
            return idx;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        std::vector<std::string> m_depths, m_rgbs;
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> m_poses;
        std::string pose_file_name_;


        Eigen::Matrix4f m_poseTransform;
    };
}

#endif //LIBSURFELRECONSTRUCTION_DATALOADER_3RSCAN_H
