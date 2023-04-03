//
// Created by sc on 4/29/21.
//

#ifndef GRAPHSLAM_DATALOADER_TUMRGBD_H
#define GRAPHSLAM_DATALOADER_TUMRGBD_H

#include "dataset_loader.h"
namespace PSLAM {
    class DatasetLoader_TUMRGBD : public DatasetLoader_base {
    public:
        DatasetLoader_TUMRGBD(std::shared_ptr<DatasetDefinitionBase> dataset);
        void Reset() override;
        int Retrieve() override;

        double GetTime(int idx);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        const std::string NameAssociation = "association.txt";
        const std::string NameGT = "groundtruth.txt";

        std::string pose_file_name_ = "";
        std::vector<std::string> m_depthPaths, m_colorPaths;
        std::vector<double> m_timeDepth, m_timeColor, m_timeTraj;
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> m_trajectories;
        std::vector<size_t> m_validFrameIdx, m_validPoseIdx;
//        std::map<size_t,size_t> m_timeStampMapping;//from depth to pose
    };
}
#endif //GRAPHSLAM_DATALOADER_TUMRGBD_H
