//
// Created by sc on 1/13/21.
//

#ifndef LIBSURFELRECONSTRUCTION_DATALOADER_3RSCAN_H
#define LIBSURFELRECONSTRUCTION_DATALOADER_3RSCAN_H
#include "dataset_loader.h"
namespace PSLAM {
    class DatasetLoader_3RScan : public DatasetLoader_base {
    public:
        DatasetLoader_3RScan(std::shared_ptr<DatasetDefinitionBase> dataset);
        void Reset() override;
        int Retrieve() override;

        [[nodiscard]] const std::string &GetRGBPath() const {return mPathRGB;}
        [[nodiscard]] const std::string &GetDepthPath() const {return mPathDepth;}
        [[nodiscard]] const std::string &GetPosePath() const {return mPathPose;}

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:

        Eigen::Matrix<float, 4, 4> rotation_matrix_Z(const float rot = M_PI);

        const std::string GetFileName(const std::string &folder,
                                      const std::string &subfolder,
                                      const std::string &prefix,
                                      const std::string &suffix,
                                      int number_length) const;
        bool IsV2() const;

        Eigen::Matrix4f m_poseTransform;
        std::string mPathDepth,mPathRGB, mPathPose;
    };
}

#endif //LIBSURFELRECONSTRUCTION_DATALOADER_3RSCAN_H
