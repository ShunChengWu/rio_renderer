//
// Created by sc on 1/17/21.
//

#ifndef DFG_MULTICON_VISION_DATALOADERDFGIMG_H
#define DFG_MULTICON_VISION_DATALOADERDFGIMG_H
#include "datasetDFGIMG.h"
#include "dataset_loader.h"
//#include <dataLoader/dataloa>
//#include "dataloaderInterface.h"
#include "DFGDataLoader.h"
namespace PSLAM {
    class DataLoader_DFGIMG : public DatasetLoader_base  {
    public:
        explicit DataLoader_DFGIMG(
                const std::shared_ptr<DatasetDefinitionBase>& dataset): DatasetLoader_base(dataset)
                {
            m_converted_dataset = dynamic_cast<DFGIMGDataset*>(dataset.get());
            if(!m_converted_dataset) {
                throw std::runtime_error("wrong input dataset type");
            }
            this->Reset();
        }

        int Retrieve() override {
            auto idx = m_idx;
            m_idx += m_dataset->frame_index_counter;
            if(idx >= 0 && idx < (int)m_multiconLoader->m_depthImages.size()){
                m_rgb = m_multiconLoader->GetRGBImage(idx);
                m_d = m_multiconLoader->GetDepthImage(idx);

                auto data = m_multiconLoader->GetPose(idx);
                if(!data)
                    m_pose =  Eigen::Matrix4f::Zero();
                else
                    m_pose = Eigen::Matrix4f(data) * m_tmat;

//                std::cerr << "pose\n" << m_pose << "\n";
            }

            return idx;
        }

        void Reset() override {
            m_idx = 0;
            // remove the last '/'
            m_converted_dataset->folder = m_converted_dataset->folder.back() == '/'?
                    m_converted_dataset->folder.substr(0, m_converted_dataset->folder.length()-1) :
                                          m_converted_dataset->folder;

            m_multiconLoader = MakeMultiConLoader(m_converted_dataset->folder,m_converted_dataset->logNames);
            m_multiconLoader->GetDepthParam(reinterpret_cast<int &>(m_cam_param_d.width),
                                            reinterpret_cast<int &>(m_cam_param_d.height),
                                            m_cam_param_d.fx, m_cam_param_d.fy, m_cam_param_d.cx, m_cam_param_d.cy,
                                            m_cam_param_d.scale);
            m_multiconLoader->GetRGBParam(reinterpret_cast<int &>(m_cam_param_rgb.width),
                                          reinterpret_cast<int &>(m_cam_param_rgb.height),
                                          m_cam_param_rgb.fx, m_cam_param_rgb.fy, m_cam_param_rgb.cx, m_cam_param_rgb.cy,
                                          m_cam_param_rgb.scale);
            m_cam_param_d = m_cam_param_rgb; // depth image is aligned to color

            // try to load the transformation matrix from Vicon to optical center coordinates.
            auto mother_path = m_converted_dataset->folder.substr(
                    0, m_converted_dataset->folder.find_last_of('/')
                    );
            std::fstream ftmat (mother_path+"/TMatrix.txt",std::ios::in);
            if(ftmat.is_open()){
                for (size_t i = 0; i < 4; i++)
                    ftmat >> m_tmat(i, 0) >> m_tmat(i, 1)
                      >> m_tmat(i, 2) >> m_tmat(i, 3);

//                m_tmat.topRightCorner<3,1>() *= 1e-3;
                m_tmat = m_tmat.inverse().eval();

                std::cerr << "loaded the transformation from Vicon to camera optical center :\n" << m_tmat << "\n";
            }
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        int m_idx=0;
        MultiConLoaderPtr m_multiconLoader;
        DFGIMGDataset* m_converted_dataset;
        Eigen::Matrix4f m_tmat = Eigen::Matrix4f::Identity();
//        cv::Mat m_lastRGB, m_lastDepth;

    };
}

#endif //DFG_MULTICON_VISION_DATALOADERDFGIMG_H
